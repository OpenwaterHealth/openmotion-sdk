"""Source protocol + concrete source implementations.

A Source produces an iterator of FrameBatch and carries the ScanMetadata
for the scan. Concrete sources:
  - CsvReplaySource  — replays raw histogram CSVs (Task 20)
  - DbReplaySource   — replays scan-DB session_raw rows (Task 21)
  - LiveUsbSource    — reads from USB on background threads (Task 22)
"""

from __future__ import annotations

import csv
import queue
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any, Iterator, Optional, Protocol, runtime_checkable

import numpy as np

from .batch import FrameBatch, TelemetryEvent
from .sinks import ScanMetadata


@runtime_checkable
class Source(Protocol):
    metadata: ScanMetadata

    def __iter__(self) -> Iterator[FrameBatch]: ...

    def close(self) -> None: ...


class _BaseSource:
    """Shared scaffolding for concrete sources."""

    def __init__(self, *, metadata: ScanMetadata,
                 normalize_timestamps: bool = True):
        self.metadata = metadata
        self._normalize_timestamps = normalize_timestamps
        self._t0: Optional[float] = None

    def _apply_timestamp_normalization(self, timestamp_s: np.ndarray) -> np.ndarray:
        """Subtract the first observed timestamp so scans always start at t=0.

        The offset is set on the first batch and held fixed for all subsequent
        batches in the same scan. Thread-safety is not a concern here because
        sources are consumed by a single thread.
        """
        if not self._normalize_timestamps:
            return timestamp_s
        if self._t0 is None:
            if len(timestamp_s) > 0:
                self._t0 = float(timestamp_s[0])
            else:
                return timestamp_s
        return timestamp_s - self._t0

    def close(self) -> None:
        pass


class CsvReplaySource(_BaseSource):
    """Replays a raw-histogram CSV produced by CsvSink.

    CSV schema (SciencePipeline.md §12 / spec §12):
        cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc

    Accepts up to two CSVs (one per side); if a side is None, only the
    other side is replayed.
    """

    def __init__(self, *,
                 raw_csv_left: Optional[Path],
                 raw_csv_right: Optional[Path],
                 batch_size_frames: int = 100,
                 metadata: ScanMetadata,
                 normalize_timestamps: bool = True):
        super().__init__(metadata=metadata, normalize_timestamps=normalize_timestamps)
        self._paths = {"left": raw_csv_left, "right": raw_csv_right}
        self._batch_size = int(batch_size_frames)

    def __iter__(self) -> Iterator[FrameBatch]:
        for side_name, path in self._paths.items():
            if path is None:
                continue
            yield from self._iter_side(side_name, path)

    def _iter_side(self, side_name: str, path: Path) -> Iterator[FrameBatch]:
        side_idx = 0 if side_name == "left" else 1
        rows_buf: list[dict] = []
        with open(path, "r", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                rows_buf.append(row)
                if len(rows_buf) >= self._batch_size:
                    yield self._rows_to_batch(side_idx, rows_buf)
                    rows_buf = []
            if rows_buf:
                yield self._rows_to_batch(side_idx, rows_buf)

    def _rows_to_batch(self, side_idx: int, rows: list[dict]) -> FrameBatch:
        n = len(rows)
        cam_ids     = np.array([int(r["cam_id"])        for r in rows], dtype=np.int8)
        frame_ids   = np.array([int(r["frame_id"])      for r in rows], dtype=np.uint8)
        timestamp_s = np.array([float(r["timestamp_s"]) for r in rows], dtype=np.float64)
        timestamp_s = self._apply_timestamp_normalization(timestamp_s)

        raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temp_arr = np.zeros((n, 2, 8), dtype=np.float32)
        for i, row in enumerate(rows):
            cam = int(row["cam_id"])
            for b in range(1024):
                raw_hist[i, side_idx, cam, b] = int(row[str(b)])
            temp_arr[i, side_idx, cam] = float(row["temperature"])

        return FrameBatch(
            cam_ids=cam_ids,
            frame_ids=frame_ids,
            raw_histograms=raw_hist,
            temperature_c=temp_arr,
            timestamp_s=timestamp_s,
            pdc=None, tcm=None, tcl=None,
        )


class DbReplaySource(_BaseSource):
    """Replays a scan-DB session by reading rows out of session_raw.

    Assumes the table layout used by ScanDBSink (see omotion/ScanDatabase.py).
    Each session_raw row carries a 4096-byte histogram blob (1024 × uint32 LE)
    in the `hist` column.
    """

    def __init__(self, *, db_path: str, session_id: int,
                 batch_size_frames: int = 100,
                 metadata: ScanMetadata,
                 normalize_timestamps: bool = True):
        super().__init__(metadata=metadata, normalize_timestamps=normalize_timestamps)
        self._db_path = db_path
        self._session_id = int(session_id)
        self._batch_size = int(batch_size_frames)

    def __iter__(self) -> Iterator[FrameBatch]:
        conn = sqlite3.connect(self._db_path)
        try:
            cur = conn.execute(
                "SELECT side, cam_id, frame_id, timestamp_s, hist, "
                "       temp, sum "
                "FROM session_raw WHERE session_id = ? "
                "ORDER BY timestamp_s, side, cam_id",
                (self._session_id,),
            )
            buf: list[tuple] = []
            for row in cur:
                buf.append(tuple(row))
                if len(buf) >= self._batch_size:
                    yield self._rows_to_batch(buf)
                    buf = []
            if buf:
                yield self._rows_to_batch(buf)
        finally:
            conn.close()

    def _rows_to_batch(self, rows: list[tuple]) -> FrameBatch:
        n = len(rows)
        cam_ids     = np.zeros(n, dtype=np.int8)
        frame_ids   = np.zeros(n, dtype=np.uint8)
        timestamp_s = np.zeros(n, dtype=np.float64)
        raw_hist    = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temp_arr    = np.zeros((n, 2, 8), dtype=np.float32)

        for i, (side, cam_id, frame_id, t, blob, temp, _row_sum) in enumerate(rows):
            side_idx = 0 if side == "left" else 1
            cam_ids[i]     = int(cam_id)
            frame_ids[i]   = int(frame_id)
            timestamp_s[i] = float(t)
            hist_bytes = bytes(blob)
            hist_arr = np.frombuffer(hist_bytes, dtype=np.uint32, count=1024)
            raw_hist[i, side_idx, int(cam_id)] = hist_arr
            if temp is not None:
                temp_arr[i, side_idx, int(cam_id)] = float(temp)

        timestamp_s = self._apply_timestamp_normalization(timestamp_s)

        return FrameBatch(
            cam_ids=cam_ids, frame_ids=frame_ids,
            raw_histograms=raw_hist, temperature_c=temp_arr,
            timestamp_s=timestamp_s,
            pdc=None, tcm=None, tcl=None,
        )


class LiveUsbSource(_BaseSource):
    """Per-side packet queues + per-side reader threads → shared batch queue.

    Each StreamInterface (one per side) pushes raw bytes into its own
    _packet_queues[side] entry.  A per-side reader thread runs
    parse_histogram_stream against that queue, accumulating the parsed
    HistogramSamples into FrameBatch objects which it pushes to the
    shared _batch_queue.  The runner pulls FrameBatches off _batch_queue
    via __iter__.
    """

    def __init__(self, *,
                 console: Any, left: Any, right: Any,
                 batch_size_frames: int = 10,
                 flush_interval_s: float = 0.25,
                 packet_queue_size: int = 64,
                 metadata: ScanMetadata):
        super().__init__(metadata=metadata)
        self._console = console
        self._sensors: dict[str, Any] = {"left": left, "right": right}
        self._batch_size = int(batch_size_frames)
        self._flush_interval = float(flush_interval_s)
        self._packet_queues: dict[str, queue.Queue] = {
            side: queue.Queue(maxsize=packet_queue_size)
            for side, sensor in self._sensors.items() if sensor is not None
        }
        self._batch_queue: queue.Queue = queue.Queue(maxsize=4)
        self._stop = threading.Event()
        self._reader_threads: list[threading.Thread] = []

    def __iter__(self) -> Iterator[FrameBatch]:
        from omotion.MotionProcessing import HISTOGRAM_BYTES

        self._expected_size = HISTOGRAM_BYTES

        for side_name in self._packet_queues:
            sensor = self._sensors[side_name]
            sensor.uart.histo.start_streaming(
                self._packet_queues[side_name],
                expected_size=HISTOGRAM_BYTES,
            )
            t = threading.Thread(
                target=self._reader_loop, args=(side_name,),
                name=f"LiveUsbSource-{side_name}", daemon=True,
            )
            t.start()
            self._reader_threads.append(t)

        while not self._stop.is_set():
            try:
                batch = self._batch_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            if batch is None:
                break
            yield batch

    def close(self) -> None:
        # Match the legacy stop sequence: stop_streaming → drain_final.
        # ScanWorkflow handles the trigger-off + ~0.35s DMA-flush wait before
        # calling close(); here we just shut down the USB stream and feed any
        # late-arriving final transfer back through the parser before the
        # reader threads exit.
        self._stop.set()
        expected_size = getattr(self, "_expected_size", None)
        for side_name, sensor in self._sensors.items():
            if sensor is None or getattr(sensor, "uart", None) is None:
                continue
            histo = getattr(sensor.uart, "histo", None)
            if histo is None:
                continue
            try:
                histo.stop_streaming()
            except Exception:
                pass
            if expected_size is not None:
                try:
                    final_chunks = histo.drain_final(expected_size=expected_size)
                    q = self._packet_queues.get(side_name)
                    if q is not None and final_chunks:
                        for chunk in final_chunks:
                            try:
                                q.put(chunk, timeout=0.5)
                            except queue.Full:
                                pass
                except Exception:
                    pass
        for t in self._reader_threads:
            t.join(timeout=2.0)

    def _reader_loop(self, side_name: str) -> None:
        """Per-side reader. Delegates packet parsing to parse_histogram_stream;
        accumulates HistogramSamples into FrameBatches and pushes them to the
        shared batch queue.
        """
        from omotion.MotionProcessing import (
            parse_histogram_stream, EXPECTED_HISTOGRAM_SUM,
        )

        side_idx = 0 if side_name == "left" else 1
        accumulated: list = []
        last_flush = time.monotonic()

        def on_row(cam_id, frame_id, ts, histogram, row_sum, temp):
            nonlocal last_flush
            accumulated.append((cam_id, frame_id, ts, histogram, row_sum, temp))
            now = time.monotonic()
            if (len(accumulated) >= self._batch_size
                    or now - last_flush >= self._flush_interval):
                self._batch_queue.put(self._build_batch(side_idx, accumulated))
                accumulated.clear()
                last_flush = now

        buf = bytearray()
        parse_histogram_stream(
            self._packet_queues[side_name], self._stop, buf,
            on_row_fn=on_row,
            expected_row_sum=EXPECTED_HISTOGRAM_SUM,
            t0_normalizer=getattr(self, "_t0_normalize", None),
        )
        # Flush any remaining samples after parse_histogram_stream returns
        if accumulated:
            self._batch_queue.put(self._build_batch(side_idx, accumulated))

    def _build_batch(self, side_idx: int, samples: list) -> FrameBatch:
        """Convert a list of (cam_id, frame_id, ts, histogram, row_sum, temp)
        tuples into one FrameBatch with (N, 2, 8, 1024) shape, populating the
        (side_idx, cam_id) slot in the histograms array for each row.
        """
        n = len(samples)
        cam_ids     = np.array([s[0] for s in samples], dtype=np.int8)
        frame_ids   = np.array([s[1] for s in samples], dtype=np.uint8)
        timestamp_s = np.array([s[2] for s in samples], dtype=np.float64)
        raw_hist    = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
        temps       = np.zeros((n, 2, 8), dtype=np.float32)
        for i, (cam_id, _, _, histogram, _, temp) in enumerate(samples):
            raw_hist[i, side_idx, cam_id] = histogram
            temps[i, side_idx, cam_id] = temp
        return FrameBatch(
            cam_ids=cam_ids, frame_ids=frame_ids,
            raw_histograms=raw_hist, temperature_c=temps,
            timestamp_s=timestamp_s, pdc=None, tcm=None, tcl=None,
        )


class ConsoleTelemetrySource:
    """Polls MotionConsole.telemetry at fixed cadence; yields TelemetryEvent with
    scan-relative timestamps.

    Used as the optional `telemetry_source` on ScanRunner. Doesn't produce
    FrameBatch — parallel input that flows to "telemetry"-channel sinks
    and feeds the pipeline's TelemetryAggregator.
    """

    def __init__(self, *, console, poll_interval_s: float = 0.1):
        self._console = console
        self._poll_interval_s = poll_interval_s
        self._stop = threading.Event()
        self._t0 = None

    def __iter__(self):
        while not self._stop.is_set():
            snap = self._console.telemetry.get_snapshot()
            if snap is None:
                time.sleep(self._poll_interval_s)
                continue
            if self._t0 is None:
                self._t0 = snap.timestamp
            # Map ConsoleTelemetry fields to TelemetryEvent.
            # tec_set_raw / tec_v_raw are raw ADC voltages; app converts to °C
            # For now, pass raw values (they'll be converted by the sink/aggregator).
            # safety_ok is a bool; convert to int (0=ok, 1=fault) for safety_status.
            yield TelemetryEvent(
                timestamp_s=snap.timestamp - self._t0,
                pdc_samples=[snap.pdc],  # pdc is a single float (mA), wrap in list
                tec_setpoint_c=snap.tec_set_raw,  # raw setpoint voltage (app converts)
                tec_actual_c=snap.tec_v_raw,      # raw measured voltage (app converts)
                console_temp_c=0.0,                # not available in ConsoleTelemetry snapshot
                fan_rpm=0,                         # not available in ConsoleTelemetry snapshot
                safety_status=0 if snap.safety_ok else 1,  # 0=ok, 1=fault
            )

    def close(self) -> None:
        self._stop.set()
