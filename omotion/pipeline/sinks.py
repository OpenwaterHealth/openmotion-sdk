"""Sink protocol + ScanMetadata.

Concrete sink implementations (CsvSink, ScanDBSink) live below the
protocol definitions. The live-plot UI sink lives in the bloodflow-app
(`motion_connector.py` — `_LivePlotSink` for the `"live"` channel and
`_FinalBatchSink` for the `"final"` channel) rather than in the SDK.
"""

from __future__ import annotations

import collections
import csv
import logging
import os
from dataclasses import dataclass
from typing import Any, Optional, Protocol, runtime_checkable

from omotion.config import HISTO_SIZE_WORDS

logger = logging.getLogger("openmotion.sdk.pipeline.sinks")


@dataclass(frozen=True)
class ScanMetadata:
    """Per-scan metadata handed to every sink at on_scan_start.
    Output gating lives elsewhere (raw-save gate is on the pipeline's
    Tee("raw"); default storage sinks are SDK-injected at start_scan)."""
    scan_id:           str
    subject_id:        str
    operator:          str
    started_at_iso:    str
    duration_sec:      int
    left_camera_mask:  int
    right_camera_mask: int
    reduced_mode:      bool


@runtime_checkable
class Sink(Protocol):
    """A consumer of pipeline output.

    Channels in this pipeline:
        "raw"          — per-frame, all non-stale frames including warmup
        "live"         — per-frame, best-effort corrected (light + dark)
        "live_side"    — reduced mode: realtime per-side average
                         (SideAverageSample), one per capture per side
        "final"        — per-dark-interval, accurately corrected
                         EnrichedCorrectedInterval; in reduced mode this
                         also carries the side-average intervals whose
                         frames have cam_id=-1
        "diagnostics"  — out-of-band events (DarkIntegrityWarning, etc.)

    A sink may set ``critical = True`` to mean "if my on_scan_start fails,
    abort the whole scan" (ScanRunner raises CriticalSinkError). The default
    is False: a failing sink is disabled for the scan and the rest continue.
    ``critical`` is optional (the runner reads it via getattr with a False
    default), so it is deliberately NOT declared as a protocol member — adding
    a data attribute here would make runtime isinstance() checks require it.
    """
    channels: set[str]

    def on_scan_start(self, meta: ScanMetadata) -> None: ...

    def consume(self, channel: str, payload: Any) -> None: ...

    def on_complete(self) -> None: ...


# ---------------------------------------------------------------------------
# Concrete sinks
# ---------------------------------------------------------------------------

# Raw CSV column order: cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc
_RAW_PIPELINE_HEADERS: list = [
    "cam_id", "frame_id", "timestamp_s", "type",
    *list(range(HISTO_SIZE_WORDS)),
    "temperature", "sum",
    "tcm", "tcl", "pdc",
]


def _corrected_headers_normal() -> list[str]:
    """83-column corrected CSV header (82 metric columns + quality)."""
    cols = ["frame_id", "timestamp_s"]
    for metric in ("bfi", "bvi", "mean", "contrast", "temp"):
        for side in ("l", "r"):
            for cam in range(1, 9):
                cols.append(f"{metric}_{side}{cam}")
    cols.append("quality")
    return cols


def _corrected_headers_reduced() -> list[str]:
    """7-column reduced corrected CSV header."""
    return ["frame_id", "timestamp_s", "bfi_left", "bfi_right", "bvi_left", "bvi_right", "quality"]


# Build the column-index lookup once at module load time.
_NORMAL_HEADERS = _corrected_headers_normal()
_REDUCED_HEADERS = _corrected_headers_reduced()


def _scalar_or_blank(arr, i):
    """Pull arr[i] as a float, returning "" for missing/None/NaN values.

    Used for raw-CSV optional cells where ``""`` is the schema's missing-value
    marker.
    """
    if arr is None:
        return ""
    try:
        v = float(arr[i])
    except (TypeError, ValueError):
        return ""
    if v != v:  # NaN
        return ""
    return v


def _source_side_indices(batch, i: int, cam_id: int, meta: ScanMetadata):
    """Yield the active source side(s) for one raw row.

    New live sources set ``side_ids`` per row; when absent, keep the older
    mask-based fallback for replay/test batches that have not been migrated.
    """
    masks = (meta.left_camera_mask, meta.right_camera_mask)
    names = ("left", "right")
    side_ids = getattr(batch, "side_ids", None)
    if side_ids is not None:
        side_idx = int(side_ids[i])
        if side_idx not in (0, 1):
            logger.warning(
                "raw frame skipped with invalid side_id=%s cam_id=%d frame_id=%s",
                side_idx, cam_id, int(batch.frame_ids[i]),
            )
            return
        mask = masks[side_idx]
        if mask and (mask & (1 << cam_id)):
            yield side_idx, names[side_idx], mask
        return

    for side_idx, side_name in enumerate(names):
        mask = masks[side_idx]
        if mask and (mask & (1 << cam_id)):
            yield side_idx, side_name, mask

# Maps (metric, side_char, cam_1indexed) -> column index in _NORMAL_HEADERS
_NORMAL_COL_IDX: dict[tuple[str, str, int], int] = {
    (metric, side, cam): _NORMAL_HEADERS.index(f"{metric}_{side}{cam}")
    for metric in ("bfi", "bvi", "mean", "contrast", "temp")
    for side in ("l", "r")
    for cam in range(1, 9)
}

# Quality ranking: higher rank means worse quality; used to keep the worst
# quality value seen across all cameras contributing to one output row.
_QUALITY_RANK: dict[str, int] = {"ok": 0, "ts_corrected": 1, "nan_filled": 2}


class CsvSink:
    """Channel-based CSV sink for the pipeline.

    Channels:
        "raw"   — per-frame raw histograms (gated by meta.write_raw_csv)
        "final" — per-interval corrected output

    Raw file naming:       ``{scan_id}_{subject_id}_{side}_mask{XX}_raw.csv``
    Corrected file naming: ``{scan_id}_{subject_id}.csv``

    The corrected stream is the default output — naming it ``_corrected``
    doesn't add information, and the ``_raw`` suffix on histogram CSVs
    already disambiguates them. See issue #44 / commit 71bee4c for the
    rationale; the pipeline cutover's first pass regressed this back to
    ``_corrected.csv`` until it was restored.

    Normal mode corrected CSV: 82-column wide format matching legacy SciencePipeline
    output (frame_id, timestamp_s, bfi_l1..bfi_r8, bvi_l1..bvi_r8,
    mean_l1..mean_r8, contrast_l1..contrast_r8, temp_l1..temp_r8).

    Reduced mode corrected CSV: 6 columns (frame_id, timestamp_s,
    bfi_left, bfi_right, bvi_left, bvi_right).

    Files are created lazily on first "final" / "raw" consume call.
    """

    channels = {"raw", "final"}

    def __init__(self, output_dir, write_corrected: bool = True) -> None:
        self._output_dir = str(output_dir)
        # When False, the corrected CSV ({scan_id}_{subject}.csv) is not
        # written — the scan DB's session_data is the system of record
        # for per-cam BFI/BVI instead. Raw histogram CSV handling (the
        # "raw" channel, separately gated by meta.write_raw_csv) is
        # unaffected. The SDK runner forces this True when no scan DB is
        # configured so there's always at least one persisted record.
        self._write_corrected = bool(write_corrected)
        self._meta: Optional[ScanMetadata] = None
        self._raw_fhs: dict[str, Any] = {}    # side -> file handle
        self._raw_csvs: dict[str, Any] = {}   # side -> csv.writer
        self._corrected_fh: Optional[Any] = None
        self._corrected_csv: Optional[Any] = None
        self._closed = False
        # Per-frame accumulator: abs_frame_id -> {"t": float, "row": list}
        self._corrected_acc: "dict[int, dict]" = {}
        self._corrected_n_cols: int = 0
        self._corrected_reduced: bool = False
        self._next_flush_id: Optional[int] = None
        self._corrected_rows_since_flush: int = 0
        self._expected_cams: "dict[str, set[int]]" = {}  # side -> set of cam_ids

    def on_scan_start(self, meta: ScanMetadata) -> None:
        self._meta = meta
        self._closed = False
        self._corrected_fh = None
        self._corrected_csv = None
        self._corrected_acc = {}
        self._corrected_reduced = meta.reduced_mode
        self._next_flush_id = None
        # Build set of expected cam_ids per side from masks
        self._expected_cams = {
            "left":  {c for c in range(8) if meta.left_camera_mask  & (1 << c)},
            "right": {c for c in range(8) if meta.right_camera_mask & (1 << c)},
        }
        if meta.reduced_mode:
            self._corrected_n_cols = len(_REDUCED_HEADERS)
        else:
            self._corrected_n_cols = len(_NORMAL_HEADERS)

    def consume(self, channel: str, payload: Any) -> None:
        if channel == "raw":
            self._consume_raw(payload)
        elif channel == "final" and self._write_corrected:
            self._consume_final(payload)

    def on_complete(self) -> None:
        if self._closed:
            return
        self._closed = True
        # Flush any partial rows (cameras that didn't all contribute).
        # No-op when corrected CSV is disabled — the accumulator is empty.
        if self._write_corrected:
            for abs_id in sorted(self._corrected_acc.keys()):
                entry = self._corrected_acc[abs_id]
                row = entry["row"]
                row[0] = abs_id
                row[1] = round(entry["t"], 9)
                row[-1] = entry.get("quality", "ok")
                self._write_corrected_row(row)
        self._corrected_acc.clear()
        for side, fh in list(self._raw_fhs.items()):
            try:
                fh.flush()
                fh.close()
            except Exception:
                logger.exception("CsvSink: failed to close %s raw CSV", side)
        self._raw_fhs.clear()
        self._raw_csvs.clear()
        if self._corrected_fh is not None:
            try:
                self._corrected_fh.flush()
                self._corrected_fh.close()
            except Exception:
                logger.exception("CsvSink: failed to close corrected CSV")
            self._corrected_fh = None
            self._corrected_csv = None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _consume_raw(self, batch) -> None:
        """Write raw histogram rows for each frame in the batch."""
        meta = self._meta
        if meta is None:
            return

        import numpy as np

        # Tee("raw")'s gate is batch-level, so stale rows (leftover packets
        # from a previous scan) can still arrive here; iter_rows skips them.
        if batch.frame_type is not None:
            n_stale = int(np.sum(batch.frame_type == "stale"))
            if n_stale:
                logger.warning(
                    "stale raw frame skipped x%d (leftover packets from a "
                    "previous scan)", n_stale,
                )

        for i, _side_idx, cam_id, frame_type in batch.iter_rows(exclude={"stale"}):
            frame_id = int(batch.frame_ids[i])
            ts = float(batch.timestamp_s[i])

            pdc_val = _scalar_or_blank(batch.pdc, i)
            tcm_val = _scalar_or_blank(batch.tcm, i)
            tcl_val = _scalar_or_blank(batch.tcl, i)

            for side_idx, side_name, mask in _source_side_indices(batch, i, cam_id, meta):
                w = self._get_or_open_raw_writer(side_name, mask)
                if w is None:
                    continue
                temp = (
                    float(batch.temperature_c[i, side_idx, cam_id])
                    if batch.temperature_c is not None else ""
                )
                histo = batch.raw_histograms[i, side_idx, cam_id, :]
                histo_list = histo.tolist()
                histo_sum = int(np.sum(histo))
                w.writerow([
                    cam_id,
                    frame_id,
                    ts,
                    frame_type,
                    *histo_list,
                    temp,
                    histo_sum,
                    tcm_val,
                    tcl_val,
                    pdc_val,
                ])

    def _consume_final(self, interval) -> None:
        """Accumulate EnrichedCorrectedFrames; flush complete rows to CSV.

        Reduced mode reads ONLY the cam_id=-1 side-average frames emitted by
        SideAverageStage (per-camera frames are skipped) so the CSV records
        the true spatial side average. Normal mode reads only per-camera
        frames (cam_id 0..7).
        """
        from .stages.dark import EnrichedCorrectedFrame
        for frame in interval.frames:
            cam_id = int(getattr(frame, "cam_id", -99))
            if self._corrected_reduced:
                if cam_id != -1:
                    continue  # reduced CSV records the side average only
            elif not (0 <= cam_id < 8):
                continue

            abs_id = int(frame.abs_frame_id)
            acc = self._corrected_acc
            if abs_id not in acc:
                row = [""] * self._corrected_n_cols
                acc[abs_id] = {"t": float(frame.t), "row": row, "quality": "ok"}
            entry = acc[abs_id]
            row = entry["row"]

            # Track worst quality across all cameras contributing to this row.
            frame_quality = getattr(frame, "quality", "ok")
            if _QUALITY_RANK.get(frame_quality, 0) > _QUALITY_RANK.get(entry["quality"], 0):
                entry["quality"] = frame_quality

            if self._corrected_reduced:
                # Reduced mode: one cam_id=-1 frame per side per capture.
                side = frame.side
                if isinstance(frame, EnrichedCorrectedFrame):
                    bfi_val = round(float(frame.bfi), 9)
                    bvi_val = round(float(frame.bvi), 9)
                else:
                    # Fallback for plain CorrectedFrame (no enrichment)
                    bfi_val = ""
                    bvi_val = ""
                if side == "left":
                    row[2] = bfi_val
                    row[4] = bvi_val
                else:
                    row[3] = bfi_val
                    row[5] = bvi_val
            else:
                # Normal mode: wide per-cam columns
                side_char = "l" if frame.side == "left" else "r"
                cam_1 = cam_id % 8 + 1
                if isinstance(frame, EnrichedCorrectedFrame):
                    for metric, val in (
                        ("bfi",      frame.bfi),
                        ("bvi",      frame.bvi),
                        ("mean",     frame.mean),
                        ("contrast", frame.contrast),
                    ):
                        col_idx = _NORMAL_COL_IDX[(metric, side_char, cam_1)]
                        row[col_idx] = round(float(val), 9)
                else:
                    # Fallback: only mean/std available
                    for metric, val in (("mean", frame.mean),):
                        col_idx = _NORMAL_COL_IDX[(metric, side_char, cam_1)]
                        row[col_idx] = round(float(val), 9)
                # temp: leave empty (not propagated through corrected path yet)

            # Check whether this frame is complete (all expected cams seen for
            # sides that have non-empty masks).
            self._maybe_flush_row(abs_id)

    def _maybe_flush_row(self, abs_id: int) -> None:
        """Flush completed rows in abs_frame_id order (watermark).

        Only flushes contiguously from the lowest buffered abs_id upward,
        preventing out-of-order writes that cause non-monotonic timestamps.
        """
        if self._next_flush_id is None:
            self._next_flush_id = min(self._corrected_acc) if self._corrected_acc else abs_id

        while self._next_flush_id in self._corrected_acc:
            fid = self._next_flush_id
            if not self._is_row_complete(fid):
                break
            self._flush_single_row(fid)
            self._next_flush_id += 1

    def _is_row_complete(self, abs_id: int) -> bool:
        entry = self._corrected_acc.get(abs_id)
        if entry is None:
            return False
        row = entry["row"]
        if self._corrected_reduced:
            left_done = not self._expected_cams["left"] or row[2] != ""
            right_done = not self._expected_cams["right"] or row[3] != ""
            return left_done and right_done
        for side_name, side_char in (("left", "l"), ("right", "r")):
            for cam_id in self._expected_cams[side_name]:
                cam_1 = cam_id % 8 + 1
                col_idx = _NORMAL_COL_IDX[("mean", side_char, cam_1)]
                if row[col_idx] == "":
                    return False
        return True

    def _flush_single_row(self, abs_id: int) -> None:
        entry = self._corrected_acc.pop(abs_id)
        row = entry["row"]
        row[0] = abs_id
        row[1] = round(entry["t"], 9)
        row[-1] = entry.get("quality", "ok")
        self._write_corrected_row(row)

    # Flush the corrected CSV every N rows rather than per row — a per-row
    # flush costs a syscall at up to 40 Hz for the whole scan. on_complete
    # flushes the tail, so at most the last N rows ride the OS buffer.
    _CORRECTED_FLUSH_EVERY = 100

    def _write_corrected_row(self, row: list) -> None:
        w = self._get_or_open_corrected_writer()
        if w is None:
            return
        w.writerow(row)
        self._corrected_rows_since_flush += 1
        if (self._corrected_rows_since_flush >= self._CORRECTED_FLUSH_EVERY
                and self._corrected_fh is not None):
            self._corrected_fh.flush()
            self._corrected_rows_since_flush = 0

    def _get_or_open_corrected_writer(self):
        if self._corrected_csv is not None:
            return self._corrected_csv
        meta = self._meta
        if meta is None:
            return None
        try:
            os.makedirs(self._output_dir, exist_ok=True)
            # Post-#44 naming: the canonical corrected CSV is just
            # {scan_id}_{subject_id}.csv — no _corrected suffix. The
            # _raw suffix on histogram CSVs disambiguates them. When
            # subject_id is empty (e.g. some calibration paths), we
            # fall back to {scan_id}.csv so the file still has a stable
            # name.
            stem = (
                f"{meta.scan_id}_{meta.subject_id}"
                if meta.subject_id
                else meta.scan_id
            )
            filename = f"{stem}.csv"
            path = os.path.join(self._output_dir, filename)
            fh = open(path, "w", newline="", encoding="utf-8")
            w = csv.writer(fh)
            if self._corrected_reduced:
                w.writerow(_REDUCED_HEADERS)
            else:
                w.writerow(_NORMAL_HEADERS)
            self._corrected_fh = fh
            self._corrected_csv = w
            return w
        except Exception:
            logger.exception("CsvSink: failed to open corrected CSV")
            return None

    def _get_or_open_raw_writer(self, side: str, mask: int):
        if side in self._raw_csvs:
            return self._raw_csvs[side]
        meta = self._meta
        if meta is None:
            return None
        try:
            os.makedirs(self._output_dir, exist_ok=True)
            mask_hex = f"{mask:02X}"
            filename = f"{meta.scan_id}_{meta.subject_id}_{side}_mask{mask_hex}_raw.csv"
            path = os.path.join(self._output_dir, filename)
            fh = open(path, "w", newline="", encoding="utf-8")
            w = csv.writer(fh)
            w.writerow(_RAW_PIPELINE_HEADERS)
            self._raw_fhs[side] = fh
            self._raw_csvs[side] = w
            return w
        except Exception:
            logger.exception("CsvSink: failed to open raw CSV for side=%s", side)
            return None


def _is_integrity_event(event) -> bool:
    """True for events that indicate a correction-integrity problem.

    TriggerStateEvent and TerminalFsyncCount are routine operational
    telemetry, and a TerminalDarkResult with found=True is the expected
    happy path — none belongs in the integrity record.
    """
    from .batch import TerminalDarkResult, TerminalFsyncCount, TriggerStateEvent
    if isinstance(event, (TriggerStateEvent, TerminalFsyncCount)):
        return False
    if isinstance(event, TerminalDarkResult) and event.found:
        return False
    return True


def _event_frame(event):
    """Best-effort frame/time locator for an event, for summaries."""
    for attr in ("abs_frame_id", "onset_fid", "first_timestamp_s"):
        v = getattr(event, attr, None)
        if v is not None:
            return v
    return None


class DiagnosticsLogSink:
    """Default consumer for the "diagnostics" channel.

    Always injected by ScanWorkflow (independent of storage flags) so
    integrity events — DarkIntegrityWarning (laser apparently on during a
    dark frame), TerminalDarkResult(found=False) (terminal interval lost),
    StencilFallback, PipelineError (batch dropped) — are logged at WARNING
    instead of silently evaporating, with a per-type summary at scan end.

    The durable counterpart lives in ScanDBSink, which also subscribes to
    "diagnostics" and writes the same summary into the session's
    session_meta so the DB record itself shows whether a scan had
    integrity warnings.
    """

    channels = {"diagnostics"}

    def __init__(self) -> None:
        self._scan_id: str = ""
        self._counts: dict[str, int] = {}

    def on_scan_start(self, meta: ScanMetadata) -> None:
        self._scan_id = meta.scan_id
        self._counts = {}

    def consume(self, channel: str, event: Any) -> None:
        if not _is_integrity_event(event):
            return
        from .batch import TimestampMisalignmentWindow
        name = type(event).__name__
        self._counts[name] = self._counts.get(name, 0) + 1
        # TimestampRepairStage already logs each window with full context
        # under its own logger — count it for the summary, don't double-log.
        if not isinstance(event, TimestampMisalignmentWindow):
            logger.warning("scan %s integrity event: %r", self._scan_id, event)

    def on_complete(self) -> None:
        if self._counts:
            logger.warning(
                "scan %s completed with integrity events: %s",
                self._scan_id,
                ", ".join(f"{k}×{v}" for k, v in sorted(self._counts.items())),
            )


class ScanDBSink:
    """Channel-based SQLite sink for the pipeline.

    Channels:
        "final" — per-dark-interval corrected output (EnrichedCorrectedInterval).
                  Normal mode: one session_data row per per-camera
                  EnrichedCorrectedFrame (cam_id 0..7), including the
                  stencilled leading dark frame — a gapless 40 Hz record.
                  Reduced mode: only the side-average frames (cam_id=-1)
                  emitted by SideAverageStage are persisted.
        "diagnostics" — integrity events are tallied and a per-type summary
                  (count + first/last frame) is written into the session's
                  session_meta at scan end, so the DB record itself shows
                  whether the scan had correction-integrity warnings.

    The DB is the corrected (final-branch) record only. Realtime values
    reach the GUI via the "live" / "live_side" channels and are never
    persisted. Raw histograms are not stored in the DB either — the raw
    CSVs written by CsvSink (fed by Tee("raw")) are the only raw record.
    Consequence: corrected rows trail the scan by up to one dark interval
    (~15 s), and an unclean shutdown loses that tail.
    """

    channels = {"final", "diagnostics"}

    # If the scan database can't be opened, abort the scan rather than run
    # it with no durable record (see ScanRunner.CriticalSinkError).
    critical = True

    _SIDE_STR_TO_INT = {"left": 0, "right": 1}

    def __init__(self, db_path: str, *, batch_size: int = 200) -> None:
        self._db_path = db_path
        self._batch_size = max(1, int(batch_size))
        self._db = None
        self._session_id: Optional[int] = None
        self._meta: Optional[ScanMetadata] = None
        self._session_meta: Optional[dict] = None
        self._buffer: list = []
        self._rows_written = 0
        self._diag: dict[str, dict] = {}
        self._closed = False

    def on_scan_start(self, meta: ScanMetadata) -> None:
        from omotion.ScanDatabase import ScanDatabase
        import time
        self._meta = meta
        self._closed = False
        self._diag = {}
        self._rows_written = 0
        label = f"{meta.scan_id}_{meta.subject_id}"
        self._db = ScanDatabase(db_path=self._db_path)
        # data_semantics distinguishes final-branch sessions from legacy
        # ones whose session_data held realtime (live-branch) values.
        # Readers treat a missing key as legacy.
        self._session_meta = {
            "scan_id": meta.scan_id,
            "subject_id": meta.subject_id,
            "operator": meta.operator,
            "started_at_iso": meta.started_at_iso,
            "duration_sec": meta.duration_sec,
            "data_semantics": "final",
            "sdk_flags": {
                "reduced_mode": meta.reduced_mode,
                "left_camera_mask": meta.left_camera_mask,
                "right_camera_mask": meta.right_camera_mask,
            },
        }
        self._session_id = self._db.create_session(
            session_label=label,
            session_start=time.time(),
            session_notes=None,
            session_meta=self._session_meta,
        )

    def consume(self, channel: str, payload: Any) -> None:
        if channel == "final":
            self._consume_final(payload)
        elif channel == "diagnostics":
            self._consume_diagnostic(payload)

    def on_complete(self) -> None:
        if self._closed:
            return
        self._closed = True
        import time
        try:
            self._flush()
            if self._db is not None and self._session_id is not None:
                if self._rows_written == 0:
                    # Nothing was persisted (scan interrupted before any dark
                    # interval closed). Drop the orphan session header so it
                    # never appears as a viewable scan in History.
                    scan_id = (self._session_meta or {}).get("scan_id", "?")
                    logger.warning(
                        "ScanDBSink: session %d (%s) recorded no corrected "
                        "rows — deleting empty session row.",
                        self._session_id, scan_id,
                    )
                    self._db.delete_session(self._session_id)
                else:
                    if self._diag and self._session_meta is not None:
                        try:
                            self._db.update_session(
                                self._session_id,
                                session_meta={**self._session_meta,
                                              "diagnostics": self._diag},
                            )
                        except Exception:
                            logger.exception(
                                "ScanDBSink: failed to write diagnostics summary"
                            )
                    self._db.close_session(self._session_id, time.time())
        except Exception:
            logger.exception("ScanDBSink.on_complete: failed to finalise session")
        finally:
            if self._db is not None:
                try:
                    self._db.close()
                except Exception:
                    logger.exception("ScanDBSink: error closing ScanDatabase")
                self._db = None

    def _consume_diagnostic(self, event) -> None:
        """Tally integrity events for the session_meta summary."""
        if not _is_integrity_event(event):
            return
        name = type(event).__name__
        rec = self._diag.get(name)
        loc = _event_frame(event)
        if rec is None:
            self._diag[name] = {"count": 1, "first": loc, "last": loc}
        else:
            rec["count"] += 1
            if loc is not None:
                rec["last"] = loc
                if rec["first"] is None:
                    rec["first"] = loc

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _consume_final(self, interval) -> None:
        """Buffer session_data rows from one corrected interval.

        Normal mode keeps per-camera frames (cam_id 0..7); reduced mode
        keeps only the cam_id=-1 side averages so the persisted record
        matches what reduced-mode replay reads.
        """
        if self._db is None or self._session_id is None:
            return
        frames = getattr(interval, "frames", None)
        if not frames:
            return
        reduced = bool(self._meta.reduced_mode) if self._meta is not None else False

        import math

        def _round(v):
            if v is None:
                return None
            try:
                v = float(v)
            except (TypeError, ValueError):
                return None
            return round(v, 9) if math.isfinite(v) else None

        for f in frames:
            cam_id = int(getattr(f, "cam_id", -99))
            if reduced:
                if cam_id != -1:
                    continue
            elif not (0 <= cam_id < 8):
                continue
            side_int = self._SIDE_STR_TO_INT.get(getattr(f, "side", None))
            if side_int is None:
                continue
            bfi = _round(getattr(f, "bfi", None))
            bvi = _round(getattr(f, "bvi", None))
            mean_v = _round(getattr(f, "mean", None))
            contrast_v = _round(getattr(f, "contrast", None))
            if bfi is None and bvi is None and mean_v is None and contrast_v is None:
                continue  # nothing finite to record for this frame
            self._buffer.append({
                "session_id": self._session_id,
                "cam_id": cam_id,
                "side": side_int,
                "frame_id": int(getattr(f, "abs_frame_id", -1)),
                "timestamp_s": round(float(getattr(f, "t", 0.0)), 6),
                "bfi": bfi,
                "bvi": bvi,
                "mean": mean_v,
                "contrast": contrast_v,
                "quality": str(getattr(f, "quality", "ok") or "ok"),
            })

        if len(self._buffer) >= self._batch_size:
            self._flush()

    def _flush(self) -> None:
        if not self._buffer or self._db is None:
            return
        n = len(self._buffer)
        try:
            self._db.insert_session_data_rows(self._buffer)
            self._rows_written += n
        except Exception:
            logger.exception(
                "ScanDBSink: failed to insert %d corrected rows", n
            )
        self._buffer = []
