"""
CsvSink — writes the per-scan CSV files the SDK has always produced.

This is the second concrete ``Sink`` implementation (alongside
``ScanDBSink``). It consumes the same callback events ``ScanWorkflow``
fans out, and writes:

* ``{ts}_{subject}.csv``                       — corrected per-frame merged
* ``{ts}_{subject}_{side}_mask{XX}_raw.csv``   — raw histogram, per side
* ``{ts}_{subject}_telemetry.csv``             — console telemetry (developerMode)

Today's ScanWorkflow still owns the telemetry CSV writer inline; this
class is being introduced incrementally so each output target can land
+ be exercised in isolation. See ``docs/ScanDatabase.md`` (Sink section)
for the rollout plan.

The corrected-CSV merge logic (per-frame buffering, complete-row
flushes, late-completion drain at scan end) is ported verbatim from
``ScanWorkflow._worker`` — the existing ``test_corrected_csv_output.py``
regression suite is the safety net for byte-identical output across
the move.
"""

from __future__ import annotations

import csv
import logging
import os
import struct
import threading
from typing import TYPE_CHECKING, Any, Optional

from omotion import _log_root
from omotion.Sink import Sink

if TYPE_CHECKING:
    from omotion.MotionProcessing import CorrectedBatch
    from omotion.ScanWorkflow import ScanRequest, ScanResult


# Raw CSV format — mirrors ScanWorkflow._RAW_CSV_HEADERS so the on-disk
# output stays byte-identical after extraction. 3 housekeeping cells +
# 1024 histogram bins + temperature + sum + 3 telemetry extras = 1032
# columns total.
_HISTO_BINS = 1024
_RAW_CSV_HEADERS: list = [
    "cam_id", "frame_id", "timestamp_s",
    *list(range(_HISTO_BINS)),
    "temperature", "sum",
    "tcm", "tcl", "pdc",
]
# Unpack a 4096-byte (1024 × uint32 little-endian) histogram blob into
# a Python list of 1024 ints. Mirrors ``HistogramSample.histogram.tolist()``
# in the pre-extraction inline writer path.
_HISTO_UNPACK = struct.Struct(f"<{_HISTO_BINS}I")

logger = logging.getLogger(
    f"{_log_root}.CsvSink" if _log_root else "CsvSink"
)


def _corrected_columns(reduced_mode: bool) -> list[str]:
    """Column ordering for the corrected CSV — ported from ScanWorkflow."""
    if reduced_mode:
        return ["bfi_left", "bfi_right", "bvi_left", "bvi_right"]
    return (
        [f"bfi_l{i}" for i in range(1, 9)]
        + [f"bfi_r{i}" for i in range(1, 9)]
        + [f"bvi_l{i}" for i in range(1, 9)]
        + [f"bvi_r{i}" for i in range(1, 9)]
        + [f"mean_l{i}" for i in range(1, 9)]
        + [f"mean_r{i}" for i in range(1, 9)]
        + [f"contrast_l{i}" for i in range(1, 9)]
        + [f"contrast_r{i}" for i in range(1, 9)]
        + [f"temp_l{i}" for i in range(1, 9)]
        + [f"temp_r{i}" for i in range(1, 9)]
    )


def _expected_col_suffixes(left_mask: int, right_mask: int) -> set[str]:
    """The set of ``"{side[0]}{cam_id+1}"`` tags the corrected writer needs
    to see before declaring a frame "complete" and emitting its row."""
    out: set[str] = set()
    for side_letter, mask in (("l", left_mask), ("r", right_mask)):
        for cam in range(8):
            if mask & (1 << cam):
                out.add(f"{side_letter}{cam + 1}")
    return out


class CsvSink(Sink):
    """Writes the SDK's existing per-scan CSV outputs.

    Step B1: implements ``on_corrected_batch`` only. ``on_raw_frame`` and
    the telemetry path land in later commits — they're still inline in
    ScanWorkflow today.
    """

    def __init__(
        self,
        *,
        enable_corrected: bool = True,
        enable_raw: bool = True,
    ) -> None:
        # Top-level toggles let ScanWorkflow cut the inline writers over
        # to CsvSink incrementally — corrected in Step B4a, raw in B4b.
        # Once both inline paths are removed these can go away.
        self._enable_corrected = enable_corrected
        self._enable_raw = enable_raw

        # State filled in by on_scan_start. None means "not started yet
        # or not configured for this output target".
        self._reduced_mode: bool = False
        self._corrected_columns: list[str] = []
        self._expected_col_suffixes: set[str] = set()
        self._corrected_path: Optional[str] = None
        self._corr_fh = None                       # type: ignore[assignment]
        self._corr_csv = None                      # type: ignore[assignment]
        self._corr_by_frame: dict[int, dict] = {}
        self._corr_lock = threading.Lock()
        self._closed: bool = False
        self._rows_written: int = 0

        # Raw CSV state — one writer per side. ``_raw_duration_s`` caps how
        # long raw rows are persisted (matches the existing
        # ``raw_csv_duration_sec`` semantics); we close that side's writer
        # the first time we see a sample past the cap so the on-disk file
        # ends at a real frame boundary.
        self._raw_paths: dict[str, str] = {}
        self._raw_fhs: dict[str, Any] = {}
        self._raw_csvs: dict[str, Any] = {}
        self._raw_lock = threading.Lock()
        self._raw_duration_s: Optional[float] = None
        self._raw_rows_written: dict[str, int] = {"left": 0, "right": 0}

    @property
    def corrected_path(self) -> Optional[str]:
        return self._corrected_path

    @property
    def rows_written(self) -> int:
        return self._rows_written

    @property
    def raw_paths(self) -> dict[str, str]:
        return dict(self._raw_paths)

    @property
    def raw_rows_written(self) -> dict[str, int]:
        return dict(self._raw_rows_written)

    # ------------------------------------------------------------------
    # Sink hooks
    # ------------------------------------------------------------------

    def on_scan_start(
        self,
        *,
        ts: str,
        session_start_ts: float,
        request: "ScanRequest",
        meta: dict,
    ) -> None:
        # The two output paths (corrected + raw) are toggled
        # independently by the request flags; the data_dir is shared.
        try:
            os.makedirs(request.data_dir, exist_ok=True)
        except Exception:
            logger.exception("CsvSink: could not create data_dir %s", request.data_dir)
            return

        self._reduced_mode = bool(getattr(request, "reduced_mode", False))
        left_mask  = int(request.left_camera_mask)
        right_mask = int(request.right_camera_mask)

        # --- corrected CSV --------------------------------------------------
        if self._enable_corrected and getattr(request, "write_corrected_csv", True):
            self._corrected_columns = _corrected_columns(self._reduced_mode)
            self._expected_col_suffixes = _expected_col_suffixes(left_mask, right_mask)
            self._corrected_path = os.path.join(
                request.data_dir, f"{ts}_{request.subject_id}.csv"
            )
            try:
                self._corr_fh = open(  # noqa: WPS515
                    self._corrected_path, "w", newline="", encoding="utf-8"
                )
                self._corr_csv = csv.writer(self._corr_fh)
                self._corr_csv.writerow(
                    ["frame_id", "timestamp_s", *self._corrected_columns]
                )
            except Exception:
                logger.exception(
                    "CsvSink: failed to open corrected CSV at %s — "
                    "corrected CSV disabled for this scan",
                    self._corrected_path,
                )
                self._corrected_path = None
                self._corr_fh = None
                self._corr_csv = None

        # --- raw CSVs (one per active side) --------------------------------
        if self._enable_raw and getattr(request, "write_raw_csv", True):
            raw_dur = getattr(request, "raw_csv_duration_sec", None)
            self._raw_duration_s = float(raw_dur) if raw_dur is not None else None
            for side, mask in (("left", left_mask), ("right", right_mask)):
                if mask == 0:
                    continue
                mask_hex = f"{mask:02X}"
                path = os.path.join(
                    request.data_dir,
                    f"{ts}_{request.subject_id}_{side}_mask{mask_hex}_raw.csv",
                )
                try:
                    fh = open(path, "w", newline="", encoding="utf-8")  # noqa: WPS515
                    w = csv.writer(fh)
                    w.writerow(_RAW_CSV_HEADERS)
                    self._raw_paths[side] = path
                    self._raw_fhs[side] = fh
                    self._raw_csvs[side] = w
                except Exception:
                    logger.exception(
                        "CsvSink: failed to open %s raw CSV at %s — "
                        "raw output for that side disabled",
                        side, path,
                    )

    def on_raw_frame(
        self,
        side: str,
        cam_id: int,
        frame_id: int,
        timestamp_s: float,
        hist: bytes,
        temp: float,
        sum_counts: int,
        tcm: float,
        tcl: float,
        pdc: float,
    ) -> None:
        with self._raw_lock:
            w = self._raw_csvs.get(side)
            if w is None:
                return
            # Duration cap: close that side's writer the first time we
            # see a sample past the limit so the file ends cleanly on a
            # frame boundary. ``timestamp_s`` is already 0-based per
            # scan (parse_histogram_stream's t0_normalizer).
            if self._raw_duration_s is not None and timestamp_s > self._raw_duration_s:
                self._close_raw_locked(side)
                return
            # Unpack the 4096-byte histogram blob into 1024 ints. Mirrors
            # ``sample.histogram.tolist()`` in the inline writer.
            try:
                bins = _HISTO_UNPACK.unpack(hist)
            except struct.error:
                logger.warning(
                    "CsvSink: %s cam %d frame %d had unexpected hist len %d "
                    "(expected %d); skipping row",
                    side, cam_id, frame_id, len(hist), _HISTO_BINS * 4,
                )
                return
            w.writerow([
                int(cam_id),
                int(frame_id),
                float(timestamp_s),
                *bins,
                float(temp) if temp is not None else "",
                int(sum_counts) if sum_counts is not None else "",
                float(tcm),
                float(tcl),
                float(pdc),
            ])
            self._raw_rows_written[side] = self._raw_rows_written.get(side, 0) + 1

    def _close_raw_locked(self, side: str) -> None:
        """Caller holds ``self._raw_lock``."""
        fh = self._raw_fhs.pop(side, None)
        self._raw_csvs.pop(side, None)
        if fh is not None:
            try:
                fh.flush()
                fh.close()
            except Exception:
                logger.exception("CsvSink: %s raw CSV close failed", side)

    def on_corrected_batch(self, batch: "CorrectedBatch") -> None:
        if self._corr_csv is None:
            return

        if self._reduced_mode:
            self._on_corrected_batch_reduced(batch)
            return

        try:
            with self._corr_lock:
                for sample in batch.samples:
                    frame_key = int(sample.absolute_frame_id)
                    col_suffix = f"{sample.side[0]}{int(sample.cam_id) + 1}"
                    frame_entry = self._corr_by_frame.get(frame_key)
                    if frame_entry is None:
                        frame_entry = {
                            "timestamp_s": float(sample.timestamp_s),
                            "values": {},
                        }
                        self._corr_by_frame[frame_key] = frame_entry
                    else:
                        # Use the earliest per-side timestamp seen for this
                        # frame_id; matches the pre-extraction behavior.
                        frame_entry["timestamp_s"] = min(
                            float(frame_entry["timestamp_s"]),
                            float(sample.timestamp_s),
                        )
                    vals = frame_entry["values"]
                    vals[f"bfi_{col_suffix}"]      = round(float(sample.bfi), 6)
                    vals[f"bvi_{col_suffix}"]      = round(float(sample.bvi), 6)
                    vals[f"mean_{col_suffix}"]     = round(float(sample.mean), 6)
                    vals[f"contrast_{col_suffix}"] = round(float(sample.contrast), 6)
                    vals[f"temp_{col_suffix}"]     = float(sample.temperature_c)

                # Flush frames where every expected (side, cam) has reported.
                if self._expected_col_suffixes:
                    complete = [
                        fid for fid, e in self._corr_by_frame.items()
                        if all(
                            f"bfi_{s}" in e["values"]
                            for s in self._expected_col_suffixes
                        )
                    ]
                    if complete:
                        for fid in sorted(complete):
                            entry = self._corr_by_frame.pop(fid)
                            row = [fid, float(entry["timestamp_s"])]
                            row.extend(
                                entry["values"].get(col, "")
                                for col in self._corrected_columns
                            )
                            self._corr_csv.writerow(row)
                            self._rows_written += 1
                        self._corr_fh.flush()
        except Exception:
            logger.exception("CsvSink: corrected-batch aggregation failed")

    def _on_corrected_batch_reduced(self, batch: "CorrectedBatch") -> None:
        """Reduced-mode path: average all active cameras per side per
        frame, write only ``bfi_left/right`` and ``bvi_left/right``."""
        try:
            with self._corr_lock:
                for sample in batch.samples:
                    frame_key = int(sample.absolute_frame_id)
                    side = sample.side
                    frame_entry = self._corr_by_frame.get(frame_key)
                    if frame_entry is None:
                        frame_entry = {
                            "timestamp_s": float(sample.timestamp_s),
                            "values": {},
                            "_accum": {},
                        }
                        self._corr_by_frame[frame_key] = frame_entry
                    else:
                        frame_entry["timestamp_s"] = min(
                            float(frame_entry["timestamp_s"]),
                            float(sample.timestamp_s),
                        )
                    accum = frame_entry.setdefault("_accum", {})
                    side_acc = accum.get(side)
                    if side_acc is None:
                        side_acc = {"bfi_sum": 0.0, "bvi_sum": 0.0, "count": 0}
                        accum[side] = side_acc
                    side_acc["bfi_sum"] += float(sample.bfi)
                    side_acc["bvi_sum"] += float(sample.bvi)
                    side_acc["count"] += 1

                # Expected per-side cam counts from the mask.
                # We need both sides to have all their cams reporting for a frame to be "complete".
                # Same logic as the inline path used.
                expected_left  = bin(int.from_bytes(bytes([]), "big")) if False else None
                # (recompute here to avoid needing request stored; use suffix set)
                expected_per_side = {"left": 0, "right": 0}
                for s in self._expected_col_suffixes:
                    expected_per_side[
                        "left" if s.startswith("l") else "right"
                    ] += 1

                # Which sides actually have any active cameras at all?
                expected_sides = {
                    sd for sd, n in expected_per_side.items() if n > 0
                }

                complete = []
                for fid, entry in self._corr_by_frame.items():
                    accum = entry.get("_accum", {})
                    if all(
                        accum.get(sd, {}).get("count", 0) >= expected_per_side.get(sd, 1)
                        for sd in expected_sides
                    ):
                        complete.append(fid)
                if complete:
                    for fid in sorted(complete):
                        entry = self._corr_by_frame.pop(fid)
                        accum = entry.get("_accum", {})
                        left_acc  = accum.get("left",  {"bfi_sum": 0, "bvi_sum": 0, "count": 1})
                        right_acc = accum.get("right", {"bfi_sum": 0, "bvi_sum": 0, "count": 1})
                        vals = {
                            "bfi_left":  round(left_acc["bfi_sum"]  / max(1, left_acc["count"]),  6),
                            "bfi_right": round(right_acc["bfi_sum"] / max(1, right_acc["count"]), 6),
                            "bvi_left":  round(left_acc["bvi_sum"]  / max(1, left_acc["count"]),  6),
                            "bvi_right": round(right_acc["bvi_sum"] / max(1, right_acc["count"]), 6),
                        }
                        row = [fid, float(entry["timestamp_s"])]
                        row.extend(vals.get(col, "") for col in self._corrected_columns)
                        self._corr_csv.writerow(row)
                        self._rows_written += 1
                    self._corr_fh.flush()
        except Exception:
            logger.exception("CsvSink: reduced-mode corrected-batch aggregation failed")

    def on_complete(self, result: "ScanResult" = None) -> None:
        if self._closed:
            return
        self._closed = True

        # Late-completion flush: any frames that didn't fill all expected
        # cameras get written out anyway so the operator sees partial data
        # rather than silent loss. Matches pre-extraction behavior.
        try:
            with self._corr_lock:
                if self._corr_csv is not None and self._corr_by_frame:
                    for fid in sorted(self._corr_by_frame.keys()):
                        entry = self._corr_by_frame[fid]
                        if self._reduced_mode:
                            accum = entry.get("_accum", {})
                            left_acc  = accum.get("left",  {"bfi_sum": 0, "bvi_sum": 0, "count": 1})
                            right_acc = accum.get("right", {"bfi_sum": 0, "bvi_sum": 0, "count": 1})
                            vals = {
                                "bfi_left":  round(left_acc["bfi_sum"]  / max(1, left_acc["count"]),  6),
                                "bfi_right": round(right_acc["bfi_sum"] / max(1, right_acc["count"]), 6),
                                "bvi_left":  round(left_acc["bvi_sum"]  / max(1, left_acc["count"]),  6),
                                "bvi_right": round(right_acc["bvi_sum"] / max(1, right_acc["count"]), 6),
                            }
                            row = [fid, float(entry["timestamp_s"])]
                            row.extend(vals.get(col, "") for col in self._corrected_columns)
                        else:
                            row = [fid, float(entry["timestamp_s"])]
                            row.extend(
                                entry["values"].get(col, "")
                                for col in self._corrected_columns
                            )
                        self._corr_csv.writerow(row)
                        self._rows_written += 1
                    self._corr_by_frame.clear()
        except Exception:
            logger.exception("CsvSink: corrected-batch final flush failed")

        if self._corr_fh is not None:
            try:
                self._corr_fh.flush()
                self._corr_fh.close()
            except Exception:
                logger.exception("CsvSink: corrected CSV close failed")
            self._corr_fh = None
            self._corr_csv = None

        # Close any raw writers still open (writers that hit the duration
        # cap are already closed via _close_raw_locked).
        with self._raw_lock:
            for side in list(self._raw_fhs.keys()):
                self._close_raw_locked(side)
