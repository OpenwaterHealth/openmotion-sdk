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
        "final"        — per-dark-interval, accurately corrected CorrectedBatch
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


def _scalar_or_default(arr, i, default=0.0):
    """Pull arr[i] as a float, returning ``default`` for missing/NaN.

    Used for DB columns where NULL would be acceptable but the schema
    is currently NOT NULL — keep the default to preserve writability.
    """
    if arr is None:
        return default
    try:
        v = float(arr[i])
    except (TypeError, ValueError):
        return default
    if v != v:  # NaN
        return default
    return v


def _batch_frame_type(batch, i: int) -> str:
    if getattr(batch, "frame_type", None) is None:
        return ""
    return str(batch.frame_type[i])


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
        from .batch import FrameBatch

        n = len(batch.cam_ids)
        for i in range(n):
            cam_id = int(batch.cam_ids[i])
            frame_id = int(batch.frame_ids[i])
            ts = float(batch.timestamp_s[i])

            frame_type = _batch_frame_type(batch, i)
            if frame_type == "stale":
                logger.warning(
                    "stale raw frame skipped cam_id=%d frame_id=%d abs_frame_id=%s",
                    cam_id, frame_id,
                    "" if batch.abs_frame_ids is None else int(batch.abs_frame_ids[i]),
                )
                continue

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
        """Accumulate EnrichedCorrectedFrames; flush complete rows to CSV."""
        from .stages.dark import EnrichedCorrectedFrame
        for frame in interval.frames:
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
                # Reduced mode: accumulate per-side bfi/bvi, flush when we
                # have seen contributions from both expected sides.
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
                cam_1 = int(frame.cam_id) % 8 + 1
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

    def _write_corrected_row(self, row: list) -> None:
        w = self._get_or_open_corrected_writer()
        if w is None:
            return
        w.writerow(row)
        if self._corrected_fh is not None:
            self._corrected_fh.flush()

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


class ScanDBSink:
    """Channel-based SQLite sink for the pipeline.

    Channels:
        "raw"        — per-frame raw histograms (gated by meta.write_raw_csv)
        "live"       — per-frame per-cam BFI/BVI/mean/contrast (uncorrected
                       realtime values), for past-scan replay. ~40 rows/sec.
                       NOT written in reduced mode (which persists only the
                       corrected side average).
        "final_side" — reduced-mode per-side dark-corrected average
                       (SideAverageStage), one SideAverageSample per
                       capture. Persisted as cam_id=-1 rows — the accurate
                       side-average record replay reads.
    """

    channels = {"raw", "live", "final_side"}

    # If the scan database can't be opened, abort the scan rather than run
    # it with no durable record (see ScanRunner.CriticalSinkError).
    critical = True

    def __init__(self, db_path: str, *, raw_batch_size: int = 200,
                 side_batch_size: int = 200) -> None:
        self._db_path = db_path
        self._raw_batch_size = max(1, int(raw_batch_size))
        self._side_batch_size = max(1, int(side_batch_size))
        self._db = None
        self._session_id: Optional[int] = None
        self._meta: Optional[ScanMetadata] = None
        self._raw_buffer: list = []
        self._side_buffer: list = []
        self._closed = False

    def on_scan_start(self, meta: ScanMetadata) -> None:
        from omotion.ScanDatabase import ScanDatabase
        import time
        self._meta = meta
        self._closed = False
        label = f"{meta.scan_id}_{meta.subject_id}"
        self._db = ScanDatabase(db_path=self._db_path)
        self._session_id = self._db.create_session(
            session_label=label,
            session_start=time.time(),
            session_notes=None,
            session_meta={"scan_id": meta.scan_id, "operator": meta.operator},
        )

    def consume(self, channel: str, payload: Any) -> None:
        if channel == "raw":
            self._consume_raw(payload)
        elif channel == "live":
            self._consume_live(payload)
        elif channel == "final_side":
            self._consume_side(payload)

    def on_complete(self) -> None:
        if self._closed:
            return
        self._closed = True
        import time
        try:
            self._flush_raw()
            self._flush_side()
            if self._db is not None and self._session_id is not None:
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

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _consume_live(self, batch) -> None:
        """Write per-frame per-cam BFI/BVI/mean/contrast rows from a
        post-BfiBvi batch. One row per frame; the source camera for that
        frame is identified by batch.side_ids[i] / batch.cam_ids[i] (the
        live pipeline interleaves frames across cameras).

        Mirrors the read pattern in bloodflow-app's _LivePlotSink so
        past-replay sees the same values the user saw live. Skips dark
        frames (no useful display signal) and non-finite samples (early
        warmup before first dark observation).

        Reduced mode persists only the corrected side average (cam_id=-1, via
        the "final_side" channel), so per-camera realtime rows are not written.
        """
        if self._db is None or self._session_id is None:
            return
        if self._meta is not None and self._meta.reduced_mode:
            return
        if batch.bfi_live is None:
            return
        side_ids = getattr(batch, "side_ids", None)
        cam_ids = getattr(batch, "cam_ids", None)
        if side_ids is None or cam_ids is None:
            return
        import math
        n = batch.bfi_live.shape[0]
        rows = []
        for i in range(n):
            ft = str(batch.frame_type[i]) if batch.frame_type is not None else "light"
            if ft in ("warmup", "stale", "dark"):
                continue
            side_idx = int(side_ids[i])
            cam_id = int(cam_ids[i])
            if side_idx < 0 or side_idx > 1 or cam_id < 0 or cam_id >= 8:
                continue
            bfi = float(batch.bfi_live[i, side_idx, cam_id])
            bvi = float(batch.bvi_live[i, side_idx, cam_id])
            if not (math.isfinite(bfi) and math.isfinite(bvi)):
                continue
            mean_v = None
            contrast_v = None
            if batch.mean_dc_rt is not None:
                m = float(batch.mean_dc_rt[i, side_idx, cam_id])
                if math.isfinite(m):
                    mean_v = round(m, 9)
            if batch.contrast_sn_rt is not None:
                c = float(batch.contrast_sn_rt[i, side_idx, cam_id])
                if math.isfinite(c):
                    contrast_v = round(c, 9)
            rows.append({
                "session_id": self._session_id,
                "session_raw_id": None,
                "cam_id": cam_id,
                "side": side_idx,
                "frame_id": int(batch.abs_frame_ids[i]) if batch.abs_frame_ids is not None else i,
                "timestamp_s": round(float(batch.timestamp_s[i]), 6),
                "bfi": round(bfi, 9),
                "bvi": round(bvi, 9),
                "mean": mean_v,
                "contrast": contrast_v,
                "quality": str(batch.quality[i]) if batch.quality is not None else "ok",
            })
        if rows:
            try:
                self._db.insert_session_data_rows(rows)
            except Exception:
                logger.exception(
                    "ScanDBSink: failed to insert %d live rows", len(rows)
                )

    def _consume_side(self, sample) -> None:
        """Buffer one corrected per-side average (SideAverageSample from
        SideAverageStage) for persistence as a cam_id=-1 row. This is
        the accurate side-average record reduced-mode replay reads."""
        if self._db is None or self._session_id is None:
            return
        import math

        def _round(v):
            if v is None:
                return None
            v = float(v)
            return round(v, 9) if math.isfinite(v) else None

        bfi = _round(getattr(sample, "bfi", None))
        bvi = _round(getattr(sample, "bvi", None))
        if bfi is None and bvi is None:
            return  # nothing finite to record for this capture
        self._side_buffer.append({
            "session_id": self._session_id,
            "session_raw_id": None,
            "cam_id": -1,
            "side": int(sample.side),
            "frame_id": int(sample.frame_id),
            "timestamp_s": round(float(sample.t), 6),
            "bfi": bfi,
            "bvi": bvi,
            "mean": _round(getattr(sample, "mean", None)),
            "contrast": _round(getattr(sample, "contrast", None)),
            "quality": getattr(sample, "quality", "ok") or "ok",
        })
        if len(self._side_buffer) >= self._side_batch_size:
            self._flush_side()

    def _flush_side(self) -> None:
        if not self._side_buffer or self._db is None:
            return
        try:
            self._db.insert_session_data_rows(self._side_buffer)
        except Exception:
            logger.exception(
                "ScanDBSink: failed to insert %d side-average rows",
                len(self._side_buffer),
            )
        self._side_buffer = []

    def _consume_raw(self, batch) -> None:
        meta = self._meta
        if meta is None:
            return
        if self._db is None or self._session_id is None:
            return

        import struct
        import numpy as np

        _pack = struct.Struct(f"<{HISTO_SIZE_WORDS}I")

        n = len(batch.cam_ids)
        for i in range(n):
            cam_id = int(batch.cam_ids[i])
            frame_id = int(batch.frame_ids[i])
            ts = float(batch.timestamp_s[i])
            frame_type = _batch_frame_type(batch, i)
            if frame_type == "stale":
                logger.warning(
                    "stale raw frame skipped cam_id=%d frame_id=%d abs_frame_id=%s",
                    cam_id, frame_id,
                    "" if batch.abs_frame_ids is None else int(batch.abs_frame_ids[i]),
                )
                continue
            pdc_val = _scalar_or_default(batch.pdc, i, 0.0)
            tcm_val = _scalar_or_default(batch.tcm, i, 0.0)
            tcl_val = _scalar_or_default(batch.tcl, i, 0.0)

            for side_idx, side_name, _mask in _source_side_indices(batch, i, cam_id, meta):
                temp = (
                    float(batch.temperature_c[i, side_idx, cam_id])
                    if batch.temperature_c is not None else None
                )
                histo = batch.raw_histograms[i, side_idx, cam_id, :]
                hist_bytes = _pack.pack(*histo.tolist())
                histo_sum = int(np.sum(histo))
                self._raw_buffer.append({
                    "session_id": self._session_id,
                    "side": side_name,
                    "cam_id": cam_id,
                    "frame_id": frame_id,
                    "timestamp_s": round(ts, 6),
                    "hist": hist_bytes,
                    "temp": round(temp, 6) if temp is not None else None,
                    "sum_counts": histo_sum,
                    "tcm": round(tcm_val, 6),
                    "tcl": round(tcl_val, 6),
                    "pdc": round(pdc_val, 6),
                })

        if len(self._raw_buffer) >= self._raw_batch_size:
            self._flush_raw()

    def _flush_raw(self) -> None:
        if not self._raw_buffer or self._db is None:
            self._raw_buffer.clear()
            return
        try:
            self._db.insert_raw_frames(self._raw_buffer)
        except Exception:
            logger.exception("ScanDBSink: failed to flush %d raw frames", len(self._raw_buffer))
        finally:
            self._raw_buffer.clear()
