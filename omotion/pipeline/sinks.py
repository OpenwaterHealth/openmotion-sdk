"""Sink protocol + ScanMetadata.

Concrete sink implementations (CsvSink, ScanDBSink, QtUiSink) live below
the protocol definitions.
"""

from __future__ import annotations

import csv
import logging
import os
from dataclasses import dataclass
from typing import Any, Optional, Protocol, runtime_checkable

logger = logging.getLogger("omotion.pipeline.sinks")


@dataclass(frozen=True)
class ScanMetadata:
    """Per-scan metadata handed to every sink at on_scan_start."""
    scan_id:               str
    subject_id:            str
    operator:              str
    started_at_iso:        str
    duration_sec:          int
    left_camera_mask:      int
    right_camera_mask:     int
    reduced_mode:          bool
    write_raw_csv:         bool
    raw_csv_duration_sec:  Optional[float]


@runtime_checkable
class Sink(Protocol):
    """A consumer of pipeline output.

    Channels in this pipeline:
        "raw"          — per-frame, all non-stale frames including warmup
        "live"         — per-frame, best-effort corrected (light + dark)
        "rolling"      — per-frame, rolling-averaged for test/calibration
        "final"        — per-dark-interval, accurately corrected CorrectedBatch
        "diagnostics"  — out-of-band events (DarkIntegrityWarning, etc.)
    """
    channels: set[str]

    def on_scan_start(self, meta: ScanMetadata) -> None: ...

    def consume(self, channel: str, payload: Any) -> None: ...

    def on_complete(self) -> None: ...


# ---------------------------------------------------------------------------
# Concrete sinks
# ---------------------------------------------------------------------------

_HISTO_BINS = 1024

# Raw CSV column order: cam_id, frame_id, timestamp_s, type, 0..1023, temperature, sum, tcm, tcl, pdc
_RAW_PIPELINE_HEADERS: list = [
    "cam_id", "frame_id", "timestamp_s", "type",
    *list(range(_HISTO_BINS)),
    "temperature", "sum",
    "tcm", "tcl", "pdc",
]


class CsvSink:
    """Channel-based CSV sink for the pipeline.

    Channels:
        "raw"   — per-frame raw histograms (gated by meta.write_raw_csv)
        "final" — per-interval corrected output (placeholder; wired in PR 3)

    Raw file naming: ``{scan_id}_{subject_id}_{side}_mask{XX}_raw.csv``
    Files are created lazily on first "raw" consume call.
    """

    channels = {"raw", "final"}

    def __init__(self, output_dir) -> None:
        self._output_dir = str(output_dir)
        self._meta: Optional[ScanMetadata] = None
        self._raw_fhs: dict[str, Any] = {}    # side -> file handle
        self._raw_csvs: dict[str, Any] = {}   # side -> csv.writer
        self._closed = False

    def on_scan_start(self, meta: ScanMetadata) -> None:
        self._meta = meta
        self._closed = False

    def consume(self, channel: str, payload: Any) -> None:
        if channel == "raw":
            self._consume_raw(payload)
        # "final" channel: no-op placeholder until PR 3 wires corrected output

    def on_complete(self) -> None:
        if self._closed:
            return
        self._closed = True
        for side, fh in list(self._raw_fhs.items()):
            try:
                fh.flush()
                fh.close()
            except Exception:
                logger.exception("CsvSink: failed to close %s raw CSV", side)
        self._raw_fhs.clear()
        self._raw_csvs.clear()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _consume_raw(self, batch) -> None:
        """Write raw histogram rows for each frame in the batch."""
        meta = self._meta
        if meta is None or not meta.write_raw_csv:
            return

        import numpy as np
        from .batch import FrameBatch

        n = len(batch.cam_ids)
        for i in range(n):
            cam_id = int(batch.cam_ids[i])
            frame_id = int(batch.frame_ids[i])
            ts = float(batch.timestamp_s[i])

            # Duration cap: skip this frame if it's past the limit
            if meta.raw_csv_duration_sec is not None and ts > meta.raw_csv_duration_sec:
                continue

            frame_type = ""
            if batch.frame_type is not None:
                frame_type = str(batch.frame_type[i])

            temp = float(batch.temperature_c[i, 0, cam_id]) if batch.temperature_c is not None else ""
            pdc_val = float(batch.pdc[i]) if batch.pdc is not None else ""
            tcm_val = float(batch.tcm[i]) if batch.tcm is not None else ""
            tcl_val = float(batch.tcl[i]) if batch.tcl is not None else ""

            # Determine side from cam_id: left=side 0, right=side 1
            # The batch has shape (N, 2, 8, 1024) for raw_histograms.
            # We write one row per frame/cam using side=0 if left_camera_mask
            # has this cam bit set, side=1 for right_camera_mask.
            for side_idx, (side_name, mask) in enumerate(
                [("left", meta.left_camera_mask), ("right", meta.right_camera_mask)]
            ):
                if mask == 0:
                    continue
                if not (mask & (1 << cam_id)):
                    continue
                w = self._get_or_open_raw_writer(side_name, mask)
                if w is None:
                    continue
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
