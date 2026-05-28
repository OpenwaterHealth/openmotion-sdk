"""LiveSideAverageStage — realtime per-side average for reduced-mode display.

One purely SPATIAL average per capture per side (across the selected cameras at
a single instant), emitted on the "live_side" channel. Display-only — the DB
record uses the corrected side average (CorrectedSideAverageStage). See
docs/SciencePipeline.md §16 and
docs/superpowers/specs/2026-05-28-reduced-mode-side-average-design.md.
"""

from __future__ import annotations

import warnings
from typing import Optional

import numpy as np

from ..batch import FrameBatch, LiveEmit, SideAverageSample


def _mask_to_cam_indices(mask: int) -> np.ndarray:
    return np.array([i for i in range(8) if mask & (1 << i)], dtype=np.int8)


def spatial_side_average(values_by_cam: np.ndarray, cam_indices: np.ndarray) -> float:
    """Nan-aware mean of the SELECTED cameras' values at one capture instant.

    This is the single definition of the reduced-mode side average — a purely
    SPATIAL operation (across cameras at one instant), with no temporal element.
    `values_by_cam` is a per-camera 1-D array (length 8); `cam_indices` selects
    the active cameras. Returns NaN when the selection is empty or every selected
    camera is non-finite."""
    if len(cam_indices) == 0:
        return float("nan")
    selected = np.asarray(values_by_cam)[cam_indices]
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", r"Mean of empty slice",
                                category=RuntimeWarning)
        with np.errstate(invalid="ignore"):
            return float(np.nanmean(selected))


class LiveSideAverageStage:
    """Reduced-mode realtime side average — one spatial mean per capture per side.

    The live USB path delivers ONE camera per frame row, so a capture's cameras
    (those sharing a frame_id) arrive as consecutive rows. A per-side accumulator
    gathers a capture's per-camera BFI/BVI; when the next capture begins, it
    averages the selected cameras (``spatial_side_average``) and emits a
    ``LiveEmit(channel="live_side", SideAverageSample)``. The final open capture
    is flushed at ``on_scan_stop``.

    Purely spatial: each capture's average uses ONLY that capture's cameras —
    no value carries across captures (that would be a temporal smear). The
    accumulator is just streaming plumbing to regroup the per-row stream.
    Gated on ``enabled`` (reduced mode); display-only, never persisted.
    """

    name = "live_side_average"

    def __init__(self, *, enabled: bool, left_camera_mask: int, right_camera_mask: int):
        self.enabled = bool(enabled)
        self._cams = (_mask_to_cam_indices(left_camera_mask),
                      _mask_to_cam_indices(right_camera_mask))
        self._reset_state()

    def _reset_state(self) -> None:
        # Per side: the open capture's frame_id (None = none open), its
        # accumulated per-camera BFI/BVI (length-8, NaN = camera not seen this
        # capture), and the capture's timestamp.
        self._open_fid: list[Optional[int]] = [None, None]
        self._acc_bfi = [np.full(8, np.nan), np.full(8, np.nan)]
        self._acc_bvi = [np.full(8, np.nan), np.full(8, np.nan)]
        self._acc_t = [0.0, 0.0]

    def process(self, batch: FrameBatch) -> FrameBatch:
        if not self.enabled:
            return batch
        if batch.bfi_live is None or batch.side_ids is None or batch.cam_ids is None:
            return batch
        fids = batch.abs_frame_ids if batch.abs_frame_ids is not None else batch.frame_ids
        n = batch.bfi_live.shape[0]
        for i in range(n):
            side = int(batch.side_ids[i])
            cam = int(batch.cam_ids[i])
            if side < 0 or side > 1 or cam < 0 or cam >= 8:
                continue
            fid = int(fids[i])
            if self._open_fid[side] is not None and fid != self._open_fid[side]:
                self._emit(side, batch)            # previous capture complete
            if self._open_fid[side] != fid:        # start a fresh capture
                self._open_fid[side] = fid
                self._acc_bfi[side][:] = np.nan
                self._acc_bvi[side][:] = np.nan
            self._acc_bfi[side][cam] = float(batch.bfi_live[i, side, cam])
            self._acc_bvi[side][cam] = float(batch.bvi_live[i, side, cam])
            self._acc_t[side] = float(batch.timestamp_s[i])
        return batch

    def _emit(self, side: int, batch: FrameBatch) -> None:
        fid = self._open_fid[side]
        if fid is None:
            return
        cams = self._cams[side]
        batch.events.append(LiveEmit(
            channel="live_side",
            payload=SideAverageSample(
                t=self._acc_t[side],
                frame_id=int(fid),
                side=side,
                bfi=spatial_side_average(self._acc_bfi[side], cams),
                bvi=spatial_side_average(self._acc_bvi[side], cams),
            ),
        ))
        self._open_fid[side] = None

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Flush the final open capture per side into the terminal batch's
        events (the runner dispatches them after on_scan_stop)."""
        for side in (0, 1):
            self._emit(side, batch)

    def reset(self) -> None:
        self._reset_state()
