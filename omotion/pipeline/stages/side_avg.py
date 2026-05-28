"""SideAveragingStage — per-side averaging for reduced-mode display.

See docs/SciencePipeline.md §16.
"""

from __future__ import annotations

import warnings

import numpy as np

from ..batch import FrameBatch


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


class SideAveragingStage:
    name = "side_averaging"

    def __init__(self, *, enabled: bool, left_camera_mask: int, right_camera_mask: int):
        self.enabled = bool(enabled)
        self._left_cams  = _mask_to_cam_indices(left_camera_mask)
        self._right_cams = _mask_to_cam_indices(right_camera_mask)
        # Running latest finite BFI/BVI per (side, cam), carried across rows
        # AND batches. The live USB path delivers ONE camera per frame row
        # (raw_hist[i, side, cam] for a single cam), so at any single row only
        # that camera's bfi_live is finite — the rest are NaN (MomentsStage
        # NaNs zero histograms). A per-row nanmean over the active cameras is
        # therefore NOT a cross-camera average; it's whichever camera owns the
        # row, which makes the reduced-mode trace hop between cameras frame to
        # frame. Holding each camera's most recent value and averaging those
        # yields a true, smooth side average. NaN = "not seen yet".
        self._last_bfi = np.full((2, 8), np.nan, dtype=np.float64)
        self._last_bvi = np.full((2, 8), np.nan, dtype=np.float64)

    def process(self, batch: FrameBatch) -> FrameBatch:
        if not self.enabled:
            return batch

        bfi = batch.bfi_live
        bvi = batch.bvi_live
        n = bfi.shape[0]
        bfi_side = np.full((n, 2), np.nan, dtype=np.float32)
        bvi_side = np.full((n, 2), np.nan, dtype=np.float32)

        sides = ((0, self._left_cams), (1, self._right_cams))
        # nanmean over the per-camera latest values. Suppress the "Mean of
        # empty slice" warning that fires before any active camera has
        # reported (all-NaN cache → NaN average, which the consumer skips).
        # nanmean raises it via warnings.warn, not numpy errstate.
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", r"Mean of empty slice",
                                    category=RuntimeWarning)
            with np.errstate(invalid="ignore"):
                for i in range(n):
                    for side, cams in sides:
                        if len(cams) == 0:
                            continue
                        # Update this side's latest with whatever cameras are
                        # finite in this row (typically just the row's camera).
                        row_bfi = bfi[i, side, cams]
                        row_bvi = bvi[i, side, cams]
                        fb = np.isfinite(row_bfi)
                        if fb.any():
                            self._last_bfi[side, cams[fb]] = row_bfi[fb]
                        fv = np.isfinite(row_bvi)
                        if fv.any():
                            self._last_bvi[side, cams[fv]] = row_bvi[fv]
                        # Emit the running average across active cameras.
                        bfi_side[i, side] = np.nanmean(self._last_bfi[side, cams])
                        bvi_side[i, side] = np.nanmean(self._last_bvi[side, cams])

        batch.bfi_live_side = bfi_side
        batch.bvi_live_side = bvi_side
        return batch

    def reset(self) -> None:
        self._last_bfi[:] = np.nan
        self._last_bvi[:] = np.nan
