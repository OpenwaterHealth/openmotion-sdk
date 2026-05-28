"""SideAveragingStage — per-side averaging for reduced-mode display.

See docs/SciencePipeline.md §16.
"""

from __future__ import annotations

import warnings

import numpy as np

from ..batch import FrameBatch


def _mask_to_cam_indices(mask: int) -> np.ndarray:
    return np.array([i for i in range(8) if mask & (1 << i)], dtype=np.int8)


class SideAveragingStage:
    name = "side_averaging"

    def __init__(self, *, enabled: bool, left_camera_mask: int, right_camera_mask: int):
        self.enabled = bool(enabled)
        self._left_cams  = _mask_to_cam_indices(left_camera_mask)
        self._right_cams = _mask_to_cam_indices(right_camera_mask)

    def process(self, batch: FrameBatch) -> FrameBatch:
        if not self.enabled:
            return batch

        n = batch.bfi_live.shape[0]
        bfi_side = np.zeros((n, 2), dtype=np.float32)
        bvi_side = np.zeros((n, 2), dtype=np.float32)

        # nanmean (not mean) so one bad camera frame doesn't poison the
        # whole side average. The realtime BFI/BVI stage emits NaN for
        # early frames before the first dark observation closes; with
        # plain mean, the side average is NaN until every active cam
        # has a finite value at the same frame index, which can take
        # several seconds and leaves the reduced-mode display empty.
        # Suppress the "Mean of empty slice" warning that fires when an
        # entire row is NaN (all active cams non-finite for that frame —
        # common in early warmup frames). nanmean raises it via
        # warnings.warn, NOT numpy's FP errstate, so catch_warnings is
        # required. Those rows correctly emit NaN.
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", r"Mean of empty slice",
                                    category=RuntimeWarning)
            with np.errstate(invalid="ignore"):
                if len(self._left_cams) > 0:
                    bfi_side[:, 0] = np.nanmean(batch.bfi_live[:, 0, self._left_cams], axis=1)
                    bvi_side[:, 0] = np.nanmean(batch.bvi_live[:, 0, self._left_cams], axis=1)
                if len(self._right_cams) > 0:
                    bfi_side[:, 1] = np.nanmean(batch.bfi_live[:, 1, self._right_cams], axis=1)
                    bvi_side[:, 1] = np.nanmean(batch.bvi_live[:, 1, self._right_cams], axis=1)

        batch.bfi_live_side = bfi_side
        batch.bvi_live_side = bvi_side
        return batch

    def reset(self) -> None:
        pass
