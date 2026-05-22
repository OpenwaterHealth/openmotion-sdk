"""SideAveragingStage — per-side averaging for reduced-mode display.

See docs/SciencePipeline.md §16.
"""

from __future__ import annotations

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

        if len(self._left_cams) > 0:
            bfi_side[:, 0] = batch.bfi_live[:, 0, self._left_cams].mean(axis=1)
            bvi_side[:, 0] = batch.bvi_live[:, 0, self._left_cams].mean(axis=1)
        if len(self._right_cams) > 0:
            bfi_side[:, 1] = batch.bfi_live[:, 1, self._right_cams].mean(axis=1)
            bvi_side[:, 1] = batch.bvi_live[:, 1, self._right_cams].mean(axis=1)

        batch.bfi_live_side = bfi_side
        batch.bvi_live_side = bvi_side
        return batch

    def reset(self) -> None:
        pass
