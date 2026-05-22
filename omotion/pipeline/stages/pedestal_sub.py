"""PedestalSubtractionStage — produce display_mean from mean_raw, per-side.

See docs/SciencePipeline.md §7.1.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch
from ..pedestal import SensorPedestals


class PedestalSubtractionStage:
    name = "pedestal_subtraction"

    def __init__(self, pedestals: SensorPedestals):
        self._pedestal = np.array(
            [pedestals.left, pedestals.right], dtype=np.float32
        ).reshape(1, 2, 1)

    def process(self, batch: FrameBatch) -> FrameBatch:
        batch.display_mean = np.maximum(
            0.0, batch.mean_raw - self._pedestal
        ).astype(np.float32)
        return batch

    def reset(self) -> None:
        pass
