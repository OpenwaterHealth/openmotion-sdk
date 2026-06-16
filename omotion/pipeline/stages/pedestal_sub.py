"""PedestalSubtractionStage — produce subtracted_mean from mean_raw, per-side.

subtracted_mean = mean_raw - pedestal. Used by dark-frame diagnostics
(ambient-light gate in CalibrationWorkflow, dark-max in ContactQualityWorkflow)
— NOT used for the science BFI/BVI path, which uses the dark-corrected mean.

Negative values are valid: they indicate the frame's mean is below the
pedestal average, which is normal noise for dark frames.

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
        batch.subtracted_mean = (
            batch.mean_raw - self._pedestal
        ).astype(np.float32)
        return batch

    def reset(self) -> None:
        pass
