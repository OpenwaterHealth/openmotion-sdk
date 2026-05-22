"""NoiseFloorStage — zero histogram bins below a count threshold.

See docs/SciencePipeline.md §5.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


class NoiseFloorStage:
    name = "noise_floor"

    def __init__(self, threshold: int = 10):
        self.threshold = int(threshold)

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self.threshold <= 0:
            return batch
        np.putmask(batch.raw_histograms, batch.raw_histograms < self.threshold, 0)
        return batch

    def reset(self) -> None:
        pass
