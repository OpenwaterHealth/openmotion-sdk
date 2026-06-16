"""MomentsStage — vectorized first/second moment + std.

See docs/SciencePipeline.md §6.

Note: contrast_raw is intentionally NOT computed here. Spec §7.1 defines
K = std / (u1 - pedestal), i.e. pedestal-subtracted mean, but MomentsStage
has no pedestal. Pedestal-subtracted contrast is computed downstream in
BfiBviStage. contrast_raw is left as None.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


class MomentsStage:
    name = "moments"

    _BIN_VALUES    = np.arange(1024, dtype=np.float64)
    _BIN_VALUES_SQ = _BIN_VALUES ** 2

    def process(self, batch: FrameBatch) -> FrameBatch:
        h = batch.raw_histograms
        counts = h.sum(axis=-1)
        safe_counts = np.where(counts > 0, counts, 1).astype(np.float64)

        u1 = np.einsum('nsci,i->nsc', h, self._BIN_VALUES)    / safe_counts
        u2 = np.einsum('nsci,i->nsc', h, self._BIN_VALUES_SQ) / safe_counts

        var = np.maximum(u2 - u1 ** 2, 0.0)
        std = np.sqrt(var)

        mean = np.where(counts > 0, u1, np.nan)

        batch.mean_raw     = mean.astype(np.float32)
        batch.std_raw      = std.astype(np.float32)
        batch.contrast_raw = None
        return batch

    def reset(self) -> None:
        pass
