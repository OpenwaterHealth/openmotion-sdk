"""ShotNoiseCorrectionStage — Poisson-variance subtraction.

See docs/SciencePipeline.md §8.3.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


class ShotNoiseCorrectionStage:
    name = "shot_noise_correction"

    def __init__(self, adc_gain: float, camera_gain_map: np.ndarray):
        self.adc_gain = float(adc_gain)
        self._gain_map = np.asarray(camera_gain_map, dtype=np.float32).reshape(1, 1, 8)

    def process(self, batch: FrameBatch) -> FrameBatch:
        mean = batch.mean_dc_rt
        std  = batch.std_dc_rt
        var  = std.astype(np.float64) ** 2

        shot_var = self.adc_gain * np.maximum(0.0, mean.astype(np.float64)) * self._gain_map
        corrected_var = np.maximum(0.0, var - shot_var)
        std_sn = np.sqrt(corrected_var).astype(np.float32)

        with np.errstate(divide='ignore', invalid='ignore'):
            # Three cases:
            #   mean > 0           — normal: contrast = std/mean
            #   mean <= 0  (real)  — no signal: contrast = 0 (legacy behaviour)
            #   mean is NaN        — slot was never populated (e.g. first dark
            #                        frame before any light has been seen);
            #                        propagate NaN so BfiBvi/sinks can skip
            #                        rather than render a spurious BFI = 10.
            mean_valid = np.isfinite(mean)
            contrast = np.where(
                mean_valid & (mean > 0),
                std_sn / mean,
                np.where(mean_valid, np.float32(0.0), np.float32("nan")),
            )

        batch.std_sn_rt      = std_sn
        batch.contrast_sn_rt = contrast.astype(np.float32)
        return batch

    def reset(self) -> None:
        pass
