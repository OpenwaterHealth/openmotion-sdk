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
            contrast = np.where(mean > 0, std_sn / mean, np.float32(0.0))

        batch.std_sn_rt      = std_sn
        batch.contrast_sn_rt = contrast.astype(np.float32)
        return batch

    def reset(self) -> None:
        pass
