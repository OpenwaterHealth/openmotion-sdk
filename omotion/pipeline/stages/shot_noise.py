"""ShotNoiseCorrectionStage — Poisson-variance subtraction.

Operates on two paths:
  Realtime: batch fields mean_dc_rt / std_dc_rt → std_sn_rt, contrast_sn_rt
  Batch:    IntervalClosed events carrying CorrectedInterval — mutates each
            CorrectedFrame's std in place and sets its contrast field.

See docs/SciencePipeline.md §8.3.
"""

from __future__ import annotations

import logging

import numpy as np

from ..batch import FrameBatch, IntervalClosed

logger = logging.getLogger("openmotion.sdk.pipeline.stages.shot_noise")
from ..pedestal import SensorPedestals, adc_gain_for_pedestal
from .dark import CorrectedInterval


class ShotNoiseCorrectionStage:
    name = "shot_noise_correction"

    def __init__(self, pedestals: SensorPedestals, camera_gain_map: np.ndarray):
        # ADC gain is (HISTO_SIZE_WORDS - pedestal) / ELECTRON_WELL_CAPACITY — different per side when the
        # two sensor modules ship with different firmware. Pre-broadcast as
        # (1, 2, 1) so the multiplication aligns with the (N, 2, 8) mean.
        self._adc_gain = np.array(
            [adc_gain_for_pedestal(pedestals.left),
             adc_gain_for_pedestal(pedestals.right)],
            dtype=np.float64,
        ).reshape(1, 2, 1)
        # Scalar per-side gains for the event path.
        self._adc_gain_scalar = (
            adc_gain_for_pedestal(pedestals.left),
            adc_gain_for_pedestal(pedestals.right),
        )
        self._gain_map = np.asarray(camera_gain_map, dtype=np.float32).reshape(1, 1, 8)
        self._gain_map_flat = np.asarray(camera_gain_map, dtype=np.float32).ravel()

    def process(self, batch: FrameBatch) -> FrameBatch:
        self._process_realtime(batch)
        self._process_events(batch)
        return batch

    def _process_realtime(self, batch: FrameBatch) -> None:
        mean = batch.mean_dc_rt
        std  = batch.std_dc_rt
        if mean is None or std is None:
            return
        var  = std.astype(np.float64) ** 2

        shot_var = self._adc_gain * np.maximum(0.0, mean.astype(np.float64)) * self._gain_map
        corrected_var = var - shot_var
        neg_mask = corrected_var < 0
        n_neg = int(np.sum(neg_mask & np.isfinite(corrected_var)))
        if n_neg > 0:
            logger.debug(
                "realtime shot-noise clamped %d/%d negative variance slots",
                n_neg, int(np.sum(np.isfinite(corrected_var))),
            )
        corrected_var = np.maximum(0.0, corrected_var)
        std_sn = np.sqrt(corrected_var).astype(np.float32)

        with np.errstate(divide='ignore', invalid='ignore'):
            mean_valid = np.isfinite(mean)
            contrast = np.where(
                mean_valid & (mean > 0),
                std_sn / mean,
                np.where(mean_valid, np.float32(0.0), np.float32("nan")),
            )

        batch.std_sn_rt      = std_sn
        batch.contrast_sn_rt = contrast.astype(np.float32)

    def _process_events(self, batch: FrameBatch) -> None:
        """Apply shot-noise correction to CorrectedFrames in IntervalClosed events."""
        for event in batch.events:
            if not isinstance(event, IntervalClosed):
                continue
            ci = event.corrected_batch
            if not isinstance(ci, CorrectedInterval):
                continue
            for f in ci.frames:
                side_idx = 0 if f.side == "left" else 1
                cam_pos = int(f.cam_id) % 8
                adc_gain = self._adc_gain_scalar[side_idx]
                g_cam = float(self._gain_map_flat[cam_pos])

                shot_var = adc_gain * max(0.0, f.mean) * g_cam
                corrected_var = f.std ** 2 - shot_var
                if corrected_var < 0:
                    logger.debug(
                        "batch shot-noise clamped negative variance: "
                        "side=%s cam=%d abs_id=%d signal_var=%.3f "
                        "shot_var=%.3f deficit=%.3f",
                        f.side, f.cam_id, f.abs_frame_id,
                        f.std ** 2, shot_var, -corrected_var,
                    )
                    corrected_var = 0.0
                f.std = corrected_var ** 0.5
                f.contrast = f.std / f.mean if f.mean > 0 else 0.0

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Process events from DarkCorrectionStage's terminal flush."""
        self._process_events(batch)

    def reset(self) -> None:
        pass
