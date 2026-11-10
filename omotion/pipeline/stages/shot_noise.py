"""ShotNoiseCorrectionStage — Poisson-variance subtraction.

Operates on two paths:
  Realtime: batch fields mean_dc_rt / std_dc_rt → std_sn_rt, contrast_sn_rt
  Batch:    IntervalClosed events carrying CorrectedInterval — mutates each
            CorrectedFrame's std in place and sets its contrast field.

See docs/SciencePipeline.md §5.9 ShotNoiseCorrectionStage (batch-path
math: §5.8.5).
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
        # A measured variance below the Poisson floor is unphysical: the
        # speckle variance is unresolvable, NOT zero. Reporting √0 = 0 here
        # made contrast exactly 0.0, which the calibration map reads as a
        # perfectly coherent speckle field — maximal flow (issue #114).
        unresolved = corrected_var < 0
        n_neg = int(np.sum(unresolved & np.isfinite(corrected_var)))
        if n_neg > 0:
            logger.debug(
                "realtime shot-noise: %d/%d slots below the Poisson floor "
                "(speckle variance unresolvable, emitted as NaN)",
                n_neg, int(np.sum(np.isfinite(corrected_var))),
            )
        std_sn = np.where(unresolved, np.float32("nan"),
                          np.sqrt(np.maximum(0.0, corrected_var))).astype(np.float32)

        # mean > 0 is False for NaN as well as for a non-positive mean, so both
        # "no frame" and "no signal above the dark baseline" fall through to
        # NaN. Contrast is undefined there — emitting 0.0 would make the
        # calibration map read it as a perfectly coherent speckle field and
        # report super-maximal flow (issue #114).
        with np.errstate(divide='ignore', invalid='ignore'):
            contrast = np.where(mean > 0, std_sn / mean, np.float32("nan"))

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

                mean = float(f.mean)
                if not np.isfinite(mean):
                    # NaN-fill row for a dropped frame: there is no
                    # measurement. The builtin max(0.0, NaN) returns 0.0
                    # (NaN comparisons are False), which used to launder the
                    # gap into a zero-shot-noise frame; propagate NaN like the
                    # realtime path's np.maximum does (issue #114).
                    f.std = float("nan")
                    f.contrast = float("nan")
                    continue

                shot_var = adc_gain * max(0.0, mean) * g_cam
                corrected_var = f.std ** 2 - shot_var
                if corrected_var < 0:
                    # Below the Poisson floor — unphysical, so the speckle
                    # variance cannot be resolved. Clamping to 0 here yielded
                    # contrast 0.0 and therefore a finite maximal BFI; a
                    # pitch-dark scan produced 4151 such frames (issue #114).
                    logger.debug(
                        "batch shot-noise below Poisson floor: "
                        "side=%s cam=%d abs_id=%d signal_var=%.3f "
                        "shot_var=%.3f deficit=%.3f — unresolvable, NaN",
                        f.side, f.cam_id, f.abs_frame_id,
                        f.std ** 2, shot_var, -corrected_var,
                    )
                    f.std = float("nan")
                    f.contrast = float("nan")
                    continue
                f.std = corrected_var ** 0.5
                # mean <= 0: the camera saw nothing above its dark baseline
                # (covered sensor, signal-starved periphery). Undefined, not 0.
                f.contrast = f.std / mean if mean > 0 else float("nan")

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Process events from DarkCorrectionStage's terminal flush."""
        self._process_events(batch)

    def reset(self) -> None:
        pass
