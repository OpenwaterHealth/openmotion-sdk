"""default_pipeline() — assembles the canonical 9-stage + 3-Tee chain."""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from .pipeline import Pipeline
from .pedestal import SensorPedestals
from .sinks import ScanMetadata
from .stages.classify import FrameClassificationStage
from .stages.noise_floor import NoiseFloorStage
from .stages.moments import MomentsStage
from .stages.pedestal_sub import PedestalSubtractionStage
from .stages.dark import (
    DarkCorrectionStage, HybridRealtimePredictor, LinearInterpolation,
    EnrichedCorrectedFrame, EnrichedCorrectedInterval,
)
from .stages.shot_noise import ShotNoiseCorrectionStage
from .stages.bfi_bvi import BfiBviStage
from .stages.side_avg import SideAveragingStage
from .stages.rolling_avg import RollingAverageStage
from .tee import Tee


ADC_GAIN = (1024 - 64) / 11_000
CAMERA_GAIN_MAP = np.array([16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32)


def default_pipeline(*,
                     metadata: ScanMetadata,
                     calibration: Any,
                     pedestals: SensorPedestals,
                     noise_floor_threshold: int = 10,
                     rolling_avg_window: int = 10,
                     discard_count: int = 9,
                     dark_interval: int = 600,
                     realtime_dark_history_size: int = 4,
                     raw_save_max_duration_s: Optional[float] = None) -> Pipeline:
    """Build the canonical pipeline. See SciencePipeline.md for the algorithm.

    Args:
        raw_save_max_duration_s: If provided and > 0, includes Tee("raw") with
            max_duration_s set. If 0 or negative, omits the raw tee. If None
            (default), includes unbounded raw tee.
    """

    not_warmup_or_stale = lambda ft: ft != "warmup" and ft != "stale"

    stages: list = [
        FrameClassificationStage(discard_count=discard_count, dark_interval=dark_interval),
    ]

    # Conditionally add raw tee based on raw_save_max_duration_s
    if raw_save_max_duration_s is None or raw_save_max_duration_s > 0:
        stages.append(
            Tee("raw", filter=lambda ft: ft != "stale", max_duration_s=raw_save_max_duration_s)
        )

    stages.extend([
        NoiseFloorStage(threshold=noise_floor_threshold),
        MomentsStage(),
        PedestalSubtractionStage(pedestals=pedestals),

        DarkCorrectionStage(
            realtime_estimator=HybridRealtimePredictor(),
            batch_estimator=LinearInterpolation(),
            pedestals=pedestals,
            realtime_history_size=realtime_dark_history_size,
            adc_gain=ADC_GAIN,
            camera_gain_map=CAMERA_GAIN_MAP,
            calibration=calibration,
        ),

        ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP),
        BfiBviStage(calibration=calibration),

        SideAveragingStage(
            enabled=metadata.reduced_mode,
            left_camera_mask=metadata.left_camera_mask,
            right_camera_mask=metadata.right_camera_mask,
        ),
        Tee("live", filter=not_warmup_or_stale),

        RollingAverageStage(window=rolling_avg_window),
        Tee("rolling", filter=not_warmup_or_stale),
    ])

    return Pipeline(stages)
