"""Optimize Laser: auto-exposure for the blood-flow cameras.

Pure algorithm, no hardware I/O: :mod:`.estimate` (saturation-aware frame
brightness), :mod:`.settings` (the settings menu, hardware limits, gain
ladder) and :mod:`.controller` (the multi-camera controller). The workflow
that applies settings to a device and captures frames builds on these.
"""

from .controller import (
    CameraObservation,
    CameraRecord,
    CameraStatus,
    ExposureController,
    ExposureSettings,
    IterationRecord,
    OptimizeStatus,
)
from .estimate import SignalEstimate, censored_gamma_mean, estimate_signal
from .settings import (
    ABSOLUTE_MAX_CURRENT_MA,
    ABSOLUTE_MAX_PULSE_WIDTH_US,
    Bounds,
    GainLadder,
    GainMode,
    GainPriority,
    HardwareLimits,
    Knob,
    Method,
    OptimizeLaserSettings,
    StartPoint,
)

__all__ = [
    "ABSOLUTE_MAX_CURRENT_MA",
    "ABSOLUTE_MAX_PULSE_WIDTH_US",
    "Bounds",
    "CameraObservation",
    "CameraRecord",
    "CameraStatus",
    "ExposureController",
    "ExposureSettings",
    "GainLadder",
    "GainMode",
    "GainPriority",
    "HardwareLimits",
    "IterationRecord",
    "Knob",
    "Method",
    "OptimizeLaserSettings",
    "OptimizeStatus",
    "SignalEstimate",
    "StartPoint",
    "censored_gamma_mean",
    "estimate_signal",
]
