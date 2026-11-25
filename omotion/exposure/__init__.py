"""Optimize Laser: auto-exposure for the blood-flow cameras.

Pure algorithm, no hardware I/O: :mod:`.estimate` (saturation-aware frame
brightness), :mod:`.settings` (the settings menu, hardware limits, gain
ladder), :mod:`.controller` (the multi-camera controller) and
:mod:`.firmware` (which console firmware may take the extended laser
limits). :mod:`.actuator` does the device writes, and
:mod:`omotion.OptimizeLaserWorkflow` runs the whole procedure.
"""

from .actuator import LaserBaseline, LaserCameraActuator
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
from .firmware import ExtendedLimitsRefused, extended_limits_problem, require_extended_limits
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
    "ExtendedLimitsRefused",
    "GainLadder",
    "GainMode",
    "GainPriority",
    "HardwareLimits",
    "IterationRecord",
    "Knob",
    "LaserBaseline",
    "LaserCameraActuator",
    "Method",
    "OptimizeLaserSettings",
    "OptimizeStatus",
    "SignalEstimate",
    "StartPoint",
    "censored_gamma_mean",
    "estimate_signal",
    "extended_limits_problem",
    "require_extended_limits",
]
