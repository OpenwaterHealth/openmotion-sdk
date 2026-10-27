"""Pure values and validation rules for WI-00015 Safety Calibration."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum
import math
from typing import Literal, Mapping

from .laser import (
    CriterionResult,
    DeviceIdentity,
    TopologySnapshot,
    validate_serial,
)
from ._procedure import (
    deeply_immutable as _deeply_immutable,
    finite_number as _finite_number,
)


SafetyController = Literal["SAFETY_OPT", "SAFETY_EE"]


class ShippingTopology(str, Enum):
    """The sensor configuration in which the console will ship.

    CONSOLE_ONLY is a bench declaration: the console-side safety
    calibration runs without any sensor modules attached, and the final
    production scan is not applicable.
    """

    SINGLE_LEFT = "single-left"
    SINGLE_RIGHT = "single-right"
    DUAL = "dual"
    CONSOLE_ONLY = "console-only"


ADC_ROUNDING_RULE = "nearest integer; exact halves round upward"
# Minimum measured console power-off dwell. Reduced from 15 s to 1 s on
# Ethan's bench ruling (2026-08-14): the persistence property being proven
# does not depend on dwell length, only on an observed real power cycle.
MINIMUM_POWER_OFF_S = 1.0
SAFETY_OPT_MULTIPLIER = 1.3
SAFETY_EE_MULTIPLIER = 1.1
PULSE_LIMIT_MULTIPLIER = 1.1
MAX_TA_PULSE_WIDTH_US = 600
MAX_PULSE_WIDTH_LIMIT_US = 660
MINIMUM_ADC_SAMPLES = 10

REQUIRED_USER_CONFIGURATION_KEYS = (
    "TA_PULSE_WIDTH",
    "TA_CURRENT_DRV",
    "SEED_CW_GAIN",
    "EE_PULSE_WIDTH_UL",
    "EE_RATE_LL",
    "EE_DRIVE_CL",
    "OPT_PULSE_WIDTH_UL",
    "OPT_RATE_LL",
    "OPT_DRIVE_CL",
    "TEC_TRIP",
)


def _positive_integer(value: object) -> bool:
    return _finite_number(value) and float(value).is_integer() and float(value) > 0


@dataclass(frozen=True)
class AdcReadEvidence:
    controller: SafetyController
    attempt: int
    timestamp: datetime
    accepted: bool
    value_ma: float | None
    rejection_reason: str | None


@dataclass(frozen=True)
class SafetyLimitCalculation:
    controller: SafetyController
    samples_ma: tuple[float, ...]
    sample_count: int
    mean_ma: float
    multiplier: float
    unrounded_limit_ma: float
    rounding_rule: str
    rounded_limit_ma: int


@dataclass(frozen=True)
class PulseLimitCalculation:
    ta_pulse_width_us: int
    multiplier: float
    unrounded_limit_us: float
    rounding_rule: str
    rounded_limit_us: int
    ee_pulse_width_ul_us: int
    opt_pulse_width_ul_us: int
    mode: Literal["derived", "retained"]


@dataclass(frozen=True)
class PowerCycleEvidence:
    off_requested_at: datetime | None
    disconnect_observed_at: datetime | None
    on_allowed_at: datetime | None
    on_requested_at: datetime | None
    reconnect_observed_at: datetime | None
    off_duration_s: float | None
    disconnect_observed: bool
    reconnect_observed: bool
    restart_proven: bool
    restart_proof: str | None
    console_serial_before: str | None
    console_serial_after: str | None


@dataclass(frozen=True)
class SafetyWarningEvidence:
    timestamp: datetime
    safety_known: bool
    safety_ok: bool
    faults: tuple[str, ...]
    raw_state: Mapping[str, object]

    def __post_init__(self) -> None:
        object.__setattr__(self, "faults", tuple(self.faults))
        object.__setattr__(self, "raw_state", _deeply_immutable(self.raw_state))


@dataclass(frozen=True)
class NormalScanEvidence:
    declared_topology: ShippingTopology
    requested_duration_s: float
    actual_duration_s: float
    started: bool
    completed: bool
    canceled: bool
    error: str | None
    topology: TopologySnapshot
    identities: tuple[DeviceIdentity, ...]
    overrides: Mapping[str, object]
    safety_observations: tuple[SafetyWarningEvidence, ...]
    warnings: tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "identities", tuple(self.identities))
        object.__setattr__(self, "overrides", _deeply_immutable(self.overrides))
        object.__setattr__(
            self, "safety_observations", tuple(self.safety_observations)
        )
        object.__setattr__(self, "warnings", tuple(self.warnings))


def nearest_integer_half_up(value: float) -> int:
    """Round a finite nonnegative value to nearest integer, with halves upward."""
    if not _finite_number(value) or float(value) < 0:
        raise ValueError("value must be a finite nonnegative number")
    return math.floor(float(value) + 0.5)


def calculate_safety_limit(
    controller: SafetyController,
    samples_ma: tuple[float, ...] | list[float],
    multiplier: float,
) -> SafetyLimitCalculation:
    """Calculate one safety-current limit from accepted scaled-mA samples."""
    if controller not in ("SAFETY_OPT", "SAFETY_EE"):
        raise ValueError("controller must be SAFETY_OPT or SAFETY_EE")
    samples = tuple(samples_ma)
    if not samples:
        raise ValueError("at least one scaled-mA sample is required")
    if any(not _finite_number(value) or float(value) < 0 for value in samples):
        raise ValueError("scaled-mA samples must be finite nonnegative numbers")
    if not _finite_number(multiplier) or float(multiplier) <= 0:
        raise ValueError("multiplier must be a finite positive number")

    numeric_samples = tuple(float(value) for value in samples)
    mean_ma = math.fsum(numeric_samples) / len(numeric_samples)
    unrounded_limit_ma = mean_ma * float(multiplier)
    return SafetyLimitCalculation(
        controller=controller,
        samples_ma=numeric_samples,
        sample_count=len(numeric_samples),
        mean_ma=mean_ma,
        multiplier=float(multiplier),
        unrounded_limit_ma=unrounded_limit_ma,
        rounding_rule=ADC_ROUNDING_RULE,
        rounded_limit_ma=nearest_integer_half_up(unrounded_limit_ma),
    )


def calculate_pulse_limits(
    ta_pulse_width_us: int | float,
    *,
    current_ee_limit_us: int | float,
    current_opt_limit_us: int | float,
) -> PulseLimitCalculation:
    """Calculate or retain the approved upper pulse-width limits."""
    if not _positive_integer(ta_pulse_width_us):
        raise ValueError("TA_PULSE_WIDTH must be a positive integer")
    pulse_width = int(ta_pulse_width_us)
    if pulse_width > MAX_TA_PULSE_WIDTH_US:
        raise ValueError("TA_PULSE_WIDTH must not exceed 600 us")

    unrounded = pulse_width * PULSE_LIMIT_MULTIPLIER
    rounded = nearest_integer_half_up(unrounded)
    mode: Literal["derived", "retained"] = "derived"
    if pulse_width == MAX_TA_PULSE_WIDTH_US:
        if (
            current_ee_limit_us != MAX_PULSE_WIDTH_LIMIT_US
            or current_opt_limit_us != MAX_PULSE_WIDTH_LIMIT_US
            or isinstance(current_ee_limit_us, bool)
            or isinstance(current_opt_limit_us, bool)
        ):
            raise ValueError("600 us requires both pulse-width limits to be 660 us")
        mode = "retained"

    return PulseLimitCalculation(
        ta_pulse_width_us=pulse_width,
        multiplier=PULSE_LIMIT_MULTIPLIER,
        unrounded_limit_us=unrounded,
        rounding_rule=ADC_ROUNDING_RULE,
        rounded_limit_us=rounded,
        ee_pulse_width_ul_us=rounded,
        opt_pulse_width_ul_us=rounded,
        mode=mode,
    )


def validate_current_configuration(
    configuration: Mapping[str, object] | object,
) -> tuple[CriterionResult, ...]:
    """Return each local Safety Calibration configuration gate."""
    is_mapping = isinstance(configuration, Mapping)
    config = configuration if is_mapping else {}
    criteria = [
        CriterionResult(
            "configuration_mapping",
            is_mapping,
            "User Configuration must be a mapping.",
        )
    ]
    for key in REQUIRED_USER_CONFIGURATION_KEYS:
        present = key in config
        criteria.append(
            CriterionResult(
                f"config_key_{key}",
                present,
                f"User Configuration must contain {key}.",
            )
        )
        if present:
            criteria.append(
                CriterionResult(
                    f"config_value_{key}",
                    _finite_number(config[key]),
                    f"{key} must be a finite numeric value and not a boolean.",
                )
            )

    ta_current = config.get("TA_CURRENT_DRV")
    criteria.append(
        CriterionResult(
            "ta_current",
            _finite_number(ta_current) and float(ta_current) > 0,
            "TA_CURRENT_DRV must be finite and positive.",
        )
    )
    pulse_width = config.get("TA_PULSE_WIDTH")
    pulse_valid = (
        _positive_integer(pulse_width)
        and int(pulse_width) <= MAX_TA_PULSE_WIDTH_US
    )
    criteria.append(
        CriterionResult(
            "ta_pulse_width",
            pulse_valid,
            "TA_PULSE_WIDTH must be a positive integer no greater than 600 us.",
        )
    )
    at_600_valid = True
    if pulse_valid and int(pulse_width) == MAX_TA_PULSE_WIDTH_US:
        at_600_valid = (
            config.get("EE_PULSE_WIDTH_UL") == MAX_PULSE_WIDTH_LIMIT_US
            and config.get("OPT_PULSE_WIDTH_UL") == MAX_PULSE_WIDTH_LIMIT_US
            and not isinstance(config.get("EE_PULSE_WIDTH_UL"), bool)
            and not isinstance(config.get("OPT_PULSE_WIDTH_UL"), bool)
        )
    criteria.append(
        CriterionResult(
            "pulse_limits_at_600",
            at_600_valid,
            "A 600 us operating point requires both pulse-width limits at 660 us.",
        )
    )
    return tuple(criteria)


def validate_shipping_topology(
    declared_topology: ShippingTopology,
    topology: TopologySnapshot,
    *,
    left_identity: DeviceIdentity,
    right_identity: DeviceIdentity,
) -> CriterionResult:
    """Require the exact declared sensor topology and all required serials."""
    if not isinstance(declared_topology, ShippingTopology):
        return CriterionResult(
            "shipping_topology",
            False,
            "Shipping topology declaration is invalid.",
        )

    expected_sides = {
        ShippingTopology.SINGLE_LEFT: (True, False),
        ShippingTopology.SINGLE_RIGHT: (False, True),
        ShippingTopology.DUAL: (True, True),
        ShippingTopology.CONSOLE_ONLY: (False, False),
    }[declared_topology]
    topology_matches = (
        topology.console_connected
        and (topology.left_connected, topology.right_connected) == expected_sides
    )
    required_identities = []
    if expected_sides[0]:
        required_identities.append(left_identity)
    if expected_sides[1]:
        required_identities.append(right_identity)
    serials_valid = all(validate_serial(identity.serial).passed for identity in required_identities)
    passed = topology_matches and serials_valid
    detail = (
        f"Shipping topology expected {declared_topology.value} with nonblank "
        "required sensor serials."
    )
    return CriterionResult("shipping_topology", passed, detail)
