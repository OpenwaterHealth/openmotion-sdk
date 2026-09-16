"""Pure values and validation helpers for WI-00015 laser calibration."""

from dataclasses import dataclass
from enum import Enum
import math
from types import MappingProxyType
from typing import Iterable, Literal


SensorSide = Literal["left", "right"]
REQUIRED_CONSOLE_FPGA_CONTROLLERS = (
    "TA",
    "SEED",
    "SAFETY_EE",
    "SAFETY_OPT",
)


class ProcedureStatus(str, Enum):
    IN_PROGRESS = "in_progress"
    PASSED = "passed"
    FAILED = "failed"
    FAILED_NCR = "failed_ncr"
    CANCELED = "canceled"
    # Written outside the acceptance criteria with explicit operator consent
    # (omotion.calibration.override). Deliberately distinct from PASSED: the
    # evidence, the script verdict and the exit code all say "override".
    OVERRIDDEN = "overridden"


class FailureKind(str, Enum):
    SETUP = "setup"
    CONFIGURATION = "configuration"
    MEASUREMENT = "measurement"
    NCR = "ncr"
    CANCELED = "canceled"
    REPORT = "report"


@dataclass(frozen=True)
class EnergyMeasurement:
    n: int
    discarded: int
    mean_uj: float
    stdev_uj: float
    rate_hz: float
    min_uj: float
    max_uj: float
    duration_s: float


@dataclass(frozen=True)
class CriterionResult:
    name: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class SettingReadback:
    name: str
    requested: float
    actual: float


@dataclass(frozen=True)
class FinalSettingCheck:
    name: str
    requested: float
    actual: float
    absolute_difference: float
    percent_difference: float
    tolerance_percent: float
    passed: bool


@dataclass(frozen=True)
class FpgaFirmwareRevision:
    controller: str
    version: str


@dataclass(frozen=True)
class DeviceIdentity:
    role: str
    serial: str | None
    firmware: str | None
    hardware_id: str | None
    fpga_firmware: str | None = None
    fpga_firmware_revisions: tuple[FpgaFirmwareRevision, ...] = ()


@dataclass(frozen=True)
class OphirIdentity:
    meter_model: str | None
    meter_serial: str | None
    sensor_model: str | None
    sensor_serial: str | None
    calibration_due: str | None


@dataclass(frozen=True)
class TopologySnapshot:
    console_connected: bool
    left_connected: bool
    right_connected: bool


@dataclass(frozen=True)
class PairMetrics:
    left_mean_uj: float
    right_mean_uj: float
    difference_uj: float
    midpoint_uj: float
    midpoint_distance_uj: float
    left_offset_uj: float
    right_offset_uj: float

TARGET_ENERGY_UJ = 350
MIN_ACCEPTABLE_ENERGY_UJ = 300
MAX_ACCEPTABLE_ENERGY_UJ = 400
MIN_PULSE_COUNT_EXCLUSIVE = 25
MAX_STDEV_UJ_EXCLUSIVE = 40
MIN_RATE_HZ = 39
MAX_RATE_HZ = 41
CURRENT_STEP_MA = 50
CURRENT_FLOOR_MA = 2000
PULSE_WIDTH_STEP_US = 10
MAX_PULSE_WIDTH_US = 600
TEMPORARY_PULSE_WIDTH_LIMIT_US = 660

DEFAULT_USER_CONFIG = MappingProxyType({
    "TA_PULSE_WIDTH": 500,
    "TA_CURRENT_DRV": 5000,
    "SEED_CW_GAIN": 140,
    "EE_PULSE_WIDTH_UL": 550,
    "EE_RATE_LL": 23125,
    "EE_DRIVE_CL": 9999,
    "OPT_PULSE_WIDTH_UL": 550,
    "OPT_RATE_LL": 23125,
    "OPT_DRIVE_CL": 9999,
    "TEC_TRIP": 40,
})


def default_user_configuration() -> dict[str, int]:
    """Return a fresh copy of the approved ten-key WI-00015 defaults."""
    return dict(DEFAULT_USER_CONFIG)


def _is_finite(value: object) -> bool:
    try:
        return math.isfinite(value)
    except TypeError:
        return False


def validate_energy_measurement(
    measurement: EnergyMeasurement,
) -> tuple[CriterionResult, ...]:
    """Return each WI-00015 measurement-quality criterion without raising."""
    pulse_count_is_valid = (
        _is_finite(measurement.n) and measurement.n > MIN_PULSE_COUNT_EXCLUSIVE
    )
    return (
        CriterionResult("n", _is_finite(measurement.n), "Pulse count must be finite."),
        CriterionResult(
            "pulse_count",
            pulse_count_is_valid,
            "Pulse count must be > 25.",
        ),
        CriterionResult(
            "discarded",
            _is_finite(measurement.discarded),
            "Discarded count must be finite.",
        ),
        CriterionResult(
            "mean_uj",
            _is_finite(measurement.mean_uj),
            "Mean energy must be finite.",
        ),
        CriterionResult(
            "stdev_uj",
            _is_finite(measurement.stdev_uj)
            and measurement.stdev_uj < MAX_STDEV_UJ_EXCLUSIVE,
            "Standard deviation must be finite and < 40 uJ.",
        ),
        CriterionResult(
            "rate_hz",
            _is_finite(measurement.rate_hz)
            and MIN_RATE_HZ <= measurement.rate_hz <= MAX_RATE_HZ,
            "Rate must be finite and between 39 and 41 Hz inclusive.",
        ),
        CriterionResult(
            "min_uj",
            _is_finite(measurement.min_uj),
            "Minimum energy must be finite.",
        ),
        CriterionResult(
            "max_uj",
            _is_finite(measurement.max_uj),
            "Maximum energy must be finite.",
        ),
        CriterionResult(
            "duration_s",
            _is_finite(measurement.duration_s),
            "Measurement duration must be finite.",
        ),
    )


def validate_exact_single_topology(
    topology: TopologySnapshot,
    side: SensorSide,
) -> CriterionResult:
    """Require a console and only the sensor declared for this run."""
    selected_present = (
        topology.left_connected and not topology.right_connected
        if side == "left"
        else topology.right_connected and not topology.left_connected
        if side == "right"
        else False
    )
    passed = topology.console_connected and selected_present
    detail = (
        "Expected a console and exactly the declared sensor side."
        if passed
        else _single_topology_failure_detail(topology, side)
    )
    return CriterionResult("topology", passed, detail)


def _single_topology_failure_detail(
    topology: TopologySnapshot, side: SensorSide
) -> str:
    if not topology.console_connected:
        return "The console is not connected."
    if topology.left_connected and topology.right_connected:
        return (
            "Both sensors are connected; use Dual-Sensor Laser Calibration "
            "instead, or disconnect the sensor not being calibrated to "
            "continue with Single-Sensor Laser Calibration."
        )
    if not topology.left_connected and not topology.right_connected:
        return "No sensor is connected; connect the declared sensor side."
    wrong_side = "right" if side == "left" else "left"
    return (
        f"The {wrong_side} sensor is connected instead of the declared "
        f"{side} sensor."
    )


def validate_exact_dual_topology(topology: TopologySnapshot) -> CriterionResult:
    """Require a console with both declared shipping sensors connected."""
    passed = (
        topology.console_connected
        and topology.left_connected
        and topology.right_connected
    )
    detail = (
        "Expected a console with both left and right sensors connected."
        if passed
        else _dual_topology_failure_detail(topology)
    )
    return CriterionResult("topology", passed, detail)


def _dual_topology_failure_detail(topology: TopologySnapshot) -> str:
    if not topology.console_connected:
        return "The console is not connected."
    if not topology.left_connected and not topology.right_connected:
        return (
            "No sensor is connected; connect both sensor modules to "
            "continue with Dual-Sensor Laser Calibration, or connect one "
            "sensor and use Single-Sensor Laser Calibration instead."
        )
    connected_side = "left" if topology.left_connected else "right"
    return (
        f"Only the {connected_side} sensor is connected; use "
        "Single-Sensor Laser Calibration instead, or connect the other "
        "sensor module to continue with Dual-Sensor Laser Calibration."
    )


def calculate_pair_metrics(
    left_mean_uj: float,
    right_mean_uj: float,
    target_energy_uj: float = TARGET_ENERGY_UJ,
) -> PairMetrics:
    """Calculate immutable, side-preserving evidence for one complete pair."""
    difference = abs(left_mean_uj - right_mean_uj)
    midpoint = (left_mean_uj + right_mean_uj) / 2.0
    return PairMetrics(
        left_mean_uj=left_mean_uj,
        right_mean_uj=right_mean_uj,
        difference_uj=difference,
        midpoint_uj=midpoint,
        midpoint_distance_uj=abs(midpoint - target_energy_uj),
        left_offset_uj=left_mean_uj - target_energy_uj,
        right_offset_uj=right_mean_uj - target_energy_uj,
    )


def both_energies_accepted(
    left_mean_uj: float,
    right_mean_uj: float,
    minimum_energy_uj: float = MIN_ACCEPTABLE_ENERGY_UJ,
    maximum_energy_uj: float = MAX_ACCEPTABLE_ENERGY_UJ,
) -> bool:
    """Return whether both finite side means satisfy the inclusive WI window."""
    return all(
        _is_finite(value)
        and minimum_energy_uj <= value <= maximum_energy_uj
        for value in (left_mean_uj, right_mean_uj)
    )


def validate_serial(serial: str | None) -> CriterionResult:
    """Require a nonblank textual serial number for reportable identity."""
    passed = isinstance(serial, str) and bool(serial.strip())
    return CriterionResult("serial", passed, "Serial must be nonblank text.")


def validate_console_fpga_revisions(identity: DeviceIdentity) -> CriterionResult:
    """Require one nonblank firmware revision for every console-board FPGA."""
    revisions = identity.fpga_firmware_revisions
    records_are_typed = isinstance(revisions, tuple) and all(
        isinstance(revision, FpgaFirmwareRevision) for revision in revisions
    )
    controllers = (
        tuple(revision.controller for revision in revisions)
        if records_are_typed
        else ()
    )

    def valid_version(version: object) -> bool:
        if not isinstance(version, str):
            return False
        parts = version.split(".")
        return len(parts) == 3 and all(
            part.isascii() and part.isdigit() and 0 <= int(part) <= 255
            for part in parts
        )

    passed = (
        records_are_typed
        and controllers == REQUIRED_CONSOLE_FPGA_CONTROLLERS
        and all(valid_version(revision.version) for revision in revisions)
    )
    return CriterionResult(
        "console_fpga_firmware_revisions",
        passed,
        "Console identity must include TA, SEED, SAFETY_EE, and SAFETY_OPT FPGA firmware revisions.",
    )


def percent_difference(requested: float, actual: float) -> float:
    """Return absolute percent difference, with defined behavior at zero."""
    if not (_is_finite(requested) and _is_finite(actual)):
        return math.nan
    if requested == 0:
        return 0.0 if actual == 0 else math.inf
    return abs(actual - requested) / abs(requested) * 100.0


def within_percent(requested: float, actual: float, tolerance_percent: float) -> bool:
    """Return whether an active setting stays within an inclusive tolerance."""
    return (
        _is_finite(tolerance_percent)
        and tolerance_percent >= 0
        and percent_difference(requested, actual) <= tolerance_percent
    )


def select_closest_valid_setting_to_target(
    candidates: Iterable[tuple[float, EnergyMeasurement]],
    target_uj: float,
) -> tuple[float, EnergyMeasurement] | None:
    """Select the valid observation nearest a finite, caller-supplied target."""
    if not _is_finite(target_uj):
        return None
    valid_candidates = [
        candidate
        for candidate in candidates
        if _is_finite(candidate[0])
        and all(
            result.passed for result in validate_energy_measurement(candidate[1])
        )
    ]
    if not valid_candidates:
        return None
    return min(
        valid_candidates,
        key=lambda candidate: (
            abs(candidate[1].mean_uj - target_uj),
            candidate[0],
        ),
    )
