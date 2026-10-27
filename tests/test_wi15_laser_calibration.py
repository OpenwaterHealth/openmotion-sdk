from dataclasses import is_dataclass, replace
import math

import pytest

from omotion.calibration.laser import (
    CURRENT_FLOOR_MA,
    CURRENT_STEP_MA,
    CriterionResult,
    DEFAULT_USER_CONFIG,
    DeviceIdentity,
    FailureKind,
    FpgaFirmwareRevision,
    MAX_ACCEPTABLE_ENERGY_UJ,
    MAX_PULSE_WIDTH_US,
    MAX_RATE_HZ,
    MAX_STDEV_UJ_EXCLUSIVE,
    MIN_ACCEPTABLE_ENERGY_UJ,
    MIN_PULSE_COUNT_EXCLUSIVE,
    MIN_RATE_HZ,
    OphirIdentity,
    PairMetrics,
    PULSE_WIDTH_STEP_US,
    ProcedureStatus,
    SettingReadback,
    TARGET_ENERGY_UJ,
    TEMPORARY_PULSE_WIDTH_LIMIT_US,
    TopologySnapshot,
    both_energies_accepted,
    calculate_pair_metrics,
    percent_difference,
    select_closest_valid_setting_to_target,
    validate_energy_measurement,
    validate_console_fpga_revisions,
    validate_exact_dual_topology,
    validate_exact_single_topology,
    validate_serial,
    within_percent,
)
from wi15_builders import valid_measurement as _valid_measurement


def test_console_identity_requires_all_four_named_fpga_firmware_revisions():
    complete = DeviceIdentity(
        "console",
        "C-1",
        "1.2.3",
        "HW-1",
        fpga_firmware_revisions=tuple(
            FpgaFirmwareRevision(controller, "1.2.3")
            for controller in ("TA", "SEED", "SAFETY_EE", "SAFETY_OPT")
        ),
    )

    assert validate_console_fpga_revisions(complete).passed
    assert not validate_console_fpga_revisions(
        replace(
            complete,
            fpga_firmware_revisions=complete.fpga_firmware_revisions[:-1],
        )
    ).passed
    assert not validate_console_fpga_revisions(
        replace(
            complete,
            fpga_firmware_revisions=tuple(
                reversed(complete.fpga_firmware_revisions)
            ),
        )
    ).passed
    assert not validate_console_fpga_revisions(
        replace(
            complete,
            fpga_firmware_revisions=(
                *complete.fpga_firmware_revisions[:-1],
                FpgaFirmwareRevision("SAFETY_OPT", "  "),
            ),
        )
    ).passed


def test_wi15_fixed_thresholds_and_default_user_configuration():
    """Changing any mandated WI-00015 threshold or default must be detected."""
    assert TARGET_ENERGY_UJ == 350
    assert (MIN_ACCEPTABLE_ENERGY_UJ, MAX_ACCEPTABLE_ENERGY_UJ) == (300, 400)
    assert MIN_PULSE_COUNT_EXCLUSIVE == 25
    assert MAX_STDEV_UJ_EXCLUSIVE == 40
    assert (MIN_RATE_HZ, MAX_RATE_HZ) == (39, 41)
    assert (CURRENT_STEP_MA, CURRENT_FLOOR_MA) == (50, 2000)
    assert (PULSE_WIDTH_STEP_US, MAX_PULSE_WIDTH_US) == (10, 600)
    assert TEMPORARY_PULSE_WIDTH_LIMIT_US == 660
    assert DEFAULT_USER_CONFIG == {
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
    }
    assert isinstance(DEFAULT_USER_CONFIG["TEC_TRIP"], int)


def _criteria(measurement):
    return {result.name: result for result in validate_energy_measurement(measurement)}


def test_energy_measurement_requires_strictly_more_than_25_pulses():
    """Changing the count gate to >= 25 would admit an invalid observation."""
    at_25 = _criteria(_valid_measurement(n=25))["pulse_count"]
    at_26 = _criteria(_valid_measurement(n=26))["pulse_count"]
    assert not at_25.passed
    assert at_25.detail == "Pulse count must be > 25."
    assert at_26.passed


def test_energy_measurement_requires_standard_deviation_below_40_uj():
    """Changing the deviation gate to <= 40 would admit a boundary failure."""
    assert _criteria(_valid_measurement(stdev_uj=39.999))["stdev_uj"].passed
    assert not _criteria(_valid_measurement(stdev_uj=40.0))["stdev_uj"].passed


@pytest.mark.parametrize(
    ("rate_hz", "passed"),
    [(38.999, False), (39.0, True), (41.0, True), (41.001, False)],
)
def test_energy_measurement_accepts_only_the_inclusive_39_to_41_hz_rate(rate_hz, passed):
    """Changing either rate boundary would accept or reject a WI boundary value."""
    assert _criteria(_valid_measurement(rate_hz=rate_hz))["rate_hz"].passed is passed


@pytest.mark.parametrize(
    "field_name",
    [
        "n",
        "discarded",
        "mean_uj",
        "stdev_uj",
        "rate_hz",
        "min_uj",
        "max_uj",
        "duration_s",
    ],
)
@pytest.mark.parametrize("non_finite", [math.nan, math.inf, -math.inf])
def test_energy_measurement_rejects_each_non_finite_statistic(field_name, non_finite):
    """Removing finite checks would allow invalid raw Ophir statistics into tuning."""
    assert not _criteria(_valid_measurement(**{field_name: non_finite}))[field_name].passed


@pytest.mark.parametrize(
    ("topology", "side", "passed"),
    [
        (TopologySnapshot(True, True, False), "left", True),
        (TopologySnapshot(True, False, True), "right", True),
        (TopologySnapshot(True, False, False), "left", False),
        (TopologySnapshot(True, True, True), "left", False),
        (TopologySnapshot(True, False, True), "left", False),
    ],
)
def test_exact_single_topology_requires_only_the_declared_side(topology, side, passed):
    """Ignoring absent/extra/wrong sensors would allow an ambiguous procedure run."""
    assert validate_exact_single_topology(topology, side).passed is passed


@pytest.mark.parametrize(
    ("topology", "passed"),
    [
        (TopologySnapshot(True, True, True), True),
        (TopologySnapshot(True, True, False), False),
        (TopologySnapshot(True, False, True), False),
        (TopologySnapshot(False, True, True), False),
    ],
)
def test_exact_dual_topology_requires_console_left_and_right(topology, passed):
    """Dropping any device from the declared shipping topology must fail preflight."""
    assert validate_exact_dual_topology(topology).passed is passed


@pytest.mark.parametrize(
    ("left", "right", "accepted"),
    [
        (300.0, 400.0, True),
        (299.999, 350.0, False),
        (350.0, 400.001, False),
        (math.nan, 350.0, False),
    ],
)
def test_dual_acceptance_requires_both_finite_values_in_inclusive_window(
    left, right, accepted
):
    """A single out-of-window or nonfinite sensor must reject the paired result."""
    assert both_energies_accepted(left, right) is accepted


@pytest.mark.parametrize(
    ("serial", "passed"),
    [(None, False), ("", False), ("  \t", False), (" SN-123 ", True)],
)
def test_serial_validation_requires_nonblank_text(serial, passed):
    """Accepting blank identity text would make the calibration evidence unusable."""
    assert validate_serial(serial).passed is passed


def test_percent_difference_handles_a_zero_requested_setting_without_raising():
    """A zero requested setting must have defined comparison behavior."""
    assert percent_difference(100.0, 102.0) == 2.0
    assert percent_difference(0.0, 0.0) == 0.0
    assert math.isinf(percent_difference(0.0, 1.0))


@pytest.mark.parametrize(
    ("actual", "passed"),
    [(98.0, True), (102.0, True), (97.999, False), (102.001, False)],
)
def test_within_percent_includes_exact_plus_or_minus_two_percent(actual, passed):
    """Changing the tolerance boundary would reject valid readbacks or admit bad ones."""
    assert within_percent(100.0, actual, 2.0) is passed


def test_closest_setting_uses_supplied_dual_target_and_rejects_nonfinite_target():
    """Using 350 instead of the calculated per-side target selects the wrong setting."""
    candidates = [
        (5000, _valid_measurement(mean_uj=390.0)),
        (4950, _valid_measurement(mean_uj=370.0)),
    ]
    assert select_closest_valid_setting_to_target(candidates, 375.0) == candidates[1]
    assert select_closest_valid_setting_to_target(candidates, math.nan) is None
