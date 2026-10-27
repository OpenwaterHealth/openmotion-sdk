from dataclasses import FrozenInstanceError, is_dataclass
from datetime import datetime, timezone
import math
from types import MappingProxyType

import pytest

from omotion.calibration.laser import (
    DeviceIdentity,
    TopologySnapshot,
)
from omotion.calibration.safety import (
    ADC_ROUNDING_RULE,
    AdcReadEvidence,
    NormalScanEvidence,
    PowerCycleEvidence,
    PulseLimitCalculation,
    SafetyLimitCalculation,
    SafetyWarningEvidence,
    ShippingTopology,
    calculate_pulse_limits,
    calculate_safety_limit,
    nearest_integer_half_up,
    validate_current_configuration,
    validate_shipping_topology,
)
from wi15_builders import valid_safety_config as _valid_config


def _criteria(config):
    return {item.name: item for item in validate_current_configuration(config)}


@pytest.mark.parametrize(
    ("value", "expected"),
    [(0.0, 0), (10.49, 10), (10.5, 11), (10.51, 11), (130.5, 131)],
)
def test_nearest_integer_uses_explicit_half_up_rule(value, expected):
    assert nearest_integer_half_up(value) == expected
    assert ADC_ROUNDING_RULE == "nearest integer; exact halves round upward"


@pytest.mark.parametrize("value", [-0.001, math.nan, math.inf, -math.inf, True])
def test_nearest_integer_rejects_values_that_cannot_be_safety_limits(value):
    with pytest.raises(ValueError, match="finite nonnegative number"):
        nearest_integer_half_up(value)


def test_safety_limit_uses_every_accepted_sample_and_half_up_rounding():
    result = calculate_safety_limit("SAFETY_EE", (118.0, 119.27272727272727), 1.1)

    assert result.sample_count == 2
    assert result.mean_ma == pytest.approx(118.63636363636364)
    assert result.unrounded_limit_ma == pytest.approx(130.5)
    assert result.rounded_limit_ma == 131


@pytest.mark.parametrize(
    "samples",
    [(), (None,), (True,), (math.nan,), (math.inf,), (-1.0,)],
)
def test_safety_limit_rejects_empty_or_invalid_scaled_samples(samples):
    with pytest.raises(ValueError):
        calculate_safety_limit("SAFETY_OPT", samples, 1.3)


@pytest.mark.parametrize(
    ("pulse_width", "ee_limit", "opt_limit", "expected"),
    [
        (499, 1, 2, (548.9, 549, "derived")),
        (500, 1, 2, (550.0, 550, "derived")),
        (599, 1, 2, (658.9, 659, "derived")),
        (600, 660, 660, (660.0, 660, "retained")),
    ],
)
def test_pulse_limit_rules_cover_below_and_exactly_600(
    pulse_width, ee_limit, opt_limit, expected
):
    result = calculate_pulse_limits(
        pulse_width,
        current_ee_limit_us=ee_limit,
        current_opt_limit_us=opt_limit,
    )

    assert result == PulseLimitCalculation(
        ta_pulse_width_us=pulse_width,
        multiplier=1.1,
        unrounded_limit_us=result.unrounded_limit_us,
        rounding_rule=ADC_ROUNDING_RULE,
        rounded_limit_us=expected[1],
        ee_pulse_width_ul_us=expected[1],
        opt_pulse_width_ul_us=expected[1],
        mode=expected[2],
    )
    assert result.unrounded_limit_us == pytest.approx(expected[0])


@pytest.mark.parametrize(
    ("pulse_width", "ee_limit", "opt_limit"),
    [
        (600, 659, 660),
        (600, 660, 661),
        (601, 660, 660),
        (500.5, 550, 550),
        (0, 0, 0),
        (True, 550, 550),
    ],
)
def test_pulse_limit_rejects_invalid_or_unapproved_operating_points(
    pulse_width, ee_limit, opt_limit
):
    with pytest.raises(ValueError):
        calculate_pulse_limits(
            pulse_width,
            current_ee_limit_us=ee_limit,
            current_opt_limit_us=opt_limit,
        )


def test_current_configuration_accepts_required_keys_and_preserves_extras():
    config = _valid_config(FACTORY_NOTE="preserve me")

    assert all(item.passed for item in validate_current_configuration(config))


@pytest.mark.parametrize(
    "missing_key",
    [
        "TA_CURRENT_DRV",
        "TA_PULSE_WIDTH",
        "SEED_CW_GAIN",
        "EE_PULSE_WIDTH_UL",
        "OPT_PULSE_WIDTH_UL",
        "EE_DRIVE_CL",
        "OPT_DRIVE_CL",
        "EE_RATE_LL",
        "OPT_RATE_LL",
        "TEC_TRIP",
    ],
)
def test_current_configuration_requires_each_work_instruction_key(missing_key):
    config = _valid_config()
    del config[missing_key]

    assert not _criteria(config)[f"config_key_{missing_key}"].passed


@pytest.mark.parametrize("value", [None, True, 0, -1, math.nan, math.inf])
def test_current_configuration_requires_positive_finite_ta_current(value):
    assert not _criteria(_valid_config(TA_CURRENT_DRV=value))["ta_current"].passed


@pytest.mark.parametrize("value", [None, True, 0, -1, 500.5, math.nan, math.inf, 601])
def test_current_configuration_requires_positive_integer_pulse_at_most_600(value):
    assert not _criteria(_valid_config(TA_PULSE_WIDTH=value))["ta_pulse_width"].passed


@pytest.mark.parametrize(
    ("ee_limit", "opt_limit", "passed"),
    [(660, 660, True), (659, 660, False), (660, 659, False)],
)
def test_600_us_configuration_requires_both_660_us_limits(
    ee_limit, opt_limit, passed
):
    criteria = _criteria(
        _valid_config(
            TA_PULSE_WIDTH=600,
            EE_PULSE_WIDTH_UL=ee_limit,
            OPT_PULSE_WIDTH_UL=opt_limit,
        )
    )
    assert criteria["pulse_limits_at_600"].passed is passed


@pytest.mark.parametrize(
    ("declared", "topology", "left_serial", "right_serial", "passed"),
    [
        ("single-left", TopologySnapshot(True, True, False), "L-1", None, True),
        ("single-left", TopologySnapshot(True, True, True), "L-1", "R-1", False),
        ("single-left", TopologySnapshot(True, False, True), None, "R-1", False),
        ("single-right", TopologySnapshot(True, False, True), None, "R-1", True),
        ("single-right", TopologySnapshot(True, True, True), "L-1", "R-1", False),
        ("dual", TopologySnapshot(True, True, True), "L-1", "R-1", True),
        ("dual", TopologySnapshot(True, True, False), "L-1", None, False),
        ("dual", TopologySnapshot(False, True, True), "L-1", "R-1", False),
        # validate_shipping_topology is exact-match; the console-only operator
        # flow tolerates attached sensors only because the workflow never
        # invokes this check for console-only (the scan stage is skipped).
        ("console-only", TopologySnapshot(True, True, True), "L-1", "R-1", False),
        ("console-only", TopologySnapshot(True, False, False), None, None, True),
    ],
)
def test_shipping_topology_requires_exact_declared_sides_and_serials(
    declared, topology, left_serial, right_serial, passed
):
    result = validate_shipping_topology(
        ShippingTopology(declared),
        topology,
        left_identity=DeviceIdentity("left sensor", left_serial, None, None),
        right_identity=DeviceIdentity("right sensor", right_serial, None, None),
    )

    assert result.passed is passed


@pytest.mark.parametrize("serial", [None, "", "  "])
def test_shipping_topology_rejects_blank_required_sensor_serial(serial):
    result = validate_shipping_topology(
        ShippingTopology.SINGLE_LEFT,
        TopologySnapshot(True, True, False),
        left_identity=DeviceIdentity("left sensor", serial, None, None),
        right_identity=DeviceIdentity("right sensor", None, None, None),
    )

    assert not result.passed
    assert "serial" in result.detail.lower()


