"""Tests for omotion/exposure/settings.py: settings validation, limits, gain ladder.

Pure-software. A bad bound must be an error, never a clamp: these values
end up as laser register writes.
"""

import dataclasses

import pytest

from omotion.exposure.settings import (
    ABSOLUTE_MAX_CURRENT_MA,
    ABSOLUTE_MAX_PULSE_WIDTH_US,
    Bounds,
    GainLadder,
    HardwareLimits,
    Knob,
    OptimizeLaserSettings,
    gain_row,
)

pytestmark = pytest.mark.unit

NARROW_LIMITS = HardwareLimits(pulse_width_max_us=600.0)


def _errors(**changes):
    return OptimizeLaserSettings(**changes).errors()


def _has(errors, fragment):
    return any(fragment in e for e in errors), errors


def test_defaults_are_valid():
    assert OptimizeLaserSettings().errors() == []
    OptimizeLaserSettings().validate()


def test_effective_pulse_width_max_fits_inside_exposure():
    assert HardwareLimits().effective_pulse_width_max_us == pytest.approx(2096.0)
    assert NARROW_LIMITS.effective_pulse_width_max_us == 600.0


def test_pulse_width_hard_cap_holds_whatever_the_limits_say():
    roomy = HardwareLimits(pulse_width_max_us=5000.0, exposure_us=10_000.0)
    assert roomy.effective_pulse_width_max_us == ABSOLUTE_MAX_PULSE_WIDTH_US
    errs = OptimizeLaserSettings(pulse_width_bounds=Bounds(100, 2300)).errors(roomy)
    assert _has(errs, "above the hardware maximum 2200")[0], errs


def test_current_hard_cap_holds_whatever_the_limits_say():
    roomy = HardwareLimits(current_max_ma=6000.0)
    errs = OptimizeLaserSettings(current_bounds=Bounds(2500, 5500)).errors(roomy)
    assert _has(errs, f"above the hardware maximum {ABSOLUTE_MAX_CURRENT_MA:g}")[0], errs


def test_pulse_width_past_the_exposure_window_is_rejected():
    errs = _errors(pulse_width_bounds=Bounds(100, 2200))
    assert _has(errs, "pulse width upper bound 2200 us is above the hardware maximum 2096")[0]


def test_narrower_limits_reject_long_pulses():
    errs = OptimizeLaserSettings().errors(NARROW_LIMITS)
    assert _has(errs, "above the hardware maximum 600")[0], errs
    assert OptimizeLaserSettings(
        pulse_width_bounds=Bounds(100, 600)).errors(NARROW_LIMITS) == []


@pytest.mark.parametrize("field, bounds, fragment", [
    ("pulse_width_bounds", Bounds(900, 500), "pulse width lower bound 900 us is above"),
    ("pulse_width_bounds", Bounds(50, 500), "below the hardware minimum 100"),
    ("current_bounds", Bounds(4800, 3000), "laser drive current lower bound 4800 mA is above"),
    ("current_bounds", Bounds(2000, 5000), "below the hardware minimum 2500"),
    ("pulse_width_bounds", Bounds(float("nan"), 500), "two finite numbers"),
])
def test_bad_laser_bounds_are_rejected(field, bounds, fragment):
    errs = _errors(**{field: bounds})
    assert _has(errs, fragment)[0], errs


def test_equal_bounds_are_valid_and_fixed():
    s = OptimizeLaserSettings(pulse_width_bounds=Bounds(500, 500))
    assert s.errors() == []
    assert s.pulse_width_bounds.fixed


def test_gain_bounds_need_four_monotonic_ladder_rows():
    rows = list(OptimizeLaserSettings().gain_bounds)
    assert _has(_errors(gain_bounds=tuple(rows[:3])), "need 4 rows")[0]
    bad = rows.copy()
    bad[2] = Bounds(8, 4)
    assert _has(_errors(gain_bounds=tuple(bad)), "row 3 gain lower bound 8x is above")[0]
    bad[2] = Bounds(1, 17)
    assert _has(_errors(gain_bounds=tuple(bad)), "above the hardware maximum 16x")[0]
    bad[2] = Bounds(1.03, 4)
    assert _has(_errors(gain_bounds=tuple(bad)), "not a multiple of 1/16")[0]
    bad[2] = Bounds(1.5, 4)
    assert _errors(gain_bounds=tuple(bad)) == []


@pytest.mark.parametrize("order", [
    ("gain", "gain", "current"),
    ("gain", "pulse_width"),
    ("gain", "pulse_width", "voltage"),
])
def test_order_must_be_a_permutation(order):
    assert _has(_errors(order=order), "optimization order")[0]


def test_every_order_permutation_is_valid_and_strings_work():
    import itertools
    for perm in itertools.permutations(["gain", "pulse_width", "current"]):
        assert _errors(order=perm) == []
    assert _errors(order=(Knob.CURRENT, Knob.GAIN, Knob.PULSE_WIDTH)) == []


@pytest.mark.parametrize("changes, fragment", [
    ({"passing_range": 0}, "must be positive"),
    ({"target_mean": 10, "passing_range": 20}, "must be above 0"),
    ({"target_mean": 1010, "passing_range": 20}, "below full scale"),
    ({"target_mean": float("inf")}, "finite"),
    ({"frames_per_iteration": 0}, "frames per iteration"),
    ({"in_range_points": True}, "in-range points"),
    ({"in_range_points": 2.5}, "in-range points"),
    ({"method": "fancy"}, "method 'fancy'"),
    ({"start_point": "middle"}, "start point"),
    ({"gain_mode": "pairs"}, "gain mode"),
    ({"gain_priority": "left"}, "gain priority"),
])
def test_other_fields_are_validated(changes, fragment):
    assert _has(_errors(**changes), fragment)[0]


def test_validate_raises_with_every_problem():
    s = OptimizeLaserSettings(passing_range=-1, current_bounds=Bounds(4000, 3000))
    with pytest.raises(ValueError) as exc:
        s.validate()
    assert "passing range" in str(exc.value) and "drive current" in str(exc.value)


def test_settings_are_immutable():
    with pytest.raises(dataclasses.FrozenInstanceError):
        OptimizeLaserSettings().target_mean = 100


def test_gain_rows_pair_mirrored_cameras():
    assert [gain_row(p) for p in range(8)] == [0, 1, 2, 3, 3, 2, 1, 0]
    with pytest.raises(ValueError):
        gain_row(8)


def test_nominal_gain_ladder_is_sixteenth_steps():
    ladder = GainLadder.nominal()
    assert ladder.codes[0] == 16 and ladder.codes[-1] == 256
    assert ladder.multipliers[1] == pytest.approx(17 / 16)
    assert ladder.nearest(3.3, Bounds(1, 16)) == (53, 53 / 16)
    assert ladder.nearest(40.0, Bounds(1, 4)) == (64, 4.0)
    assert ladder.nearest(0.2, Bounds(2, 16)) == (32, 2.0)


def test_gain_ladder_uses_measured_multipliers_for_matching():
    # A measured table where code 48 really gives 3.3x: asking for 3.3x
    # picks code 48 even though its nominal gain is 3.0x.
    codes = tuple(range(16, 257))
    mults = tuple(3.3 if c == 48 else (c / 16 if c < 48 else c / 16 + 0.31) for c in codes)
    ladder = GainLadder(codes, mults)
    assert ladder.nearest(3.3, Bounds(1, 16))[0] == 48
    with pytest.raises(ValueError):
        GainLadder((16, 17), (1.0, 0.9))
