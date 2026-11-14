"""Tests for omotion/exposure/controller.py: the Optimize Laser controller.

Pure-software simulation, ported from the reference test (bloodflow-app
docs/auto_exposure/test_exposure_control.py) and extended to the device:
16 cameras on a shared laser, per-camera brightness spread, gain priority,
row gains, every knob order, both start points.

Synthetic frames are gamma-distributed speckle, clipped at 1023, from a
simulated laser whose threshold and knee vary unit to unit. Settings are
assumed to take effect on the next capture.
"""

import itertools
import math

import numpy as np
import pytest

from omotion.exposure import (
    Bounds,
    CameraObservation,
    CameraStatus,
    ExposureController,
    ExposureSettings,
    HardwareLimits,
    OptimizeLaserSettings,
    OptimizeStatus,
)
from omotion.exposure.estimate import SAT_LEVEL

pytestmark = pytest.mark.unit

LIMITS = HardwareLimits()
PW_MAX = LIMITS.effective_pulse_width_max_us
TRUNC = LIMITS.pulse_truncation_us
TARGET, BAND = 200.0, 20.0

ALL_CAMS = [(s, p) for s in (0, 1) for p in range(8)]


def laser_power(current, threshold, knee):
    """Relative laser output vs drive current: dark, linear, then flat with slight rollover."""
    if current < threshold:
        return 0.0
    if current < knee:
        return (current - threshold) / (knee - threshold)
    return 1.0 - 0.02 * (current - knee) / 500


def speckle_frame(rng, mean, shape, dark, pixels):
    x = rng.gamma(shape, mean / shape, pixels) if mean > 0 else np.zeros(pixels)
    return np.bincount(np.clip(x + dark, 0, SAT_LEVEL).astype(int), minlength=SAT_LEVEL + 1)


def run(ctrl, brightness, *, shape=4.0, threshold=2700.0, knee=4500.0, dark=0.0,
        pixels=20_000, seed=0, pulsatility=0.0):
    """Drive ``ctrl`` to completion against a simulated subject.

    ``brightness[cam]`` is the unclipped mean (DN, dark-subtracted) the
    camera would see at 16x / PW_MAX / 5000 mA. Returns the controller.
    """
    rng = np.random.default_rng(seed)
    p_full = laser_power(5000.0, threshold, knee)
    frame_no = 0
    while not ctrl.done:
        s = ctrl.settings
        light = ((s.pulse_width_us - TRUNC) / (PW_MAX - TRUNC)
                 * laser_power(s.current_ma, threshold, knee) / p_full)
        obs = {}
        for cam in ctrl.cameras:
            frames = []
            for k in range(ctrl.frames_needed):
                beat = 1.0 + pulsatility * math.sin(2 * math.pi * (frame_no + k) / 16)
                mean = brightness[cam] * s.gains[cam] / 16.0 * light * beat
                frames.append(speckle_frame(rng, mean, shape, dark, pixels))
            obs[cam] = CameraObservation(frames, dark)
        frame_no += ctrl.frames_needed
        ctrl.update(obs)
    return ctrl


def final(ctrl):
    """The last record: frames captured at the settings the run kept."""
    return ctrl.records[-1]


def assert_on_target(ctrl, cams):
    rec = final(ctrl)
    for cam in cams:
        sig = rec.cameras[cam].signal
        assert sig is not None and abs(sig - TARGET) <= BAND, (cam, sig, ctrl.status)


# ---- the reference cases, one camera ---------------------------------------

# (brightness at max settings, speckle shape, laser threshold mA, laser knee mA)
REFERENCE_CASES = [
    (150, 3, 2700, 4500),        # dimmer than target: stays at max
    (800, 1.5, 2700, 4500),      # gain only, high speckle contrast
    (800, 12, 2700, 4500),       # gain only, low speckle contrast (high flow)
    (3000, 3, 2700, 4500),       # down to 1x gain
    (50000, 4, 2700, 4500),      # pulse width
    (300000, 6, 2700, 4500),     # laser current, typical unit
    (300000, 6, 3200, 4600),     # laser current, high-threshold unit
    (300000, 6, 2300, 4400),     # laser current, low-threshold unit
    (2e6, 6, 2900, 4550),        # extreme brightness
]


@pytest.mark.parametrize("method", ["fit", "basic"])
@pytest.mark.parametrize("case", REFERENCE_CASES, ids=lambda c: f"b{c[0]:g}-t{c[2]}")
def test_reference_cases_single_camera(method, case):
    brightness, shape, threshold, knee = case
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(method=method), [cam])
    run(ctrl, {cam: brightness}, shape=shape, threshold=threshold, knee=knee, pixels=100_000)

    assert ctrl.iteration <= 12, (method, case, ctrl.iteration)
    if brightness < TARGET:
        assert ctrl.status is OptimizeStatus.TOO_DIM_AT_MAX
        s = ctrl.settings
        assert (s.gains[cam], s.pulse_width_us, s.current_ma) == (16.0, PW_MAX, 5000.0)
        assert ctrl.iteration == 1
    else:
        assert ctrl.status is OptimizeStatus.ON_TARGET, (method, case, ctrl.status)
        assert_on_target(ctrl, [cam])


def test_fit_is_never_slower_than_basic_on_saturated_starts():
    for brightness in (3000, 50000, 300000):
        counts = {}
        for method in ("fit", "basic"):
            ctrl = ExposureController(OptimizeLaserSettings(method=method), [(0, 0)])
            run(ctrl, {(0, 0): brightness}, shape=4, pixels=50_000)
            counts[method] = ctrl.iteration
        assert counts["fit"] <= counts["basic"], (brightness, counts)


# ---- multiple cameras on one laser -----------------------------------------

# Outer cameras (1 & 8) see much less light than the middle ones, as on the
# device; the right module is 30% brighter than the left.
POSITION_BRIGHTNESS = [400, 3000, 8000, 20000, 20000, 8000, 3000, 400]
SPREAD = {(s, p): POSITION_BRIGHTNESS[p] * (1.0 if s == 0 else 1.3) for s, p in ALL_CAMS}


def test_far_priority_keeps_cameras_1_and_8_on_target():
    ctrl = ExposureController(OptimizeLaserSettings(gain_priority="far"), ALL_CAMS)
    run(ctrl, SPREAD)
    far = [c for c in ALL_CAMS if c[1] in (0, 7)]
    near = [c for c in ALL_CAMS if c[1] in (3, 4)]
    assert ctrl.status is OptimizeStatus.PARTIAL
    assert_on_target(ctrl, far)
    # The laser dims only as far as the far cameras allow (the dimmer,
    # left pair ends at full gain), which brings the too-bright middle
    # cameras as close as they can get.
    assert max(ctrl.settings.gains[c] for c in far) == 16.0
    assert ctrl.settings.pulse_width_us < PW_MAX
    status = ctrl.camera_status
    assert all(status[c] is CameraStatus.TOO_BRIGHT for c in near)
    assert all(ctrl.settings.gains[c] == 1.0 for c in near)


def test_priority_positions():
    """Near = cameras 4 & 5 (nearest the laser), far = 1 & 8, none = every
    camera in the run (the Scan Settings masks)."""
    some = [(0, 0), (0, 3), (0, 4), (1, 7), (1, 5)]
    pick = {p: ExposureController(OptimizeLaserSettings(gain_priority=p), some).priority_cameras
            for p in ("near", "far", "none")}
    assert pick["near"] == ((0, 3), (0, 4))
    assert pick["far"] == ((0, 0), (1, 7))
    assert pick["none"] == tuple(sorted(some))


def test_priority_is_ignored_when_every_camera_can_reach_target():
    brightness = {(s, p): [1500, 3000, 6000, 12000, 11000, 5000, 2500, 1800][p] * (1 + 0.1 * s)
                  for s, p in ALL_CAMS}
    results = {}
    for priority in ("near", "far", "none"):
        ctrl = ExposureController(OptimizeLaserSettings(gain_priority=priority), ALL_CAMS)
        run(ctrl, brightness)
        results[priority] = ctrl.settings.pulse_width_us
    assert len(set(results.values())) == 1, results


def test_near_priority_dims_the_laser_for_cameras_4_and_5():
    ctrl = ExposureController(OptimizeLaserSettings(gain_priority="near"), ALL_CAMS)
    run(ctrl, SPREAD)
    far = [c for c in ALL_CAMS if c[1] in (0, 7)]
    near = [c for c in ALL_CAMS if c[1] in (3, 4)]
    assert ctrl.status is OptimizeStatus.PARTIAL
    assert_on_target(ctrl, near)
    assert ctrl.settings.pulse_width_us < PW_MAX
    assert all(ctrl.camera_status[c] is CameraStatus.TOO_DIM for c in far)
    assert all(ctrl.settings.gains[c] == 16.0 for c in far)


def test_every_camera_on_target_when_the_spread_fits_in_gain():
    brightness = {(s, p): [1500, 3000, 6000, 12000, 11000, 5000, 2500, 1800][p] * (1 + 0.1 * s)
                  for s, p in ALL_CAMS}
    for priority in ("near", "far", "none"):
        ctrl = ExposureController(OptimizeLaserSettings(gain_priority=priority), ALL_CAMS)
        run(ctrl, brightness)
        assert ctrl.status is OptimizeStatus.ON_TARGET, (priority, ctrl.status)
        assert_on_target(ctrl, ALL_CAMS)
        assert ctrl.iteration <= 6, (priority, ctrl.iteration)


def test_no_priority_balances_a_spread_that_cannot_fit():
    ctrl = ExposureController(OptimizeLaserSettings(gain_priority="none"), ALL_CAMS)
    run(ctrl, SPREAD)
    assert ctrl.status is OptimizeStatus.SPREAD_TOO_WIDE
    status = ctrl.camera_status
    assert any(v is CameraStatus.TOO_DIM for v in status.values())
    assert any(v is CameraStatus.TOO_BRIGHT for v in status.values())
    assert any(v is CameraStatus.ON_TARGET for v in status.values())
    # Balanced: the dimmest camera is as far below target as the brightest is above.
    signals = [r.signal for r in final(ctrl).cameras.values()]
    below, above = TARGET / min(signals), max(signals) / TARGET
    assert below == pytest.approx(above, rel=0.15)


def test_row_mode_shares_one_gain_per_mirrored_pair_across_modules():
    brightness = {(s, p): [2000, 4000, 7000, 9000, 9500, 7300, 4200, 2100][p] * (1 + 0.04 * s)
                  for s, p in ALL_CAMS}
    ctrl = ExposureController(OptimizeLaserSettings(gain_mode="row", gain_priority="none"),
                              ALL_CAMS)
    run(ctrl, brightness)
    for rec in ctrl.records:
        codes = rec.next_settings.gain_codes
        for row in range(4):
            members = {codes[(s, p)] for s in (0, 1) for p in (row, 7 - row)}
            assert len(members) == 1, (rec.iteration, row, members)
    assert ctrl.status is OptimizeStatus.ON_TARGET
    assert_on_target(ctrl, ALL_CAMS)


def test_row_gain_bounds_apply_per_row():
    bounds = (Bounds(8, 16), Bounds(1, 16), Bounds(1, 16), Bounds(1, 2))
    ctrl = ExposureController(OptimizeLaserSettings(gain_bounds=bounds, gain_priority="none"),
                              ALL_CAMS)
    run(ctrl, {c: 3000.0 for c in ALL_CAMS})
    for rec in ctrl.records:
        for (s, p), g in rec.next_settings.gains.items():
            row = min(p, 7 - p)
            assert bounds[row].low <= g <= bounds[row].high


# ---- order, start point, fixed knobs ---------------------------------------

ORDERS = list(itertools.permutations(["gain", "pulse_width", "current"]))


@pytest.mark.parametrize("order", ORDERS, ids=lambda o: ">".join(k[0] for k in o))
@pytest.mark.parametrize("start", ["max", "current"])
def test_every_order_and_start_point_reaches_target(order, start):
    cam = (0, 0)
    initial = ExposureSettings({cam: 64}, {cam: 4.0}, 900.0, 4700.0)
    ctrl = ExposureController(OptimizeLaserSettings(order=order, start_point=start), [cam],
                              initial=initial)
    run(ctrl, {cam: 3000.0}, threshold=2600.0)
    assert ctrl.status is OptimizeStatus.ON_TARGET, (order, start, ctrl.status, ctrl.iteration)
    assert_on_target(ctrl, [cam])
    assert ctrl.iteration <= 10


def test_order_decides_which_knob_dims_first():
    cam = (0, 0)
    gain_first = ExposureController(OptimizeLaserSettings(), [cam])
    run(gain_first, {cam: 1600.0})
    assert gain_first.settings.pulse_width_us == PW_MAX
    assert gain_first.settings.gains[cam] < 16.0

    pw_first = ExposureController(
        OptimizeLaserSettings(order=("pulse_width", "gain", "current")), [cam])
    run(pw_first, {cam: 1600.0})
    assert pw_first.settings.gains[cam] == 16.0
    assert pw_first.settings.pulse_width_us < PW_MAX


def test_start_from_current_settings_begins_there():
    cam = (1, 3)
    initial = ExposureSettings({cam: 32}, {cam: 2.0}, 800.0, 4600.0)
    ctrl = ExposureController(OptimizeLaserSettings(start_point="current"), [cam],
                              initial=initial)
    first = ctrl.settings
    assert (first.gains[cam], first.pulse_width_us, first.current_ma) == (2.0, 800.0, 4600.0)
    run(ctrl, {cam: 9000.0})
    assert ctrl.status is OptimizeStatus.ON_TARGET
    assert_on_target(ctrl, [cam])


def test_start_point_current_needs_the_device_settings():
    with pytest.raises(ValueError, match="initial"):
        ExposureController(OptimizeLaserSettings(start_point="current"), [(0, 0)])


def test_fixed_knobs_never_move():
    cam = (0, 0)
    settings = OptimizeLaserSettings(pulse_width_bounds=Bounds(500, 500),
                                     current_bounds=Bounds(5000, 5000))
    ctrl = ExposureController(settings, [cam])
    run(ctrl, {cam: 300000.0})
    assert {r.next_settings.pulse_width_us for r in ctrl.records} == {500.0}
    assert {r.next_settings.current_ma for r in ctrl.records} == {5000.0}
    assert ctrl.status is OptimizeStatus.TOO_BRIGHT_AT_MIN
    assert ctrl.settings.gains[cam] == 1.0


def test_pulse_width_stays_inside_narrower_limits():
    limits = HardwareLimits(pulse_width_max_us=600.0)
    settings = OptimizeLaserSettings(pulse_width_bounds=Bounds(100, 600))
    cam = (0, 0)
    ctrl = ExposureController(settings, [cam], limits=limits)
    assert ctrl.settings.pulse_width_us == 600.0
    run(ctrl, {cam: 300000.0})
    assert max(r.next_settings.pulse_width_us for r in ctrl.records) <= 600.0
    assert ctrl.status is OptimizeStatus.ON_TARGET


def test_invalid_settings_are_refused_before_anything_runs():
    with pytest.raises(ValueError, match="pulse width"):
        ExposureController(OptimizeLaserSettings(pulse_width_bounds=Bounds(100, 2200)),
                           [(0, 0)])
    with pytest.raises(ValueError, match="no cameras"):
        ExposureController(OptimizeLaserSettings(), [])
    with pytest.raises(ValueError, match="side"):
        ExposureController(OptimizeLaserSettings(), [(2, 0)])


# ---- saturation, darks, exit condition -------------------------------------

def test_too_clipped_frame_cuts_brightness_16x():
    cam = (0, 0)
    initial = ExposureSettings({cam: 256}, {cam: 16.0}, 1000.0, 5000.0)
    ctrl = ExposureController(OptimizeLaserSettings(start_point="current"), [cam],
                              initial=initial)
    hist = np.zeros(SAT_LEVEL + 1)
    hist[SAT_LEVEL], hist[700] = 900, 100
    rec = ctrl.update({cam: CameraObservation([hist])})
    assert rec.cameras[cam].kind == "clipped"
    assert rec.next_settings.gains[cam] == 1.0      # 16x -> 1x is the whole 16x cut
    assert rec.next_settings.pulse_width_us == 1000.0


def test_dark_level_is_subtracted():
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(), [cam])
    run(ctrl, {cam: 5000.0}, dark=40.0)
    rec = final(ctrl).cameras[cam]
    assert ctrl.status is OptimizeStatus.ON_TARGET
    assert abs(rec.signal - TARGET) <= BAND
    assert rec.mean_raw == pytest.approx(rec.signal + 40.0)


def _flat(dn, pixels=10_000):
    h = np.zeros(SAT_LEVEL + 1)
    h[int(dn)] = pixels
    return h


def test_exit_needs_consecutive_in_range_points():
    cam = (0, 3)
    settings = OptimizeLaserSettings(frames_per_iteration=2, in_range_points=3,
                                     start_point="current")
    initial = ExposureSettings({cam: 64}, {cam: 4.0}, 1000.0, 5000.0)
    ctrl = ExposureController(settings, [cam], initial=initial)
    assert ctrl.frames_needed == 6

    # Points: (200, 200) (260, 260) (200, 200) -> the middle one is out of range.
    rec = ctrl.update({cam: CameraObservation([_flat(v) for v in (200, 200, 260, 260, 200, 200)])})
    cr = rec.cameras[cam]
    assert (cr.points, cr.points_in_range, cr.on_target) == (3, 2, False)
    assert not ctrl.done

    rec = ctrl.update({cam: CameraObservation([_flat(v) for v in (195, 205, 210, 190, 200, 200)])})
    assert rec.cameras[cam].on_target
    assert ctrl.done and ctrl.status is OptimizeStatus.ON_TARGET


def test_pulsatile_signal_inside_the_band_still_converges():
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(in_range_points=4), [cam])
    run(ctrl, {cam: 5000.0}, pulsatility=0.04)
    assert ctrl.status is OptimizeStatus.ON_TARGET


def test_pulsatility_wider_than_the_band_does_not_fake_an_exit():
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(in_range_points=4), [cam], max_iterations=8)
    run(ctrl, {cam: 5000.0}, pulsatility=0.25)
    assert ctrl.status is OptimizeStatus.NOT_CONVERGED
    assert not ctrl.status.passed


def test_resume_after_not_converged_keeps_iterating():
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(in_range_points=4), [cam], max_iterations=8)
    run(ctrl, {cam: 5000.0}, pulsatility=0.25)
    assert ctrl.status is OptimizeStatus.NOT_CONVERGED and ctrl.iteration == 8
    # The device holds what the last frame used; resume moves on from the
    # settings that iteration had chosen.
    held = ctrl.settings
    assert held == ctrl.records[-1].settings
    ctrl.resume(3)
    assert not ctrl.done and ctrl.status is None and ctrl.max_iterations == 11
    rec = ctrl.update({cam: CameraObservation([_flat(TARGET)] * ctrl.frames_needed)})
    assert rec.iteration == 9 and ctrl.done and ctrl.status is OptimizeStatus.ON_TARGET


def test_resume_only_after_not_converged():
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(), [cam])
    with pytest.raises(RuntimeError):
        ctrl.resume(5)
    ctrl = ExposureController(OptimizeLaserSettings(in_range_points=4), [cam], max_iterations=2)
    run(ctrl, {cam: 5000.0}, pulsatility=0.25)
    with pytest.raises(ValueError):
        ctrl.resume(0)


def test_missing_camera_reports_no_data_without_blocking():
    # Camera 4 (near, the default priority) delivers; camera 1 doesn't.
    cams = [(0, 0), (0, 3)]
    ctrl = ExposureController(OptimizeLaserSettings(), cams)
    rng = np.random.default_rng(3)
    while not ctrl.done:
        s = ctrl.settings
        mean = 6000.0 * s.gains[(0, 3)] / 16 * (s.pulse_width_us - TRUNC) / (PW_MAX - TRUNC)
        ctrl.update({(0, 3): CameraObservation([speckle_frame(rng, mean, 4, 0, 20_000)])})
    assert ctrl.status is OptimizeStatus.PARTIAL
    assert ctrl.camera_status[(0, 0)] is CameraStatus.NO_DATA


def test_a_missing_priority_camera_does_not_converge():
    cams = [(0, 0), (0, 3)]
    ctrl = ExposureController(OptimizeLaserSettings(gain_priority="far"), cams)
    rng = np.random.default_rng(3)
    while not ctrl.done:
        s = ctrl.settings
        mean = 6000.0 * s.gains[(0, 3)] / 16 * (s.pulse_width_us - TRUNC) / (PW_MAX - TRUNC)
        ctrl.update({(0, 3): CameraObservation([speckle_frame(rng, mean, 4, 0, 20_000)])})
    assert ctrl.status is OptimizeStatus.NOT_CONVERGED


def test_update_after_done_raises():
    cam = (0, 0)
    ctrl = ExposureController(OptimizeLaserSettings(), [cam])
    run(ctrl, {cam: 100.0})
    with pytest.raises(RuntimeError):
        ctrl.update({})


# ---- laser current search --------------------------------------------------

def test_learned_threshold_speeds_up_the_next_run():
    cam = (0, 0)
    true_threshold = 3200.0
    first = ExposureController(OptimizeLaserSettings(), [cam])
    run(first, {cam: 300000.0}, shape=6, threshold=true_threshold, knee=4600.0)
    assert first.status is OptimizeStatus.ON_TARGET
    learned = first.learned_threshold()
    assert learned is not None and abs(learned - true_threshold) < 150, learned

    second = ExposureController(OptimizeLaserSettings(), [cam], threshold_guess=learned)
    run(second, {cam: 300000.0}, shape=6, threshold=true_threshold, knee=4600.0, seed=1)
    assert second.status is OptimizeStatus.ON_TARGET
    assert second.iteration <= first.iteration


def test_current_never_leaves_its_bounds():
    cam = (0, 0)
    settings = OptimizeLaserSettings(current_bounds=Bounds(3000, 4800))
    ctrl = ExposureController(settings, [cam])
    run(ctrl, {cam: 2e6}, shape=6, threshold=2900.0)
    currents = [r.next_settings.current_ma for r in ctrl.records]
    assert min(currents) >= 3000.0 and max(currents) <= 4800.0
    assert ctrl.status.passed
