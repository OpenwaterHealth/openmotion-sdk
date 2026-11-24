"""Seedless C_min procedure logic, against a simulated phantom. Pure software.

The fake bench models the physics the procedure relies on: seeded frames at
a fixed mean and contrast; seedless frames whose mean grows linearly with
the TA pulse width (bench 2026-07-17: ~0.975x nominal at 2 ms) at a much
lower contrast.
"""

import json
import math

import pytest

from omotion.calibration.seedless_cmin import (
    PHASE_CMIN,
    PHASE_NOMINAL,
    PHASE_PLATEAU,
    PHASE_SEARCH,
    BASIS_MATCHED,
    BASIS_PLATEAU,
    VERDICT_FAIL,
    VERDICT_PASS,
    BurstResult,
    CameraStats,
    CminSettings,
    FrameRecord,
    ProcedureStop,
    SeedlessCminProcedure,
    camera_stats,
    contrast_spread,
    next_ta_us,
    plateau_widths,
    width_for_ratio,
)
from omotion.calibration.seedless_cmin_report import write_cmin_report
from omotion.seedless import MAX_TA_PULSE_WIDTH_US

CAMS = {("right", c) for c in range(8)}


class FakeBench:
    def __init__(self, *, nominal_mean=200.0, nominal_k=0.40,
                 seedless_per_us=0.975 / 2000, seedless_k=0.09,
                 seed_stays_on=False, dead_cam=None, plateau_us=None,
                 enter_error=None, restore_problem=None, drift=1.0,
                 k_slope=0.0):
        self.nominal_mean = nominal_mean
        self.nominal_k = nominal_k
        self.per_us = seedless_per_us
        self.seedless_k = seedless_k
        self.seed_stays_on = seed_stays_on
        self.dead_cam = dead_cam
        self.plateau_us = plateau_us
        self.enter_error = enter_error
        self.restore_problem = restore_problem
        self.drift = drift
        # Seedless contrast rises as the image dims: k * (1 + k_slope * (1 - ratio)).
        self.k_slope = k_slope
        self.ta = None
        self.seedless = False
        self.log = []
        self.captures = 0

    def _frames(self, n, mean, k):
        out = []
        for cam in range(8):
            if cam == self.dead_cam:
                continue
            for i in range(n):
                m = mean * (1 + 0.001 * ((i % 3) - 1))
                out.append(FrameRecord("right", cam, 11 + i, m, k * m, k))
        return out

    def capture(self, light_frames, *, seedless):
        assert seedless == self.seedless
        self.captures += 1
        self.log.append(("capture", seedless, self.ta, light_frames))
        if not seedless or self.seed_stays_on:
            return self._frames(light_frames, self.nominal_mean, self.nominal_k)
        width = self.ta if self.plateau_us is None else min(self.ta, self.plateau_us)
        mean = self.nominal_mean * self.per_us * width
        if light_frames == 40:
            mean *= self.drift
        k = self.seedless_k * (1 + self.k_slope * (1 - mean / self.nominal_mean))
        return self._frames(light_frames, mean, k)

    def enter_seedless(self, us):
        self.log.append(("enter", us))
        if self.enter_error:
            raise RuntimeError(self.enter_error)
        self.seedless, self.ta = True, us

    def set_ta_pulse_width_us(self, us):
        assert self.seedless and us <= MAX_TA_PULSE_WIDTH_US
        self.log.append(("set", us))
        self.ta = us

    def exit_seedless(self):
        self.log.append(("exit",))
        self.seedless = False
        return self.restore_problem


def run(bench, **kw):
    lines = []
    result = SeedlessCminProcedure(bench, CminSettings(**kw), CAMS,
                                   output_func=lines.append).run()
    return result, lines


def test_happy_path_matches_and_reports_cmin():
    bench = FakeBench()
    result, _ = run(bench)
    assert result.verdict == VERDICT_PASS, result.problem
    assert result.basis == BASIS_MATCHED and result.plateau == []
    assert abs(result.cmin - 0.09) < 1e-6
    assert set(result.cmin_per_camera) == {f"R{c}" for c in range(1, 9)}
    # Matched width: 1 / (0.975/2000) = 2051 us, within tolerance.
    assert abs(result.cmin_capture.ratio - 1) <= 0.03
    assert 1950 <= result.matched_ta_us <= MAX_TA_PULSE_WIDTH_US
    # First burst at the normal 500 us; pulse never more than doubles.
    widths = [b.ta_pulse_width_us for b in result.bursts]
    assert abs(widths[0] - 500) < 0.5
    assert all(b <= 2 * a + 0.5 for a, b in zip(widths, widths[1:]))
    assert bench.log[-1] == ("exit",)
    phases = {f.phase for f in result.frames}
    assert phases == {PHASE_NOMINAL, PHASE_SEARCH, PHASE_CMIN}
    # Every search burst is 10 light frames per camera.
    assert all(c[3] == 10 for c in bench.log
               if c[0] == "capture" and c[1] and c[3] != 40)
    assert sum(1 for f in result.frames if f.phase == PHASE_CMIN) == 8 * 40


def test_seed_still_on_stops_after_first_burst():
    bench = FakeBench(seed_stays_on=True)
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "seed may not be off" in result.problem
    assert len(result.bursts) == 1
    assert bench.log[-1] == ("exit",)


def test_bad_nominal_readout_stops_before_seed_is_touched():
    bench = FakeBench(nominal_mean=10.0)
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "does not look like the phantom" in result.problem
    assert not any(c[0] == "enter" for c in bench.log)
    assert ("exit",) not in bench.log


def test_first_burst_contrast_not_lower_stops():
    result, _ = run(FakeBench(seedless_k=0.38))
    assert result.verdict == VERDICT_FAIL
    assert "unseeded" in result.problem


def test_ta_shutdown_like_dark_first_burst_stops():
    result, _ = run(FakeBench(seedless_per_us=1e-6))
    assert result.verdict == VERDICT_FAIL
    assert "shut down" in result.problem


def test_unreachable_target_fails_at_the_limit_without_plateau_check():
    result, _ = run(FakeBench(seedless_per_us=0.75 / 2150), plateau_check=False)
    assert result.verdict == VERDICT_FAIL
    assert "out of reach" in result.problem
    assert result.bursts[-1].ta_pulse_width_us <= MAX_TA_PULSE_WIDTH_US
    assert result.cmin is None


# The bench case (2026-11-24): 2150 us reaches ~0.75x nominal.
BENCH_REACH = 0.75 / 2150


def test_out_of_reach_flat_contrast_passes_on_the_plateau_check():
    bench = FakeBench(seedless_per_us=BENCH_REACH)
    result, lines = run(bench)
    assert result.verdict == VERDICT_PASS, result.problem
    assert result.basis == BASIS_PLATEAU and result.matched_ta_us is None
    assert result.cmin == pytest.approx(0.09)
    cap = result.cmin_capture
    assert cap.ta_pulse_width_us == result.bursts[-1].ta_pulse_width_us
    assert cap.ratio == pytest.approx(0.75, abs=0.01)
    # Two plateau bursts below the capture, starting near 0.5x brightness.
    assert len(result.plateau) == 2
    assert result.plateau[0].ratio == pytest.approx(0.5, abs=0.03)
    assert result.plateau[0].ta_pulse_width_us < result.plateau[1].ta_pulse_width_us \
        < cap.ta_pulse_width_us
    assert set(result.plateau_spread) == {f"R{c}" for c in range(1, 9)}
    assert all(v < 1e-9 for v in result.plateau_spread.values())
    assert {f.phase for f in result.frames} == {PHASE_NOMINAL, PHASE_SEARCH,
                                                 PHASE_PLATEAU, PHASE_CMIN}
    assert bench.log[-1] == ("exit",)
    assert any("out of reach" in line for line in lines)


def test_out_of_reach_contrast_that_follows_brightness_fails_per_camera():
    # 0.5x -> 0.75x brightness moves contrast by ~10 %: not flat.
    bench = FakeBench(seedless_per_us=BENCH_REACH, k_slope=0.4)
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "not flat" in result.problem and "R1" in result.problem
    assert result.cmin is not None            # measured and reported anyway
    assert all(v > 0.05 for v in result.plateau_spread.values())
    assert bench.log[-1] == ("exit",)


def test_plateau_check_tolerance_is_a_setting():
    bench = FakeBench(seedless_per_us=BENCH_REACH, k_slope=0.4)
    result, _ = run(bench, plateau_tolerance=0.2)
    assert result.verdict == VERDICT_PASS, result.problem


def test_too_dim_for_the_plateau_check_fails():
    result, _ = run(FakeBench(seedless_per_us=0.4 / 2150))
    assert result.verdict == VERDICT_FAIL
    assert "out of reach" in result.problem and "plateau check" in result.problem
    assert result.plateau == [] and result.cmin is None


def test_plateau_range_too_narrow_fails():
    # ~0.54x at the limit: the 0.5x width is within 10 % of the longest pulse.
    result, _ = run(FakeBench(seedless_per_us=0.54 / 2150))
    assert result.verdict == VERDICT_FAIL
    assert "too narrow" in result.problem


def test_plateau_helpers():
    hist = [_burst(1, 500.0, 0.2), _burst(2, 1000.0, 0.4), _burst(3, 2000.0, 0.8)]
    assert width_for_ratio(hist, 0.5, 500.0, 2000.0) == pytest.approx(1250.0)
    assert width_for_ratio(hist, 0.01, 500.0, 2000.0) == 500.0     # clamped
    w = plateau_widths(1250.0, 2149.76, 3)
    assert len(w) == 3 and w[-1] == 2149.76 and w[0] < w[1] < w[2]
    assert contrast_spread([0.10, 0.105, 0.095]) == pytest.approx(0.1)


def test_mean_not_responding_to_pulse_width_stops():
    result, _ = run(FakeBench(plateau_us=500))
    assert result.verdict == VERDICT_FAIL
    assert "did not raise the image mean" in result.problem


def test_missing_camera_fails():
    result, _ = run(FakeBench(dead_cam=3))
    assert result.verdict == VERDICT_FAIL
    assert "R4 (0 of 80)" in result.problem


def test_drift_during_cmin_capture_fails():
    result, _ = run(FakeBench(drift=0.85))
    assert result.verdict == VERDICT_FAIL
    assert "drifted" in result.problem


def test_enter_failure_still_restores():
    bench = FakeBench(enter_error="console refused EE_PULSE_WIDTH_UL")
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "refused" in result.problem
    assert bench.log[-1] == ("exit",)


def test_restore_problem_fails_a_good_run():
    result, _ = run(FakeBench(restore_problem="OPT_RATE_LL read back wrong"))
    assert result.verdict == VERDICT_FAIL
    assert result.restore_problem
    assert result.cmin is not None      # measured, but the run is not clean


def _burst(i, w, r):
    return BurstResult(i, w, r, 0.1, ())


def test_next_width_proportional_then_secant_and_step_limit():
    s = CminSettings()
    # 500 us at 0.25x: proportional says 2000, step limit says 1000.
    assert abs(next_ta_us([_burst(1, 500, 0.25)], s) - 1000) < 0.5
    # Secant through (500, .25) and (1000, .5): 2000.
    w = next_ta_us([_burst(1, 500, 0.25), _burst(2, 1000, 0.5)], s)
    assert abs(w - 2000) < 0.5
    # Overshoot comes back down.
    w = next_ta_us([_burst(1, 1000, 0.5), _burst(2, 2000, 1.2)], s)
    assert w < 2000


def test_next_width_never_exceeds_max():
    s = CminSettings(max_ta_us=1800)
    w = next_ta_us([_burst(1, 1000, 0.5), _burst(2, 1500, 0.7)], s)
    assert w <= 1800


def test_settings_reject_widths_beyond_the_hard_limit():
    with pytest.raises(ValueError):
        CminSettings(max_ta_us=MAX_TA_PULSE_WIDTH_US + 10).validate()


def test_camera_stats_contrast_is_average_of_ratios():
    frames = [FrameRecord("left", 0, 11, 100.0, 10.0, 0.10),
              FrameRecord("left", 0, 12, 300.0, 90.0, 0.30)]
    (c,) = camera_stats(frames)
    assert c.contrast == pytest.approx(0.20)   # not 100/400
    assert c.mean == pytest.approx(200.0)


def test_report_files(tmp_path):
    result, _ = run(FakeBench())
    paths = write_cmin_report(result, CminSettings(), folder=tmp_path,
                              stem="SN-seedless-cmin-x", run_info={"operator": "t"})
    doc = json.loads(paths["json"].read_text())
    assert doc["written_to_device"] is False
    assert doc["result"]["c_min"] == pytest.approx(result.cmin)
    assert doc["result"]["matched_ta_pulse_width_us"] == result.matched_ta_us
    assert doc["result"]["c_min_basis"] == BASIS_MATCHED
    rows = paths["frames"].read_text().splitlines()
    assert rows[0].startswith("phase,burst,seed,ta_pulse_width_us")
    assert len(rows) - 1 == len(result.frames)
    assert "C_min" in paths["text"].read_text()
    assert not math.isnan(result.cmin)


def test_report_files_plateau(tmp_path):
    result, _ = run(FakeBench(seedless_per_us=BENCH_REACH))
    paths = write_cmin_report(result, CminSettings(), folder=tmp_path,
                              stem="p", run_info={})
    doc = json.loads(paths["json"].read_text())
    assert doc["result"]["c_min_basis"] == BASIS_PLATEAU
    assert len(doc["result"]["plateau"]) == 2
    assert set(doc["result"]["plateau_contrast_spread"]) == {f"R{c}" for c in range(1, 9)}
    text = paths["text"].read_text()
    assert "plateau" in text and "Contrast spread per camera" in text
    assert ",plateau," not in paths["frames"].read_text().splitlines()[0]
    assert any(r.startswith("plateau,") for r in paths["frames"].read_text().splitlines())
