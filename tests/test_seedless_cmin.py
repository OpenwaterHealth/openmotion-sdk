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
    PHASE_NOMINAL_AFTER,
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
    contrast_slope,
    contrast_spread,
    next_ta_us,
    plateau_widths,
    width_for_ratio,
)
from omotion.calibration.seedless_cmin_report import write_cmin_report
from omotion.seedless import MAX_TA_CURRENT_MA, MAX_TA_PULSE_WIDTH_US

CAMS = {("right", c) for c in range(8)}


class FakeBench:
    def __init__(self, *, nominal_mean=200.0, nominal_k=0.40,
                 seedless_per_us=0.975 / 2000, seedless_k=0.09,
                 seed_stays_on=False, dead_cam=None, plateau_us=None,
                 enter_error=None, restore_problem=None, drift=1.0,
                 k_slope=0.0, current_ma=3900.0, threshold_ma=1500.0,
                 current_ignored=False):
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
        # Unseeded output above threshold scales with drive current (a
        # simple model; the bench says nothing yet about the real curve).
        self.start_ma = current_ma
        self.current = current_ma
        self.threshold_ma = threshold_ma
        self.current_ignored = current_ignored
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
        if not self.current_ignored:
            mean *= ((self.current - self.threshold_ma)
                     / (self.start_ma - self.threshold_ma))
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

    def ta_current_ma(self):
        return self.current

    def set_ta_current_ma(self, ma):
        assert self.seedless and ma <= MAX_TA_CURRENT_MA + 1e-6
        self.log.append(("current", ma))
        self.current = ma

    def exit_seedless(self):
        self.log.append(("exit",))
        self.seedless = False
        return self.restore_problem


def restored(bench):
    """Exited seedless exactly once, and only seeded captures (the nominal
    re-check) came after."""
    exits = [i for i, e in enumerate(bench.log) if e == ("exit",)]
    return (len(exits) == 1 and not bench.seedless
            and all(e[0] == "capture" and e[1] is False
                    for e in bench.log[exits[0] + 1:]))


def run(bench, **kw):
    # Most tests here cover the search and gates with the earlier width
    # match and no warm-up; the default (current match, warm-up) has its
    # own tests at the end.
    kw.setdefault("warmup", False)
    kw.setdefault("match_by", "width")
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
    # First burst at the normal 500 us; reachable at the unit's own current.
    widths = [b.ta_pulse_width_us for b in result.bursts]
    assert abs(widths[0] - 500) < 0.5
    assert not any(c[0] == "current" for c in bench.log)
    assert result.matched_current_ma == result.start_current_ma == 3900.0
    assert restored(bench)
    phases = {f.phase for f in result.frames}
    assert phases == {PHASE_NOMINAL, PHASE_SEARCH, PHASE_CMIN,
                      PHASE_NOMINAL_AFTER}
    # Every search burst is 10 light frames per camera.
    assert all(c[3] == 10 for c in bench.log
               if c[0] == "capture" and c[1] and c[3] != 40)
    # Three 1 s repeats at the matched setting, then the seeded re-check.
    assert sum(1 for f in result.frames if f.phase == PHASE_CMIN) == 3 * 8 * 40
    assert len(result.cmin_captures) == 3
    assert [c.label for c in result.cameras] == [f"R{c}" for c in range(1, 9)]
    for cam in result.cameras:
        assert len(cam.repeats) == 3 and cam.repeat_spread == pytest.approx(0)
        assert cam.nominal_drift == pytest.approx(0, abs=1e-4)
        assert cam.cmin_over_nominal == pytest.approx(0.09 / 0.40)
    assert result.notes == []


def test_seed_still_on_stops_after_first_burst():
    bench = FakeBench(seed_stays_on=True)
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "seed may not be off" in result.problem
    assert len(result.bursts) == 1
    assert restored(bench)


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
    result, _ = run(FakeBench(seedless_per_us=0.75 / 2150), current_stepping=False)
    assert result.verdict == VERDICT_FAIL
    assert "out of reach" in result.problem
    assert result.bursts[-1].ta_pulse_width_us <= MAX_TA_PULSE_WIDTH_US
    assert result.cmin is None


# The bench case (2026-11-24): 2150 us reaches ~0.75x nominal.
BENCH_REACH = 0.75 / 2150


PLATEAU = dict(current_stepping=False, plateau_check=True)


def test_out_of_reach_flat_contrast_passes_on_the_plateau_check():
    bench = FakeBench(seedless_per_us=BENCH_REACH)
    result, lines = run(bench, **PLATEAU)
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
    assert restored(bench)
    assert any("out of reach" in line for line in lines)


def test_out_of_reach_contrast_that_follows_brightness_fails_per_camera():
    # 0.5x -> 0.75x brightness moves contrast by ~10 %: not flat.
    bench = FakeBench(seedless_per_us=BENCH_REACH, k_slope=0.4)
    result, _ = run(bench, **PLATEAU)
    assert result.verdict == VERDICT_FAIL
    assert "not flat" in result.problem and "R1" in result.problem
    assert result.cmin is not None            # measured and reported anyway
    assert all(v > 0.05 for v in result.plateau_spread.values())
    assert restored(bench)


def test_plateau_check_tolerance_is_a_setting():
    bench = FakeBench(seedless_per_us=BENCH_REACH, k_slope=0.4)
    result, _ = run(bench, plateau_tolerance=0.2, **PLATEAU)
    assert result.verdict == VERDICT_PASS, result.problem


def test_too_dim_for_the_plateau_check_fails():
    result, _ = run(FakeBench(seedless_per_us=0.4 / 2150), **PLATEAU)
    assert result.verdict == VERDICT_FAIL
    assert "out of reach" in result.problem and "plateau check" in result.problem
    assert result.plateau == [] and result.cmin is None


def test_plateau_range_too_narrow_fails():
    # ~0.54x at the limit: the 0.5x width is within 10 % of the longest pulse.
    result, _ = run(FakeBench(seedless_per_us=0.54 / 2150), **PLATEAU)
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
    assert restored(bench)


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
    result, _ = run(FakeBench(seedless_per_us=BENCH_REACH), **PLATEAU)
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


# ---------------------------------------------------------------------------
# Drive-current stepping (Brad / Ethan 2026-11-24)
# ---------------------------------------------------------------------------

def test_current_steps_up_100ma_until_the_brightness_is_reachable():
    # 0.75x at 2150 us at 3.9 A (the bench); with a 1.5 A threshold the
    # model needs 1/0.75 more light: 1500 + 2400 / 0.75 = 4700 mA.
    bench = FakeBench(seedless_per_us=BENCH_REACH)
    result, lines = run(bench)
    assert result.verdict == VERDICT_PASS, result.problem
    assert result.basis == BASIS_MATCHED
    steps = [c[1] for c in bench.log if c[0] == "current"]
    assert steps == pytest.approx([4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700])
    assert result.currents_tried_ma == pytest.approx([3900] + steps)
    assert result.matched_current_ma == pytest.approx(4700)
    assert result.start_current_ma == pytest.approx(3900)
    assert result.matched_ta_us <= MAX_TA_PULSE_WIDTH_US
    assert abs(result.cmin_capture.ratio - 1) <= 0.03
    assert result.cmin_capture.ta_current_ma == pytest.approx(4700)
    # Each round: 3 progressive widths at one current (round 1 adds 500 us).
    at_3900 = [b.ta_pulse_width_us for b in result.bursts if b.ta_current_ma == 3900]
    assert at_3900 == pytest.approx([499.84, 1075.2, 1612.8, 2149.76], abs=0.4)
    assert restored(bench)
    assert any("Raising the TA drive current to 4000 mA" in line for line in lines)
    # Every frame carries its current.
    assert all(f.ta_current_ma is not None for f in result.frames)


def test_out_of_reach_at_5a_fails_and_never_passes_the_ceiling():
    bench = FakeBench(seedless_per_us=0.5 / 2150)   # needs ~6.3 A in the model
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "out of reach" in result.problem and "5000 mA" in result.problem
    steps = [c[1] for c in bench.log if c[0] == "current"]
    assert max(steps) == pytest.approx(MAX_TA_CURRENT_MA)
    assert steps[-2:] == pytest.approx([4900, MAX_TA_CURRENT_MA])
    assert result.cmin is None
    assert restored(bench)


def test_max_current_setting_caps_the_search():
    bench = FakeBench(seedless_per_us=BENCH_REACH)
    result, _ = run(bench, max_current_ma=4300)
    assert result.verdict == VERDICT_FAIL
    assert max(c[1] for c in bench.log if c[0] == "current") == pytest.approx(4300)


def test_current_that_does_not_raise_the_mean_stops():
    bench = FakeBench(seedless_per_us=BENCH_REACH, current_ignored=True)
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "raising the drive current to 4000 mA did not raise" in result.problem
    assert restored(bench)


def test_settings_reject_current_above_5a():
    with pytest.raises(ValueError):
        CminSettings(max_current_ma=5100).validate()


def test_report_records_the_current(tmp_path):
    result, _ = run(FakeBench(seedless_per_us=BENCH_REACH))
    paths = write_cmin_report(result, CminSettings(), folder=tmp_path,
                              stem="c", run_info={})
    doc = json.loads(paths["json"].read_text())
    assert doc["result"]["matched_ta_current_ma"] == pytest.approx(4700)
    assert doc["result"]["start_ta_current_ma"] == pytest.approx(3900)
    rows = paths["frames"].read_text().splitlines()
    assert rows[0].split(",")[4] == "ta_current_ma"
    assert "TA drive current for matched brightness: 4700.00 mA" in paths["text"].read_text()


class PerCamBench(FakeBench):
    """FakeBench with per-camera seedless brightness, repeat noise and a
    seeded brightness that moves after the seedless part."""

    def __init__(self, *, cam_gain=None, repeat_k=None, after_gain=1.0, **kw):
        super().__init__(**kw)
        self.cam_gain = cam_gain or {}
        self.repeat_k = list(repeat_k or [])
        self.after_gain = after_gain
        self.exited = False

    def exit_seedless(self):
        self.exited = True
        return super().exit_seedless()

    def capture(self, light_frames, *, seedless):
        frames = super().capture(light_frames, seedless=seedless)
        out = []
        for f in frames:
            g = 1.0
            k = f.contrast
            if seedless:
                g = self.cam_gain.get(f.cam_id, 1.0)
                if light_frames == 40 and self.repeat_k:
                    k = k * self.repeat_k[0]
            elif self.exited:
                g = self.after_gain
            out.append(FrameRecord(f.side, f.cam_id, f.abs_frame_id, f.mean * g,
                                   f.std * g * k / f.contrast, k))
        if seedless and light_frames == 40 and self.repeat_k:
            self.repeat_k.pop(0)
        return out


def test_one_camera_out_of_brightness_fails_with_its_name():
    bench = PerCamBench(cam_gain={3: 0.90})
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "brightness not matched per camera" in result.problem
    assert "R4 0.9" in result.problem
    # Per-camera numbers are still in the result for the report.
    assert len(result.cameras) == 8
    assert restored(bench)


def test_cmin_that_does_not_repeat_fails():
    bench = PerCamBench(repeat_k=[1.0, 1.03, 1.0])
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "does not repeat within 2%" in result.problem
    assert all(c.repeat_spread == pytest.approx(0.03, rel=0.05)
               for c in result.cameras)
    assert restored(bench)


def test_nominal_drift_fails_after_the_restore():
    bench = PerCamBench(after_gain=0.97)
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "normal (seeded) brightness moved" in result.problem
    assert all(c.nominal_drift == pytest.approx(-0.03, abs=1e-4)
               for c in result.cameras)
    assert len(result.nominal_after) == 8
    assert restored(bench)


def test_nominal_recheck_can_be_skipped():
    bench = FakeBench()
    result, _ = run(bench, nominal_recheck=False)
    assert result.verdict == VERDICT_PASS
    assert bench.log[-1] == ("exit",)
    assert all(c.nominal_drift is None for c in result.cameras)


def test_restore_problem_before_recheck_fails_without_recapturing():
    bench = FakeBench(restore_problem="TA width read back wrong")
    result, _ = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "TA width read back wrong" in result.problem
    assert bench.log[-1] == ("exit",)
    assert result.nominal_after == ()


def test_cmin_is_corrected_to_exactly_one_times_brightness():
    # Contrast rises as the image dims; the capture lands a little dim
    # (drift 0.985), so the raw C_min is a bit high and the fit pulls it
    # back to the contrast at 1.0x.
    bench = FakeBench(k_slope=0.5, drift=0.985)
    result, _ = run(bench)
    assert result.verdict == VERDICT_PASS, result.problem
    for cam in result.cameras:
        assert cam.ratio < 1.0
        assert cam.cmin_raw > cam.cmin
        assert cam.slope == pytest.approx(-0.09 * 0.5, rel=0.02)
        # The model's contrast at exactly 1.0x is seedless_k.
        assert cam.cmin == pytest.approx(0.09, abs=2e-4)


def test_contrast_slope_needs_a_spread_of_brightness():
    assert contrast_slope([(1.0, 0.1)]) is None
    assert contrast_slope([(0.99, 0.1), (1.0, 0.1)]) is None
    assert contrast_slope([(0.8, 0.12), (1.0, 0.10)]) == pytest.approx(-0.1)


@pytest.mark.parametrize("kw", [{"cmin_repeats": 1}, {"repeat_spread_max": 0},
                                {"nominal_drift_max": 1.5},
                                {"camera_tolerance": 0}])
def test_settings_reject_bad_confidence_gates(kw):
    with pytest.raises(ValueError):
        CminSettings(**kw).validate()


def test_report_has_per_camera_table_and_repeats(tmp_path):
    bench = FakeBench()
    result, _ = run(bench)
    paths = write_cmin_report(result, CminSettings(), folder=tmp_path,
                              stem="t", run_info={})
    text = paths["text"].read_text()
    assert "Per camera C_min" in text and "Repeat captures" in text
    assert "Nominal again after the restore" in text
    doc = json.loads(paths["json"].read_text())
    cams = doc["result"]["cameras"]
    assert [c["label"] for c in cams] == [f"R{c}" for c in range(1, 9)]
    assert len(doc["result"]["cmin_repeats"]) == 3
    assert cams[0]["cmin"] == pytest.approx(0.09)
    rows = paths["frames"].read_text().splitlines()
    assert sum(1 for r in rows if r.startswith("nominal_after,")) == 8 * 40


# --- default: warm-up, then match by current at the full pulse -------------

class WarmingBench(FakeBench):
    """The seeded image brightens by `rise` per warm-up chunk, settling
    after `settle` chunks."""

    def __init__(self, *, rise=0.01, settle=4, **kw):
        super().__init__(**kw)
        self.rise, self.settle, self.chunks = rise, settle, 0
        self.base = self.nominal_mean

    def capture(self, light_frames, *, seedless):
        if not seedless and light_frames == 800:
            self.chunks += 1
            self.nominal_mean = self.base * (
                1 + self.rise * min(self.chunks, self.settle))
        return super().capture(light_frames, seedless=seedless)


def run_default(bench, **kw):
    lines = []
    result = SeedlessCminProcedure(bench, CminSettings(**kw), CAMS,
                                   output_func=lines.append).run()
    return result, lines


def test_default_holds_the_pulse_at_the_ceiling_and_tunes_current():
    # 0.74x at 2150 us at 3900 mA: needs more current.
    bench = FakeBench(seedless_per_us=0.74 / 2150)
    result, lines = run_default(bench, warmup=False)
    assert result.verdict == VERDICT_PASS, result.problem
    assert result.match_by == "current"
    assert result.matched_ta_us == pytest.approx(MAX_TA_PULSE_WIDTH_US, abs=0.5)
    # Model: ratio = 0.74 (I - 1500) / 2400 -> 1.0x at 4743 mA.
    assert result.matched_current_ma == pytest.approx(4743, abs=30)
    assert abs(result.cmin_capture.ratio - 1) <= 0.01 + 1e-9
    assert any("tuning the TA drive current" in line for line in lines)
    # Refine bursts are all at the full pulse.
    refine = [b for b in result.bursts if b.ta_current_ma % 100]
    assert refine and all(abs(b.ta_pulse_width_us - MAX_TA_PULSE_WIDTH_US) < 0.5
               for b in refine)
    assert restored(bench)


def test_default_lowers_the_current_when_the_full_pulse_is_too_bright():
    bench = FakeBench()               # 1.048x at 2150 us, 3900 mA
    result, _ = run_default(bench, warmup=False)
    assert result.verdict == VERDICT_PASS, result.problem
    assert result.matched_ta_us == pytest.approx(MAX_TA_PULSE_WIDTH_US, abs=0.5)
    assert result.matched_current_ma < 3900
    assert abs(result.cmin_capture.ratio - 1) <= 0.01 + 1e-9


def test_current_match_corrects_along_the_full_pulse_bursts():
    bench = FakeBench(seedless_per_us=0.75 / 2150, k_slope=0.5)
    result, _ = run_default(bench, warmup=False)
    assert result.verdict == VERDICT_PASS, result.problem
    for cam in result.cameras:
        assert cam.slope == pytest.approx(-0.045, rel=0.05)
        assert cam.cmin == pytest.approx(0.09, abs=2e-4)


def test_current_that_does_nothing_fails_the_current_match():
    bench = FakeBench(current_ignored=True)
    result, _ = run_default(bench, warmup=False)
    assert result.verdict == VERDICT_FAIL
    assert "current" in result.problem
    assert restored(bench)


def test_warm_up_runs_until_the_seeded_image_settles():
    bench = WarmingBench(rise=0.01, settle=4)
    result, lines = run_default(bench, match_by="width")
    assert result.verdict == VERDICT_PASS, result.problem
    # Chunks 2-4 change ~1 %, chunk 5 doesn't: warm after 5 chunks.
    assert bench.chunks == 5
    assert result.warmup[-1] <= 0.002 and result.warmup[0] > 0.005
    assert any("Laser warm after 100 s" in line for line in lines)
    assert {f.phase for f in result.frames} >= {"warmup"}
    # The nominal is the warm one.
    assert result.nominal[0].mean == pytest.approx(200 * 1.04, rel=0.01)


def test_warm_up_that_never_settles_fails_before_the_seed_is_touched():
    bench = WarmingBench(rise=0.01, settle=100)
    result, _ = run_default(bench, warmup_max_chunks=4)
    assert result.verdict == VERDICT_FAIL
    assert "did not settle" in result.problem
    assert not any(e[0] == "enter" for e in bench.log)


def test_gate_failure_still_rechecks_the_seeded_image():
    bench = PerCamBench(cam_gain={3: 0.90})
    result, lines = run(bench)
    assert result.verdict == VERDICT_FAIL
    assert "brightness not matched per camera" in result.problem
    assert len(result.nominal_after) == 8
    assert all(c.nominal_drift is not None for c in result.cameras)
    assert any("seed back ON" in line for line in lines)


def test_early_failure_says_the_seed_is_back_on():
    bench = FakeBench(seed_stays_on=True)
    _, lines = run(bench)
    assert "Laser settings restored, seed back ON." in lines
