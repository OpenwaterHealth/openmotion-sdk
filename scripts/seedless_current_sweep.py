"""Seedless contrast vs TA drive current at a fixed pulse width (engineering).

Ethan / laser team, 2026-11-24: how does the unseeded TA's speckle contrast
change with drive current at the same pulse width? This runs the seedless
C_min operator script's bring-up, guard rails and restore unchanged, but
replaces the C_min search with a sweep:

1. seeded warm-up until the image holds (as the C_min procedure);
2. the seeded nominal (80 light frames), for brightness ratios;
3. seed OFF, TA pulse fixed (default 2150 us); for each drive current from
   the unit's own up to 4999.68 mA in 100 mA steps, in shuffled order, three
   1 s captures; the first current is captured again at the end to show
   drift over the sweep;
4. restore, then the seeded nominal again.

Every frame lands in the usual seedless C_min report (frames CSV: phase
"sweep", burst = capture order, ta_current_ma, per-camera contrast). The
per-current, per-camera table is printed and written to
<run folder>/<serial>-current-sweep.csv. Nothing is written to the device.

LASER SAFETY: as omotion.scripts.seedless_cmin (EE/OPT limits widened and
restored, current bounded to the 5 A EE rating, STATUS checked after every
seedless capture). Static phantom only.

    python scripts/seedless_current_sweep.py --side right \
        --output-dir cmin_out --operator op --fixture-id phantom
"""

from __future__ import annotations

import csv
import math
import random
import sys
from pathlib import Path

import omotion.scripts.seedless_cmin as cmin_script
from omotion.calibration.seedless_cmin import (
    PHASE_NOMINAL,
    PHASE_NOMINAL_AFTER,
    VERDICT_PASS,
    CminResult,
    ProcedureStop,
    SeedlessCminProcedure,
    _check_frame_counts,
    camera_stats,
    contrast_spread,
    quantize_ta_us,
)
from omotion.seedless import MAX_TA_CURRENT_MA

PHASE_SWEEP = "sweep"
STEP_MA = 100.0
REPEATS = 3
FRAMES = 40
PULSE_US = 2150.0
SHUFFLE_SEED = 20261124


def sweep_currents(start_ma: float, top_ma: float = MAX_TA_CURRENT_MA,
                   step_ma: float = STEP_MA) -> list[float]:
    out = []
    c = start_ma
    while c < top_ma - 0.5:
        out.append(round(c, 2))
        c += step_ma
    out.append(top_ma)
    return out


class CurrentSweep(SeedlessCminProcedure):
    """Stands in for SeedlessCminProcedure inside the operator script."""

    rows: list[dict] = []

    def run(self) -> CminResult:
        result = CminResult(match_by="sweep")
        s = self._s
        rows: list[dict] = []
        CurrentSweep.rows = rows
        try:
            if s.warmup:
                self._warm_up(result)
            self._out(f"Measuring the normal image ({s.nominal_light_frames} "
                      "frames, seed on) ...")
            frames = self._bench.capture(s.nominal_light_frames, seedless=False)
            self._tag(result, frames, PHASE_NOMINAL, 0, quantize_ta_us(500.0), "on")
            cams = camera_stats(frames)
            _check_frame_counts(cams, self._cams, s.nominal_light_frames,
                                s.min_frame_fraction, "normal capture")
            result.nominal = tuple(c for c in cams
                                   if (c.side, c.cam_id) in self._cams)
            pulse = quantize_ta_us(min(PULSE_US, s.max_ta_us))
            self._out(f"Turning the seed OFF. TA pulse fixed at {pulse:.1f} us.")
            result.seedless_entered = True
            self._bench.enter_seedless(pulse)
            self._current = self._bench.ta_current_ma()
            result.start_current_ma = self._current
            currents = sweep_currents(self._current, s.max_current_ma)
            order = list(currents)
            random.Random(SHUFFLE_SEED).shuffle(order)
            order.append(order[0])            # drift check: first one again
            self._out(f"Sweeping {len(currents)} currents "
                      f"({currents[0]:.0f}..{currents[-1]:.0f} mA), "
                      f"{REPEATS} x {FRAMES / 40:.0f} s each, shuffled; the "
                      f"first ({order[0]:.0f} mA) is repeated at the end.")
            index = 0
            for step, ma in enumerate(order, start=1):
                self._bench.set_ta_current_ma(ma)
                self._current = self._bench.ta_current_ma()
                for rep in range(1, REPEATS + 1):
                    index += 1
                    b = self._burst(result, index, pulse, FRAMES, PHASE_SWEEP,
                                    f"{self._current:.0f} mA capture {rep}")
                    result.bursts.append(b)
                    if index == 1:
                        k_nom = sum(c.contrast for c in result.nominal) / len(
                            result.nominal)
                        if not b.contrast <= 0.75 * k_nom:
                            raise ProcedureStop(
                                f"seedless contrast {b.contrast:.3f} is close "
                                f"to the seeded {k_nom:.3f}: the seed may not "
                                "be off")
                    nominal = {(c.side, c.cam_id): c for c in result.nominal}
                    for c in b.cameras:
                        if (c.side, c.cam_id) not in self._cams:
                            continue
                        rows.append(dict(
                            order=step, capture=index, repeat=rep,
                            final_repeat=step == len(order),
                            ta_current_ma=round(self._current, 2),
                            ta_pulse_width_us=round(pulse, 2),
                            camera=c.label, n=c.n, mean=round(c.mean, 3),
                            ratio=round(c.mean / nominal[(c.side, c.cam_id)].mean, 4),
                            contrast=round(c.contrast, 5)))
                self._out(f"  {step:2d}/{len(order)}: {self._current:.0f} mA -> "
                          f"{b.ratio:.3f}x normal brightness, contrast "
                          f"{b.contrast:.4f}")
            result.seedless_exited = True
            result.restore_problem = self._bench.exit_seedless()
            if result.restore_problem:
                raise ProcedureStop("the normal laser settings were not "
                                    "restored: " + result.restore_problem)
            self._out("Laser settings restored, seed back ON. Re-measuring the "
                      "normal image ...")
            frames = self._bench.capture(s.nominal_recheck_frames, seedless=False)
            self._tag(result, frames, PHASE_NOMINAL_AFTER, 0,
                      quantize_ta_us(500.0), "on", result.start_current_ma)
            result.nominal_after = tuple(c for c in camera_stats(frames)
                                         if (c.side, c.cam_id) in self._cams)
            result.verdict = VERDICT_PASS
        except ProcedureStop as exc:
            result.problem = str(exc)
        except Exception as exc:
            result.problem = f"{type(exc).__name__}: {exc}"
        finally:
            if result.seedless_entered and not result.seedless_exited:
                try:
                    result.restore_problem = self._bench.exit_seedless()
                except Exception as exc:
                    result.restore_problem = f"{type(exc).__name__}: {exc}"
                if not result.restore_problem:
                    self._out("Laser settings restored, seed back ON.")
                elif not result.problem:
                    result.problem = ("the normal laser settings were not "
                                      "restored: " + result.restore_problem)
        self._summarise(result, rows)
        return result

    def _summarise(self, result: CminResult, rows: list[dict]) -> None:
        if not rows:
            return
        labels = sorted({r["camera"] for r in rows},
                        key=lambda x: (x[0], int(x[1:])))
        main = [r for r in rows if not r["final_repeat"]]
        currents = sorted({r["ta_current_ma"] for r in main})

        def avg(vals):
            vals = [v for v in vals if math.isfinite(v)]
            return sum(vals) / len(vals) if vals else math.nan

        table = {}
        for ma in currents:
            for lab in labels:
                vals = [r["contrast"] for r in main
                        if r["ta_current_ma"] == ma and r["camera"] == lab]
                table[(ma, lab)] = (avg(vals), contrast_spread(vals))
        self._out("")
        self._out("Contrast per camera (average of the repeats) vs TA current:")
        self._out(f"  {'mA':>7}{'bright':>8}" + "".join(f"{l:>8}" for l in labels)
                  + f"{'all':>8}{'rep sp':>8}")
        for ma in currents:
            bright = avg([r["ratio"] for r in main if r["ta_current_ma"] == ma])
            cs = [table[(ma, l)][0] for l in labels]
            sp = max(table[(ma, l)][1] for l in labels)
            self._out(f"  {ma:>7.0f}{bright:>7.3f}x" + "".join(f"{c:>8.4f}" for c in cs)
                      + f"{avg(cs):>8.4f}{sp:>8.1%}")
        last = [r for r in rows if r["final_repeat"]]
        if last:
            ma = last[0]["ta_current_ma"]
            first = {l: table[(ma, l)][0] for l in labels}
            again = {l: avg([r["contrast"] for r in last if r["camera"] == l])
                     for l in labels}
            self._out(f"  {ma:>7.0f} again at the end: " + ", ".join(
                f"{l} {again[l] / first[l] - 1:+.1%}" for l in labels))
            result.notes.append(f"drift check at {ma:.0f} mA: " + ", ".join(
                f"{l} {again[l] / first[l] - 1:+.1%}" for l in labels))


def main(argv=None) -> int:
    rows_holder = {}
    original_writer = cmin_script.write_cmin_report

    def write_report(result, settings, *, folder, stem, run_info):
        paths = original_writer(result, settings, folder=folder, stem=stem,
                                run_info=run_info)
        rows = CurrentSweep.rows
        if rows:
            p = Path(folder) / f"{stem.split('-seedless-cmin-')[0]}-current-sweep.csv"
            with open(p, "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)
            paths = dict(paths, sweep=p)
        rows_holder["paths"] = paths
        return paths

    cmin_script.SeedlessCminProcedure = CurrentSweep
    cmin_script.write_cmin_report = write_report
    return cmin_script.main(argv)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
