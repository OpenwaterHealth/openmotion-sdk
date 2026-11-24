"""Report files for the seedless C_min measurement.

Three files side by side in the run folder, named
``<prefix>seedless-cmin-<run id>.<ext>``:

* ``-frames.csv`` - every corrected light frame of every phase: phase,
  burst, seed on/off, TA pulse width, side, camera, frame id, mean, std,
  contrast.
* ``.json`` - run identity, settings, the nominal per-camera stats, the
  search history (pulse width -> brightness ratio per burst), the C_min
  capture, the per-camera and overall C_min, the verdict and any problem.
* ``.txt`` - the same summary in plain text for a quick read.

The report is the only place C_min goes; nothing is written to the device.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict
from pathlib import Path
from typing import Mapping

from .seedless_cmin import CminResult, CminSettings

FRAME_COLUMNS = ("phase", "burst", "seed", "ta_pulse_width_us",
                 "ta_current_ma", "side",
                 "camera", "abs_frame_id", "mean", "std", "contrast")


def write_cmin_report(result: CminResult, settings: CminSettings, *,
                      folder: Path, stem: str,
                      run_info: Mapping[str, object]) -> dict[str, Path]:
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    paths = {
        "frames": folder / f"{stem}-frames.csv",
        "json": folder / f"{stem}.json",
        "text": folder / f"{stem}.txt",
    }

    with open(paths["frames"], "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(FRAME_COLUMNS)
        for fr in result.frames:
            w.writerow([fr.phase, fr.burst, fr.seed,
                        f"{fr.ta_pulse_width_us:.2f}",
                        "" if fr.ta_current_ma is None else f"{fr.ta_current_ma:.2f}",
                        fr.side, fr.cam_id + 1,
                        fr.abs_frame_id, f"{fr.mean:.4f}", f"{fr.std:.4f}",
                        f"{fr.contrast:.6f}"])

    doc = {
        "procedure": "Seedless C_min measurement",
        "run": dict(run_info),
        "settings": asdict(settings),
        "result": result.to_dict(),
        "written_to_device": False,
        "files": {k: p.name for k, p in paths.items()},
    }
    with open(paths["json"], "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, default=str)

    with open(paths["text"], "w", encoding="utf-8") as f:
        f.write(format_summary(result, run_info))
    return paths


def format_summary(result: CminResult, run_info: Mapping[str, object]) -> str:
    lines = ["Seedless C_min measurement", ""]
    for key, value in run_info.items():
        lines.append(f"{key}: {value}")
    lines.append("")
    lines.append(f"Verdict: {result.verdict}")
    if result.problem:
        lines.append(f"Problem: {result.problem}")
    if result.cmin is not None:
        lines.append(f"C_min (average contrast, all cameras): {result.cmin:.4f}")
        if result.matched_ta_us is not None:
            lines.append(f"TA pulse width for matched brightness: "
                         f"{result.matched_ta_us:.2f} us")
            if result.matched_current_ma is not None:
                lines.append(f"TA drive current for matched brightness: "
                             f"{result.matched_current_ma:.2f} mA (unit's own: "
                             f"{result.start_current_ma:.2f} mA)")
        else:
            lines.append("Brightness not matched: C_min from the plateau "
                         "check (contrast flatness per camera, below)")
    lines.append("Nothing was written to the device.")
    if result.restore_problem:
        lines.append(f"RESTORE PROBLEM: {result.restore_problem}")
    if result.nominal:
        lines += ["", "Nominal (seed on)", f"{'cam':<5}{'n':>4}{'mean':>10}"
                  f"{'contrast':>10}"]
        for c in result.nominal:
            lines.append(f"{c.label:<5}{c.n:>4}{c.mean:>10.2f}{c.contrast:>10.4f}")
    if result.bursts:
        lines += ["", "Seedless search", f"{'burst':<7}{'TA mA':>9}{'TA us':>9}"
                  f"{'mean ratio':>12}{'contrast':>10}"]
        for b in result.bursts:
            ma = "" if b.ta_current_ma is None else f"{b.ta_current_ma:.0f}"
            lines.append(f"{b.burst:<7}{ma:>9}{b.ta_pulse_width_us:>9.1f}"
                         f"{b.ratio:>12.3f}{b.contrast:>10.4f}")
    if result.plateau:
        lines += ["", "Plateau check (below the C_min capture)",
                  f"{'burst':<7}{'TA us':>9}{'mean ratio':>12}{'contrast':>10}"]
        for b in result.plateau:
            lines.append(f"{b.burst:<7}{b.ta_pulse_width_us:>9.1f}"
                         f"{b.ratio:>12.3f}{b.contrast:>10.4f}")
    if result.plateau_spread:
        lines += ["", "Contrast spread per camera across the plateau widths "
                  "((max - min) / mean)"]
        for label, spread in result.plateau_spread.items():
            lines.append(f"{label:<5}{spread:>8.1%}")
    if result.cmin_capture is not None:
        cap = result.cmin_capture
        lines += ["", f"C_min capture at {cap.ta_pulse_width_us:.1f} us "
                  f"(mean ratio {cap.ratio:.3f})",
                  f"{'cam':<5}{'n':>4}{'mean':>10}{'C_min':>10}"]
        for c in cap.cameras:
            lines.append(f"{c.label:<5}{c.n:>4}{c.mean:>10.2f}{c.contrast:>10.4f}")
    return "\n".join(lines) + "\n"
