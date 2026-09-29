#!/usr/bin/env python3
"""Detect and characterize intensity dips in reduced per-frame stats CSVs.

The 20-min duty-cycle scans showed no dip below the warm-up trend; the dip
is a long-timescale (tens of minutes) feature, so this detrends each camera's
mean with a long rolling-median baseline and flags sustained negative
excursions. For each dip it reports center time, depth (% below baseline),
width, and the concurrent die temperature; it also checks whether dips are
synchronized across cameras (common cause: light source / thermal) or
per-camera (sensor-specific).

Usage:
  python dip_analyze.py <data_dir> [--side right] [--depth-pct 1.0]
Reads <data_dir>/derived/*_<side>_*_stats.csv, writes <data_dir>/analysis/
dips.csv and per-scan dip figures.
"""
import argparse
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CAM_COLORS = ["#2a78d6", "#1baf7a", "#eda100", "#008300",
              "#4a3aa7", "#e34948", "#e87ba4", "#eb6834"]
INK, MUTED, GRID, SURFACE = "#14130f", "#898781", "#e1e0d9", "#fcfcfb"
plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "savefig.dpi": 140, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.7,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
    "axes.edgecolor": "#c3c2b7", "text.color": INK, "axes.labelcolor": "#52514e",
    "xtick.color": MUTED, "ytick.color": MUTED,
})

FS = 40.0                       # frames/s (per camera)
BASELINE_S = 600.0              # long rolling-median baseline window (10 min)
SMOOTH_S = 4.0                  # short smoothing before excursion test


def _roll(series, win, fn="median"):
    r = pd.Series(series).rolling(int(win), center=True, min_periods=max(5, int(win) // 10))
    return (r.median() if fn == "median" else r.mean()).to_numpy()


def find_dips(t, m, depth_frac, min_width_s):
    """Return list of dips as dicts. A dip = contiguous run where the smoothed
    fractional deviation (m/baseline - 1) stays below -depth_frac for at least
    min_width_s; characterized at its minimum. The min-width gate rejects the
    single-frame ambient flicker that crosses the threshold transiently."""
    if len(t) < int(FS * 120):
        return [], None, None
    base = _roll(m, FS * BASELINE_S)                 # 10-min median baseline
    sm = _roll(m, FS * SMOOTH_S)
    frac = sm / base - 1.0
    below = frac < -depth_frac
    dips = []
    i = 0
    n = len(frac)
    while i < n:
        if below[i]:
            j = i
            while j < n and below[j]:
                j += 1
            width = float(t[j - 1] - t[i])
            if width >= min_width_s:
                seg = slice(i, j)
                k = i + int(np.nanargmin(frac[seg]))
                dips.append({
                    "t_center_s": float(t[k]),
                    "t_center_min": float(t[k] / 60),
                    "depth_pct": float(-frac[k] * 100),
                    "width_s": width,
                    "i0": i, "i1": j, "kmin": k,
                })
            i = j
        else:
            i += 1
    return dips, base, frac


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("data_dir")
    ap.add_argument("--side", default="right")
    ap.add_argument("--depth-pct", type=float, default=1.0,
                    help="min depth below baseline to count as a dip (percent)")
    ap.add_argument("--min-width-s", type=float, default=10.0,
                    help="min sustained width to count as a dip (s); rejects flicker")
    a = ap.parse_args()
    data = Path(a.data_dir)
    out = data / "analysis"
    out.mkdir(exist_ok=True)
    depth_frac = a.depth_pct / 100.0

    files = sorted((data / "derived").glob(f"*_{a.side}_*_stats.csv"))
    if not files:
        print(f"no {a.side} stats files in {data/'derived'}")
        return
    rows = []
    for f in files:
        m = re.search(r"_([A-Z0-9]+_\d+)_" + a.side, f.name)
        label = m.group(1) if m else f.stem
        df = pd.read_csv(f)
        df = df[(df.timestamp_s >= 0) & (df.type == "light")]
        cams = sorted(df.cam_id.unique())
        fig, axes = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True,
                                 gridspec_kw={"height_ratios": [3, 1]})
        per_cam_dips = {}
        for cam in cams:
            g = df[df.cam_id == cam].sort_values("timestamp_s")
            t = g.timestamp_s.to_numpy(); mv = g["mean"].to_numpy()
            dips, base, frac = find_dips(t, mv, depth_frac, a.min_width_s)
            per_cam_dips[cam] = dips
            c = CAM_COLORS[cam % 8]
            axes[0].plot(t / 60, mv, lw=0.8, color=c, alpha=0.75, label=f"cam {cam}")
            if base is not None:
                axes[0].plot(t / 60, base, lw=1.0, color=c, alpha=0.5, ls="--")
            for d in dips:
                axes[0].plot(d["t_center_min"], mv[d["kmin"]], "v", color=c, ms=7,
                             markeredgecolor=SURFACE)
                rows.append({"scan": label, "cam": cam, **{k: d[k] for k in
                            ("t_center_min", "depth_pct", "width_s")},
                            "T_at_dip": float(g.temperature.to_numpy()[d["kmin"]]),
                            "mean_level": float(np.nanmedian(mv))})
            gt = g.sort_values("timestamp_s")
            axes[1].plot(gt.timestamp_s / 60, gt.temperature, lw=0.8, color=c, alpha=0.7)
        axes[0].set_ylabel("light-frame mean (counts)")
        axes[0].legend(ncol=8, fontsize=7, loc="upper right")
        axes[0].set_title(f"{label} ({a.side}): mean intensity (dashed = 10-min baseline, "
                          f"▼ = dip >{a.depth_pct:g}%)", color=INK)
        axes[1].set_ylabel("die temp (°C)")
        axes[1].set_xlabel("time in scan (min)")
        fig.tight_layout()
        fig.savefig(out / f"dips_{label}_{a.side}.png", bbox_inches="tight")
        plt.close(fig)
        ndip = sum(len(v) for v in per_cam_dips.values())
        print(f"{label} {a.side}: {ndip} dip(s) across {len(cams)} cams; "
              f"per-cam counts { {c: len(per_cam_dips[c]) for c in cams} }")

    if rows:
        dd = pd.DataFrame(rows).sort_values(["scan", "t_center_min", "cam"])
        dd.to_csv(out / "dips.csv", index=False)
        print(f"\n{len(dd)} dips -> {out/'dips.csv'}")
        print(dd.to_string(index=False))
    else:
        print(f"\nNo dips >{a.depth_pct:g}% found (side={a.side}).")


if __name__ == "__main__":
    main()
