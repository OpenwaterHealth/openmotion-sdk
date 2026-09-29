#!/usr/bin/env python3
"""Overlay new cold-start dip runs on the Jul-Aug 2026 control band.

Per camera: dark-corrected mean (mean_dc), 120-frame rolling median, divided by
the run's own plateau (median 600-1500 s) -> % of plateau. Same smoothing and
plateau window as dip_from_meandc.py.

SOURCE-STEP MASK: the illumination source sometimes comes back from a dark
window ~5% brighter or dimmer and holds that level for exactly one dark-window
interval (seen in 16 of 17 runs, Jul-Sep). The photodiode and every camera move
together while the dark pedestal stays flat, so it is a real light change, not
a dark-correction error. Intervals whose photodiode median deviates more than
STEP_TOL_PCT from the median of the neighbouring intervals are dropped before
plotting and before the dip minimum is taken.

Usage:
  python bench/plot_dip_overlay.py [--mask c3|ff] <out.png> <data_dir> <subject> [<subject> ...]
  e.g. python bench/plot_dip_overlay.py dip.png bench/rebaseline_out REBASE_01 REBASE_02
       python bench/plot_dip_overlay.py --mask ff dip_ff.png bench/ff_rebaseline_out FFBASE_01

--mask picks the matching historical control set and camera panels: c3 (default,
cams 1/2/7/8, n=14 controls) or ff (all 8 cams, n=3 July controls).
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BENCH = Path(__file__).resolve().parent
HIST = {  # plain 2-h fan-soak, mask 0xC3 controls
    "clinical_dip_out": ["CLIN_02", "CLIN_04", "CLIN_05"],
    "crop_dip_out": ["CROPEXP_01", "CROPEXP_04"],
    "warmup_dip_out": ["FANEXP_01", "FANEXP_04"],
    "ob_dip_out": ["OBDIP_01", "OBDIP_02", "OBDIP_03"],
    "vendor_reg_out": ["VREG1_01", "VREG1_03"],
    "moved_dip_out": ["MOVED_01", "MOVED_03"],
}
HIST_FF = {  # plain 2-h fan-soak, mask 0xFF controls (Jul 16)
    "clinical_dip_out": ["CLIN_01"],
    "deepsoak_out": ["DEEPSOAK_01", "DEEPSOAK_02"],
}
PRESETS = {  # --mask -> (controls, camera panels (cam_id), label, grid)
    "c3": (HIST, [0, 1, 6, 7], "0xC3", (2, 2)),
    "ff": (HIST_FF, list(range(8)), "0xFF", (2, 4)),
}
DARK_INTERVAL_S = 60.0
STEP_TOL_PCT = 2.0
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]   # categorical slots 1-5
HIST_GREY = "#b9b8b0"
INK, INK2 = "#1f1f1c", "#6b6a63"
T_MAX = 900


def step_intervals(df: pd.DataFrame) -> set:
    """Dark-window intervals where the source stepped (photodiode vs. neighbours)."""
    light = df[~df.is_dark & (df.photodiode_w > 10e-6)]
    iv = (light.timestamp_s // DARK_INTERVAL_S).astype(int)
    pd_iv = light.groupby(iv).photodiode_w.median()
    ref = pd_iv.rolling(5, center=True, min_periods=3).median()
    dev = 100 * (pd_iv / ref - 1)
    return set(dev[dev.abs() > STEP_TOL_PCT].index)


def curves(path: Path):
    df = pd.read_csv(path, usecols=["cam_id", "timestamp_s", "is_dark", "mean_dc", "photodiode_w"])
    bad = step_intervals(df)
    out = {}
    for cam, g in df[~df.is_dark].dropna(subset=["mean_dc"]).groupby("cam_id"):
        s = g.sort_values("timestamp_s").set_index("timestamp_s").mean_dc
        stepped = (s.index // DARK_INTERVAL_S).astype(int).isin(bad)
        s[stepped] = np.nan                     # mask BEFORE smoothing so no edge bleed
        s = s.rolling(120, center=True, min_periods=15).median()
        s[stepped] = np.nan                     # keep the masked span as a visible gap
        plat = s[(s.index >= 600) & (s.index <= 1500)].median()
        if plat and plat > 0:
            s = 100 * s / plat
            out[cam] = s[s.index <= T_MAX].iloc[::10]
    return out, bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mask", choices=sorted(PRESETS), default="c3")
    ap.add_argument("out_png")
    ap.add_argument("data_dir", type=Path)
    ap.add_argument("runs", nargs="+")
    args = ap.parse_args()
    out_png, data_dir, runs = args.out_png, args.data_dir, args.runs
    hist, cams, mask_label, (nrows, ncols) = PRESETS[args.mask]
    fig, axes = plt.subplots(nrows, ncols, figsize=(5.5 * ncols, 3.5 * nrows), sharex=True)
    axes = dict(zip(cams, axes.ravel()))
    bottom_row, left_col = cams[-ncols:], cams[::ncols]
    n_hist = 0
    for d, subjs in hist.items():
        for s in subjs:
            f = BENCH / d / f"{s}_analysis.csv"
            if not f.exists():
                continue
            n_hist += 1
            cs, bad = curves(f)
            if bad:
                print(f"{s}: masked source-step intervals {sorted(int(b * DARK_INTERVAL_S) for b in bad)} s")
            for cam, c in cs.items():
                if cam in axes:
                    axes[cam].plot(c.index, c.values, color=HIST_GREY, lw=1, alpha=0.7, zorder=1)
    for i, r in enumerate(runs):
        cs, bad = curves(data_dir / f"{r}_analysis.csv")
        if bad:
            print(f"{r}: masked source-step intervals {sorted(int(b * DARK_INTERVAL_S) for b in bad)} s")
        for cam, c in cs.items():
            if cam not in axes:
                continue
            ax = axes[cam]
            ax.plot(c.index, c.values, color=SERIES[i % len(SERIES)], lw=2, zorder=3, label=r)
            body = c[(c.index >= 60) & (c.index <= 420)].dropna()
            if body.empty:
                continue
            tmin = body.idxmin()
            if 100 - body[tmin] >= 2.0:   # label real dips only (dip_from_meandc threshold)
                ax.annotate(f"{100 - body[tmin]:.1f}%", (tmin, body[tmin]), xytext=(8 + 34 * i, -14),
                            textcoords="offset points", color=INK, fontsize=8)
    for cam, ax in axes.items():
        if not ax.lines:   # all-dark camera: analyze_drift_scan yields no mean_dc
            ax.text(0.5, 0.5, "no signal (all-dark)", transform=ax.transAxes,
                    ha="center", va="center", color=INK2, fontsize=9)
        ax.set_title(f"cam {cam + 1}", loc="left", color=INK, fontsize=10)
        ax.axhline(100, color=INK2, lw=0.6, ls=(0, (2, 3)))
        ax.axvspan(60, 420, color="#f0efe9", zorder=0)
        ax.grid(axis="y", color="#e6e5df", lw=0.6)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
        ax.tick_params(colors=INK2, labelsize=8)
    for cam in bottom_row:
        axes[cam].set_xlabel("time since scan start (s)", color=INK2, fontsize=9)
    for cam in left_col:
        axes[cam].set_ylabel("% of own plateau (dark-corrected)", color=INK2, fontsize=9)
    h, l = axes[cams[0]].get_legend_handles_labels()
    h.append(plt.Line2D([], [], color=HIST_GREY, lw=1)); l.append(f"historical controls (n={n_hist})")
    fig.legend(h, l, loc="upper left", bbox_to_anchor=(0.005, 0.955), frameon=False, fontsize=9, ncol=len(l))
    fig.suptitle(f"Cold-start warm-up dip vs. historical controls (2-h rig-off soak, mask {mask_label}; "
                 "source-step intervals masked)", x=0.01, ha="left", color=INK, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(out_png, dpi=130)
    print("wrote", out_png)


if __name__ == "__main__":
    main()
