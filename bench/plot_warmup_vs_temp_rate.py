#!/usr/bin/env python3
"""Warmup-transient diagnostic: for cameras that show the early dip-then-rise
curve in photodiode-normalized intensity, overlay mean_norm (DN/uW) with the
RATE of change of that camera's die temperature (dT/dt, C/min).

If the dip tracks dT/dt (rate), the suppression is a transient/gradient
effect (thermal gradients across the die/optics while heating fast). If it
instead tracks absolute temperature, the sensitivity-vs-temperature curve
itself is non-monotonic. Reads {subject}_analysis.csv from
analyze_drift_scan.py.
"""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BIN_S = 5.0  # time-bin width for both series

parser = argparse.ArgumentParser()
parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"))
parser.add_argument("--subject-id", default="DRIFT30C")
parser.add_argument("--cams", default="7,8", help="1-indexed camera list, comma-separated. Default: 7,8.")
parser.add_argument("--tmax-min", type=float, default=15.0, help="x-axis limit in minutes. Default: 15.")
args = parser.parse_args()

cams = [int(c) for c in args.cams.split(",")]

df = pd.read_csv(args.data_dir / f"{args.subject_id}_analysis.csv",
                 usecols=["cam_id", "timestamp_s", "is_dark", "mean_norm", "temperature"])

fig, axes = plt.subplots(len(cams), 1, figsize=(12, 3.4 * len(cams)), sharex=True)
if len(cams) == 1:
    axes = [axes]

for ax, cam1 in zip(axes, cams):
    d = df[df["cam_id"] == cam1 - 1].sort_values("timestamp_s").copy()
    d["tb"] = (d["timestamp_s"] // BIN_S) * BIN_S + BIN_S / 2

    # normalized intensity: light frames only (mean_norm > 0 excludes the
    # forced-zero dark/transition frames)
    light = d[(~d["is_dark"]) & (d["mean_norm"] > 0)]
    mn = light.groupby("tb")["mean_norm"].median()

    # temperature -> smoothed -> time derivative in C/min
    tc = d.groupby("tb")["temperature"].median()
    tc_smooth = tc.rolling(5, center=True, min_periods=1).mean()
    dTdt = np.gradient(tc_smooth.to_numpy(), tc.index.to_numpy()) * 60.0

    ax.plot(mn.index / 60.0, mn.values, color="#2a78d6", lw=1.4, label="mean_norm (DN/µW)")
    ax.set_ylabel("mean_norm (DN/µW)", color="#2a78d6")
    ax.tick_params(axis="y", labelcolor="#2a78d6")
    ax.grid(True, alpha=0.3)

    ax2 = ax.twinx()
    ax2.plot(tc.index / 60.0, dTdt, color="#e34948", lw=1.2, ls="--", label="dT/dt (°C/min)")
    ax2.axhline(0, color="#e34948", lw=0.6, alpha=0.4)
    ax2.set_ylabel("dT/dt (°C/min)", color="#e34948")
    ax2.tick_params(axis="y", labelcolor="#e34948")

    t_final = d["temperature"].iloc[-int(len(d) * 0.05):].median()
    ax.set_title(f"Cam {cam1}  (final die temp ~{t_final:.0f}°C)", fontsize=11)

axes[-1].set_xlabel("Time (min)")
axes[-1].set_xlim(0, args.tmax_min)
fig.suptitle(f"Normalized intensity vs. temperature ramp rate -- {args.subject_id}", y=0.995)
fig.tight_layout()
out = args.data_dir / f"{args.subject_id}_warmup_vs_temp_rate.png"
fig.savefig(out, dpi=150)
print(f"[+] Saved {out}")
plt.show()
