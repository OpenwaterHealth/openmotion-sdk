#!/usr/bin/env python3
"""Animate one camera's average histogram through the cold-start warmup dip.

Each GIF frame is the light-frame average histogram over a short time bin
(default 5 s), swept from scan start through the dip and recovery, with the
plateau-average histogram drawn as a static reference. Shows the raw
histogram (no dark correction needed), so the first minute -- before the
first dark anchor -- is visible too.

Usage:
    python bench/make_dip_histogram_gif.py --subject-id DRIFT30C --cam 7
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BIN_COLS = [str(i) for i in range(1024)]
BINS = np.arange(1024, dtype=np.float64)
LIGHT_THRESHOLD_DN = 133.0
CHUNK_SIZE = 20_000

parser = argparse.ArgumentParser()
parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"))
parser.add_argument("--subject-id", default="DRIFT30C")
parser.add_argument("--cam", type=int, default=7, help="1-indexed camera.")
parser.add_argument("--t-end", type=float, default=600.0, help="Animation covers 0..t_end seconds. Default 600.")
parser.add_argument("--bin-sec", type=float, default=5.0, help="Seconds of frames averaged per GIF frame. Default 5.")
parser.add_argument("--plateau-window", type=float, nargs=2, default=[1140.0, 1260.0])
parser.add_argument("--fps", type=int, default=10)
args = parser.parse_args()

cam_id = args.cam - 1
meta = json.loads((args.data_dir / f"{args.subject_id}_drift_meta.json").read_text())
raw_csv = meta["raw_csv_path"]

n_tbins = int(np.ceil(args.t_end / args.bin_sec))
hists = np.zeros((n_tbins, 1024))
counts = np.zeros(n_tbins)
temp_sum = np.zeros(n_tbins)
h_plat = np.zeros(1024)
n_plat = 0

usecols = ["cam_id", "timestamp_s", "temperature", "sum", *BIN_COLS]
for chunk in pd.read_csv(raw_csv, usecols=usecols, chunksize=CHUNK_SIZE):
    sel = chunk[chunk["cam_id"] == cam_id]
    if sel.empty:
        continue
    bins_arr = sel[BIN_COLS].to_numpy(dtype=np.float64)
    total = sel["sum"].to_numpy(dtype=np.float64)
    u1 = (bins_arr @ BINS) / np.where(total > 0, total, 1.0)
    t = sel["timestamp_s"].to_numpy()
    temps = sel["temperature"].to_numpy(dtype=np.float64)
    light = u1 > LIGHT_THRESHOLD_DN

    m_anim = light & (t >= 0) & (t < args.t_end)
    if m_anim.any():
        idx = (t[m_anim] // args.bin_sec).astype(int)
        np.add.at(hists, idx, bins_arr[m_anim])
        np.add.at(counts, idx, 1.0)
        np.add.at(temp_sum, idx, temps[m_anim])

    m_plat = light & (t >= args.plateau_window[0]) & (t <= args.plateau_window[1])
    if m_plat.any():
        h_plat += bins_arr[m_plat].sum(axis=0)
        n_plat += int(m_plat.sum())

print(f"[+] {int(counts.sum())} animation frames binned into {n_tbins} steps; {n_plat} plateau frames")
p_plat = h_plat / h_plat.sum()
plat_mean = float((BINS * p_plat).sum())

# x-limit: a little past the plateau distribution's extreme tail
xmax = float(BINS[np.nonzero(p_plat > 1e-9)[0].max()]) * 1.15

fig, ax = plt.subplots(figsize=(9, 5))
ax.semilogy(BINS, np.maximum(p_plat, 1e-12), color="gray", lw=1.2, alpha=0.7,
            label=f"plateau reference (mean {plat_mean:.0f} DN)")
line, = ax.semilogy([], [], color="#eb6834", lw=1.5, label="current window")
mean_line = ax.axvline(plat_mean, color="#eb6834", lw=0.8, ls="--", alpha=0.6)
title = ax.set_title("")
ax.set_xlim(0, xmax)
ax.set_ylim(1e-7, 0.1)
ax.set_xlabel("Pixel value (DN)")
ax.set_ylabel("Probability")
ax.legend(loc="upper right", fontsize=9)
ax.grid(True, alpha=0.3)

valid_bins = [k for k in range(n_tbins) if counts[k] > 0]

def update(frame_k):
    k = valid_bins[frame_k]
    p = hists[k] / hists[k].sum()
    line.set_data(BINS, np.maximum(p, 1e-12))
    mu = float((BINS * p).sum())
    mean_line.set_xdata([mu, mu])
    tc = temp_sum[k] / counts[k]
    title.set_text(f"{args.subject_id} cam {args.cam}   t = {k * args.bin_sec:5.0f} s"
                   f"   mean = {mu:6.1f} DN ({mu / plat_mean * 100:5.1f}% of plateau)   die = {tc:5.1f} °C")
    return line, mean_line, title

anim = animation.FuncAnimation(fig, update, frames=len(valid_bins), blit=False)
out = args.data_dir / f"{args.subject_id}_cam{args.cam}_warmup_histogram.gif"
anim.save(out, writer=animation.PillowWriter(fps=args.fps))
print(f"[+] Saved {out} ({out.stat().st_size / 1e6:.1f} MB, {len(valid_bins)} frames @ {args.fps} fps)")
