#!/usr/bin/env python3
"""Pop up a matplotlib window: each camera's raw mean intensity divided by
the Thorlabs photodiode reading, both resampled onto a common time grid.
When the photodiode reads near its dark noise floor (light off), the ratio
is forced to zero instead of blowing up on a near-zero denominator."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BIN_COLS = [str(i) for i in range(1024)]
BIN_VALUES = np.arange(1024, dtype=np.float64)
N_CAMERAS = 8
CHUNK_SIZE = 20_000
BIN_WIDTH_S = 3.0
PHOTODIODE_OFF_THRESHOLD_W = 10e-6  # off-state noise floor ~0.2-0.3uW, on-state ~250+uW

parser = argparse.ArgumentParser()
parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"))
parser.add_argument("--subject-id", default="DRIFT30B")
args = parser.parse_args()

meta = json.loads((args.data_dir / f"{args.subject_id}_drift_meta.json").read_text())
raw_csv_path = meta["raw_csv_path"]
duration = meta["duration_sec"]

n_bins = int(duration // BIN_WIDTH_S) + 1
edges = np.arange(0, (n_bins + 1) * BIN_WIDTH_S, BIN_WIDTH_S)

sum_intensity = np.zeros((N_CAMERAS, n_bins))
sum_weight = np.zeros((N_CAMERAS, n_bins))

usecols = ["cam_id", "timestamp_s", "sum", *BIN_COLS]
for chunk in pd.read_csv(raw_csv_path, usecols=usecols, chunksize=CHUNK_SIZE):
    bins_arr = chunk[BIN_COLS].to_numpy(dtype=np.float64)
    total = chunk["sum"].to_numpy(dtype=np.float64)
    safe_total = np.where(total > 0, total, 1.0)
    u1 = (bins_arr @ BIN_VALUES) / safe_total
    u1 = np.where(total > 0, u1, np.nan)

    bin_idx = np.clip((chunk["timestamp_s"].to_numpy() // BIN_WIDTH_S).astype(int), 0, n_bins - 1)
    cam_idx = chunk["cam_id"].to_numpy()

    valid = ~np.isnan(u1)
    np.add.at(sum_intensity, (cam_idx[valid], bin_idx[valid]), u1[valid])
    np.add.at(sum_weight, (cam_idx[valid], bin_idx[valid]), 1.0)

mean_intensity = np.where(sum_weight > 0, sum_intensity / np.maximum(sum_weight, 1), np.nan)

thorlabs = pd.read_csv(args.data_dir / f"{args.subject_id}_thorlabs.csv")
tl_bin_idx = np.clip((thorlabs["elapsed_s"].to_numpy() // BIN_WIDTH_S).astype(int), 0, n_bins - 1)
photodiode_sum = np.zeros(n_bins)
photodiode_count = np.zeros(n_bins)
np.add.at(photodiode_sum, tl_bin_idx, thorlabs["power"].to_numpy())
np.add.at(photodiode_count, tl_bin_idx, 1.0)
photodiode_mean = np.where(photodiode_count > 0, photodiode_sum / np.maximum(photodiode_count, 1), np.nan)

bin_centers_min = (edges[:-1] + edges[1:]) / 2 / 60

light_on = photodiode_mean >= PHOTODIODE_OFF_THRESHOLD_W
ratio = np.zeros_like(mean_intensity)
for cam in range(N_CAMERAS):
    valid = light_on & ~np.isnan(mean_intensity[cam])
    ratio[cam][valid] = mean_intensity[cam][valid] / photodiode_mean[valid]

COLORS = ['#2a78d6', '#1baf7a', '#eda100', '#008300', '#4a3aa7', '#e34948', '#e87ba4', '#eb6834']
fig, ax = plt.subplots(figsize=(13, 6))
for cam in range(N_CAMERAS):
    ax.plot(bin_centers_min, ratio[cam], color=COLORS[cam], lw=1, label=f"Cam {cam + 1}")
ax.set_xlabel("Time (min)")
ax.set_ylabel("Mean intensity / photodiode power (DN / W)")
ax.set_title(f"Camera mean intensity normalized by photodiode reading -- {args.subject_id}\n"
             f"(ratio forced to 0 when photodiode < {PHOTODIODE_OFF_THRESHOLD_W * 1e6:.0f} uW)")
ax.legend(ncol=4, fontsize=9)
ax.grid(True, alpha=0.3)
fig.tight_layout()
plt.show()
