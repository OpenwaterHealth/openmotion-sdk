#!/usr/bin/env python3
"""Downsample DRIFT30 analysis + thorlabs CSVs into a compact JSON for the
interactive drift-scan widget. Not part of the acquisition/analysis pair --
a one-off bridge from analyze_drift_scan.py's output to a Chart.js dashboard.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

DATA_DIR = Path("bench/drift_scan_out")
SUBJECT = "DRIFT30"
N_BINS = 120  # points per series in the time-series charts

df = pd.read_csv(DATA_DIR / f"{SUBJECT}_analysis.csv")
meta = json.loads((DATA_DIR / f"{SUBJECT}_drift_meta.json").read_text())
thorlabs = pd.read_csv(DATA_DIR / f"{SUBJECT}_thorlabs.csv")

duration = meta["duration_sec"]
bin_edges = np.linspace(0, duration, N_BINS + 1)
bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2

def binned_mean(t, v, edges):
    idx = np.digitize(t, edges) - 1
    idx = np.clip(idx, 0, len(edges) - 2)
    out = np.full(len(edges) - 1, np.nan)
    for b in range(len(edges) - 1):
        sel = v[idx == b]
        if len(sel):
            out[b] = np.nanmean(sel)
    return out

cams = {}
for cam_id in range(8):
    sub = df[df["cam_id"] == cam_id].sort_values("timestamp_s")
    light = sub[sub["is_dark"] == False]  # noqa: E712
    t = light["timestamp_s"].to_numpy()

    raw_mean = binned_mean(t, light["u1"].to_numpy(), bin_edges)
    raw_var = np.maximum(light["u2"].to_numpy() - light["u1"].to_numpy() ** 2, 0.0)
    raw_std = binned_mean(t, np.sqrt(raw_var), bin_edges)
    corr_mean = binned_mean(t, light["mean_dc"].to_numpy(), bin_edges)
    corr_std = binned_mean(t, light["std_dc"].to_numpy(), bin_edges)
    temp = binned_mean(sub["timestamp_s"].to_numpy(), sub["temperature"].to_numpy(), bin_edges)

    def clean(arr, nd=1):
        return [None if (x is None or (isinstance(x, float) and np.isnan(x))) else round(float(x), nd) for x in arr]

    cams[cam_id] = {
        "raw_mean": clean(raw_mean),
        "raw_std": clean(raw_std),
        "corr_mean": clean(corr_mean),
        "corr_std": clean(corr_std),
        "temperature": clean(temp),
        "last_t": round(float(sub["timestamp_s"].max()), 1),
    }

# Dark-event drift (already sparse -- one row per event per camera)
dark_drift = {cam_id: {"t": [], "mean": [], "std": []} for cam_id in range(8)}
for cam_id in range(8):
    sub = df[(df["cam_id"] == cam_id) & (df["is_dark"] == True)]  # noqa: E712
    grp = sub.groupby("dark_event_idx")
    for ev_idx, g in grp:
        if ev_idx < 0:
            continue
        w = g["total"].to_numpy()
        w_sum = w.sum()
        u1 = float((g["u1"].to_numpy() * w).sum() / w_sum)
        u2 = float((g["u2"].to_numpy() * w).sum() / w_sum)
        var = max(0.0, u2 - u1 ** 2)
        dark_drift[cam_id]["t"].append(round(float(g["timestamp_s"].mean()), 1))
        dark_drift[cam_id]["mean"].append(round(u1, 1))
        dark_drift[cam_id]["std"].append(round(var ** 0.5, 1))

# Thorlabs -- convert W to uW so we can round to 2 decimals instead of 8
tl_edges = np.linspace(0, thorlabs["elapsed_s"].max(), N_BINS + 1)
tl_binned = binned_mean(thorlabs["elapsed_s"].to_numpy(), thorlabs["power"].to_numpy() * 1e6, tl_edges)
tl_centers = (tl_edges[:-1] + tl_edges[1:]) / 2

out = {
    "duration_sec": round(duration, 1),
    "bin_centers": [round(float(x), 1) for x in bin_centers],
    "cams": cams,
    "dark_drift": dark_drift,
    "thorlabs": {
        "t": [round(float(x), 1) for x in tl_centers],
        "power_uw": [None if np.isnan(x) else round(float(x), 2) for x in tl_binned],
    },
    "camera_dropout_note": "cam 4 (cam_id=3) stopped streaming at t=1550.99s",
}

out_path = DATA_DIR / f"{SUBJECT}_widget_data.json"
out_path.write_text(json.dumps(out, separators=(",", ":")))
print(f"wrote {out_path} ({out_path.stat().st_size / 1024:.1f} KB)")
