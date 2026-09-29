#!/usr/bin/env python3
"""Root-cause fingerprinting for the cold-start warmup dip.

Compares the full 1024-bin histogram of one camera during the dip against
the same camera at plateau, three ways:

1. Overlaid average histograms (log-y) -- gross shape check.
2. Quantile-quantile plot with two competing model fits:
     multiplicative about the pedestal:  q_dip - PED = m * (q_plateau - PED)
       (gain / QE / exposure change -- histogram compresses toward pedestal)
     additive offset:                    q_dip = q_plateau + c
       (black-level / offset error -- histogram translates)
   Whichever fits the quantiles with smaller residual is the better story.
3. Contrast timeline (std_dc / mean_dc from the analysis CSV): constant
   contrast through the dip => multiplicative; contrast spike => additive;
   erratic => shape change (optical).

Usage:
    python bench/analyze_dip_histogram.py --subject-id DRIFT30C --cam 7 \
        --dip-window 113 173 --plateau-window 1140 1260
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

BIN_COLS = [str(i) for i in range(1024)]
BINS = np.arange(1024, dtype=np.float64)
PEDESTAL_DN = 128.0
LIGHT_THRESHOLD_DN = 133.0
CHUNK_SIZE = 20_000

parser = argparse.ArgumentParser()
parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"))
parser.add_argument("--subject-id", default="DRIFT30C")
parser.add_argument("--cam", type=int, default=7, help="1-indexed camera.")
parser.add_argument("--dip-window", type=float, nargs=2, default=[113.0, 173.0], metavar=("LO", "HI"))
parser.add_argument("--plateau-window", type=float, nargs=2, default=[1140.0, 1260.0], metavar=("LO", "HI"))
args = parser.parse_args()

cam_id = args.cam - 1

import json
meta = json.loads((args.data_dir / f"{args.subject_id}_drift_meta.json").read_text())
raw_csv = meta["raw_csv_path"]

# ---- accumulate average histograms for the two windows ----
h_dip = np.zeros(1024)
h_plat = np.zeros(1024)
n_dip = n_plat = 0

usecols = ["cam_id", "timestamp_s", "sum", *BIN_COLS]
for chunk in pd.read_csv(raw_csv, usecols=usecols, chunksize=CHUNK_SIZE):
    sel = chunk[chunk["cam_id"] == cam_id]
    if sel.empty:
        continue
    bins_arr = sel[BIN_COLS].to_numpy(dtype=np.float64)
    total = sel["sum"].to_numpy(dtype=np.float64)
    u1 = (bins_arr @ BINS) / np.where(total > 0, total, 1.0)
    t = sel["timestamp_s"].to_numpy()
    light = u1 > LIGHT_THRESHOLD_DN

    m_dip = light & (t >= args.dip_window[0]) & (t <= args.dip_window[1])
    m_plat = light & (t >= args.plateau_window[0]) & (t <= args.plateau_window[1])
    if m_dip.any():
        h_dip += bins_arr[m_dip].sum(axis=0)
        n_dip += int(m_dip.sum())
    if m_plat.any():
        h_plat += bins_arr[m_plat].sum(axis=0)
        n_plat += int(m_plat.sum())

print(f"[+] {n_dip} dip frames, {n_plat} plateau frames (cam {args.cam})")
p_dip = h_dip / h_dip.sum()
p_plat = h_plat / h_plat.sum()

def stats(p):
    u1 = (BINS * p).sum()
    u2 = (BINS ** 2 * p).sum()
    sd = max(u2 - u1 ** 2, 0) ** 0.5
    return u1, sd

u1_d, sd_d = stats(p_dip)
u1_p, sd_p = stats(p_plat)
print(f"    dip:     mean={u1_d:.2f}  std={sd_d:.2f}  contrast(ped-sub)={sd_d/(u1_d-PEDESTAL_DN):.4f}")
print(f"    plateau: mean={u1_p:.2f}  std={sd_p:.2f}  contrast(ped-sub)={sd_p/(u1_p-PEDESTAL_DN):.4f}")

# ---- quantiles ----
probs = np.linspace(0.01, 0.99, 99)
cdf_d = np.cumsum(p_dip)
cdf_p = np.cumsum(p_plat)
q_d = np.interp(probs, cdf_d, BINS)
q_p = np.interp(probs, cdf_p, BINS)

# multiplicative-about-pedestal fit
x = q_p - PEDESTAL_DN
y = q_d - PEDESTAL_DN
m = float((x * y).sum() / (x * x).sum())
res_mult = np.sqrt(np.mean((y - m * x) ** 2))
# additive fit
c = float(np.mean(q_d - q_p))
res_add = np.sqrt(np.mean((q_d - (q_p + c)) ** 2))
print(f"    multiplicative fit: gain m={m:.4f}, RMS residual={res_mult:.2f} DN")
print(f"    additive fit:       offset c={c:.2f} DN, RMS residual={res_add:.2f} DN")

# ---- contrast timeline from analysis CSV ----
an = pd.read_csv(args.data_dir / f"{args.subject_id}_analysis.csv",
                 usecols=["cam_id", "timestamp_s", "is_dark", "mean_dc", "std_dc"])
d = an[(an["cam_id"] == cam_id) & (~an["is_dark"])].sort_values("timestamp_s").copy()
d = d.dropna(subset=["mean_dc", "std_dc"])
d = d[d["mean_dc"] > 1.0]
d["tb"] = (d["timestamp_s"] // 10.0) * 10.0 + 5.0
g = d.groupby("tb")[["mean_dc", "std_dc"]].median()
contrast = g["std_dc"] / g["mean_dc"]
plateau_mean = g["mean_dc"][(g.index >= 600)].median()

# ---- figure ----
fig, (axa, axb, axc) = plt.subplots(1, 3, figsize=(16, 5))

axa.semilogy(BINS, p_plat, lw=1, label=f"plateau ({args.plateau_window[0]:.0f}-{args.plateau_window[1]:.0f}s)")
axa.semilogy(BINS, p_dip, lw=1, label=f"dip ({args.dip_window[0]:.0f}-{args.dip_window[1]:.0f}s)")
axa.set_xlabel("Pixel value (DN)")
axa.set_ylabel("Probability")
axa.set_title(f"Cam {args.cam} average histogram")
axa.legend(fontsize=9)
axa.grid(True, alpha=0.3)

axb.plot(q_p, q_d, "o", ms=3, label="quantiles")
axb.plot(q_p, PEDESTAL_DN + m * (q_p - PEDESTAL_DN), "-", lw=1,
         label=f"multiplicative m={m:.3f} (rms {res_mult:.2f})")
axb.plot(q_p, q_p + c, "--", lw=1, label=f"additive c={c:+.1f} DN (rms {res_add:.2f})")
axb.plot(q_p, q_p, ":", lw=0.8, color="gray", label="y=x")
axb.set_xlabel("Plateau quantile (DN)")
axb.set_ylabel("Dip quantile (DN)")
axb.set_title("Q-Q: dip vs plateau")
axb.legend(fontsize=8)
axb.grid(True, alpha=0.3)

axc2 = axc.twinx()
axc.plot(g.index / 60.0, g["mean_dc"] / plateau_mean, color="#2a78d6", lw=1.2, label="mean_dc / plateau")
axc2.plot(contrast.index / 60.0, contrast, color="#e34948", lw=1.2, label="contrast (std/mean)")
axc.set_xlabel("Time (min)")
axc.set_ylabel("mean_dc / plateau", color="#2a78d6")
axc2.set_ylabel("contrast", color="#e34948")
axc.set_xlim(0, 15)
axc.set_title("Dip vs contrast timeline")
axc.grid(True, alpha=0.3)

fig.suptitle(f"{args.subject_id} cam {args.cam}: dip histogram fingerprint", y=1.0)
fig.tight_layout()
out = args.data_dir / f"{args.subject_id}_cam{args.cam}_dip_fingerprint.png"
fig.savefig(out, dpi=150, bbox_inches="tight")
print(f"[+] Saved {out}")
