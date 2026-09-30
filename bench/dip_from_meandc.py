#!/usr/bin/env python3
"""Measure the warm-up dip on the DARK-CORRECTED CAMERA SIGNAL (mean_dc), NOT the
photodiode-normalized mean_norm.

Why: the Thorlabs photodiode exhibits intermittent ~5% discrete output steps that
the cameras do not track (e.g. SHORTDIP_01: +4.8% for one 60 s dark-window
interval). Dividing by it (mean_norm) manufactures false dips. mean_dc is the
camera's own dark-corrected response; we use the photodiode only to CONFIRM the
source was stable across a candidate dip (so a real mean_dc dip isn't just the
source dimming).

For each run's {subject}_analysis.csv: per camera, smooth mean_dc, take the
plateau (median 600-1500 s), find the minimum in 60-420 s, and report the dip
depth. Flag it CONFIRMED only if the photodiode drift between dip and plateau is
small (< pd-tol), else PHOTODIODE-SUSPECT.

Usage:
  python bench/dip_from_meandc.py <analysis_dir> <subj1> [subj2 ...]
"""
import sys
from pathlib import Path

import numpy as np
import pandas as pd

PLATEAU_LO, PLATEAU_HI = 600.0, 1500.0
DIP_LO, DIP_HI = 60.0, 420.0
PD_TOL_PCT = 1.5   # allowed |photodiode(dip) - photodiode(plateau)| for a CONFIRMED dip
MIN_COVERAGE = 0.8  # fraction of dip-window light frames that must have a mean_dc


def main():
    adir = Path(sys.argv[1])
    subjects = sys.argv[2:]
    print(f"{'run':>14} {'cam':>3} {'plat_dc':>7} {'dip_dc':>7} {'dip%':>6} {'t_min':>6} "
          f"{'pd_dip':>7} {'pd_plat':>7} {'pd_d%':>6} {'verdict':>16}")
    for subj in subjects:
        f = adir / f"{subj}_analysis.csv"
        if not f.exists():
            print(f"{subj:>14}  (no analysis csv)")
            continue
        df = pd.read_csv(f, usecols=["cam_id", "timestamp_s", "is_dark", "mean_dc", "photodiode_w"])
        for cam in sorted(df.cam_id.unique()):
            g = df[(df.cam_id == cam) & (~df.is_dark)].sort_values("timestamp_s")
            gd = g.dropna(subset=["mean_dc"])
            if len(gd) < 3000:
                continue
            s = gd.set_index("timestamp_s").mean_dc.rolling(120, center=True, min_periods=15).median()
            plat = s[(s.index >= PLATEAU_LO) & (s.index <= PLATEAU_HI)].median()
            body = s[(s.index >= DIP_LO) & (s.index <= DIP_HI)]
            # Coverage of the dip window: light frames there that actually have a
            # mean_dc. analyze_drift_scan leaves gaps where it could not resolve a
            # dark window; a min over a half-empty window is not a dip measurement.
            in_win = g[(g.timestamp_s >= DIP_LO) & (g.timestamp_s <= DIP_HI)]
            coverage = in_win.mean_dc.notna().mean() if len(in_win) else 0.0
            if coverage < MIN_COVERAGE:
                print(f"{subj:>14} {cam+1:>3}  dip window only {100 * coverage:.0f}% covered by mean_dc "
                      f"-- {'INSUFFICIENT-COVERAGE':>16}")
                continue
            if body.empty or not np.isfinite(plat) or plat <= 0:
                continue
            dmin = body.min(); tmin = body.idxmin()
            dip = (plat - dmin) / plat * 100
            # photodiode at dip vs plateau (source stability check)
            pdser = g.dropna(subset=["photodiode_w"])
            pdser = pdser[pdser.photodiode_w > 10e-6].set_index("timestamp_s").photodiode_w
            pd_dip = pdser[(pdser.index >= tmin - 20) & (pdser.index <= tmin + 20)].median() * 1e6
            pd_plat = pdser[(pdser.index >= PLATEAU_LO) & (pdser.index <= PLATEAU_HI)].median() * 1e6
            pd_delta = 100 * (pd_dip - pd_plat) / pd_plat if pd_plat else float("nan")
            if dip < 2.0:
                verdict = "no-dip"
            elif abs(pd_delta) <= PD_TOL_PCT:
                verdict = "CONFIRMED"
            else:
                verdict = "PHOTODIODE-SUSPECT"
            print(f"{subj:>14} {cam+1:>3} {plat:>7.1f} {dmin:>7.1f} {dip:>6.1f} {tmin:>6.0f} "
                  f"{pd_dip:>7.1f} {pd_plat:>7.1f} {pd_delta:>+6.1f} {verdict:>16}")


if __name__ == "__main__":
    main()
