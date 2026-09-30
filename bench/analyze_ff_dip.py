#!/usr/bin/env python3
"""Analyze one ff_dip_capture step: dip curve + spatial dip map from full frames.

Input: a step directory written by bench/ff_dip_capture.py
(soak/ with run.json + images/left/camN/*.npz + camera_telemetry.csv,
dark_events.csv, thorlabs.csv).

Per imaged camera:
  * Dark rows = composite rows whose arrival time falls inside a logged
    Keysight source-off span (validated to line up within <0.05 s). They are
    excluded from the lit signal and give the dark pedestal: per-window median
    of the dark rows' means, linearly interpolated in time (the histogram
    analysis's method); windows with < MIN_DARK_ROWS rows are skipped.
  * Lit signal per composite = mean over lit rows of (row mean - pedestal(t)),
    central columns only (COL_LO:COL_HI); timestamped at the median lit-row time
    relative to imaging start (t0). Composites overlapping a source step
    (photodiode > 2 % off the plateau level) are rescaled by the photodiode ratio.
  * Dip = min over DIP window of the smoothed trace vs the plateau median.
  * Spatial map: mean dark-corrected image over the composites around the
    trough (+-TROUGH_HALF_S) divided by the mean over the plateau window,
    block-averaged to BLOCK x BLOCK pixels -> where on the sensor the loss is.

Outputs in <step_dir>/analysis/: ff_dip_summary.csv, ff_dip_trace.csv,
ff_dip_<cam>.png (trace + temperature + plateau / trough / ratio maps).

Usage:
  python bench/analyze_ff_dip.py bench/ff_dip_out/FFDIP_01_cams12
  python bench/analyze_ff_dip.py <step_dir> --plateau 120 220 --dip 5 110   # short validation runs
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

COL_LO, COL_HI = 200, 1720          # central columns for the trace (avoid edge vignetting)
MIN_DARK_ROWS = 20
DARK_PRE_S = 0.05                   # before a logged source-off
DARK_POST_S = 0.20                  # after a logged source-on: exposures integrating across the turn-on are
                                    # only partly lit (thin dark lines every STRIDE rows otherwise)
STEP_TOL_PCT = 2.0
TROUGH_HALF_S = 20.0
BLOCK = 32
INK, INK2 = "#1f1f1c", "#6b6a63"


def parse():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step_dir", type=Path)
    ap.add_argument("--plateau", nargs=2, type=float, default=(600.0, 1500.0), metavar=("LO", "HI"))
    ap.add_argument("--dip", nargs=2, type=float, default=(0.0, 420.0), metavar=("LO", "HI"),
                    help="window searched for the trough, s after imaging start")
    return ap.parse_args()


def load_step(step: Path):
    meta = json.loads((step / "capture_meta.json").read_text())
    run = json.loads((step / "soak" / "run.json").read_text())
    start = dt.datetime.fromisoformat(run["start"]).timestamp()
    darks = pd.read_csv(step / "dark_events.csv")
    darks["on_epoch"] = darks["on_epoch"].fillna(darks["off_epoch"] + 1e6)   # tail dark: open-ended
    pd_log = pd.read_csv(step / "thorlabs.csv") if (step / "thorlabs.csv").exists() else None
    tel = pd.read_csv(step / "soak" / "camera_telemetry.csv")
    return meta, start, darks, pd_log, tel


def source_factor(pd_log: pd.DataFrame | None, darks: pd.DataFrame, t0: float, plateau: tuple):
    """Per-interval source-step correction from the photodiode.

    The Keysight source sometimes returns from a dark window ~5 % brighter or
    dimmer and holds that level for one OR SEVERAL intervals; cameras track the
    photodiode exactly (checked on 17 histogram runs + FFVAL_02). Each interval
    between dark windows gets its photodiode median; intervals more than
    STEP_TOL_PCT off the reference level (median over the plateau window's
    intervals) are rescaled to it, the rest are left untouched (factor 1).
    Returns (factor(t_epoch) -> float, table)."""
    if pd_log is None or pd_log.empty:
        return (lambda t: 1.0), pd.DataFrame()
    edges = sorted(darks["on_epoch"].clip(upper=pd_log.epoch_s.max()).tolist())
    lit = pd_log[pd_log.power_w > 10e-6]
    rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        s_ = lit[(lit.epoch_s > a + 2.5) & (lit.epoch_s < b - 0.5)].power_w   # skip post-toggle settling
        rows.append({"a": a, "b": b, "level": s_.median() if len(s_) else np.nan})
    tab = pd.DataFrame(rows)
    mid = (tab.a + tab.b) / 2 - t0
    in_plat = tab[(mid >= plateau[0]) & (mid <= plateau[1])].level
    ref = in_plat.median() if in_plat.notna().any() else tab.level.median()
    tab["dev_pct"] = 100 * (tab.level / ref - 1)
    tab["factor"] = np.where(tab.dev_pct.abs() > STEP_TOL_PCT, tab.level / ref, 1.0)

    def factor(t):
        hit = tab[(tab.a <= t) & (t < tab.b)]
        return float(hit.factor.iloc[0]) if len(hit) and np.isfinite(hit.factor.iloc[0]) else 1.0
    return factor, tab


def analyze_cam(files, start, t0, darks, factor):
    in_dark_span = lambda t: np.any([(t >= r.off_epoch - DARK_PRE_S) & (t <= r.on_epoch + DARK_POST_S)
                                     for r in darks.itertuples()], axis=0)
    comps = []           # per composite: epoch times, row means, dark mask, image path
    for f in files:
        z = np.load(f)
        t = start + z["row_t_s"]
        rm = z["image"][:, COL_LO:COL_HI].astype(np.float32).mean(axis=1)
        comps.append((f, t, rm, in_dark_span(t)))
    # pedestal per dark window from the dark rows of every composite
    ped = []
    for r in darks.itertuples():
        vals = np.concatenate([rm[(t >= r.off_epoch) & (t <= min(r.on_epoch, r.off_epoch + 5)) & dk]
                               for _, t, rm, dk in comps]) if comps else np.array([])
        vals = vals[vals < np.median(vals) + 20] if vals.size else vals   # drop stray lit edge rows
        if vals.size >= MIN_DARK_ROWS:
            ped.append((r.off_epoch + 0.5, float(np.median(vals)), int(vals.size)))
    ped = pd.DataFrame(ped, columns=["epoch", "pedestal", "n_rows"])
    if ped.empty:
        raise RuntimeError("no dark window captured enough dark rows for a pedestal")

    def pedestal(t):
        return np.interp(t, ped.epoch, ped.pedestal)          # flat extrapolation at the ends

    rows = []
    for f, t, rm, dk in comps:
        lit = ~dk
        if lit.sum() < 400:
            continue
        tc = float(np.median(t[lit]))
        k = factor(tc)
        rows.append({"file": f, "t_s": tc - t0, "lit_dc": float(np.mean(rm[lit] - pedestal(t[lit]))) / k,
                     "n_lit_rows": int(lit.sum()), "n_dark_rows": int(dk.sum()), "source_factor": k})
    return pd.DataFrame(rows).sort_values("t_s"), ped, pedestal


def mean_image(files_t, pedestal, start, darks, factor):
    """Mean dark-corrected image over composites, dark rows excluded (NaN) -> nanmean."""
    acc, n = None, None
    for f in files_t:
        z = np.load(f)
        t = start + z["row_t_s"]
        img = z["image"].astype(np.float32) - pedestal(t)[:, None]
        img /= factor(float(np.median(t)))
        dk = np.any([(t >= r.off_epoch - DARK_PRE_S) & (t <= r.on_epoch + DARK_POST_S)
                     for r in darks.itertuples()], axis=0)
        img[dk, :] = np.nan
        ok = ~np.isnan(img)
        acc = np.where(ok, img, 0.0) if acc is None else acc + np.where(ok, img, 0.0)
        n = ok.astype(np.float32) if n is None else n + ok
    return acc / np.where(n > 0, n, np.nan)


def block_mean(img, b=BLOCK):
    h, w = (img.shape[0] // b) * b, (img.shape[1] // b) * b
    return np.nanmean(img[:h, :w].reshape(h // b, b, w // b, b), axis=(1, 3))


def main():
    a = parse()
    step = a.step_dir.resolve()
    out = step / "analysis"
    out.mkdir(exist_ok=True)
    meta, start, darks, pd_log, tel = load_step(step)
    t0 = meta["t0_epoch"]
    factor, steps = source_factor(pd_log, darks, t0, a.plateau)
    if not steps.empty and (steps.factor != 1).any():
        st = steps[steps.factor != 1]
        print("source steps corrected (s after t0: %):",
              [(round(r.a - t0), round(r.b - t0), round(r.dev_pct, 1)) for r in st.itertuples()])
        steps.assign(a_s=steps.a - t0, b_s=steps.b - t0).to_csv(out / "source_levels.csv", index=False)
    summary, traces = [], []
    for cam in meta["cams_0based"]:
        files = sorted(glob.glob(str(step / "soak" / "images" / "left" / f"cam{cam}" / "*.npz")))
        if not files:
            print(f"cam {cam + 1}: no images")
            continue
        tr, ped, pedestal = analyze_cam(files, start, t0, darks, factor)
        good = tr.set_index("t_s")
        smooth = good.lit_dc.rolling(3, center=True, min_periods=1).median()
        plat = smooth[(smooth.index >= a.plateau[0]) & (smooth.index <= a.plateau[1])].median()
        body = smooth[(smooth.index >= a.dip[0]) & (smooth.index <= a.dip[1])]
        t_min = float(body.idxmin())
        dip = 100 * (1 - body.min() / plat)
        # spatial: trough composites vs plateau composites
        tro_f = tr[tr.t_s.between(t_min - TROUGH_HALF_S, t_min + TROUGH_HALF_S)].file
        pla_f = tr[tr.t_s.between(*a.plateau)].file
        img_tro = mean_image(tro_f, pedestal, start, darks, factor)
        img_pla = mean_image(pla_f, pedestal, start, darks, factor)
        ratio = block_mean(img_tro) / block_mean(img_pla)
        loss = 100 * (1 - ratio)
        core = loss[2:-2, 2:-2]
        summary.append({"cam": cam + 1, "plateau_dc": round(plat, 2), "dip_pct": round(dip, 2), "t_min_s": round(t_min),
                        "n_composites": len(tr), "n_trough_comps": len(tro_f), "n_plateau_comps": len(pla_f),
                        "pedestal_median": round(ped.pedestal.median(), 2), "n_dark_windows_used": len(ped),
                        "loss_map_p5": round(float(np.nanpercentile(core, 5)), 2),
                        "loss_map_p95": round(float(np.nanpercentile(core, 95)), 2),
                        "loss_map_std": round(float(np.nanstd(core)), 2)})
        tr.assign(cam=cam + 1, pct_of_plateau=100 * tr.lit_dc / plat).drop(columns="file").to_csv(
            out / f"ff_dip_trace_cam{cam + 1}.csv", index=False)
        traces.append((cam, tr, plat, t_min, dip))
        # figure: trace + temperature, then plateau / trough / loss map
        fig = plt.figure(figsize=(13, 7.2))
        gs = fig.add_gridspec(2, 3, height_ratios=[1, 1.15])
        ax = fig.add_subplot(gs[0, :2])
        ax.plot(tr.t_s, 100 * tr.lit_dc * tr.source_factor / plat, color="#b9b8b0", lw=1, label="uncorrected")
        ax.plot(good.index, 100 * smooth / plat, color="#2a78d6", lw=2, label="source-step corrected, 3-pt median")
        ax.axhline(100, color=INK2, lw=0.6, ls=(0, (2, 3)))
        ax.axvline(t_min, color=INK2, lw=0.6)
        ax.set_ylabel("% of plateau (dark-corrected)", color=INK2, fontsize=9)
        ax.set_title(f"cam {cam + 1}: dip {dip:.1f}% at {t_min:.0f} s", loc="left", color=INK, fontsize=11)
        ax.legend(frameon=False, fontsize=8, loc="lower right")
        axt = fig.add_subplot(gs[0, 2])
        tt = tel[tel.cam == cam]
        axt.plot(tt.t_s - (t0 - start), tt.tpm_avg_c, color="#eb6834", lw=1.5)
        axt.set_title("die temperature (°C)", loc="left", color=INK, fontsize=10)
        axt.set_xlabel("s since imaging start", color=INK2, fontsize=9)
        for x in (ax, axt):
            x.spines[["top", "right"]].set_visible(False)
            x.tick_params(colors=INK2, labelsize=8)
            x.grid(axis="y", color="#e6e5df", lw=0.6)
        ax.set_xlabel("s since imaging start", color=INK2, fontsize=9)
        vmax = np.nanpercentile(img_pla, 99)
        for j, (im, title, kw) in enumerate([
                (img_pla, f"plateau mean ({len(pla_f)} comps), DN", dict(cmap="gray", vmin=0, vmax=vmax)),
                (img_tro, f"trough mean ({len(tro_f)} comps), DN", dict(cmap="gray", vmin=0, vmax=vmax)),
                (loss, f"loss at trough, % ({BLOCK}px blocks)", dict(cmap="magma", vmin=max(0, np.nanpercentile(core, 1)),
                                                                      vmax=np.nanpercentile(core, 99)))]):
            axi = fig.add_subplot(gs[1, j])
            h = axi.imshow(im, aspect="auto", **kw)
            axi.set_title(title, loc="left", color=INK, fontsize=9)
            axi.set_xticks([]); axi.set_yticks([])
            fig.colorbar(h, ax=axi, fraction=0.046, pad=0.02).ax.tick_params(labelsize=7)
        fig.tight_layout()
        fig.savefig(out / f"ff_dip_cam{cam + 1}.png", dpi=110)
        plt.close(fig)
        print(f"cam {cam + 1}: dip {dip:.1f}% @ {t_min:.0f}s, plateau {plat:.1f} DN, {len(tr)} composites, "
              f"pedestal {ped.pedestal.median():.2f} from {len(ped)} windows; loss map p5-p95 "
              f"{np.nanpercentile(core, 5):.1f}-{np.nanpercentile(core, 95):.1f}%")
    pd.DataFrame(summary).to_csv(out / "ff_dip_summary.csv", index=False)
    print("wrote", out)


if __name__ == "__main__":
    main()
