#!/usr/bin/env python3
"""
analyze_drift_scan.py

Post-processing for drift_scan.py's output: reads the SDK's raw per-frame
histogram CSV (1024 bins/frame) plus the JSON sidecar describing when the
external light source was switched off for a dark reference, computes
per-frame mean/std intensity, locates the true dark dip near each scheduled
dark window, and applies the same linear-interpolation dark-pedestal
correction the real pipeline uses (see omotion/pipeline/stages/dark.py,
LinearInterpolation.correct_interval / docs/SciencePipeline.md Sec.8):

    t_frac       = (t - t_dark_prev) / (t_dark_next - t_dark_prev)
    baseline_u1  = u1_prev + t_frac * (u1_next - u1_prev)
    baseline_var = var_prev + t_frac * (var_next - var_prev)
    mean_dc      = raw_u1 - baseline_u1
    std_dc       = sqrt(max(0, raw_var - baseline_var))

One deviation from the real pipeline: the SDK tags "dark" frames itself only
when its own internal laser is blanked on a scheduled trigger position. This
bench rig's illumination is an external Keysight-driven source the SDK has
no notion of, so dark frames are classified per-frame by threshold instead:
a frame is LIGHT iff its mean exceeds pedestal (128 DN) + 5 DN — the same
threshold the SDK's DarkIntegrityGuard uses — and DARK otherwise.
Consecutive dark frames are clustered into dark events for the baseline
interpolation. A camera that never exceeds the threshold (no real signal)
is treated as all-dark. The correction *math* is identical to the real
pipeline; only dark-frame *identification* differs.

Photodiode normalization: the light source does not reach the same
illumination for the same setpoint on every turn-on, so each light frame's
dark-corrected mean is additionally divided by the Thorlabs photodiode
power (interpolated at the frame's timestamp) to cancel source-level
variation. Dark frames are forced to exactly 0 in this normalized output
(column ``mean_norm``, plotted in {subject}_mean_normalized.png).

Requires pandas (chunked CSV read -- the raw file can be several GB).

Usage
-----
    python bench/analyze_drift_scan.py --data-dir bench/drift_scan_out --subject-id DRIFT
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

BIN_COLS = [str(i) for i in range(1024)]
BINS = np.arange(1024, dtype=np.float64)
BINS_SQ = BINS ** 2
N_CAMERAS = 8
CHUNK_SIZE = 20_000
PEDESTAL_DN = 128.0
DARK_THRESHOLD_DN = PEDESTAL_DN + 5.0   # frame is LIGHT iff u1 > this (SDK DarkIntegrityGuard threshold)
DARK_EVENT_GAP_S = 5.0                  # consecutive dark frames farther apart than this start a new event
PD_OFF_THRESHOLD_W = 10e-6              # photodiode below this = source off (noise floor ~0.3uW, on-state ~250uW+)
PD_ON_FRACTION = 0.8                    # ratio only valid when photodiode >= this fraction of its median on-state
                                        # (guards against divide-by-small blowups on window-edge transition frames)
# Dark-window edge guard. The camera toggles light<->dark in ~1 frame, so the
# u1<=133 threshold separates dark/light cleanly and mean_dc is correct across a
# window. But the Thorlabs photodiode lags/overshoots ~0.5-2 s after each toggle,
# so dividing the (clean) camera mean_dc by that unsettled photodiode leaves a
# small ~0.5% notch in mean_norm right after every window. Drop frames within
# this guard of a recorded dark off/on time from the NORMALIZED trace ONLY --
# mean_dc / the dark correction are untouched.
GUARD_PRE_OFF_S = 0.75                  # exclude this long before a window's light-off
GUARD_POST_ON_S = 2.5                   # exclude this long after a window's light-on (photodiode settling)


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"))
    parser.add_argument("--subject-id", default="DRIFT")
    parser.add_argument("--out-prefix", default=None, help="Prefix for output PNGs/CSV. Default: <data-dir>/<subject-id>.")
    return parser.parse_args()


def load_meta(data_dir: Path, subject_id: str) -> dict:
    meta_path = data_dir / f"{subject_id}_drift_meta.json"
    return json.loads(meta_path.read_text())


def stream_frame_stats(raw_csv_path: str) -> pd.DataFrame:
    """Single pass over the raw histogram CSV -> per-frame (cam_id, frame_id,
    timestamp_s, temperature, total, u1, u2) summary. Never holds more than
    one chunk's worth of 1024-bin data in memory at a time."""
    usecols = ["cam_id", "frame_id", "timestamp_s", "temperature", "sum", *BIN_COLS]
    rows = []
    reader = pd.read_csv(raw_csv_path, usecols=usecols, chunksize=CHUNK_SIZE)
    for chunk in reader:
        bins_arr = chunk[BIN_COLS].to_numpy(dtype=np.float64)
        total = chunk["sum"].to_numpy(dtype=np.float64)
        safe_total = np.where(total > 0, total, 1.0)
        u1 = (bins_arr @ BINS) / safe_total
        u2 = (bins_arr @ BINS_SQ) / safe_total
        u1 = np.where(total > 0, u1, np.nan)
        u2 = np.where(total > 0, u2, np.nan)
        rows.append(pd.DataFrame({
            "cam_id": chunk["cam_id"].to_numpy(),
            "frame_id": chunk["frame_id"].to_numpy(),
            "timestamp_s": chunk["timestamp_s"].to_numpy(dtype=np.float64),
            "temperature": chunk["temperature"].to_numpy(dtype=np.float64),
            "total": total,
            "u1": u1,
            "u2": u2,
        }))
    return pd.concat(rows, ignore_index=True)


def cluster_dark_events(dark_t: np.ndarray, gap_s: float) -> list[np.ndarray]:
    """Split sorted dark-frame timestamps into contiguous events wherever the
    gap between consecutive dark frames exceeds gap_s. Returns a list of
    index arrays (positions into dark_t), one per event."""
    if dark_t.size == 0:
        return []
    boundaries = np.flatnonzero(np.diff(dark_t) > gap_s)
    return np.split(np.arange(dark_t.size), boundaries + 1)


def main() -> int:
    args = parse_cli()
    meta = load_meta(args.data_dir, args.subject_id)
    raw_csv_path = meta["raw_csv_path"]
    if not raw_csv_path:
        raise RuntimeError("meta JSON has no raw_csv_path -- did drift_scan.py finish successfully?")
    print(f"[*] Reading {raw_csv_path} ...")
    frames = stream_frame_stats(raw_csv_path)
    print(f"[+] Loaded {len(frames)} frames across {frames['cam_id'].nunique()} cameras")

    n_scheduled = len(meta["dark_events"])

    # Per-frame dark classification. Preferred: SCHEDULE-based — a frame is DARK iff
    # its timestamp falls inside a recorded dark window [off+0.15, on-0.05] (the
    # wall-clock/frame-clock alignment is sub-100 ms, proven by the guard-band work).
    # This is pedestal-agnostic, which matters in CAMERA_RAW mode where BLC is off and
    # the black level sits at the raw ADC pedestal (~255+ DN) and may drift with
    # temperature — the old value threshold (u1 <= 133) would classify nothing dark.
    # Fallback (no recorded event times): the legacy value threshold.
    events_timed = [ev for ev in meta["dark_events"] if ev.get("elapsed_off_sec") is not None]
    if events_timed:
        # Schedule-LOCATED, content-RESOLVED: the frame clock can be skewed from the
        # recorded wall-clock off/on times by a per-run ~0.5 s (observed -0.43 to
        # -0.5 s), so a pure schedule window catches a light/dark mix. Instead, take a
        # generous neighborhood around each recorded window and split it by its OWN
        # bimodal u1 levels (per camera) -- self-aligning, and pedestal-agnostic (works
        # at BLC's 128 and at RAW mode's ~507 alike).
        SKEW_MARGIN_S = 1.5
        # Minimum p95-p5 u1 spread for a window to count as light/dark for a camera.
        # Separates "no usable light" from "dim": unlit cameras (FFBASE cams 4/5) show
        # 1.0-1.6 DN of stray-light contrast, dim-but-real ones (cams 3/6) 8.6-10.7 DN,
        # sagging to the low end during the warm-up dip; dark-frame noise is ~0.01 DN.
        # The former 10 DN gate sat inside the dim range and silently dropped every
        # early window on cams 3/6 -> mean_dc NaN over the whole dip.
        MIN_CONTRAST_DN = 4.0
        ts_arr = frames["timestamp_s"].to_numpy()
        u1_arr = frames["u1"].to_numpy()
        cam_arr = frames["cam_id"].to_numpy()
        is_dark = np.zeros(len(frames), dtype=bool)
        n_resolved = 0
        unresolved = {cam: [] for cam in range(N_CAMERAS)}   # cam -> window off-times skipped
        for ev in events_timed:
            off = float(ev["elapsed_off_sec"])
            on = ev.get("elapsed_on_sec")
            hi = (float(on) if on is not None else float(frames["timestamp_s"].max()))
            near = (ts_arr >= off - SKEW_MARGIN_S) & (ts_arr <= hi + SKEW_MARGIN_S)
            resolved_any = False
            for cam_id in np.unique(cam_arr[near]):
                m = near & (cam_arr == cam_id)
                vals = u1_arr[m]
                vals = vals[~np.isnan(vals)]
                if vals.size < 8:
                    continue
                lo, hi_v = np.percentile(vals, 5), np.percentile(vals, 95)
                if hi_v - lo < MIN_CONTRAST_DN:
                    unresolved[int(cam_id)].append(off)
                    continue  # window has no light/dark contrast for this camera
                thresh = lo + 0.35 * (hi_v - lo)
                is_dark |= m & (u1_arr < thresh)
                resolved_any = True
            n_resolved += int(resolved_any)
        frames["is_dark"] = is_dark
        print(f"[*] Dark classification: schedule-located + content-resolved "
              f"({n_resolved}/{len(events_timed)} windows resolved)")
        for cam_id, skipped in unresolved.items():
            # A camera with SOME resolved windows but not all has holes in its dark
            # reference; light frames not bracketed by two resolved windows get no
            # mean_dc. Say so -- a silent gap reads downstream as "no dip".
            if skipped and len(skipped) < len(events_timed):
                shown = ", ".join(f"{t:.0f}" for t in skipped[:8]) + (" ..." if len(skipped) > 8 else "")
                print(f"[!] cam {cam_id + 1}: {len(skipped)}/{len(events_timed)} dark windows below "
                      f"{MIN_CONTRAST_DN:.0f} DN contrast, skipped (t = {shown} s) -- mean_dc has gaps there")
    else:
        frames["is_dark"] = frames["u1"] <= DARK_THRESHOLD_DN
        print("[*] Dark classification: value-threshold fallback (no timed dark events in meta)")
    frames["dark_event_idx"] = -1

    dark_stats = {cam: [] for cam in range(N_CAMERAS)}  # cam -> list of dicts per event

    for cam_id in range(N_CAMERAS):
        cam_df = frames.loc[frames["cam_id"] == cam_id].sort_values("timestamp_s")
        dark_df = cam_df.loc[cam_df["is_dark"]]
        if len(dark_df) == len(cam_df):
            print(f"  cam {cam_id + 1}: never exceeds {DARK_THRESHOLD_DN:.0f} DN -- all-dark (no usable signal)")
            continue
        if len(dark_df) == 0:
            print(f"  cam {cam_id + 1}: no light/dark contrast in any window -- no usable signal")
            continue

        events = cluster_dark_events(dark_df["timestamp_s"].to_numpy(), DARK_EVENT_GAP_S)
        for k, ev_pos in enumerate(events):
            sub = dark_df.iloc[ev_pos]
            frames.loc[sub.index, "dark_event_idx"] = k
            # Per-window pedestal via MEDIAN (not weighted mean): the light<->dark
            # toggle beats against the 40 fps frame clock, so ~40% of windows catch
            # 1-2 partial-transition frames at ~131-133 DN (still <=133, so counted
            # dark). A mean lets those 2 frames spike the pedestal ~0.2 DN; the median
            # ignores them -- removing the spike at its source (no cross-window
            # smoothing needed) while preserving the real warm-up dark-current drift.
            u1_arr = sub["u1"].to_numpy()
            u2_arr = sub["u2"].to_numpy()
            dark_u1 = float(np.median(u1_arr))
            dark_var = float(np.median(np.maximum(u2_arr - u1_arr ** 2, 0.0)))
            dark_stats[cam_id].append({
                "event_index": k,
                "t": float(sub["timestamp_s"].mean()),
                "u1": dark_u1,
                "var": dark_var,
                "std": dark_var ** 0.5,
                "n_frames": int(len(sub)),
                "temperature": float(sub["temperature"].mean()),
            })
        print(f"  cam {cam_id + 1}: {len(dark_df)} dark frames in {len(events)} events "
              f"(schedule had {n_scheduled} windows)")

    # ---- dark-pedestal linear-interpolation correction for bounded light frames ----
    frames["mean_dc"] = np.nan
    frames["std_dc"] = np.nan

    for cam_id in range(N_CAMERAS):
        events_for_cam = [d for d in dark_stats[cam_id] if d is not None]
        if len(events_for_cam) < 2:
            continue
        cam_mask_global = (frames["cam_id"] == cam_id) & (~frames["is_dark"])
        cam_df = frames.loc[cam_mask_global]

        # Interpolate the (robust, per-window-median) pedestal linearly between
        # consecutive windows. No cross-window smoothing needed -- the median
        # already removes the timing-beat spikes at their source.
        for k in range(len(events_for_cam) - 1):
            d_prev, d_next = events_for_cam[k], events_for_cam[k + 1]
            in_between = (cam_df["timestamp_s"] > d_prev["t"]) & (cam_df["timestamp_s"] < d_next["t"])
            idx = cam_df.index[in_between]
            if len(idx) == 0:
                continue
            t = frames.loc[idx, "timestamp_s"].to_numpy()
            span = d_next["t"] - d_prev["t"]
            t_frac = (t - d_prev["t"]) / span if span > 0 else np.zeros_like(t)

            baseline_u1 = d_prev["u1"] + t_frac * (d_next["u1"] - d_prev["u1"])
            baseline_var = d_prev["var"] + t_frac * (d_next["var"] - d_prev["var"])

            raw_u1 = frames.loc[idx, "u1"].to_numpy()
            raw_u2 = frames.loc[idx, "u2"].to_numpy()
            raw_var = np.maximum(raw_u2 - raw_u1 ** 2, 0.0)

            mean_dc = raw_u1 - baseline_u1
            corr_var = np.maximum(raw_var - baseline_var, 0.0)
            std_dc = np.sqrt(corr_var)

            frames.loc[idx, "mean_dc"] = mean_dc
            frames.loc[idx, "std_dc"] = std_dc

    # ---- photodiode normalization ----
    # The source doesn't hit the same illumination for the same setpoint on
    # every turn-on, so divide each light frame's dark-corrected mean by the
    # photodiode power at that frame's timestamp. Dark frames -> exactly 0.
    frames["photodiode_w"] = np.nan
    frames["mean_norm"] = np.nan
    thorlabs_csv = meta.get("thorlabs_csv_path")
    if thorlabs_csv and Path(thorlabs_csv).exists():
        tl = pd.read_csv(thorlabs_csv).sort_values("elapsed_s")
        pd_w = np.interp(frames["timestamp_s"].to_numpy(),
                         tl["elapsed_s"].to_numpy(), tl["power"].to_numpy())
        frames["photodiode_w"] = pd_w

        is_dark_arr = frames["is_dark"].to_numpy()
        mean_dc_arr = frames["mean_dc"].to_numpy()
        mean_norm = np.full(len(frames), np.nan)
        # The ratio is only meaningful when the source is FULLY on. A frame at
        # a dark-window edge can classify light (camera integrated mid-toggle)
        # while the photodiode already reads partway down -- dividing by that
        # small denominator produces huge spikes. Require the photodiode to be
        # at >= PD_ON_FRACTION of its median on-state before trusting a ratio.
        on_samples = pd_w[pd_w >= PD_OFF_THRESHOLD_W]
        pd_on_median = float(np.median(on_samples)) if on_samples.size else np.nan
        fully_on = pd_w >= PD_ON_FRACTION * pd_on_median
        # Time guard around each recorded dark window: exclude the photodiode's
        # lag/overshoot edge from the divide (mean_dc is left untouched). off/on
        # times are scan-relative in meta["dark_events"]; the tail window has
        # on=None (light stays off to scan end), the front window has off=0.
        ts = frames["timestamp_s"].to_numpy()
        in_guard = np.zeros(len(frames), dtype=bool)
        for ev in meta["dark_events"]:
            off = ev.get("elapsed_off_sec")
            if off is None:
                continue
            on = ev.get("elapsed_on_sec")
            lo = off - GUARD_PRE_OFF_S
            hi = (on + GUARD_POST_ON_S) if on is not None else np.inf
            in_guard |= (ts >= lo) & (ts <= hi)
        frames["in_guard"] = in_guard
        light_ok = (~is_dark_arr) & fully_on & (~in_guard)
        mean_norm[light_ok] = mean_dc_arr[light_ok] / (pd_w[light_ok] * 1e6)  # DN per uW
        # Everything else -- dark frames, source-off stragglers, and photodiode
        # edge-transition frames -- is forced to exactly 0.
        mean_norm[~light_ok] = 0.0
        frames["mean_norm"] = mean_norm
        n_guard = int((in_guard & ~is_dark_arr).sum())
        print(f"[+] Photodiode normalization applied (mean_norm, DN/uW; "
              f"median on-state {pd_on_median * 1e6:.1f} uW; "
              f"guarded {n_guard} photodiode-edge frames)")
    else:
        print("[!] No Thorlabs CSV available -- skipping photodiode normalization")

    out_prefix = args.out_prefix or str(args.data_dir / args.subject_id)
    frames_csv = f"{out_prefix}_analysis.csv"
    frames.to_csv(frames_csv, index=False)
    print(f"[+] Wrote {frames_csv}")

    # ---- plots ----
    import matplotlib.pyplot as plt

    light = frames.loc[~frames["is_dark"]]

    # Plot 1: raw mean (left) vs corrected mean (right)
    fig1, (ax1a, ax1b) = plt.subplots(1, 2, figsize=(14, 5.5), sharex=True)
    for cam_id in range(N_CAMERAS):
        sub = light[light["cam_id"] == cam_id].sort_values("timestamp_s")
        ax1a.plot(sub["timestamp_s"] / 60.0, sub["u1"], lw=0.8, label=f"cam {cam_id + 1}")
        corr = sub.dropna(subset=["mean_dc"])
        ax1b.plot(corr["timestamp_s"] / 60.0, corr["mean_dc"], lw=0.8, label=f"cam {cam_id + 1}")
    ax1a.set_title("Raw mean intensity")
    ax1b.set_title("Dark-corrected mean intensity")
    for ax in (ax1a, ax1b):
        ax.set_xlabel("Time (min)")
        ax.grid(True, alpha=0.3)
    ax1a.set_ylabel("Mean pixel intensity (DN)")
    ax1b.legend(ncol=4, fontsize=8)
    fig1.suptitle("Per-camera mean intensity over the drift scan")
    fig1.tight_layout()
    fig1.savefig(f"{out_prefix}_mean_raw_vs_corrected.png", dpi=150)
    print(f"[+] Saved {out_prefix}_mean_raw_vs_corrected.png")

    # Plot 2: raw std (left) vs corrected std (right)
    fig2, (ax2a, ax2b) = plt.subplots(1, 2, figsize=(14, 5.5), sharex=True)
    for cam_id in range(N_CAMERAS):
        sub = light[light["cam_id"] == cam_id].sort_values("timestamp_s")
        raw_std = np.sqrt(np.maximum(sub["u2"] - sub["u1"] ** 2, 0.0))
        ax2a.plot(sub["timestamp_s"] / 60.0, raw_std, lw=0.8, label=f"cam {cam_id + 1}")
        corr = sub.dropna(subset=["std_dc"])
        ax2b.plot(corr["timestamp_s"] / 60.0, corr["std_dc"], lw=0.8, label=f"cam {cam_id + 1}")
    ax2a.set_title("Raw std intensity")
    ax2b.set_title("Dark-corrected std intensity")
    for ax in (ax2a, ax2b):
        ax.set_xlabel("Time (min)")
        ax.grid(True, alpha=0.3)
    ax2a.set_ylabel("Std pixel intensity (DN)")
    ax2b.legend(ncol=4, fontsize=8)
    fig2.suptitle("Per-camera intensity std-dev over the drift scan")
    fig2.tight_layout()
    fig2.savefig(f"{out_prefix}_std_raw_vs_corrected.png", dpi=150)
    print(f"[+] Saved {out_prefix}_std_raw_vs_corrected.png")

    # Plot 3: temperature over time (all frames, light + dark, for a continuous trace)
    fig3, ax3 = plt.subplots(figsize=(9, 5.5))
    for cam_id in range(N_CAMERAS):
        sub = frames[frames["cam_id"] == cam_id].sort_values("timestamp_s")
        ax3.plot(sub["timestamp_s"] / 60.0, sub["temperature"], lw=0.8, label=f"cam {cam_id + 1}")
    ax3.set_xlabel("Time (min)")
    ax3.set_ylabel("Temperature (C)")
    ax3.set_title("Per-camera temperature over the drift scan")
    ax3.grid(True, alpha=0.3)
    ax3.legend(ncol=4, fontsize=8)
    fig3.tight_layout()
    fig3.savefig(f"{out_prefix}_temperature.png", dpi=150)
    print(f"[+] Saved {out_prefix}_temperature.png")

    # Plot 4: dark mean / std drift across events
    fig4, (ax4a, ax4b) = plt.subplots(1, 2, figsize=(14, 5.5), sharex=True)
    for cam_id in range(N_CAMERAS):
        evs = [d for d in dark_stats[cam_id] if d is not None]
        if not evs:
            continue
        tmin = [d["t"] / 60.0 for d in evs]
        ax4a.plot(tmin, [d["u1"] for d in evs], marker="o", ms=3, lw=0.8, label=f"cam {cam_id + 1}")
        ax4b.plot(tmin, [d["std"] for d in evs], marker="o", ms=3, lw=0.8, label=f"cam {cam_id + 1}")
    ax4a.set_title("Dark mean intensity per event")
    ax4b.set_title("Dark std intensity per event")
    for ax in (ax4a, ax4b):
        ax.set_xlabel("Time (min)")
        ax.grid(True, alpha=0.3)
    ax4a.set_ylabel("Dark mean pixel intensity (DN)")
    ax4b.legend(ncol=4, fontsize=8)
    fig4.suptitle("Dark reference drift over the scan")
    fig4.tight_layout()
    fig4.savefig(f"{out_prefix}_dark_drift.png", dpi=150)
    print(f"[+] Saved {out_prefix}_dark_drift.png")

    # Plot 5: photodiode-normalized mean (all frames -- darks show as 0)
    if frames["mean_norm"].notna().any():
        fig5, ax5 = plt.subplots(figsize=(13, 5.5))
        for cam_id in range(N_CAMERAS):
            sub = frames[frames["cam_id"] == cam_id].sort_values("timestamp_s")
            ax5.plot(sub["timestamp_s"] / 60.0, sub["mean_norm"], lw=0.8, label=f"cam {cam_id + 1}")
        ax5.set_xlabel("Time (min)")
        ax5.set_ylabel("Dark-corrected mean / photodiode power (DN/uW)")
        ax5.set_title("Photodiode-normalized mean intensity (dark frames forced to 0)")
        ax5.grid(True, alpha=0.3)
        ax5.legend(ncol=4, fontsize=9)
        fig5.tight_layout()
        fig5.savefig(f"{out_prefix}_mean_normalized.png", dpi=150)
        print(f"[+] Saved {out_prefix}_mean_normalized.png")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
