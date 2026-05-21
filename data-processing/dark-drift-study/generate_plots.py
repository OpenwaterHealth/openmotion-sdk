"""Regenerate every plot used in findings.md from the long dark-drift scan.

Single entry point so the study can be reproduced by re-running this
file against the two source CSVs. Labels use 1-indexed cam IDs to
match the firmware / QML / docs naming convention.
"""

from __future__ import annotations

import csv
import json
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np


LEFT_CSV  = "C:/Users/ethan/Projects/scan_data/20260520_163109_owEENEJ6_left_mask66_raw.csv"
RIGHT_CSV = "C:/Users/ethan/Projects/scan_data/20260520_163109_owEENEJ6_right_mask66_raw.csv"
OUT_DIR   = "C:/Users/ethan/Projects/openmotion-sdk/data-processing/dark-drift-study"

# Sensor firmware gain table — openmotion-sensor-fw/Core/Src/0X02C1B.c.
# Indexed by 0-based cam_id (raw CSV cam_id column).
GAIN = [16, 4, 2, 1, 1, 2, 4, 16]

_BIN_IDX    = np.arange(1024, dtype=np.float64)
_BIN_IDX_SQ = _BIN_IDX * _BIN_IDX


def _is_dark(absolute_frame: int, discard: int = 9, interval: int = 600) -> bool:
    if absolute_frame == discard + 1:
        return True
    return absolute_frame > discard + 1 and (absolute_frame - 1) % interval == 0


def _load(path: str, side: str) -> dict[tuple[str, int], list[tuple]]:
    """Return {(side, cam_id_1indexed): [(ts, u1, std, T), ...]}."""
    out: dict[tuple[str, int], list[tuple]] = defaultdict(list)
    last_fid: dict[int, int] = {}
    offset:   dict[int, int] = defaultdict(int)
    with open(path, newline="", encoding="utf-8") as fh:
        rd = csv.reader(fh)
        header = next(rd)
        temp_col = header.index("temperature")
        for row in rd:
            cam_id = int(row[0])
            raw_fid = int(row[1])
            ts = float(row[2])
            prev = last_fid.get(cam_id)
            if prev is not None and raw_fid < prev:
                offset[cam_id] += 256
            last_fid[cam_id] = raw_fid
            f = raw_fid + offset[cam_id]
            if not _is_dark(f):
                continue
            hist = np.fromiter(
                (int(x) for x in row[3:3 + 1024]),
                dtype=np.int64, count=1024,
            )
            s = int(hist.sum())
            if s <= 0:
                continue
            u1 = float(_BIN_IDX @ hist) / s
            m2 = float(_BIN_IDX_SQ @ hist) / s
            std = float(np.sqrt(max(0.0, m2 - u1 * u1)))
            T = float(row[temp_col])
            # Store with 1-indexed cam ID for downstream labelling.
            out[(side, cam_id + 1)].append((ts, u1, std, T))
    return out


def _label(side: str, cam_1: int, with_gain: bool = False) -> str:
    if with_gain:
        return f"{side}-cam{cam_1} (g={GAIN[cam_1 - 1]})"
    return f"{side}-cam{cam_1}"


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    print("loading raw CSVs (this takes ~50s) ...")
    groups: dict[tuple[str, int], list[tuple]] = {}
    for path, side in ((LEFT_CSV, "left"), (RIGHT_CSV, "right")):
        for k, v in _load(path, side).items():
            groups[k] = v
        print(f"  loaded {path}")

    keys = sorted(groups)
    cmap = plt.get_cmap("tab10")
    print(f"  {len(keys)} (side, cam) groups, {sum(len(v) for v in groups.values())} dark samples\n")

    # =====================================================================
    # 01. Stacked dark drift over time — u1, std, T as separate panels.
    # =====================================================================
    fig, (ax_u1, ax_std, ax_T) = plt.subplots(
        3, 1, sharex=True, figsize=(13, 11), dpi=150,
        gridspec_kw={"hspace": 0.08},
    )
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        ts   = np.array([p[0] for p in pts])
        u1   = np.array([p[1] for p in pts])
        std  = np.array([p[2] for p in pts])
        T    = np.array([p[3] for p in pts])
        c = cmap(i % 10)
        lbl = _label(side, cam_1)
        ax_u1.plot(ts, u1, marker=".", ms=3, lw=1.2, color=c, label=lbl)
        ax_std.plot(ts, std, marker=".", ms=3, lw=1.2, color=c, label=lbl)
        ax_T.plot(ts, T, marker=".", ms=3, lw=1.2, color=c, label=lbl)
    ax_u1.set_ylabel("dark u1 (raw bin index)")
    ax_u1.set_title("Dark-frame drift over 60-minute scan")
    ax_u1.legend(fontsize=9, ncol=2)
    ax_u1.grid(True, alpha=0.3)
    ax_std.set_ylabel("dark std (bin index)")
    ax_std.grid(True, alpha=0.3)
    ax_T.set_ylabel("sensor temperature (DN)")
    ax_T.set_xlabel("timestamp (s)")
    ax_T.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "01_dark_drift_3panel.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # 02. u1 vs time — alone, large.
    # =====================================================================
    fig, ax = plt.subplots(figsize=(14, 6), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        ts = np.array([p[0] for p in pts])
        u1 = np.array([p[1] for p in pts])
        ax.plot(ts, u1, marker=".", ms=4, lw=1.4, color=cmap(i % 10),
                label=_label(side, cam_1))
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("dark u1 (raw bin index)")
    ax.set_title("Dark u1 (mean) over 60 minutes — per-camera drift")
    ax.legend(fontsize=11, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "02_u1_vs_time.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # 03. std vs time — alone, large.
    # =====================================================================
    fig, ax = plt.subplots(figsize=(14, 6), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        ts = np.array([p[0] for p in pts])
        std = np.array([p[2] for p in pts])
        ax.plot(ts, std, marker=".", ms=4, lw=1.4, color=cmap(i % 10),
                label=_label(side, cam_1))
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("dark std (bin index)")
    ax.set_title("Dark std over 60 minutes — per-camera drift")
    ax.legend(fontsize=11, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "03_std_vs_time.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # 04. Temperature vs time — alone, large.
    # =====================================================================
    fig, ax = plt.subplots(figsize=(14, 6), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        ts = np.array([p[0] for p in pts])
        T  = np.array([p[3] for p in pts])
        ax.plot(ts, T, marker=".", ms=4, lw=1.4, color=cmap(i % 10),
                label=_label(side, cam_1))
    ax.set_xlabel("timestamp (s)")
    ax.set_ylabel("sensor temperature (DN)")
    ax.set_title("Per-camera sensor temperature over 60 minutes")
    ax.legend(fontsize=11, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "04_temperature_vs_time.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # 05. u1 vs T parametric scatter.
    # =====================================================================
    fig, ax = plt.subplots(figsize=(14, 9), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        u1 = np.array([p[1] for p in pts])
        T  = np.array([p[3] for p in pts])
        ax.scatter(T, u1, s=10, color=cmap(i % 10), alpha=0.7,
                   label=_label(side, cam_1))
    ax.set_xlabel("sensor temperature (DN)")
    ax.set_ylabel("dark u1 (raw bin index)")
    ax.set_title("Dark u1 vs camera temperature (each point is one dark frame)")
    ax.legend(fontsize=11, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "05_u1_vs_T.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # 06. std vs T parametric scatter.
    # =====================================================================
    fig, ax = plt.subplots(figsize=(14, 9), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        std = np.array([p[2] for p in pts])
        T   = np.array([p[3] for p in pts])
        ax.scatter(T, std, s=10, color=cmap(i % 10), alpha=0.7,
                   label=_label(side, cam_1))
    ax.set_xlabel("sensor temperature (DN)")
    ax.set_ylabel("dark std (bin index)")
    ax.set_title("Dark std vs camera temperature (each point is one dark frame)")
    ax.legend(fontsize=11, ncol=2)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "06_std_vs_T.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # 07. Per-camera quadratic fit overlay — u1 (top), std (bottom).
    # =====================================================================
    calibration: dict[str, dict] = {}
    fig, (ax_u1, ax_std) = plt.subplots(2, 1, figsize=(14, 11), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        u1  = np.array([p[1] for p in pts])
        std = np.array([p[2] for p in pts])
        T   = np.array([p[3] for p in pts])
        pu1  = np.polyfit(T, u1,  2)
        pstd = np.polyfit(T, std, 2)
        rmse_u1  = float(np.sqrt(np.mean((u1  - np.polyval(pu1,  T)) ** 2)))
        rmse_std = float(np.sqrt(np.mean((std - np.polyval(pstd, T)) ** 2)))
        calibration[_label(side, cam_1)] = {
            "u1_poly":  pu1.tolist(),
            "std_poly": pstd.tolist(),
            "rmse_u1":  rmse_u1,
            "rmse_std": rmse_std,
            "gain":     GAIN[cam_1 - 1],
            "T_range":  [float(T.min()), float(T.max())],
            "n_points": int(len(T)),
        }
        c = cmap(i % 10)
        T_smooth = np.linspace(T.min(), T.max(), 200)
        lbl = _label(side, cam_1)
        ax_u1.scatter(T, u1, s=8, color=c, alpha=0.4)
        ax_u1.plot(T_smooth, np.polyval(pu1, T_smooth), color=c, lw=1.8, label=lbl)
        ax_std.scatter(T, std, s=8, color=c, alpha=0.4)
        ax_std.plot(T_smooth, np.polyval(pstd, T_smooth), color=c, lw=1.8, label=lbl)
    ax_u1.set_xlabel("sensor temperature (DN)")
    ax_u1.set_ylabel("dark u1 (raw bin index)")
    ax_u1.set_title("u1 = poly2(T) per camera — dots = measured darks, lines = fit")
    ax_u1.legend(fontsize=10, ncol=2)
    ax_u1.grid(True, alpha=0.3)
    ax_std.set_xlabel("sensor temperature (DN)")
    ax_std.set_ylabel("dark std (bin index)")
    ax_std.set_title("std = poly2(T) per camera — dots = measured darks, lines = fit")
    ax_std.legend(fontsize=10, ncol=2)
    ax_std.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "07_polyfit_overlay.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    with open(os.path.join(OUT_DIR, "calibration.json"), "w") as fh:
        json.dump(calibration, fh, indent=2)

    # =====================================================================
    # 08. Gain normalization test — does std/gain collapse to one curve?
    # =====================================================================
    fig, (ax_raw, ax_norm) = plt.subplots(1, 2, figsize=(16, 7), dpi=150)
    for i, key in enumerate(keys):
        side, cam_1 = key
        pts = sorted(groups[key])
        std = np.array([p[2] for p in pts])
        T   = np.array([p[3] for p in pts])
        g = GAIN[cam_1 - 1]
        c = cmap(i % 10)
        lbl = _label(side, cam_1, with_gain=True)
        ax_raw.scatter(T, std, s=8, color=c, alpha=0.5, label=lbl)
        ax_norm.scatter(T, std / g, s=8, color=c, alpha=0.5, label=lbl)
    ax_raw.set_xlabel("sensor temperature (DN)")
    ax_raw.set_ylabel("dark std (bin index)")
    ax_raw.set_title("Raw std vs T")
    ax_raw.legend(fontsize=9, loc="upper left")
    ax_raw.grid(True, alpha=0.3)
    ax_norm.set_xlabel("sensor temperature (DN)")
    ax_norm.set_ylabel("dark std / gain  (bin index)")
    ax_norm.set_title("Gain-normalized std vs T — does it collapse to one curve?")
    ax_norm.legend(fontsize=9, loc="upper left")
    ax_norm.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "08_gain_normalized.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # =====================================================================
    # Print summary table for findings.md.
    # =====================================================================
    print()
    print(f"{'camera':>13s}  {'gain':>4s}  {'u1 range':>9s}  {'std range':>10s}  "
          f"{'u1 RMSE':>8s}  {'std RMSE':>9s}  {'T range':>15s}")
    print("-" * 95)
    for key in sorted(calibration):
        info = calibration[key]
        u1_range  = "(see plot)"
        std_range = "(see plot)"
        T_min, T_max = info["T_range"]
        print(f"{key:>13s}  {info['gain']:>4d}  {u1_range:>9s}  {std_range:>10s}  "
              f"{info['rmse_u1']:>8.4f}  {info['rmse_std']:>9.4f}  "
              f"{T_min:>6.1f}..{T_max:<6.1f}")

    print(f"\nWrote 8 PNGs and calibration.json to: {OUT_DIR}")


if __name__ == "__main__":
    main()
