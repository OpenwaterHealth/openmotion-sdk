#!/usr/bin/env python3
"""Plot the dark-frame mean (u1) and std over time per camera.

Given one or more ``*_raw.csv`` files, pick out the scheduled dark
frames per camera, compute u1 (raw histogram mean) and std (sqrt of
the bin-variance) for each, and plot them against ``timestamp_s``.

This is the diagnostic step for the "predict the next dark mean so we
can do real-time dark correction" experiment. Look for monotonic drift,
periodicity, or noise floors that an estimator would have to track.

Usage
-----
    python scripts/plot_dark_drift.py left_maskXX_raw.csv
    python scripts/plot_dark_drift.py left.csv right.csv --save fig.png
    python scripts/plot_dark_drift.py raw.csv --pedestal 64 --discard-count 9 --dark-interval 600

Standalone — only depends on ``csv``, ``numpy``, and ``matplotlib``.
Schedule formula matches ``SciencePipeline._is_dark_frame``: first
dark at ``discard_count + 1`` (= 10), subsequent at
``(absolute_frame - 1) % dark_interval == 0`` for
absolute_frame > discard+1. ``frame_id`` is unwrapped per camera
because the raw CSV stores the firmware's 8-bit counter.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from omotion.config import HISTO_BINS, HISTO_BINS_SQ, HISTO_SIZE_WORDS


@dataclass
class DarkPoint:
    cam_id: int
    side: str
    abs_frame: int
    timestamp_s: float
    u1: float       # raw histogram mean (bin index)
    std: float      # sqrt of bin variance
    temperature: float  # per-camera sensor temperature reported on this row


def _is_dark_frame(abs_frame: int, discard: int, interval: int) -> bool:
    """Same rule as SciencePipeline._is_dark_frame."""
    if abs_frame == discard + 1:
        return True
    return abs_frame > discard + 1 and (abs_frame - 1) % interval == 0


def _read_raw_csv_dark_points(
    path: str,
    side_label: str,
    *,
    discard_count: int,
    dark_interval: int,
) -> list[DarkPoint]:
    """Stream the raw CSV row-by-row, emit a DarkPoint for every
    row whose unwrapped absolute frame_id is scheduled-dark."""
    out: list[DarkPoint] = []
    # Per-camera unwrap state.
    last_raw_fid: dict[int, int] = {}
    abs_offset: dict[int, int] = defaultdict(int)

    with open(path, newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader)
        if header[:3] != ["cam_id", "frame_id", "timestamp_s"]:
            raise ValueError(
                f"{path}: header doesn't look like a raw histogram CSV "
                f"(got first 3 cols {header[:3]!r})"
            )
        # Bins start at column 3. ``temperature`` is the first
        # post-bin scalar (column 3 + 1024). Locate it from the header
        # so this doesn't fragile-out if columns get reordered later.
        bin_lo, bin_hi = 3, 3 + HISTO_SIZE_WORDS
        try:
            temp_col = header.index("temperature")
        except ValueError:
            temp_col = None

        for row in reader:
            cam_id     = int(row[0])
            raw_fid    = int(row[1])
            ts         = float(row[2])

            # Per-camera unwrap of the firmware's 8-bit frame counter.
            prev = last_raw_fid.get(cam_id)
            if prev is not None and raw_fid < prev:
                abs_offset[cam_id] += 256
            last_raw_fid[cam_id] = raw_fid
            abs_frame = raw_fid + abs_offset[cam_id]

            if not _is_dark_frame(abs_frame, discard_count, dark_interval):
                continue

            # Parse the histogram once we know we need it. Cheap rows are
            # the dominant case; this skips ~599/600 of them.
            hist = np.fromiter(
                (int(x) for x in row[bin_lo:bin_hi]),
                dtype=np.int64,
                count=HISTO_SIZE_WORDS,
            )
            row_sum = int(hist.sum())
            if row_sum <= 0:
                continue
            u1 = float(HISTO_BINS @ hist) / row_sum
            mean2 = float(HISTO_BINS_SQ @ hist) / row_sum
            var = max(0.0, mean2 - u1 * u1)
            std = float(np.sqrt(var))

            temp = float(row[temp_col]) if temp_col is not None else float("nan")
            out.append(DarkPoint(
                cam_id=cam_id,
                side=side_label,
                abs_frame=abs_frame,
                timestamp_s=ts,
                u1=u1,
                std=std,
                temperature=temp,
            ))
    return out


def _side_label_from_filename(path: str) -> str:
    """Infer 'left'/'right' from the standard raw CSV naming."""
    name = os.path.basename(path).lower()
    if "_left_" in name:
        return "left"
    if "_right_" in name:
        return "right"
    return os.path.splitext(os.path.basename(path))[0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csvs", nargs="+", help="One or more raw histogram CSV files")
    ap.add_argument("--pedestal", type=float, default=64.0,
                    help="Pedestal subtracted to get the dark-correction "
                         "baseline (default: 64).")
    ap.add_argument("--discard-count", type=int, default=9,
                    help="Frames discarded at scan start before the first "
                         "scheduled dark (default: 9 — first dark at frame 10).")
    ap.add_argument("--dark-interval", type=int, default=600,
                    help="Dark frames every N frames after the first "
                         "(default: 600).")
    ap.add_argument("--save", default=None,
                    help="If set, write the figure to this path instead of "
                         "showing it interactively.")
    args = ap.parse_args()

    import matplotlib.pyplot as plt

    all_points: list[DarkPoint] = []
    for path in args.csvs:
        side = _side_label_from_filename(path)
        n_before = len(all_points)
        pts = _read_raw_csv_dark_points(
            path, side,
            discard_count=args.discard_count,
            dark_interval=args.dark_interval,
        )
        all_points.extend(pts)
        print(f"  {path} ({side}): {len(pts)} dark points")
        if pts:
            ts0, ts1 = pts[0].timestamp_s, pts[-1].timestamp_s
            print(f"    timestamp range: {ts0:.1f}s .. {ts1:.1f}s")

    if not all_points:
        print("No dark frames found — check schedule constants?", file=sys.stderr)
        return 1

    # Group by (side, cam_id).
    groups: dict[tuple[str, int], list[DarkPoint]] = defaultdict(list)
    for p in all_points:
        groups[(p.side, p.cam_id)].append(p)

    print(f"\n{len(groups)} (side, cam) groups, totaling {len(all_points)} dark samples")

    # Plot: three stacked subplots — u1 (top), std (middle), per-camera
    # temperature (bottom) — colored consistently across panels so the
    # eye can compare drift shapes across (u1, std, T).
    keys = sorted(groups.keys())
    fig, (ax_u1, ax_std, ax_temp) = plt.subplots(
        3, 1, sharex=True, figsize=(13, 10),
        gridspec_kw={"hspace": 0.08},
    )
    cmap = plt.get_cmap("tab10")
    have_temp = False
    for i, key in enumerate(keys):
        side, cam_id = key
        pts = sorted(groups[key], key=lambda p: p.timestamp_s)
        ts   = np.array([p.timestamp_s for p in pts])
        u1   = np.array([p.u1 for p in pts])
        std  = np.array([p.std for p in pts])
        temp = np.array([p.temperature for p in pts])
        # Labels use 1-indexed cam IDs (cam_id+1) to match the firmware
        # / QML / camera-arrangement naming. The raw CSV stores 0-indexed
        # cam_id but everywhere else in the project (QML, docs, the
        # sensor firmware gain table) the cameras are cam1..cam8.
        label = f"{side}-cam{cam_id + 1}"
        color = cmap(i % 10)
        # Mark each dark sample so the per-interval cadence is visible.
        ax_u1.plot(ts, u1, marker="o", ms=4, lw=1.2, color=color, label=label)
        ax_std.plot(ts, std, marker="o", ms=4, lw=1.2, color=color, label=label)
        if np.isfinite(temp).any():
            ax_temp.plot(ts, temp, marker="o", ms=4, lw=1.2, color=color, label=label)
            have_temp = True

    # Don't draw a pedestal reference line — u1 sits well above it and
    # the dashed reference was pinning the y-axis open, hiding the actual
    # drift signal. Pedestal value is reported in the title instead.
    ax_u1.set_ylabel("dark u1 (raw bin index)")
    ax_u1.set_title(
        f"Dark-frame drift  —  {len(args.csvs)} file(s), "
        f"{len(all_points)} dark samples across {len(groups)} camera streams"
        f"   (pedestal = {args.pedestal:g})"
    )
    ax_u1.legend(loc="best", fontsize=8, ncol=2)
    ax_u1.grid(True, alpha=0.3)

    ax_std.set_ylabel("dark std (bin index)")
    ax_std.grid(True, alpha=0.3)

    ax_temp.set_ylabel("sensor temperature")
    ax_temp.set_xlabel("timestamp (s)")
    ax_temp.grid(True, alpha=0.3)
    if not have_temp:
        ax_temp.text(0.5, 0.5, "(no temperature column in raw CSV)",
                     transform=ax_temp.transAxes, ha="center", va="center",
                     fontsize=12, alpha=0.6)

    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=120, bbox_inches="tight")
        print(f"\nFigure written: {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
