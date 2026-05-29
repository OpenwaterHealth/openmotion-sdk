#!/usr/bin/env python3
"""Plot a corrected scan CSV (BFI / BVI / mean / contrast over time).

Reads the per-camera corrected CSV that ``sdk_examples.py scan`` writes
(``{scan_id}_{subject}.csv`` — columns ``timestamp_s, bfi_l1..bfi_r8,
bvi_*, mean_*, contrast_*``) and renders a 2x2 plot: one panel per metric,
all 16 cameras drawn faint with the per-side average overlaid bold
(left = blue, right = red).

    python scripts/visualize_scan.py --csv <corrected.csv> [--out plot.png]

If ``--out`` is omitted the PNG is written next to the CSV as
``<csv-stem>_viz.png``. See docs/API.md §3.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless: write a file, never open a window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

SIDES = ("l", "r")
PANELS = [
    ("bfi", "BFI", (0, 10)),
    ("bvi", "BVI", (0, 10)),
    ("mean", "Mean (DN)", None),
    ("contrast", "Contrast", None),
]


def visualize(csv_path: Path, out_path: Path) -> None:
    df = pd.read_csv(csv_path)
    if "timestamp_s" not in df.columns:
        raise SystemExit(f"{csv_path} has no 'timestamp_s' column — not a corrected scan CSV?")
    t = df["timestamp_s"].to_numpy(dtype=float)

    left_cmap = plt.get_cmap("Blues")
    right_cmap = plt.get_cmap("Reds")

    fig, axes = plt.subplots(2, 2, figsize=(15, 9), sharex=True)
    for ax, (key, title, ylim) in zip(axes.ravel(), PANELS):
        plotted = False
        for side, cmap in zip(SIDES, (left_cmap, right_cmap)):
            cols = [f"{key}_{side}{i}" for i in range(1, 9) if f"{key}_{side}{i}" in df.columns]
            if not cols:
                continue
            data = df[cols].to_numpy(dtype=float)
            for ci in range(data.shape[1]):
                ax.plot(t, data[:, ci], color=cmap(0.35 + 0.07 * ci), linewidth=0.6, alpha=0.5)
            ax.plot(t, np.nanmean(data, axis=1), color=cmap(0.95), linewidth=2.2,
                    label=f"{'left' if side == 'l' else 'right'} avg")
            plotted = True
        ax.set_title(title)
        ax.set_ylabel(title)
        if ylim:
            ax.set_ylim(*ylim)
        ax.grid(True, alpha=0.25)
        if plotted:
            ax.legend(loc="upper right", fontsize=9)
        else:
            ax.text(0.5, 0.5, f"no {key}_* columns", ha="center", va="center",
                    transform=ax.transAxes, color="grey")
    for ax in axes[1]:
        ax.set_xlabel("time (s)")

    n = len(df)
    dur = float(t[-1] - t[0]) if n else 0.0
    rate = (n - 1) / dur if dur > 0 else 0.0
    fig.suptitle(f"{csv_path.stem}   ({n} corrected frames, {dur:.1f}s, ~{rate:.0f} Hz, 16 cameras)",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_path, dpi=130)
    print(f"wrote {out_path}  ({n} frames, {dur:.1f}s, ~{rate:.0f} Hz)")


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot a corrected scan CSV.")
    ap.add_argument("--csv", required=True, help="Corrected scan CSV path.")
    ap.add_argument("--out", default=None, help="Output PNG (default: <csv-stem>_viz.png).")
    args = ap.parse_args()
    csv_path = Path(args.csv)
    out_path = Path(args.out) if args.out else csv_path.with_name(f"{csv_path.stem}_viz.png")
    visualize(csv_path, out_path)


if __name__ == "__main__":
    main()
