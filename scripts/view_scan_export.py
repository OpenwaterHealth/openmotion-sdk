"""
Visualize a bloodflow-app scan-export CSV (History -> Export).

The app's History screen writes ``<YYYYMMDD_HHMMSS>_<label>_export.csv``
per selected scan via ``omotion.SessionPlayback.materialize_corrected_csv``
(``include_quality=True``). Two layouts exist:

- **Per-camera** (normal recording): ``frame_id, timestamp_s`` then
  ``bfi/bvi/mean/contrast/temp[/quality]`` x ``l1..l8, r1..r8``. Cells
  are empty for cameras that were masked off (and ``temp`` for sessions
  recorded before schema v2).
- **Reduced** (clinical side-average recording): ``frame_id,
  timestamp_s, bfi_left, bfi_right, bvi_left, bvi_right``.

This script reads only the export file — no scan DB or raw CSVs — and
renders one row of axes per available metric (Left | Right columns),
with per-camera traces, the nan-aware spatial side average (same
definition as the pipeline's reduced mode), and a per-camera quality
strip (ok / ts_corrected / nan_filled) when quality columns are present.

Usage:
    python scripts/view_scan_export.py <export.csv | folder>
        [--save out.png] [--no-show] [--decimate N] [--full-range]

With a folder, the newest ``*_export.csv`` inside is used.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

SIDES = ("l", "r")
SIDE_NAMES = {"l": "Left", "r": "Right"}
PER_CAM_METRICS = ("bfi", "bvi", "mean", "contrast", "temp")
METRIC_LABELS = {
    "bfi": "BFI (a.u.)",
    "bvi": "BVI (a.u.)",
    "mean": "Mean (dark-corrected DN)",
    "contrast": "Contrast (σ/μ)",
    "temp": "Camera temp (°C)",
}
# Codes match omotion.pipeline.stages.side_avg._QUALITY_RANK, plus a
# catch-all for values a future SDK might add.
QUALITY_CODES = {"ok": 0, "ts_corrected": 1, "nan_filled": 2}
QUALITY_NAMES = ["ok", "ts_corrected", "nan_filled", "other"]
QUALITY_COLORS = ["#43a047", "#ffb300", "#e53935", "#9e9e9e"]


@dataclass
class Export:
    path: Path
    frame_id: np.ndarray                # (N,) int
    t: np.ndarray                       # (N,) float, scan-relative seconds
    reduced: bool
    # Per-camera layout: metric -> side -> (8, N) float (NaN where empty).
    per_cam: Dict[str, Dict[str, np.ndarray]] = field(default_factory=dict)
    # side -> (8, N) int quality codes, -1 where the cell was empty.
    quality: Dict[str, np.ndarray] = field(default_factory=dict)
    # side -> 0-based indices of cameras that produced any finite sample.
    active: Dict[str, List[int]] = field(default_factory=dict)
    # Reduced layout: metric -> side -> (N,) float.
    side: Dict[str, Dict[str, np.ndarray]] = field(default_factory=dict)


def _floats(col: np.ndarray) -> np.ndarray:
    return np.where(col == "", "nan", col).astype(np.float64)


def _nan_spatial_mean(a: np.ndarray) -> np.ndarray:
    """Row-ignoring-NaN mean of a (K, N) array -> (N,), NaN where no
    camera contributed (no all-NaN RuntimeWarnings)."""
    cnt = np.sum(np.isfinite(a), axis=0)
    total = np.nansum(a, axis=0)
    out = np.full(a.shape[1], np.nan)
    np.divide(total, cnt, out=out, where=cnt > 0)
    return out


def load_export(path: Path) -> Export:
    with path.open("r", newline="", encoding="utf-8") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        if not header:
            raise SystemExit(f"Empty file: {path}")
        ncol = len(header)
        rows = [r for r in reader if len(r) == ncol]
    if not rows:
        raise SystemExit(f"No data rows in {path}")

    columns = {name: np.asarray(vals) for name, vals in zip(header, zip(*rows))}
    if "frame_id" not in columns or "timestamp_s" not in columns:
        raise SystemExit(f"Not a scan-export CSV (no frame_id/timestamp_s): {path}")

    exp = Export(
        path=path,
        frame_id=_floats(columns["frame_id"]).astype(np.int64),
        t=_floats(columns["timestamp_s"]),
        reduced="bfi_left" in columns,
    )

    if exp.reduced:
        for metric in ("bfi", "bvi"):
            exp.side[metric] = {
                s: _floats(columns[f"{metric}_{name}"])
                for s, name in (("l", "left"), ("r", "right"))
                if f"{metric}_{name}" in columns
            }
        return exp

    if "bfi_l1" not in columns:
        raise SystemExit(f"Not a scan-export CSV (no bfi_l1/bfi_left column): {path}")

    n = len(exp.t)
    for metric in PER_CAM_METRICS:
        by_side = {}
        for s in SIDES:
            grid = np.full((8, n), np.nan)
            found = False
            for cam in range(8):
                col = columns.get(f"{metric}_{s}{cam + 1}")
                if col is not None:
                    grid[cam] = _floats(col)
                    found = True
            if found and np.isfinite(grid).any():
                by_side[s] = grid
        if by_side:
            exp.per_cam[metric] = by_side

    for s in SIDES:
        grid = np.full((8, n), -1, dtype=np.int8)
        found = False
        for cam in range(8):
            col = columns.get(f"quality_{s}{cam + 1}")
            if col is not None:
                found = True
                codes = np.full(n, -1, dtype=np.int8)
                nonempty = col != ""
                codes[nonempty] = 3  # "other" until matched below
                for name, code in QUALITY_CODES.items():
                    codes[col == name] = code
                grid[cam] = codes
        if found and (grid >= 0).any():
            exp.quality[s] = grid

    bfi = exp.per_cam.get("bfi", {})
    for s in SIDES:
        grid = bfi.get(s)
        exp.active[s] = (
            [] if grid is None else
            [c for c in range(8) if np.isfinite(grid[c]).any()]
        )
    return exp


def _mask_str(cams: List[int]) -> str:
    if not cams:
        return "none"
    mask = sum(1 << c for c in cams)
    return f"{','.join(str(c + 1) for c in cams)} (mask 0x{mask:02X})"


def print_summary(exp: Export) -> None:
    t, fid = exp.t, exp.frame_id
    span = t[-1] - t[0] if len(t) > 1 else 0.0
    print(f"File:      {exp.path}")
    print(f"Layout:    {'reduced (side averages)' if exp.reduced else 'per-camera'}")
    print(f"Frames:    {len(fid)} rows, frame_id {fid[0]}..{fid[-1]}, "
          f"t {t[0]:.3f}..{t[-1]:.3f} s ({span:.1f} s span)")
    deltas = np.diff(fid)
    missing = int(np.sum(deltas[deltas > 1] - 1)) if len(deltas) else 0
    print(f"Gaps:      {missing} missing frame id(s)")
    if exp.reduced:
        return
    for s in SIDES:
        print(f"{SIDE_NAMES[s]} cams: {_mask_str(exp.active.get(s, []))}")
    if exp.quality:
        counts = {name: 0 for name in QUALITY_NAMES}
        total = 0
        for grid in exp.quality.values():
            valid = grid[grid >= 0]
            total += valid.size
            for code, name in enumerate(QUALITY_NAMES):
                counts[name] += int(np.sum(valid == code))
        parts = [f"{name}={counts[name]}" for name in QUALITY_NAMES if counts[name]]
        pct_bad = 100.0 * (total - counts["ok"]) / total if total else 0.0
        print(f"Quality:   {', '.join(parts) or 'no samples'} "
              f"({pct_bad:.2f}% non-ok cells)")


def _robust_ylim(arrays: List[np.ndarray], full_range: bool):
    finite = np.concatenate([a[np.isfinite(a)].ravel() for a in arrays]) \
        if arrays else np.empty(0)
    if finite.size == 0:
        return None
    if full_range:
        lo, hi = float(finite.min()), float(finite.max())
    else:
        # Robust limits so a handful of railed/spiked frames (e.g. the
        # BFI=10 rail) can't flatten the trace everyone came to see.
        lo, hi = np.percentile(finite, [0.5, 99.5])
    if hi <= lo:
        lo, hi = lo - 0.5, hi + 0.5
    pad = 0.08 * (hi - lo)
    return lo - pad, hi + pad


def plot_export(exp: Export, decimate: int, full_range: bool):
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    sl = slice(None, None, max(1, decimate))
    t = exp.t[sl]

    if exp.reduced:
        metrics = [m for m in ("bfi", "bvi") if exp.side.get(m)]
        fig, axes = plt.subplots(
            len(metrics), 1, figsize=(14, 3.2 * len(metrics)),
            sharex=True, squeeze=False, constrained_layout=True)
        for ax, metric in zip(axes[:, 0], metrics):
            for s, color in (("l", "#1976d2"), ("r", "#d32f2f")):
                y = exp.side[metric].get(s)
                if y is not None:
                    ax.plot(t, y[sl], color=color, linewidth=1.2,
                            label=SIDE_NAMES[s])
            ax.set_ylabel(METRIC_LABELS[metric])
            ax.grid(True, alpha=0.2)
            ax.legend(loc="upper right")
        axes[-1, 0].set_xlabel("Time (s, scan-relative)")
        fig.suptitle(exp.path.name)
        return fig

    metrics = [m for m in PER_CAM_METRICS if m in exp.per_cam]
    n_rows = len(metrics) + (1 if exp.quality else 0)
    height_ratios = [3] * len(metrics) + ([1.4] if exp.quality else [])
    fig, axes = plt.subplots(
        n_rows, 2, figsize=(15, 2.6 * len(metrics) + 2.2),
        sharex=True, sharey="row", squeeze=False, constrained_layout=True,
        gridspec_kw={"height_ratios": height_ratios})

    cmap = plt.get_cmap("tab10")
    cam_colors = [cmap(c % 10) for c in range(8)]

    for row, metric in enumerate(metrics):
        ylim = _robust_ylim(
            [exp.per_cam[metric][s] for s in SIDES if s in exp.per_cam[metric]],
            full_range)
        for col, s in enumerate(SIDES):
            ax = axes[row, col]
            grid = exp.per_cam[metric].get(s)
            if grid is None or not np.isfinite(grid).any():
                ax.text(0.5, 0.5, "no data", transform=ax.transAxes,
                        ha="center", va="center", color="0.5")
            else:
                for cam in range(8):
                    if np.isfinite(grid[cam]).any():
                        ax.plot(t, grid[cam][sl], color=cam_colors[cam],
                                linewidth=0.7, alpha=0.55)
                ax.plot(t, _nan_spatial_mean(grid)[sl], color="black",
                        linewidth=1.6)
            if row == 0:
                ax.set_title(SIDE_NAMES[s])
            if col == 0:
                ax.set_ylabel(METRIC_LABELS[metric])
            if ylim:
                ax.set_ylim(*ylim)
            ax.grid(True, alpha=0.2)

    if exp.quality:
        qcmap = ListedColormap(QUALITY_COLORS)
        qcmap.set_bad(alpha=0.0)
        norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5], qcmap.N)
        for col, s in enumerate(SIDES):
            ax = axes[-1, col]
            grid = exp.quality.get(s)
            if grid is None:
                ax.text(0.5, 0.5, "no data", transform=ax.transAxes,
                        ha="center", va="center", color="0.5")
                continue
            shown = np.ma.masked_less(grid[:, sl], 0).astype(np.float64)
            # imshow assumes uniform frame spacing; dropped frames shift
            # columns by at most their own width — fine for a strip.
            ax.imshow(shown, aspect="auto", origin="lower", cmap=qcmap,
                      norm=norm, interpolation="nearest",
                      extent=[t[0], t[-1], 0.5, 8.5])
            ax.set_yticks(range(1, 9))
            if col == 0:
                ax.set_ylabel("Quality (cam)")

    for ax in axes[-1]:
        ax.set_xlabel("Time (s, scan-relative)")

    handles = [Line2D([], [], color=cam_colors[c], linewidth=1.5,
                      label=f"cam {c + 1}") for c in range(8)]
    handles.append(Line2D([], [], color="black", linewidth=2,
                          label="side avg"))
    if exp.quality:
        handles += [Patch(facecolor=c, label=n)
                    for n, c in zip(QUALITY_NAMES[:3], QUALITY_COLORS[:3])]
    fig.legend(handles=handles, loc="outside lower center",
               ncol=len(handles), fontsize=8, frameon=False)
    fig.suptitle(exp.path.name)
    return fig


def _resolve_input(raw: str) -> Path:
    p = Path(raw).expanduser().resolve()
    if p.is_dir():
        candidates = sorted(p.glob("*_export.csv"),
                            key=lambda c: c.stat().st_mtime, reverse=True)
        if not candidates:
            raise SystemExit(f"No *_export.csv files in {p}")
        return candidates[0]
    if not p.exists():
        raise SystemExit(f"File not found: {p}")
    return p


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualize a bloodflow-app scan-export CSV "
                    "(<label>_export.csv from History -> Export).")
    parser.add_argument("path", help="Export CSV, or a folder to pick the "
                                     "newest *_export.csv from.")
    parser.add_argument("--save", type=str, default=None,
                        help="Write the figure to this image path.")
    parser.add_argument("--no-show", action="store_true",
                        help="Do not open an interactive window.")
    parser.add_argument("--decimate", type=int, default=1, metavar="N",
                        help="Plot every Nth frame (summary uses all).")
    parser.add_argument("--full-range", action="store_true",
                        help="Autoscale y to the full data range instead of "
                             "the robust 0.5..99.5 percentile window.")
    args = parser.parse_args()

    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")

    path = _resolve_input(args.path)
    exp = load_export(path)
    print_summary(exp)

    fig = plot_export(exp, args.decimate, args.full_range)

    if args.save:
        out = Path(args.save).expanduser().resolve()
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=160)
        print(f"Saved figure: {out}")

    if not args.no_show:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
