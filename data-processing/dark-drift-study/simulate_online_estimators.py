"""Leave-one-out simulation of online dark-frame estimators.

For each scheduled dark frame in the recording, predict its u1 and std
using only earlier-in-time darks for that same (side, cam) — then compare
the prediction against the actually-measured value. The prediction RMSE
tells us how well the strategy would do as a real-time predictor.

This first iteration evaluates exactly two strategies, picked per the
v1 spec:

* **u1** — zero-order hold of the *average* of the last 3 darks.
  Rationale: u1 barely drifts (0.5 bin over an hour) and per-sample
  noise dominates two-point extrapolation. Averaging suppresses noise
  while still tracking slow drift.

* **std** — linear extrapolation of the last 2 darks in time.
  Rationale: std follows a smooth concave curve. Two recent points
  give a usable slope; extrapolation tracks the underlying trend.

Outputs: per-strategy overall RMSE, per-camera RMSE breakdown, and a
time-resolved error plot so warmup vs steady-state behaviour is visible.
"""

from __future__ import annotations

import csv
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np


LEFT_CSV  = "C:/Users/ethan/Projects/scan_data/20260520_163109_owEENEJ6_left_mask66_raw.csv"
RIGHT_CSV = "C:/Users/ethan/Projects/scan_data/20260520_163109_owEENEJ6_right_mask66_raw.csv"
OUT_DIR   = "C:/Users/ethan/Projects/openmotion-sdk/data-processing/dark-drift-study"

GAIN = [16, 4, 2, 1, 1, 2, 4, 16]  # 0-indexed; firmware

_BIN_IDX    = np.arange(1024, dtype=np.float64)
_BIN_IDX_SQ = _BIN_IDX * _BIN_IDX


def _is_dark(absolute_frame: int, discard: int = 9, interval: int = 600) -> bool:
    if absolute_frame == discard + 1:
        return True
    return absolute_frame > discard + 1 and (absolute_frame - 1) % interval == 0


def _load(path: str, side: str) -> dict[tuple[str, int], list[tuple]]:
    """Same loader as generate_plots.py — keep them in sync."""
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
            u1  = float(_BIN_IDX    @ hist) / s
            m2  = float(_BIN_IDX_SQ @ hist) / s
            std = float(np.sqrt(max(0.0, m2 - u1 * u1)))
            T   = float(row[temp_col])
            out[(side, cam_id + 1)].append((ts, u1, std, T))
    return out


# ---------------------------------------------------------------------------
# Strategies — each takes (history, target dark frame) and returns a
# predicted (u1 or std) value, or None if it needs more warmup history.
# ``history`` is the chronological list of darks observed *strictly before*
# the target frame, as (ts, u1, std, T) tuples. The target is also
# (ts, u1, std, T); strategies should not peek at its u1 or std but may
# use its ts and T (those are knowable at the corresponding light-frame
# in the real system).
# ---------------------------------------------------------------------------


def u1_zoh1(history, _target):
    """Last-observed dark u1 — simplest possible baseline."""
    if not history: return None
    return history[-1][1]


def u1_avg_last_n(history, _target, *, n: int):
    if not history: return None
    return float(np.mean([p[1] for p in history[-n:]]))


def u1_avg3 (history, target):  return u1_avg_last_n(history, target, n=3)
def u1_avg5 (history, target):  return u1_avg_last_n(history, target, n=5)
def u1_avg10(history, target):  return u1_avg_last_n(history, target, n=10)


def std_zoh1(history, _target):
    """Last-observed dark std — simplest possible baseline."""
    if not history: return None
    return history[-1][2]


def std_linear_extrap_time(history, target):
    """Extend the line through the last two (ts, std) points to target.ts."""
    if len(history) < 2: return None
    t_a, _, std_a, _ = history[-2]
    t_b, _, std_b, _ = history[-1]
    dt = t_b - t_a
    if dt <= 0: return std_b
    slope = (std_b - std_a) / dt
    return float(std_b + slope * (target[0] - t_b))


def std_linear_extrap_temp(history, target):
    """Extend the line through the last two (T, std) points to target.T.

    Self-correcting at the thermal asymptote: as T stops changing,
    target.T − T_last → 0, so the projection term vanishes and the
    predictor degrades gracefully to ZOH.
    """
    if len(history) < 2: return None
    _, _, std_a, T_a = history[-2]
    _, _, std_b, T_b = history[-1]
    dT = T_b - T_a
    # Guard against tiny T denominator — at steady state T quantization
    # can yield T_b == T_a or worse, T_b < T_a from quantization noise.
    if abs(dT) < 1e-3:
        return std_b
    slope = (std_b - std_a) / dT
    return float(std_b + slope * (target[3] - T_b))


def std_avg3(history, _target):
    """Comparison: average last 3 (treats std as noise around a slow level)."""
    if not history: return None
    return float(np.mean([p[2] for p in history[-3:]]))


U1_STRATEGIES = {
    "zoh1":   u1_zoh1,
    "avg3":   u1_avg3,
    "avg5":   u1_avg5,
    "avg10":  u1_avg10,
}
STD_STRATEGIES = {
    "zoh1":          std_zoh1,
    "avg3":          std_avg3,
    "linear_time":   std_linear_extrap_time,
    "linear_T":      std_linear_extrap_temp,
}


def simulate(groups: dict[tuple[str, int], list[tuple]]) -> dict:
    """Leave-one-out simulation across all U1_STRATEGIES × STD_STRATEGIES.

    Returns a nested dict keyed by (side, cam) → strategy class → strategy
    name → {ts, true, pred, err}.
    """
    results: dict = {}
    for key, darks in groups.items():
        darks = sorted(darks)
        # Pre-allocate per-strategy arrays.
        per_strat = {
            "u1":  {name: {"ts": [], "true": [], "pred": [], "err": []}
                    for name in U1_STRATEGIES},
            "std": {name: {"ts": [], "true": [], "pred": [], "err": []}
                    for name in STD_STRATEGIES},
        }
        for i, target in enumerate(darks):
            history = darks[:i]
            for name, fn in U1_STRATEGIES.items():
                p = fn(history, target)
                if p is None: continue
                per_strat["u1"][name]["ts"].append(target[0])
                per_strat["u1"][name]["true"].append(target[1])
                per_strat["u1"][name]["pred"].append(p)
                per_strat["u1"][name]["err"].append(target[1] - p)
            for name, fn in STD_STRATEGIES.items():
                p = fn(history, target)
                if p is None: continue
                per_strat["std"][name]["ts"].append(target[0])
                per_strat["std"][name]["true"].append(target[2])
                per_strat["std"][name]["pred"].append(p)
                per_strat["std"][name]["err"].append(target[2] - p)
        # Convert lists to arrays.
        for klass in per_strat.values():
            for d in klass.values():
                for k in d: d[k] = np.array(d[k])
        results[key] = per_strat
    return results


def _overall_rmse(results: dict, klass: str, name: str) -> tuple[float, float, int]:
    errs = []
    for r in results.values():
        errs.extend(r[klass][name]["err"].tolist())
    a = np.array(errs)
    if a.size == 0: return float("nan"), float("nan"), 0
    return float(np.sqrt(np.mean(a**2))), float(np.mean(a)), int(a.size)


def _split_rmse_by_phase(results: dict, klass: str, name: str,
                         t_split: float) -> tuple[float, float]:
    """Return (RMSE_early, RMSE_late) where the split is at t_split."""
    early, late = [], []
    for r in results.values():
        d = r[klass][name]
        mask_early = d["ts"] < t_split
        mask_late  = ~mask_early
        early.extend(d["err"][mask_early].tolist())
        late.extend(d["err"][mask_late].tolist())
    rmse_early = float(np.sqrt(np.mean(np.array(early)**2))) if early else float("nan")
    rmse_late  = float(np.sqrt(np.mean(np.array(late )**2))) if late  else float("nan")
    return rmse_early, rmse_late


def report(results: dict) -> None:
    # The std curve transitions from steep rise to near-asymptote roughly
    # around t = 600 s based on the drift plot. Split there to see how
    # each strategy behaves in the two regimes.
    T_SPLIT = 600.0

    print()
    print("==== u1 strategies — overall ====")
    print(f"  {'strategy':>10s}  {'RMSE':>8s}  {'bias':>9s}  {'n':>6s}  "
          f"{'RMSE early <{0:.0f}s':>15s}  {'RMSE late >={0:.0f}s':>16s}".format(T_SPLIT))
    print("  " + "-" * 70)
    for name in U1_STRATEGIES:
        rmse, bias, n = _overall_rmse(results, "u1", name)
        e, l = _split_rmse_by_phase(results, "u1", name, T_SPLIT)
        print(f"  {name:>10s}  {rmse:>8.4f}  {bias:>+9.4f}  {n:>6d}  "
              f"{e:>15.4f}  {l:>16.4f}")

    print()
    print("==== std strategies — overall ====")
    print(f"  {'strategy':>15s}  {'RMSE':>8s}  {'bias':>9s}  {'n':>6s}  "
          f"{'RMSE early <{0:.0f}s':>15s}  {'RMSE late >={0:.0f}s':>16s}".format(T_SPLIT))
    print("  " + "-" * 75)
    for name in STD_STRATEGIES:
        rmse, bias, n = _overall_rmse(results, "std", name)
        e, l = _split_rmse_by_phase(results, "std", name, T_SPLIT)
        print(f"  {name:>15s}  {rmse:>8.4f}  {bias:>+9.4f}  {n:>6d}  "
              f"{e:>15.4f}  {l:>16.4f}")


def _windowed_rmse(t: np.ndarray, err: np.ndarray,
                   bins: np.ndarray) -> np.ndarray:
    """Per-window RMSE across the scan, for an error trace at times t."""
    out = np.full(len(bins) - 1, np.nan)
    for i in range(len(bins) - 1):
        mask = (t >= bins[i]) & (t < bins[i + 1])
        if mask.any():
            out[i] = float(np.sqrt(np.mean(err[mask] ** 2)))
    return out


def plot_strategy_comparison(results: dict) -> None:
    """Two panels — one per metric — comparing strategy windowed-RMSE
    across the scan. Aggregates across all cameras into a single curve
    per strategy so the time-resolved behavior is visible at a glance."""
    bin_edges = np.linspace(0.0, 3600.0, 31)  # 30 windows × 120 s each
    centers   = 0.5 * (bin_edges[:-1] + bin_edges[1:])
    cmap = plt.get_cmap("tab10")

    # u1 strategies
    fig, ax = plt.subplots(figsize=(14, 6), dpi=150)
    for i, name in enumerate(U1_STRATEGIES):
        ts_all, err_all = [], []
        for r in results.values():
            d = r["u1"][name]
            ts_all.extend(d["ts"].tolist())
            err_all.extend(d["err"].tolist())
        ts_all = np.array(ts_all); err_all = np.array(err_all)
        rmse_curve = _windowed_rmse(ts_all, err_all, bin_edges)
        ax.plot(centers, rmse_curve, marker="o", ms=5, lw=1.8,
                color=cmap(i % 10), label=name)
    ax.set_xlabel("scan time (s)")
    ax.set_ylabel("windowed u1 RMSE (bin index)")
    ax.set_title("u1 estimator strategies — RMSE in 120 s windows, aggregated across all cameras")
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "11_u1_strategy_comparison.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)

    # std strategies
    fig, ax = plt.subplots(figsize=(14, 6), dpi=150)
    for i, name in enumerate(STD_STRATEGIES):
        ts_all, err_all = [], []
        for r in results.values():
            d = r["std"][name]
            ts_all.extend(d["ts"].tolist())
            err_all.extend(d["err"].tolist())
        ts_all = np.array(ts_all); err_all = np.array(err_all)
        rmse_curve = _windowed_rmse(ts_all, err_all, bin_edges)
        ax.plot(centers, rmse_curve, marker="o", ms=5, lw=1.8,
                color=cmap(i % 10), label=name)
    ax.set_xlabel("scan time (s)")
    ax.set_ylabel("windowed std RMSE (bin index)")
    ax.set_title("std estimator strategies — RMSE in 120 s windows, aggregated across all cameras")
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "12_std_strategy_comparison.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    print("loading raw CSVs ...")
    groups: dict[tuple[str, int], list[tuple]] = {}
    for path, side in ((LEFT_CSV, "left"), (RIGHT_CSV, "right")):
        for k, v in _load(path, side).items():
            groups[k] = v
        print(f"  loaded {path}")
    print(f"  {len(groups)} (side, cam) groups, "
          f"{sum(len(v) for v in groups.values())} dark samples")

    results = simulate(groups)
    report(results)
    plot_strategy_comparison(results)
    print(f"\nWrote 11_u1_strategy_comparison.png and 12_std_strategy_comparison.png to {OUT_DIR}")


if __name__ == "__main__":
    main()
