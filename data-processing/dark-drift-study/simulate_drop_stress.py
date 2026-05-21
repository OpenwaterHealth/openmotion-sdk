"""Dropped-dark stress test for the chosen online estimators.

Holds the predictors fixed (u1 = avg-of-last-3, std = linear-extrap-of-
last-2-in-time) and varies how many scheduled dark frames are
synthetically "dropped" before the predictor sees them. Measures
prediction quality under three failure scenarios:

1. **Random drops at rate p** — sweep p in [0, 5, 10, 20, 30, 40, 50]%.
   Each scheduled dark has independent probability ``p`` of being
   hidden from the predictor. The dropped dark's true value is still
   used as ground truth for the prediction-error computation, the
   predictor just can't see it.

2. **Single-gap recovery** — drop exactly one dark at a specific
   timepoint, then track the per-dark prediction error for the
   following darks to see how quickly the predictor recovers.

3. **Burst drops** — drop K consecutive darks (K=1, 3, 5) starting at
   a specific timepoint. Same recovery analysis as #2 but stresses the
   slope estimate which suddenly has to project across a 2K × 15-s gap.

Output: console summary tables and PNG plots.
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

GAIN = [16, 4, 2, 1, 1, 2, 4, 16]

_BIN_IDX    = np.arange(1024, dtype=np.float64)
_BIN_IDX_SQ = _BIN_IDX * _BIN_IDX


def _is_dark(absolute_frame: int, discard: int = 9, interval: int = 600) -> bool:
    if absolute_frame == discard + 1:
        return True
    return absolute_frame > discard + 1 and (absolute_frame - 1) % interval == 0


def _load(path: str, side: str) -> dict[tuple[str, int], list[tuple]]:
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


# ---------- Estimators (frozen at the v1 picks) ------------------------------

def predict_u1(history: list[tuple]) -> float | None:
    if not history: return None
    return float(np.mean([p[1] for p in history[-3:]]))


def predict_std(history: list[tuple], t_pred: float) -> float | None:
    if len(history) < 2: return None
    t_a, _, std_a, _ = history[-2]
    t_b, _, std_b, _ = history[-1]
    dt = t_b - t_a
    if dt <= 0: return std_b
    slope = (std_b - std_a) / dt
    return float(std_b + slope * (t_pred - t_b))


# ---------- Simulation under a drop pattern ----------------------------------

def simulate_with_drops(darks: list[tuple], drop_mask: np.ndarray
                        ) -> tuple[np.ndarray, np.ndarray, np.ndarray,
                                   np.ndarray, np.ndarray]:
    """Walk darks chronologically, predict each one using only non-dropped
    prior darks. Returns (ts, u1_err, std_err, gap_since_last_visible,
    n_gaps_in_window).

    ``drop_mask[i]`` True means dark ``i`` is hidden from the predictor but
    still used as ground-truth for the prediction-error metric.
    """
    ts_arr:   list[float] = []
    u1_err:   list[float] = []
    std_err:  list[float] = []
    gap_arr:  list[float] = []
    visible:  list[tuple] = []
    n_consec_dropped = 0
    for i, target in enumerate(darks):
        u1_pred  = predict_u1(visible)
        std_pred = predict_std(visible, target[0])
        if u1_pred is not None and std_pred is not None:
            ts_arr.append(target[0])
            u1_err.append(target[1] - u1_pred)
            std_err.append(target[2] - std_pred)
            # gap = how many consecutive scheduled darks were dropped
            # immediately before this prediction. 0 means the predictor
            # had a fresh observation on the previous interval.
            gap_arr.append(float(n_consec_dropped))
        if drop_mask[i]:
            n_consec_dropped += 1
        else:
            visible.append(target)
            n_consec_dropped = 0
    return (
        np.array(ts_arr),
        np.array(u1_err),
        np.array(std_err),
        np.array(gap_arr),
        None,
    )


# ---------- Test 1: random drop rate sweep -----------------------------------

def run_random_drop_sweep(groups: dict[tuple[str, int], list[tuple]],
                          *, rates=(0, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50),
                          n_seeds: int = 5) -> dict:
    """For each drop rate, run ``n_seeds`` independent simulations
    (averaging across seeds reduces sampling noise on the RMSE estimate).
    Aggregates across all (side, cam) groups."""
    out = {"rate": [], "u1_rmse": [], "std_rmse": [], "u1_bias": [], "std_bias": []}
    rng_master = np.random.default_rng(0xc0ffee)
    for p in rates:
        u1_all, std_all = [], []
        for s in range(n_seeds):
            seed = int(rng_master.integers(0, 2**31 - 1))
            rng = np.random.default_rng(seed)
            for darks in groups.values():
                darks = sorted(darks)
                mask = rng.random(len(darks)) < p
                _, u1_e, std_e, _, _ = simulate_with_drops(darks, mask)
                u1_all.extend(u1_e.tolist())
                std_all.extend(std_e.tolist())
        u1_arr  = np.array(u1_all)
        std_arr = np.array(std_all)
        out["rate"].append(p)
        out["u1_rmse"].append(float(np.sqrt(np.mean(u1_arr  ** 2))) if u1_arr.size  else float("nan"))
        out["std_rmse"].append(float(np.sqrt(np.mean(std_arr ** 2))) if std_arr.size else float("nan"))
        out["u1_bias"].append(float(np.mean(u1_arr)) if u1_arr.size  else float("nan"))
        out["std_bias"].append(float(np.mean(std_arr)) if std_arr.size else float("nan"))
    return out


# ---------- Test 2 & 3: scheduled gap at a specific timepoint ----------------

def run_burst_gap(groups: dict[tuple[str, int], list[tuple]],
                  *, gap_time_s: float, burst_size: int
                  ) -> dict:
    """Drop ``burst_size`` consecutive darks starting at the dark whose
    timestamp is closest to ``gap_time_s``. Returns per-group error
    traces aligned by "darks-since-end-of-gap" so the recovery can be
    plotted on a single relative axis."""
    out = {"label": f"gap@{int(gap_time_s)}s, K={burst_size}",
           "u1_err_by_offset":  defaultdict(list),
           "std_err_by_offset": defaultdict(list)}
    for darks in groups.values():
        darks = sorted(darks)
        # Find the index of the first dark whose t >= gap_time_s.
        ts_arr = np.array([d[0] for d in darks])
        idx_gap_start = int(np.searchsorted(ts_arr, gap_time_s))
        if idx_gap_start >= len(darks) - burst_size:
            continue
        mask = np.zeros(len(darks), dtype=bool)
        mask[idx_gap_start : idx_gap_start + burst_size] = True
        ts, u1_e, std_e, _, _ = simulate_with_drops(darks, mask)
        # Build "darks-since-end-of-gap" offset for each evaluated dark.
        # The first dark whose index > idx_gap_start + burst_size − 1
        # is the first "post-gap" dark, offset 0.
        evaluated_indices = []  # which absolute indices in `darks` produced an output
        # simulate_with_drops emits one output per dark where prediction
        # was possible, in order. So evaluated_indices walks through
        # darks in order, skipping any whose predict returned None.
        running_visible = 0
        for i in range(len(darks)):
            if running_visible >= 2:
                evaluated_indices.append(i)
            if not mask[i]:
                running_visible += 1
        evaluated_indices = evaluated_indices[: len(ts)]  # safety
        post_gap_start = idx_gap_start + burst_size
        for j, abs_idx in enumerate(evaluated_indices):
            offset = abs_idx - post_gap_start
            if -2 <= offset <= 10:  # show a window around the gap
                out["u1_err_by_offset"][offset].append(u1_e[j])
                out["std_err_by_offset"][offset].append(std_e[j])
    return out


# ---------- Plotting --------------------------------------------------------

def plot_random_drop_sweep(res: dict) -> None:
    fig, (ax_u1, ax_std) = plt.subplots(1, 2, figsize=(15, 6), dpi=150)
    rates_pct = [r * 100 for r in res["rate"]]
    ax_u1.plot(rates_pct, res["u1_rmse"], marker="o", ms=8, lw=2,
               color="tab:blue", label="u1 (avg-3)")
    ax_u1.axhline(0.0206, color="tab:blue", lw=0.8, ls="--",
                  label="u1 baseline @ 0% drop")
    ax_u1.set_xlabel("dark-frame drop rate (%)")
    ax_u1.set_ylabel("u1 RMSE (bin index)")
    ax_u1.set_title("u1 estimator robustness to random drops")
    ax_u1.legend(fontsize=11)
    ax_u1.grid(True, alpha=0.3)

    ax_std.plot(rates_pct, res["std_rmse"], marker="o", ms=8, lw=2,
                color="tab:orange", label="std (linear-extrap-time)")
    ax_std.axhline(0.0220, color="tab:orange", lw=0.8, ls="--",
                   label="std baseline @ 0% drop")
    ax_std.set_xlabel("dark-frame drop rate (%)")
    ax_std.set_ylabel("std RMSE (bin index)")
    ax_std.set_title("std estimator robustness to random drops")
    ax_std.legend(fontsize=11)
    ax_std.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "13_drop_rate_sweep.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_burst_recovery(burst_results: list[dict]) -> None:
    """Side-by-side: u1 and std error vs darks-since-end-of-gap, one
    series per burst scenario."""
    fig, (ax_u1, ax_std) = plt.subplots(1, 2, figsize=(15, 6), dpi=150)
    cmap = plt.get_cmap("tab10")
    for i, res in enumerate(burst_results):
        offsets = sorted(res["u1_err_by_offset"])
        u1_med  = [np.median(np.abs(res["u1_err_by_offset"][o])) for o in offsets]
        u1_max  = [np.max   (np.abs(res["u1_err_by_offset"][o])) for o in offsets]
        std_med = [np.median(np.abs(res["std_err_by_offset"][o])) for o in offsets]
        std_max = [np.max   (np.abs(res["std_err_by_offset"][o])) for o in offsets]
        c = cmap(i % 10)
        ax_u1.plot(offsets, u1_med, marker="o", ms=6, lw=1.8, color=c,
                   label=f"{res['label']}  (median)")
        ax_u1.plot(offsets, u1_max, marker="x", ms=6, lw=0.8, color=c,
                   linestyle="--", alpha=0.6,
                   label=f"{res['label']}  (max)")
        ax_std.plot(offsets, std_med, marker="o", ms=6, lw=1.8, color=c,
                    label=f"{res['label']}  (median)")
        ax_std.plot(offsets, std_max, marker="x", ms=6, lw=0.8, color=c,
                    linestyle="--", alpha=0.6,
                    label=f"{res['label']}  (max)")
    for ax in (ax_u1, ax_std):
        ax.axvline(0, color="red", lw=1.2, ls=":",
                   label="end of gap (0 = first post-gap dark)")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("dark index relative to end of gap")
    ax_u1.set_ylabel("|u1 prediction error| (bin)")
    ax_u1.set_title("u1 estimator recovery after dropped-dark burst")
    ax_u1.legend(fontsize=8, loc="upper right")
    ax_std.set_ylabel("|std prediction error| (bin)")
    ax_std.set_title("std estimator recovery after dropped-dark burst")
    ax_std.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "14_burst_recovery.png"),
                dpi=150, bbox_inches="tight")
    plt.close(fig)


# ---------- Reporting --------------------------------------------------------

def report_rate_sweep(res: dict) -> None:
    print("\n==== Test 1 — random drop rate sweep ====")
    print(f"  {'rate':>6s}  {'u1 RMSE':>9s}  {'u1 bias':>9s}  "
          f"{'std RMSE':>9s}  {'std bias':>9s}")
    print("  " + "-" * 55)
    for i in range(len(res["rate"])):
        print(f"  {res['rate'][i]*100:>5.0f}%  "
              f"{res['u1_rmse'][i]:>9.4f}  {res['u1_bias'][i]:>+9.4f}  "
              f"{res['std_rmse'][i]:>9.4f}  {res['std_bias'][i]:>+9.4f}")


def report_burst(burst_results: list[dict]) -> None:
    print("\n==== Tests 2 & 3 — burst recovery (median |err| in bin index) ====")
    for res in burst_results:
        offsets = sorted(res["u1_err_by_offset"])
        u1  = {o: float(np.median(np.abs(res["u1_err_by_offset"][o])))  for o in offsets}
        std = {o: float(np.median(np.abs(res["std_err_by_offset"][o]))) for o in offsets}
        print(f"\n  {res['label']}")
        print(f"    {'offset':>7s}  {'u1':>8s}  {'std':>8s}")
        for o in offsets:
            print(f"    {o:>7d}  {u1[o]:>8.4f}  {std[o]:>8.4f}")


def main() -> None:
    print("loading raw CSVs ...")
    groups: dict[tuple[str, int], list[tuple]] = {}
    for path, side in ((LEFT_CSV, "left"), (RIGHT_CSV, "right")):
        for k, v in _load(path, side).items():
            groups[k] = v
        print(f"  loaded {path}")
    print(f"  {len(groups)} (side, cam) groups\n")

    # Test 1
    rate_res = run_random_drop_sweep(groups)
    report_rate_sweep(rate_res)
    plot_random_drop_sweep(rate_res)

    # Tests 2 & 3 — burst sizes 1, 3, 5 at two timepoints (rising and asymptote).
    burst_results = []
    for gap_t in (300.0, 2400.0):
        for K in (1, 3, 5):
            burst_results.append(run_burst_gap(
                groups, gap_time_s=gap_t, burst_size=K
            ))
    report_burst(burst_results)
    plot_burst_recovery(burst_results)

    print(f"\nWrote 13_drop_rate_sweep.png and 14_burst_recovery.png to {OUT_DIR}")


if __name__ == "__main__":
    main()
