#!/usr/bin/env python3
"""
plot_pdc.py — plot the `pdc` column (photodiode current, mA) from a
ConsoleTelemetry CSV produced by ScanWorkflow.

The PDC value is the photodiode-measured laser output current
(see omotion/ConsoleTelemetry.py: _PDC_MA_PER_LSB = 1.9). It samples
at ~1 Hz alongside the rest of the console telemetry.

Usage:
    python plot_pdc.py <telemetry_csv>
    python plot_pdc.py                        # auto-find newest *_telemetry.csv
    python plot_pdc.py --show                 # also open an interactive window
    python plot_pdc.py --window 30            # add a 30-sample rolling mean
    python plot_pdc.py file1.csv file2.csv    # overlay multiple scans
"""

import argparse
import glob
import os
import sys

import matplotlib
_want_show = "--show" in sys.argv
matplotlib.use("TkAgg" if _want_show else "Agg")
import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_SCAN_DIRS = [
    r"C:\Users\ethan\Projects\scan_data",
    r"C:\Users\ethan\Projects\openmotion-bloodflow-app\scan_data",
]


def _newest_telemetry_csv() -> str:
    candidates: list[str] = []
    for d in DEFAULT_SCAN_DIRS:
        candidates.extend(glob.glob(os.path.join(d, "*_telemetry.csv")))
    if not candidates:
        sys.exit("No *_telemetry.csv found in default scan_data folders.")
    return max(candidates, key=os.path.getmtime)


def _load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    if "pdc" not in df.columns:
        sys.exit(f"{path}: no 'pdc' column (columns: {list(df.columns)})")
    if "timestamp" in df.columns:
        df["t"] = df["timestamp"] - df["timestamp"].iloc[0]
    else:
        df["t"] = range(len(df))
    return df


def _summarize(label: str, df: pd.DataFrame) -> None:
    pdc = df["pdc"]
    print(
        f"{label}: n={len(pdc)}  "
        f"duration={df['t'].iloc[-1]:.1f}s  "
        f"mean={pdc.mean():.2f} mA  std={pdc.std():.2f}  "
        f"min={pdc.min():.2f}  max={pdc.max():.2f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", nargs="*", help="telemetry CSV(s); auto-find if omitted")
    ap.add_argument("--show", action="store_true", help="open an interactive window")
    ap.add_argument(
        "--window", type=int, default=0,
        help="rolling-mean window in samples (0 = off)",
    )
    ap.add_argument(
        "--out", default=None,
        help="output PNG path (default: alongside first CSV)",
    )
    args = ap.parse_args()

    paths = args.csv or [_newest_telemetry_csv()]

    fig, ax = plt.subplots(figsize=(12, 5))
    for path in paths:
        df = _load(path)
        label = os.path.basename(path).replace("_telemetry.csv", "")
        _summarize(label, df)
        ax.plot(df["t"], df["pdc"], linewidth=1.0, alpha=0.85, label=label)
        if args.window > 0:
            ax.plot(
                df["t"],
                df["pdc"].rolling(args.window, min_periods=1).mean(),
                linewidth=1.5,
                label=f"{label} (rolling {args.window})",
            )

    ax.set_title("Photodiode current (PDC) vs time")
    ax.set_xlabel("Time since scan start (s)")
    ax.set_ylabel("PDC (mA)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()

    out_path = args.out or os.path.join(
        os.path.dirname(os.path.abspath(paths[0])),
        os.path.basename(paths[0]).replace("_telemetry.csv", "_pdc.png"),
    )
    fig.savefig(out_path, dpi=120)
    print(f"Saved {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
