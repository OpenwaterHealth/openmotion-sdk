#!/usr/bin/env python3
"""
plot_pdc_tec.py — overlay PDC with TEC and trigger channels on a shared
time axis so we can see what (if anything) co-moves with the startup
overshoot.

Usage:
    python plot_pdc_tec.py <telemetry_csv> [--show] [--max-t SECONDS]
"""

import argparse
import os
import sys

import matplotlib
_want_show = "--show" in sys.argv
matplotlib.use("TkAgg" if _want_show else "Agg")
import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--max-t", type=float, default=None,
                    help="limit x-axis to first N seconds (default: full scan)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df["t"] = df["timestamp"] - df["timestamp"].iloc[0]
    if args.max_t is not None:
        df = df[df["t"] <= args.max_t]

    # tcl is a cumulative counter; turn it into a per-second rate so it
    # is comparable to the other 1 Hz channels.
    df["tcl_rate"] = df["tcl"].diff() / df["t"].diff()

    fig, axes = plt.subplots(5, 1, figsize=(14, 14), sharex=True)

    axes[0].plot(df["t"], df["pdc"], linewidth=0.8, color="C0", label="pdc (mA)")
    if len(df) > 30:
        axes[0].plot(df["t"], df["pdc"].rolling(30, min_periods=1).mean(),
                     linewidth=1.5, color="C1", label="pdc rolling-30")
    axes[0].set_ylabel("PDC (mA)")
    axes[0].set_title(f"PDC + TEC overlay — {os.path.basename(args.csv)}")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)

    axes[1].plot(df["t"], df["tec_v_raw"], linewidth=0.8,
                 label="tec_v_raw (measured)")
    axes[1].plot(df["t"], df["tec_set_raw"], linewidth=0.8,
                 label="tec_set_raw (setpoint)")
    axes[1].set_ylabel("TEC thermistor (V)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", fontsize=8)

    # Plot |measured - setpoint| to see lock quality at the mV level.
    axes[2].plot(df["t"], (df["tec_v_raw"] - df["tec_set_raw"]) * 1000.0,
                 linewidth=0.8, color="C3")
    axes[2].axhline(0.0, color="k", linewidth=0.5, alpha=0.5)
    axes[2].set_ylabel("v_raw - set_raw (mV)")
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title("TEC lock error (TMPGD threshold = ±100 mV)")

    axes[3].plot(df["t"], df["tec_curr_raw"], linewidth=0.8,
                 label="tec_curr_raw")
    axes[3].plot(df["t"], df["tec_volt_raw"], linewidth=0.8,
                 label="tec_volt_raw")
    axes[3].set_ylabel("TEC drive (raw V)")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right", fontsize=8)

    axes[4].plot(df["t"], df["tcl_rate"], linewidth=0.8, color="C4")
    axes[4].set_ylabel("tcl rate (counts/s)")
    axes[4].set_xlabel("Time since scan start (s)")
    axes[4].grid(True, alpha=0.3)
    axes[4].set_title("Laser trigger counter rate (should be ~constant)")

    fig.tight_layout()

    out_path = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.path.basename(args.csv).replace("_telemetry.csv", "_pdc_tec.png"),
    )
    fig.savefig(out_path, dpi=120)
    print(f"Saved {out_path}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
