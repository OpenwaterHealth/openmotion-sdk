#!/usr/bin/env python3
"""
plot_pdc_fft.py — single-sided FFT of PDC steady-state, plus correlation
matrix between PDC and TEC drive channels.

The PDC signal is sampled at ~1 Hz, so the useful spectrum reaches the
Nyquist limit of ~0.5 Hz — anything we see is sub-Hertz (slow drift,
breathing-rate-class artifacts, fan PWM beats aliased into the band).

Usage:
    python plot_pdc_fft.py <telemetry_csv> [--start SECONDS]
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv")
    ap.add_argument("--start", type=float, default=1000.0,
                    help="skip first N seconds (default 1000 to drop warm-up)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df["t"] = df["timestamp"] - df["timestamp"].iloc[0]
    df["dt"] = df["timestamp"].diff()
    df_ss = df[df["t"] >= args.start].reset_index(drop=True)

    # ConsoleTelemetry targets 1.0 s but jitters. Resample to a uniform 1 Hz
    # grid so the FFT bins are interpretable.
    fs = 1.0
    dt_med = df_ss["dt"].median()
    print(f"steady-state samples: {len(df_ss)}  "
          f"median dt: {dt_med:.3f}s  "
          f"min: {df_ss['dt'].min():.3f}s  max: {df_ss['dt'].max():.3f}s")

    t_uniform = np.arange(0, df_ss["t"].iloc[-1] - df_ss["t"].iloc[0], 1.0 / fs)
    pdc_uniform = np.interp(
        t_uniform,
        df_ss["t"].values - df_ss["t"].iloc[0],
        df_ss["pdc"].values,
    )
    pdc_detrended = pdc_uniform - pdc_uniform.mean()

    # Single-sided amplitude spectrum, Hann-windowed.
    win = np.hanning(len(pdc_detrended))
    spec = np.fft.rfft(pdc_detrended * win)
    freqs = np.fft.rfftfreq(len(pdc_detrended), d=1.0 / fs)
    amps = (2.0 / win.sum()) * np.abs(spec)

    # Print top-5 peaks above 1 mHz (skip DC / ultra-slow drift).
    above = freqs > 1e-3
    top_idx = np.argsort(amps[above])[-5:][::-1]
    print("Top spectral peaks (above 1 mHz):")
    for k in top_idx:
        f = freqs[above][k]
        a = amps[above][k]
        period_s = 1.0 / f if f > 0 else float("inf")
        print(f"  f={f:.5f} Hz  period={period_s:7.1f} s  amp={a:.3f} mA")

    # Correlations across the full scan (not just steady-state) — slow
    # drifts in TEC drive should track slow drifts in PDC if they share a
    # thermal cause.
    cols = ["pdc", "tec_v_raw", "tec_set_raw", "tec_curr_raw", "tec_volt_raw"]
    print("\nPearson correlations with PDC (full scan):")
    for c in cols:
        if c == "pdc":
            continue
        r = df["pdc"].corr(df[c])
        print(f"  {c:14s} r={r:+.3f}")

    # Plot spectrum.
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    ax1.plot(t_uniform, pdc_uniform, linewidth=0.7)
    ax1.set_title(f"PDC steady-state (t ≥ {args.start:.0f}s) — "
                  f"{os.path.basename(args.csv)}")
    ax1.set_xlabel("Time since steady-state start (s)")
    ax1.set_ylabel("PDC (mA)")
    ax1.grid(True, alpha=0.3)

    ax2.loglog(freqs[1:], amps[1:], linewidth=0.7)
    ax2.set_xlabel("Frequency (Hz)")
    ax2.set_ylabel("Amplitude (mA)")
    ax2.set_title("Single-sided amplitude spectrum (Hann-windowed)")
    ax2.grid(True, which="both", alpha=0.3)

    fig.tight_layout()
    out_path = args.out or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        os.path.basename(args.csv).replace("_telemetry.csv", "_pdc_fft.png"),
    )
    fig.savefig(out_path, dpi=120)
    print(f"\nSaved {out_path}")


if __name__ == "__main__":
    main()
