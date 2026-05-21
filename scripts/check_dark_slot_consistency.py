#!/usr/bin/env python3
"""Compare a telemetry CSV's dark_slot column against the science pipeline's
predicted dark-frame schedule.

Usage:
    python scripts/check_dark_slot_consistency.py <telemetry.csv> \
        [--discard-count 9] [--dark-interval 600]

Reports:
    - Total rows
    - Rows where firmware dark_slot disagrees with the predicted schedule
    - First 10 mismatched frame_idx values (if any)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd


def predicted_dark(n: int, discard_count: int, dark_interval: int) -> bool:
    if n <= discard_count:
        return False
    if n == discard_count + 1:
        return True
    return (n - 1) % dark_interval == 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path)
    p.add_argument("--discard-count", type=int, default=9)
    p.add_argument("--dark-interval", type=int, default=600)
    args = p.parse_args()

    df = pd.read_csv(args.csv)
    required = {"frame_idx", "dark_slot"}
    if not required.issubset(df.columns):
        print(f"ERROR: CSV missing columns {required - set(df.columns)}", file=sys.stderr)
        return 2

    df = df[df["frame_idx"].notna()].copy()
    df["predicted_dark"] = df["frame_idx"].astype(int).apply(
        lambda n: predicted_dark(n, args.discard_count, args.dark_interval)
    )
    df["fw_dark"] = df["dark_slot"].astype(int).astype(bool)
    df["mismatch"] = df["fw_dark"] != df["predicted_dark"]
    # Warmup window (frame_idx <= discard_count): the firmware fires the laser
    # in long-slot here for sensor settling, but the science pipeline discards
    # these frames regardless. Disagreement in this window is expected and
    # cosmetic — report it separately so it doesn't mask real issues.
    df["in_warmup"] = df["frame_idx"].astype(int) <= args.discard_count
    df["science_mismatch"] = df["mismatch"] & (~df["in_warmup"])

    total = len(df)
    n_fw_dark = int(df["fw_dark"].sum())
    n_pred_dark = int(df["predicted_dark"].sum())
    n_warmup_only = int((df["mismatch"] & df["in_warmup"]).sum())
    n_real_mismatch = int(df["science_mismatch"].sum())

    print(f"Rows analyzed:                 {total}")
    print(f"FW dark_slot count:            {n_fw_dark}")
    print(f"Predicted dark count (post-warmup): {n_pred_dark}")
    print(f"Warmup-window darks (cosmetic): {n_warmup_only}")
    print(f"Science-critical mismatches:   {n_real_mismatch}")

    if n_real_mismatch:
        print("\nFirst 10 science-critical mismatches:")
        bad = df.loc[df["science_mismatch"], ["frame_idx", "fw_dark", "predicted_dark"]].head(10)
        print(bad.to_string(index=False))
        return 1
    print("\nOK — firmware and science-pipeline dark-frame schedule agree "
          "on all post-warmup frames.")
    if n_warmup_only:
        print(f"(Note: {n_warmup_only} cosmetic mismatches in the warmup window "
              f"frames 1..{args.discard_count} — firmware marks them dark because "
              f"the laser physically fires in long-slot during sensor warmup. "
              f"Science pipeline discards warmup frames so these don't matter.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
