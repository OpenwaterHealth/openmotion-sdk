"""Functional test: feed a recorded raw-CSV scan through the new pipeline
and diff the corrected output per-cell against the legacy SciencePipeline's
output.

Run:
    python scripts/test_pipeline_real_scan.py \\
        --scan 20260519_161117_owW1WI5T \\
        --scan-dir C:/Users/ethan/Projects/scan_data \\
        --calibration C:/Users/ethan/Projects/scan_data/calibrations/calibration-20260519_160651.json

Loads the raw CSVs through the new CsvReplaySource (legacy schema works —
the reader only needs cam_id, frame_id, timestamp_s, the 1024 bin columns,
and temperature; the new 'type' column is optional). Runs default_pipeline.
Writes a corrected CSV via CsvSink (now in the legacy wide format with
per-cam columns). Then loads both CSVs and diffs each per-cam column.

Output: per-column abs-diff stats over the rows the two files share.
"""

from __future__ import annotations

import argparse
import csv
import json
import pathlib
import sys
import tempfile
from dataclasses import dataclass

import numpy as np


@dataclass
class _Cal:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


def load_calibration(path: str) -> _Cal:
    with open(path) as f:
        d = json.load(f)
    c = d["calibration"]
    return _Cal(
        c_min=np.array(c["c_min"], dtype=np.float32),
        c_max=np.array(c["c_max"], dtype=np.float32),
        i_min=np.array(c["i_min"], dtype=np.float32),
        i_max=np.array(c["i_max"], dtype=np.float32),
    )


def detect_mask(scan_dir: pathlib.Path, scan_id: str, side: str) -> int:
    candidates = sorted(scan_dir.glob(f"{scan_id}_{side}_mask*_raw.csv"))
    if not candidates:
        raise FileNotFoundError(f"no raw csv for {side} found under {scan_dir}/{scan_id}_*")
    name = candidates[0].name
    mask_str = name.split(f"_{side}_mask", 1)[1].split("_raw", 1)[0]
    return int(mask_str, 16)


def run_new_pipeline(scan_dir: pathlib.Path, scan_id: str, calibration: _Cal,
                     out_dir: pathlib.Path) -> pathlib.Path:
    from omotion.pipeline.factory import default_pipeline
    from omotion.pipeline.runner import ScanRunner
    from omotion.pipeline.sources import CsvReplaySource
    from omotion.pipeline.sinks import CsvSink, ScanMetadata
    from omotion.pipeline.pedestal import SensorPedestals

    left_csv = next(scan_dir.glob(f"{scan_id}_left_mask*_raw.csv"))
    right_csv = next(scan_dir.glob(f"{scan_id}_right_mask*_raw.csv"))
    left_mask = detect_mask(scan_dir, scan_id, "left")
    right_mask = detect_mask(scan_dir, scan_id, "right")

    meta = ScanMetadata(
        scan_id=scan_id, subject_id="real", operator="test",
        started_at_iso="2026-05-19T16:11:17Z", duration_sec=30,
        left_camera_mask=left_mask, right_camera_mask=right_mask,
        reduced_mode=False, write_raw_csv=False, raw_csv_duration_sec=None,
    )

    pedestals = SensorPedestals(left=128.0, right=128.0)

    pipeline = default_pipeline(
        metadata=meta, calibration=calibration, pedestals=pedestals,
    )

    source = CsvReplaySource(
        raw_csv_left=left_csv, raw_csv_right=right_csv,
        batch_size_frames=100, metadata=meta,
    )
    sink = CsvSink(output_dir=out_dir)

    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    print(f"  running pipeline on raw CSVs (left mask 0x{left_mask:02x},"
          f" right mask 0x{right_mask:02x})...")
    runner.run()

    out = [p for p in out_dir.glob("*.csv") if not p.name.endswith("_raw.csv")]
    if not out:
        raise RuntimeError("CsvSink produced no corrected CSV")
    return out[0]


def numeric_diff(new_csv: pathlib.Path, legacy_csv: pathlib.Path,
                 left_mask: int, right_mask: int) -> None:
    """Match rows by frame_id and diff per-cell across BFI/BVI/mean/contrast."""

    def _load(path):
        with open(path) as fh:
            rdr = csv.DictReader(fh)
            return list(rdr), rdr.fieldnames or []

    new_rows, new_cols = _load(new_csv)
    leg_rows, leg_cols = _load(legacy_csv)

    print()
    print("-" * 78)
    print(f"Legacy:  {legacy_csv.name}  ({len(leg_rows)} rows, {len(leg_cols)} cols)")
    print(f"New:     {new_csv.name}     ({len(new_rows)} rows, {len(new_cols)} cols)")
    print("-" * 78)

    # Build maps: frame_id -> row
    new_by_frame = {int(r["frame_id"]): r for r in new_rows if r.get("frame_id")}
    leg_by_frame = {int(r["frame_id"]): r for r in leg_rows if r.get("frame_id")}

    common = sorted(set(new_by_frame.keys()) & set(leg_by_frame.keys()))
    only_new = sorted(set(new_by_frame.keys()) - set(leg_by_frame.keys()))
    only_leg = sorted(set(leg_by_frame.keys()) - set(new_by_frame.keys()))

    print(f"Common frame_ids:  {len(common)}")
    print(f"Only in new:       {len(only_new)}  (first few: {only_new[:5]})")
    print(f"Only in legacy:    {len(only_leg)}  (first few: {only_leg[:5]})")
    print()

    if not common:
        print("No frame overlap — cannot diff. Aborting numeric comparison.")
        return

    # Active cameras from masks (1-indexed for column names)
    def active_cams(mask: int) -> list[int]:
        return [i for i in range(8) if mask & (1 << i)]

    left_active  = active_cams(left_mask)
    right_active = active_cams(right_mask)

    # Per-column abs-diff stats
    metric_groups = ["bfi", "bvi", "mean", "contrast"]
    print(f"{'column':<14} {'n':>6} {'mean |Δ|':>12} {'median |Δ|':>12} "
          f"{'max |Δ|':>12} {'leg mean':>10} {'new mean':>10}")
    print("-" * 80)

    issues = []  # (column, severity, message)

    for metric in metric_groups:
        for side_letter, cam_set in (("l", left_active), ("r", right_active)):
            for cam in cam_set:
                col = f"{metric}_{side_letter}{cam + 1}"
                if col not in new_cols or col not in leg_cols:
                    continue

                diffs = []
                leg_vals = []
                new_vals = []
                for f in common:
                    l = leg_by_frame[f].get(col, "")
                    n = new_by_frame[f].get(col, "")
                    if not l or not n:
                        continue
                    try:
                        lf = float(l)
                        nf = float(n)
                    except (ValueError, TypeError):
                        continue
                    if not (np.isfinite(lf) and np.isfinite(nf)):
                        continue
                    diffs.append(abs(lf - nf))
                    leg_vals.append(lf)
                    new_vals.append(nf)

                if not diffs:
                    print(f"{col:<14} {0:>6}    (no overlapping numeric values)")
                    continue

                arr = np.array(diffs)
                lm = float(np.mean(leg_vals))
                nm = float(np.mean(new_vals))
                print(f"{col:<14} {len(diffs):>6} {arr.mean():>12.4f} "
                      f"{np.median(arr):>12.4f} {arr.max():>12.4f} "
                      f"{lm:>10.3f} {nm:>10.3f}")

                # Surface big divergences
                if metric in ("bfi", "bvi"):
                    if arr.mean() > 0.5:
                        issues.append((col, "WARN", f"BFI/BVI mean |Δ| = {arr.mean():.3f} > 0.5"))
                    if arr.max() > 2.0:
                        issues.append((col, "WARN", f"BFI/BVI max |Δ| = {arr.max():.3f} > 2.0"))

    print()
    if issues:
        print("Notable divergences:")
        for col, sev, msg in issues:
            print(f"  [{sev}] {col}: {msg}")
    else:
        print("All per-cell diffs within expected tolerance.")
    print("-" * 78)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scan", required=True)
    p.add_argument("--scan-dir", required=True, type=pathlib.Path)
    p.add_argument("--calibration", required=True)
    args = p.parse_args()

    scan_dir = args.scan_dir
    scan_id = args.scan
    legacy_corrected = scan_dir / f"{scan_id}.csv"
    if not legacy_corrected.exists():
        print(f"ERROR: legacy corrected CSV not found at {legacy_corrected}")
        return 1

    print(f"Scan id:           {scan_id}")
    print(f"Scan dir:          {scan_dir}")
    print(f"Legacy corrected:  {legacy_corrected}")
    print(f"Calibration:       {args.calibration}")
    print()

    cal = load_calibration(args.calibration)
    left_mask = detect_mask(scan_dir, scan_id, "left")
    right_mask = detect_mask(scan_dir, scan_id, "right")

    with tempfile.TemporaryDirectory() as tmp:
        out_dir = pathlib.Path(tmp)
        new_corrected = run_new_pipeline(scan_dir, scan_id, cal, out_dir)
        print(f"  -> new corrected CSV: {new_corrected.name} "
              f"({new_corrected.stat().st_size:,} bytes)")
        numeric_diff(new_corrected, legacy_corrected, left_mask, right_mask)

    return 0


if __name__ == "__main__":
    sys.exit(main())
