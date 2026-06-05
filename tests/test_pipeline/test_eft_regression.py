"""EFT clean-scan regression — corrected output must not change for clean scans.

These tests replay clean scans through the pipeline and verify the corrected
CSV is numerically identical to current output (excluding the new quality column).
"""

import csv
from pathlib import Path

import numpy as np
import pytest

from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.runner import ScanRunner
from omotion.pipeline.sinks import CsvSink, ScanMetadata
from omotion.pipeline.sources import CsvReplaySource


SCANS_DIR = Path(r"C:\Users\ethan\Projects\eft-testing\scans")

CLEAN_SCANS = [
    {
        "name": "owYWB8TN",
        "left_raw": SCANS_DIR / "20260602_135759_owYWB8TN_left_mask66_raw.csv",
        "right_raw": SCANS_DIR / "20260602_135759_owYWB8TN_right_mask66_raw.csv",
        "baseline_corrected": SCANS_DIR / "20260602_135759_owYWB8TN.csv",
        "left_mask": 0x66,
        "right_mask": 0x66,
    },
    {
        "name": "owYZ7T66_clean",
        "left_raw": SCANS_DIR / "20260603_130423_owYZ7T66_left_maskC3_raw.csv",
        "right_raw": SCANS_DIR / "20260603_130423_owYZ7T66_right_maskC3_raw.csv",
        "baseline_corrected": SCANS_DIR / "20260603_130423_owYZ7T66.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
]


class _NullCalibration:
    """Identity calibration — BFI = contrast * 10, BVI = mean * 10."""
    c_min = np.zeros((2, 8))
    c_max = np.zeros((2, 8))
    i_min = np.zeros((2, 8))
    i_max = np.zeros((2, 8))


def _replay_scan(scan_info, output_dir):
    """Replay a scan through the full pipeline and return the corrected CSV path."""
    meta = ScanMetadata(
        scan_id=scan_info["name"],
        subject_id="test",
        operator="regression",
        started_at_iso="2026-01-01T00:00:00",
        duration_sec=600,
        left_camera_mask=scan_info["left_mask"],
        right_camera_mask=scan_info["right_mask"],
        reduced_mode=False,
    )
    source = CsvReplaySource(
        raw_csv_left=scan_info["left_raw"],
        raw_csv_right=scan_info["right_raw"],
        metadata=meta,
        batch_size_frames=100,
    )
    pipeline = default_pipeline(
        metadata=meta,
        calibration=_NullCalibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    sink = CsvSink(output_dir=str(output_dir))
    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    runner.run()
    # Find the corrected CSV in output_dir
    csvs = list(Path(output_dir).glob("*.csv"))
    corrected = [c for c in csvs if "_raw" not in c.name]
    assert len(corrected) == 1, f"Expected 1 corrected CSV, found {corrected}"
    return corrected[0]


def _compare_corrected_csvs(new_csv: Path, baseline_csv: Path):
    """Compare corrected CSVs: all pre-existing columns must match numerically."""
    with open(baseline_csv) as f:
        baseline_reader = csv.DictReader(f)
        baseline_rows = list(baseline_reader)
        baseline_fields = baseline_reader.fieldnames

    with open(new_csv) as f:
        new_reader = csv.DictReader(f)
        new_rows = list(new_reader)
        new_fields = new_reader.fieldnames

    # All baseline fields must exist in new (new may have extra like "quality")
    for field in baseline_fields:
        assert field in new_fields, f"Baseline field '{field}' missing from new CSV"

    assert len(new_rows) == len(baseline_rows), (
        f"Row count mismatch: baseline={len(baseline_rows)}, new={len(new_rows)}"
    )

    for i, (new_row, base_row) in enumerate(zip(new_rows, baseline_rows)):
        for field in baseline_fields:
            new_val = new_row[field]
            base_val = base_row[field]
            if new_val == base_val:
                continue
            # Try numeric comparison
            try:
                assert float(new_val) == pytest.approx(float(base_val), abs=1e-6), (
                    f"Row {i}, field '{field}': new={new_val}, baseline={base_val}"
                )
            except (ValueError, TypeError):
                assert new_val == base_val, (
                    f"Row {i}, field '{field}': new={new_val!r}, baseline={base_val!r}"
                )

    # Quality column must be all "ok"
    if "quality" in new_fields:
        for i, row in enumerate(new_rows):
            assert row["quality"] == "ok", (
                f"Row {i}: expected quality='ok', got '{row['quality']}'"
            )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", CLEAN_SCANS, ids=[s["name"] for s in CLEAN_SCANS])
def test_clean_scan_regression(scan_info, tmp_path):
    """Corrected CSV from clean scan must match baseline (excluding quality column)."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    new_csv = _replay_scan(scan_info, tmp_path)
    _compare_corrected_csvs(new_csv, scan_info["baseline_corrected"])
