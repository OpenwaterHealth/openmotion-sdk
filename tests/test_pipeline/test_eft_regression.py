"""EFT clean-scan regression — repair stage must be a no-op on clean scans.

Replays clean scans through the pipeline and verifies the timestamp repair
stage did not alter any data: all quality flags are "ok", timestamps are
monotonic, and the row count matches the live-captured baseline.
"""

import csv
import logging
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
    c_min = np.zeros((2, 8))
    c_max = np.zeros((2, 8))
    i_min = np.zeros((2, 8))
    i_max = np.zeros((2, 8))


def _replay_scan(scan_info, output_dir):
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
        pedestals=SensorPedestals(left=128.0, right=128.0),
    )
    sink = CsvSink(output_dir=str(output_dir))
    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    runner.run()
    csvs = list(Path(output_dir).glob("*.csv"))
    corrected = [c for c in csvs if "_raw" not in c.name]
    assert len(corrected) == 1, f"Expected 1 corrected CSV, found {corrected}"
    return corrected[0]


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", CLEAN_SCANS, ids=[s["name"] for s in CLEAN_SCANS])
def test_clean_scan_quality_all_ok(scan_info, tmp_path):
    """All quality flags must be 'ok' — the repair stage is a no-op on clean scans."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_scan(scan_info, tmp_path)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert len(rows) > 0, "Corrected CSV is empty"
    assert "quality" in reader.fieldnames
    non_ok = [(i, r["quality"]) for i, r in enumerate(rows) if r["quality"] != "ok"]
    assert len(non_ok) == 0, (
        f"{len(non_ok)} rows with non-ok quality (first 5: {non_ok[:5]})"
    )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", CLEAN_SCANS, ids=[s["name"] for s in CLEAN_SCANS])
def test_clean_scan_monotonic_timestamps(scan_info, tmp_path):
    """Corrected CSV timestamps must be monotonic non-decreasing."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_scan(scan_info, tmp_path)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        timestamps = [float(row["timestamp_s"]) for row in reader]
    for i in range(1, len(timestamps)):
        assert timestamps[i] >= timestamps[i - 1], (
            f"Non-monotonic at row {i}: {timestamps[i-1]} > {timestamps[i]}"
        )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", CLEAN_SCANS, ids=[s["name"] for s in CLEAN_SCANS])
def test_clean_scan_row_count_matches_baseline(scan_info, tmp_path):
    """Row count should match the live-captured baseline corrected CSV."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_scan(scan_info, tmp_path)
    with open(csv_path) as f:
        new_count = sum(1 for _ in csv.reader(f)) - 1  # subtract header
    with open(scan_info["baseline_corrected"]) as f:
        baseline_count = sum(1 for _ in csv.reader(f)) - 1
    assert new_count == baseline_count, (
        f"Row count mismatch: new={new_count}, baseline={baseline_count}"
    )
