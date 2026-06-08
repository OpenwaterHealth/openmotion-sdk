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


def _read_raw_rows(path):
    """Read (cam_id, frame_id, timestamp_s, sum) tuples in file order."""
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append((
                int(r["cam_id"]), int(r["frame_id"]),
                float(r["timestamp_s"]), int(r["sum"]),
            ))
    return rows


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", CLEAN_SCANS, ids=[s["name"] for s in CLEAN_SCANS])
def test_raw_csv_is_faithful_capture(scan_info, tmp_path):
    """The raw CSV must be a byte-faithful capture, NOT processed data.

    Regression guard for SDK_BUGREPORT.md Defect 1: the raw tee captured the
    batch by reference, so downstream in-place mutation leaked into the raw
    CSV — NoiseFloorStage reduced the histogram sums (1b) and
    TimestampRepairStage synthesized a smooth timestamp grid (1a). The fix
    snapshots the batch at the raw tee.

    Asserts the replayed raw output matches the input capture:
      - histogram sums identical row-by-row (no noise-floor leak)
      - timestamp *structure* identical (deltas match; only the t=0 origin
        shifts under the source's documented normalization). A synthesized
        grid would flatten the jitter and fail the delta check.
    """
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    _replay_scan(scan_info, tmp_path)

    for side in ("left", "right"):
        in_path = scan_info[f"{side}_raw"]
        out_path = next(tmp_path.glob(f"*_{side}_*_raw.csv"))
        in_rows = _read_raw_rows(in_path)
        out_rows = _read_raw_rows(out_path)

        assert len(out_rows) == len(in_rows), (
            f"{side}: raw row count changed {len(in_rows)} -> {len(out_rows)} "
            "(no synthetic rows may be added to the raw CSV)"
        )

        # 1b: histogram sums must be untouched (cam_id, frame_id, sum).
        in_sums = [(c, f, s) for (c, f, _t, s) in in_rows]
        out_sums = [(c, f, s) for (c, f, _t, s) in out_rows]
        assert out_sums == in_sums, (
            f"{side}: histogram sums altered in raw CSV (noise-floor leak)"
        )

        # 1a: timestamp jitter structure must survive (deltas match; origin
        # may shift by a constant under t0 normalization).
        in_ts = np.array([t for (_c, _f, t, _s) in in_rows])
        out_ts = np.array([t for (_c, _f, t, _s) in out_rows])
        np.testing.assert_allclose(
            np.diff(out_ts), np.diff(in_ts), atol=1e-9,
            err_msg=f"{side}: raw timestamps were re-synthesized (jitter lost)",
        )
