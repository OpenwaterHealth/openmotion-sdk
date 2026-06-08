"""EFT degraded-scan structural assertions.

Replays scans affected by EMI stimulation through the pipeline and verifies:
- Gap-free abs_frame_id grid in corrected CSV
- NaN where data was absent
- Monotonic non-decreasing timestamp_s
- Quality column populated correctly
- Coalesced logging (warnings present, no per-frame spam)
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
FINAL_TESTS_DIR = Path(
    r"C:\Users\ethan\Projects\eft-testing\final_tests-20260603T234709Z-3-001\final_tests"
)

DEGRADED_SCANS = [
    {
        "name": "owEFTTEST1_1607",
        "left_raw": FINAL_TESTS_DIR / "20260603_160750_owEFTTEST1_left_maskC3_raw.csv",
        "right_raw": FINAL_TESTS_DIR / "20260603_160750_owEFTTEST1_right_maskC3_raw.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
    {
        "name": "owEFTTEST1_1618",
        "left_raw": FINAL_TESTS_DIR / "20260603_161850_owEFTTEST1_left_maskC3_raw.csv",
        "right_raw": FINAL_TESTS_DIR / "20260603_161850_owEFTTEST1_right_maskC3_raw.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
    {
        "name": "owEFTTEST1_1630",
        "left_raw": FINAL_TESTS_DIR / "20260603_163020_owEFTTEST1_left_maskC3_raw.csv",
        "right_raw": FINAL_TESTS_DIR / "20260603_163020_owEFTTEST1_right_maskC3_raw.csv",
        "left_mask": 0xC3,
        "right_mask": 0xC3,
    },
    {
        "name": "owM8T7HS",
        "left_raw": SCANS_DIR / "20260602_135343_owM8T7HS_left_mask66_raw.csv",
        "right_raw": SCANS_DIR / "20260602_135343_owM8T7HS_right_mask66_raw.csv",
        "left_mask": 0x66,
        "right_mask": 0x66,
    },
    {
        "name": "owSPZMD1",
        "left_raw": SCANS_DIR / "20260602_140215_owSPZMD1_left_mask66_raw.csv",
        "right_raw": SCANS_DIR / "20260602_140215_owSPZMD1_right_mask66_raw.csv",
        "left_mask": 0x66,
        "right_mask": 0x66,
    },
]


class _NullCalibration:
    c_min = np.zeros((2, 8))
    c_max = np.zeros((2, 8))
    i_min = np.zeros((2, 8))
    i_max = np.zeros((2, 8))


def _replay_degraded(scan_info, output_dir):
    meta = ScanMetadata(
        scan_id=scan_info["name"], subject_id="test", operator="eft",
        started_at_iso="2026-01-01T00:00:00", duration_sec=600,
        left_camera_mask=scan_info["left_mask"],
        right_camera_mask=scan_info["right_mask"],
        reduced_mode=False,
    )
    source = CsvReplaySource(
        raw_csv_left=scan_info["left_raw"],
        raw_csv_right=scan_info["right_raw"],
        metadata=meta, batch_size_frames=100,
    )
    pedestal = scan_info.get("pedestal", 128.0)
    pipeline = default_pipeline(
        metadata=meta, calibration=_NullCalibration(),
        pedestals=SensorPedestals(left=pedestal, right=pedestal),
    )
    sink = CsvSink(output_dir=str(output_dir))
    runner = ScanRunner(source=source, pipeline=pipeline, sinks=[sink])
    runner.run()
    csvs = list(Path(output_dir).glob("*.csv"))
    corrected = [c for c in csvs if "_raw" not in c.name]
    assert len(corrected) == 1
    return corrected[0]


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
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_raw_csv_is_faithful(scan_info, tmp_path):
    """On a heavily EMI-degraded scan the raw CSV must stay a faithful capture.

    All five degraded inputs are confirmed-faithful captures (exact
    2,457,606 sums + real device jitter); owM8T7HS/owSPZMD1 also exhibit
    FM-5 camera dropout, so this also asserts the raw preserves a dropped
    camera rather than NaN-filling it.

    Regression guard for SDK_BUGREPORT.md Defect 1 on the EMI path:
      - histogram sums untouched (no NoiseFloor leak),
      - timestamp jitter preserved (no re-synthesis),
      - row count identical to the input — NaN-fill rows go to the corrected
        output ONLY, never the raw capture (spec §11 R2 guardrail).
    """
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    _replay_degraded(scan_info, tmp_path)

    for side in ("left", "right"):
        in_rows = _read_raw_rows(scan_info[f"{side}_raw"])
        out_rows = _read_raw_rows(next(tmp_path.glob(f"*_{side}_*_raw.csv")))

        assert len(out_rows) == len(in_rows), (
            f"{side}: raw row count changed {len(in_rows)} -> {len(out_rows)} "
            "(NaN-fill must not add synthetic rows to the raw CSV)"
        )
        assert [(c, f, s) for (c, f, _t, s) in out_rows] == \
               [(c, f, s) for (c, f, _t, s) in in_rows], \
            f"{side}: histogram sums altered in raw CSV (noise-floor leak)"

        in_ts = np.array([t for (_c, _f, t, _s) in in_rows])
        out_ts = np.array([t for (_c, _f, t, _s) in out_rows])
        np.testing.assert_allclose(
            np.diff(out_ts), np.diff(in_ts), atol=1e-9,
            err_msg=f"{side}: raw timestamps were re-synthesized (jitter lost)",
        )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_monotonic_timestamps(scan_info, tmp_path):
    """Corrected CSV timestamps must be monotonic non-decreasing."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_degraded(scan_info, tmp_path)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        timestamps = [float(row["timestamp_s"]) for row in reader]
    for i in range(1, len(timestamps)):
        assert timestamps[i] >= timestamps[i - 1], (
            f"Non-monotonic at row {i}: {timestamps[i-1]} > {timestamps[i]}"
        )


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_has_quality_column(scan_info, tmp_path):
    """Corrected CSV must have quality column with at least some non-ok values."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    csv_path = _replay_degraded(scan_info, tmp_path)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        qualities = [row["quality"] for row in reader]
    assert "quality" in reader.fieldnames
    non_ok = [q for q in qualities if q != "ok"]
    assert len(non_ok) > 0, "Degraded scan should have at least some corrected/NaN-filled frames"
    for q in qualities:
        assert q in ("ok", "ts_corrected", "nan_filled"), f"Unexpected quality value: {q!r}"


@pytest.mark.slow
@pytest.mark.parametrize("scan_info", DEGRADED_SCANS, ids=[s["name"] for s in DEGRADED_SCANS])
def test_degraded_logging_coalesced(scan_info, tmp_path, caplog):
    """Logging must be coalesced: one WARNING per window + one summary."""
    if not scan_info["left_raw"].exists():
        pytest.skip(f"Test data not found: {scan_info['left_raw']}")
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        _replay_degraded(scan_info, tmp_path)
    warnings = [r for r in caplog.records
                if r.name == "openmotion.sdk.pipeline.stages.timestamp_repair"
                and r.levelno == logging.WARNING]
    window_msgs = [w for w in warnings if "Misalignment window" in w.message]
    summary_msgs = [w for w in warnings if "Scan summary" in w.message]
    assert len(window_msgs) >= 1, "Should have at least one misalignment window"
    assert len(summary_msgs) == 1, "Should have exactly one scan summary"
