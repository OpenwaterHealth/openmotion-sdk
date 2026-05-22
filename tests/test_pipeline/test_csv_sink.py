"""New CsvSink — channel-based, with the 'type' column in raw output."""

import csv
import numpy as np
import pytest
from dataclasses import dataclass
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.sinks import CsvSink, ScanMetadata, _NORMAL_HEADERS, _REDUCED_HEADERS
from omotion.pipeline.stages.dark import EnrichedCorrectedInterval, EnrichedCorrectedFrame


def _meta_with_raw(write_raw, duration):
    return ScanMetadata(
        scan_id="abc", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=300,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=write_raw, raw_csv_duration_sec=duration,
    )


def _dummy_raw_batch():
    """Minimal FrameBatch with one frame in side=0 cam=0, with type='light'."""
    n = 1
    raw = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw[0, 0, 0, 100] = 2_457_606
    return FrameBatch(
        cam_ids=np.array([0], dtype=np.int8),
        frame_ids=np.array([10], dtype=np.uint8),
        raw_histograms=raw,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([0.25], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([10], dtype=np.int64),
        frame_type=np.array(["light"], dtype="<U8"),
    )


def test_csv_sink_skips_raw_when_write_raw_csv_is_false(tmp_path):
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_with_raw(write_raw=False, duration=None))
    sink.consume("raw", _dummy_raw_batch())
    sink.on_complete()
    raw_files = list(tmp_path.glob("*raw*.csv"))
    assert raw_files == []


def test_csv_sink_writes_type_column_when_raw_enabled(tmp_path):
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_with_raw(write_raw=True, duration=None))
    sink.consume("raw", _dummy_raw_batch())
    sink.on_complete()
    raw_files = list(tmp_path.glob("*raw*.csv"))
    assert len(raw_files) >= 1
    with open(raw_files[0]) as fh:
        header = next(csv.reader(fh))
    assert "type" in header
    assert header.index("type") == 3


def test_csv_sink_raw_duration_cap_stops_writes(tmp_path):
    """Frames with timestamp_s > raw_csv_duration_sec should not be written."""
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_with_raw(write_raw=True, duration=0.10))

    # batch within cap
    early = _dummy_raw_batch()  # timestamp_s = 0.25 — EXCEEDS 0.10
    sink.consume("raw", early)
    sink.on_complete()

    raw_files = list(tmp_path.glob("*raw*.csv"))
    # File may exist (was opened) but should have no data rows
    if raw_files:
        with open(raw_files[0]) as fh:
            rows = list(csv.reader(fh))
        # Only the header row, no data
        assert len(rows) <= 1


def test_csv_sink_raw_writes_data_rows_within_cap(tmp_path):
    """A batch within the duration cap should write data rows."""
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_with_raw(write_raw=True, duration=10.0))

    batch = _dummy_raw_batch()  # timestamp_s = 0.25 — within 10s cap
    sink.consume("raw", batch)
    sink.on_complete()

    raw_files = list(tmp_path.glob("*raw*.csv"))
    assert len(raw_files) >= 1
    with open(raw_files[0]) as fh:
        rows = list(csv.reader(fh))
    # Header + at least one data row (one cam per frame that has mask bit set)
    assert len(rows) >= 2


def test_csv_sink_channels_attribute():
    sink = CsvSink(output_dir=".")
    assert "raw" in sink.channels
    assert "final" in sink.channels


# ---------------------------------------------------------------------------
# Corrected (final) channel — wide format tests
# ---------------------------------------------------------------------------

def _make_enriched_interval(side: str = "left", cam_id: int = 0,
                             abs_frame_id: int = 10, t: float = 0.25):
    """Helper: one-frame EnrichedCorrectedInterval."""
    f = EnrichedCorrectedFrame(
        abs_frame_id=abs_frame_id, t=t,
        side=side, cam_id=cam_id,
        mean=100.0, std=5.0, contrast=0.05,
        bfi=7.5, bvi=8.0,
    )
    return EnrichedCorrectedInterval(left_abs=0, right_abs=20, frames=[f])


def _meta_normal():
    return ScanMetadata(
        scan_id="test", subject_id="s", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0x01,   # cam 0 only
        right_camera_mask=0,
        reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )


def _meta_reduced():
    return ScanMetadata(
        scan_id="test_red", subject_id="s", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0x01,
        right_camera_mask=0,
        reduced_mode=True,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )


def test_corrected_csv_normal_mode_header_has_82_columns(tmp_path):
    """Normal mode corrected CSV must have exactly 82 columns matching legacy format."""
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_normal())
    sink.consume("final", _make_enriched_interval())
    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    assert len(files) == 1
    with open(files[0]) as fh:
        header = next(csv.reader(fh))
    assert header == _NORMAL_HEADERS, (
        f"Header mismatch.\nExpected: {_NORMAL_HEADERS}\nGot:      {header}"
    )
    assert len(header) == 82


def test_corrected_csv_reduced_mode_header_has_6_columns(tmp_path):
    """Reduced mode corrected CSV must have 6 columns."""
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_reduced())
    sink.consume("final", _make_enriched_interval())
    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    assert len(files) == 1
    with open(files[0]) as fh:
        header = next(csv.reader(fh))
    assert header == _REDUCED_HEADERS
    assert len(header) == 6


def test_corrected_csv_normal_mode_places_values_in_correct_columns(tmp_path):
    """bfi_l1 must appear in the correct column for cam_id=0 side=left."""
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_normal())
    # cam_id=0 -> l1; side=left
    sink.consume("final", _make_enriched_interval(side="left", cam_id=0,
                                                    abs_frame_id=5, t=0.1))
    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    with open(files[0]) as fh:
        rows = list(csv.reader(fh))
    assert len(rows) >= 2, "No data rows produced"
    header = rows[0]
    data = rows[1]
    bfi_l1_idx = header.index("bfi_l1")
    mean_l1_idx = header.index("mean_l1")
    contrast_l1_idx = header.index("contrast_l1")
    assert float(data[bfi_l1_idx]) == pytest.approx(7.5)
    assert float(data[mean_l1_idx]) == pytest.approx(100.0)
    assert float(data[contrast_l1_idx]) == pytest.approx(0.05)


def test_corrected_csv_normal_mode_right_side_cam3(tmp_path):
    """cam_id=2 side=right -> r3 columns."""
    meta = ScanMetadata(
        scan_id="right_test", subject_id="s", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0,
        right_camera_mask=0x04,  # cam 2 only
        reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(meta)
    sink.consume("final", _make_enriched_interval(side="right", cam_id=2,
                                                    abs_frame_id=7, t=0.175))
    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    with open(files[0]) as fh:
        rows = list(csv.reader(fh))
    header = rows[0]
    data = rows[1]
    bfi_r3_idx = header.index("bfi_r3")
    assert float(data[bfi_r3_idx]) == pytest.approx(7.5)


def test_corrected_csv_reduced_mode_populates_left_columns(tmp_path):
    """Reduced mode: left-side frame populates bfi_left/bvi_left columns."""
    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(_meta_reduced())
    sink.consume("final", _make_enriched_interval(side="left", cam_id=0))
    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    with open(files[0]) as fh:
        rows = list(csv.reader(fh))
    header = rows[0]
    data = rows[1]
    assert float(data[header.index("bfi_left")]) == pytest.approx(7.5)
    assert float(data[header.index("bvi_left")]) == pytest.approx(8.0)


def test_csv_sink_flushes_row_when_all_expected_cams_contribute_including_nan(tmp_path):
    """Row must flush when all expected cameras have contributed, even if some
    values (bfi, bvi) are NaN.  The completion check uses the 'mean' column
    presence, not whether values are finite.

    Scenario: 2 expected cams (cam0 left, cam1 left).
    cam0 contributes normal values; cam1 contributes with mean=0 which still
    counts as 'contributed' (non-empty string).  Both rows should flush.
    """
    meta = ScanMetadata(
        scan_id="nan_test", subject_id="s", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0x03,  # cams 0 and 1
        right_camera_mask=0,
        reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )

    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(meta)

    abs_id = 42

    # cam0: normal values
    f0 = EnrichedCorrectedFrame(
        abs_frame_id=abs_id, t=1.05,
        side="left", cam_id=0,
        mean=200.0, std=5.0, contrast=0.025,
        bfi=8.0, bvi=7.5,
    )
    iv0 = EnrichedCorrectedInterval(left_abs=0, right_abs=100, frames=[f0])
    sink.consume("final", iv0)

    # Row should NOT flush yet (cam1 hasn't contributed)
    assert abs_id in sink._corrected_acc, (
        "Accumulator should still hold the partial row"
    )

    # cam1: mean=0.0 (zero is still a valid contribution, not empty)
    f1 = EnrichedCorrectedFrame(
        abs_frame_id=abs_id, t=1.05,
        side="left", cam_id=1,
        mean=0.0, std=0.0, contrast=0.0,
        bfi=0.0, bvi=0.0,
    )
    iv1 = EnrichedCorrectedInterval(left_abs=0, right_abs=100, frames=[f1])
    sink.consume("final", iv1)

    # Now all expected cams have contributed — row should have flushed
    assert abs_id not in sink._corrected_acc, (
        "Row should have been flushed once all expected cams contributed"
    )

    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    assert len(files) == 1
    with open(files[0]) as fh:
        rows = list(csv.reader(fh))
    # 1 header + 1 data row
    assert len(rows) == 2, f"Expected 1 data row, got {len(rows) - 1}"
    header = rows[0]
    data = rows[1]
    # cam0 values
    assert float(data[header.index("mean_l1")]) == pytest.approx(200.0)
    assert float(data[header.index("bfi_l1")]) == pytest.approx(8.0)
    # cam1 values (zero)
    assert float(data[header.index("mean_l2")]) == pytest.approx(0.0)


def test_csv_sink_partial_row_flushed_on_complete(tmp_path):
    """on_complete flushes any partial rows (not all cams contributed).

    Scenario: 2 expected cams, only 1 contributes. on_complete should
    flush the partial row anyway.
    """
    meta = ScanMetadata(
        scan_id="partial_test", subject_id="s", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0x03,  # cams 0 and 1
        right_camera_mask=0,
        reduced_mode=False,
        write_raw_csv=False, raw_csv_duration_sec=None,
    )

    sink = CsvSink(output_dir=tmp_path)
    sink.on_scan_start(meta)

    abs_id = 55
    f0 = EnrichedCorrectedFrame(
        abs_frame_id=abs_id, t=1.375,
        side="left", cam_id=0,
        mean=150.0, std=4.0, contrast=0.027,
        bfi=7.8, bvi=6.5,
    )
    iv = EnrichedCorrectedInterval(left_abs=0, right_abs=100, frames=[f0])
    sink.consume("final", iv)

    # Only 1 of 2 expected cams — row should NOT flush yet
    assert abs_id in sink._corrected_acc

    sink.on_complete()

    files = list(tmp_path.glob("*corrected*.csv"))
    assert len(files) == 1
    with open(files[0]) as fh:
        rows = list(csv.reader(fh))
    # Should still emit the partial row via on_complete flush
    assert len(rows) == 2, f"Expected 1 data row from partial flush, got {len(rows) - 1}"
    header = rows[0]
    data = rows[1]
    assert float(data[header.index("mean_l1")]) == pytest.approx(150.0)
    # cam1 (not contributed) should be empty
    assert data[header.index("mean_l2")] == ""
