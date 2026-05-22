"""New CsvSink — channel-based, with the 'type' column in raw output."""

import csv
import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.sinks import CsvSink, ScanMetadata


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
