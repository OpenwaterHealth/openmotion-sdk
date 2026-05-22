"""Tests for ScanWorkflow and ScanRequest."""

import dataclasses
from omotion.ScanWorkflow import ScanRequest


def test_scan_request_carries_sinks_field():
    """ScanRequest should carry sinks, skip_default_storage, raw_save_max_duration_s fields."""
    field_names = {f.name for f in dataclasses.fields(ScanRequest)}
    assert "sinks" in field_names
    assert "skip_default_storage" in field_names
    assert "raw_save_max_duration_s" in field_names


def test_scan_request_drops_legacy_fields():
    """ScanRequest should not carry legacy callback/csv/operator fields."""
    field_names = {f.name for f in dataclasses.fields(ScanRequest)}
    assert "on_uncorrected_fn" not in field_names
    assert "on_corrected_batch_fn" not in field_names
    assert "on_dark_frame_fn" not in field_names
    assert "on_rolling_avg_fn" not in field_names
    assert "on_realtime_corrected_fn" not in field_names
    assert "on_raw_frame_fn" not in field_names
    assert "write_raw_csv" not in field_names
    assert "raw_csv_duration_sec" not in field_names
    assert "operator" not in field_names


def test_scan_request_sinks_default_empty_list():
    """ScanRequest should default to empty sinks list and allow construction without these fields."""
    req = ScanRequest(
        subject_id="x", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        disable_laser=False,
    )
    assert req.sinks == []
    assert req.skip_default_storage is False
    assert req.raw_save_max_duration_s is None
