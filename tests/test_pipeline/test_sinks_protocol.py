"""Tests for the Sink protocol and ScanMetadata."""

import pytest
from omotion.pipeline.sinks import Sink, ScanMetadata


def test_scan_metadata_carries_scan_identification_fields():
    meta = ScanMetadata(
        scan_id="abc-123",
        subject_id="subj-001",
        operator="bloodflow-app",
        started_at_iso="2026-05-22T10:00:00Z",
        duration_sec=300,
        left_camera_mask=0x66,
        right_camera_mask=0x66,
        reduced_mode=True,
    )
    assert meta.scan_id == "abc-123"
    assert meta.reduced_mode is True


def test_scan_metadata_basic_construction():
    """ScanMetadata basic construction without raw CSV fields."""
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T10:00:00Z",
        duration_sec=300, left_camera_mask=0, right_camera_mask=0xFF,
        reduced_mode=False,
    )
    assert meta.scan_id == "x"


class _MinimalSink:
    channels = {"live"}
    def on_scan_start(self, meta): pass
    def consume(self, channel, payload): pass
    def on_complete(self): pass


def test_sink_protocol_runtime_check_accepts_a_minimal_implementation():
    sink = _MinimalSink()
    assert isinstance(sink, Sink)


def test_sink_protocol_runtime_check_rejects_missing_methods():
    class _Bad:
        channels = {"live"}
        def on_scan_start(self, meta): pass
        def on_complete(self): pass
    assert not isinstance(_Bad(), Sink)


def test_scan_metadata_does_not_carry_raw_csv_fields():
    import dataclasses
    field_names = {f.name for f in dataclasses.fields(ScanMetadata)}
    assert "write_raw_csv" not in field_names
    assert "raw_csv_duration_sec" not in field_names


def test_scan_metadata_constructs_without_raw_csv_kwargs():
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T10:00:00Z", duration_sec=300,
        left_camera_mask=0x66, right_camera_mask=0x66, reduced_mode=True,
    )
    assert meta.scan_id == "x"
