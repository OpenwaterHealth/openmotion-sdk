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
        write_raw_csv=True,
        raw_csv_duration_sec=60.0,
    )
    assert meta.scan_id == "abc-123"
    assert meta.reduced_mode is True
    assert meta.raw_csv_duration_sec == 60.0


def test_scan_metadata_raw_csv_duration_can_be_none():
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T10:00:00Z",
        duration_sec=300, left_camera_mask=0, right_camera_mask=0xFF,
        reduced_mode=False, write_raw_csv=True, raw_csv_duration_sec=None,
    )
    assert meta.raw_csv_duration_sec is None


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
