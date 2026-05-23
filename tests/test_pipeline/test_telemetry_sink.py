"""Tests for TelemetrySink — writes the per-scan _telemetry.csv."""

import csv
import pytest
from omotion.pipeline.batch import TelemetryEvent
from omotion.pipeline.sinks import TelemetrySink, ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="abc", subject_id="x", operator="bloodflow-app",
        started_at_iso="2026-05-22T10:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )


def _ev(t, pdc):
    return TelemetryEvent(
        timestamp_s=t, pdc_samples=[pdc, pdc + 0.01, pdc + 0.02],
        tec_setpoint_c=25.0, tec_actual_c=25.1, console_temp_c=37.4,
        fan_rpm=2400, safety_status=0,
    )


def test_telemetry_sink_header(tmp_path):
    sink = TelemetrySink(output_path=str(tmp_path / "test_telemetry.csv"))
    sink.on_scan_start(_meta())
    sink.on_complete()
    with open(tmp_path / "test_telemetry.csv") as f:
        header = next(csv.reader(f))
    assert header == [
        "timestamp_s", "pdc_samples_ma", "tec_setpoint_c",
        "tec_actual_c", "console_temp_c", "fan_rpm", "safety_status",
    ]


def test_telemetry_sink_writes_one_row_per_event(tmp_path):
    sink = TelemetrySink(output_path=str(tmp_path / "test_telemetry.csv"))
    sink.on_scan_start(_meta())
    sink.consume("telemetry", _ev(0.0, 1.10))
    sink.consume("telemetry", _ev(0.1, 1.11))
    sink.on_complete()
    with open(tmp_path / "test_telemetry.csv") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 2
    assert float(rows[0]["timestamp_s"]) == 0.0
    assert float(rows[0]["tec_setpoint_c"]) == 25.0
