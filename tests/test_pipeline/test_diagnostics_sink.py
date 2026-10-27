"""Diagnostics consumers — DiagnosticsLogSink + ScanDBSink session_meta summary.

Integrity events (DarkIntegrityWarning, TerminalDarkResult(found=False),
StencilFallback, PipelineError) must not evaporate: the log sink WARNs on
each, and ScanDBSink folds a per-type summary into the session's
session_meta at scan end. Routine events (TriggerStateEvent, successful
TerminalDarkResult) are excluded from the integrity record.
"""

import json
import logging
import sqlite3
from types import SimpleNamespace

from omotion.pipeline.batch import (
    CameraStreamGap,
    DarkIntegrityWarning,
    FrameIdConsensusCorrection,
    FrameQuarantined,
    PipelineError,
    TerminalDarkResult,
    TriggerStateEvent,
)
from omotion.pipeline.sinks import DiagnosticsLogSink, ScanDBSink, ScanMetadata


def _meta():
    return ScanMetadata(
        scan_id="diag", subject_id="subj", operator="op",
        started_at_iso="2026-06-10T00:00:00Z", duration_sec=60,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
    )


def _dark_warning(abs_id=42):
    return DarkIntegrityWarning(
        side="left", cam_id=0, abs_frame_id=abs_id,
        u1=90.0, pedestal=64.0, threshold=5.0,
    )


def _final_interval():
    """One corrected frame so the session persists as a real scan. Empty scans
    (no corrected rows) are deleted by ScanDBSink — that path is covered by
    test_scan_db_sink_empty.py; these tests exercise the session_meta /
    diagnostics summary on a scan that actually produced rows."""
    frame = SimpleNamespace(
        cam_id=0, side="left", abs_frame_id=1, t=0.1,
        bfi=1.0, bvi=2.0, mean=3.0, contrast=0.4, quality="ok",
    )
    return SimpleNamespace(frames=[frame])


def test_log_sink_warns_on_integrity_events(caplog):
    sink = DiagnosticsLogSink()
    sink.on_scan_start(_meta())
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.sinks"):
        sink.consume("diagnostics", _dark_warning())
        sink.consume("diagnostics", PipelineError(
            error="RuntimeError('x')", n_frames=10, first_timestamp_s=1.0))
        sink.on_complete()
    assert "integrity event" in caplog.text
    assert "DarkIntegrityWarning" in caplog.text
    assert "PipelineError" in caplog.text


def test_log_sink_ignores_routine_events(caplog):
    from omotion.pipeline.batch import TerminalFsyncCount
    sink = DiagnosticsLogSink()
    sink.on_scan_start(_meta())
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.sinks"):
        sink.consume("diagnostics", TriggerStateEvent(state="ON", timestamp_s=0.1))
        sink.consume("diagnostics", TerminalDarkResult(
            side="left", cam_id=0, abs_frame_id=100, u1=64.0,
            threshold=69.0, found=True))
        sink.consume("diagnostics", TerminalFsyncCount(count=842, timestamp_s=60.0))
        sink.on_complete()
    assert caplog.text == ""


def test_log_sink_coalesces_high_volume_frame_integrity_events(caplog):
    sink = DiagnosticsLogSink()
    sink.on_scan_start(_meta())
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.sinks"):
        for fid in range(10, 60):
            sink.consume("diagnostics", FrameIdConsensusCorrection(
                side=0, cam_id=0, timestamp_s=fid * 0.025,
                wire_frame_id=(fid + 1) & 0xFF,
                corrected_frame_id=fid & 0xFF,
                abs_frame_id=fid,
            ))
        for fid in range(60, 70):
            sink.consume("diagnostics", FrameQuarantined(
                side=0, cam_id=0, packet_id=fid,
                timestamp_s=fid * 0.025, wire_frame_id=fid & 0xFF,
                previous_abs_frame_id=fid - 1,
                clock_anchor_abs_frame_id=fid - 1,
                clock_anchor_timestamp_s=(fid - 1) * 0.025,
                step=1, elapsed_s=0.0, reason="timestamp_inconsistent",
            ))
        sink.on_complete()

    warnings = [r.message for r in caplog.records if r.levelno >= logging.WARNING]
    assert len(warnings) == 1
    assert "FrameIdConsensusCorrection×50" in warnings[0]
    assert "FrameQuarantined×10" in warnings[0]


def test_scan_db_sink_writes_diagnostics_summary_to_session_meta(tmp_path):
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta())
    sink.consume("final", _final_interval())
    sink.consume("diagnostics", _dark_warning(abs_id=10))
    sink.consume("diagnostics", _dark_warning(abs_id=610))
    sink.consume("diagnostics", TerminalDarkResult(
        side="left", cam_id=0, abs_frame_id=2400, u1=120.0,
        threshold=69.0, found=False))
    # Routine events must not pollute the summary.
    sink.consume("diagnostics", TriggerStateEvent(state="OFF", timestamp_s=60.0))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    meta = json.loads(conn.execute(
        "SELECT session_meta FROM sessions").fetchone()[0])
    conn.close()
    diag = meta["diagnostics"]
    assert diag["DarkIntegrityWarning"] == {"count": 2, "first": 10, "last": 610}
    assert diag["TerminalDarkResult"]["count"] == 1
    assert "TriggerStateEvent" not in diag
    # The rest of the stamped meta survives the update.
    assert meta["data_semantics"] == "final"
    assert meta["sdk_flags"]["reduced_mode"] is False


def test_scan_db_sink_meta_has_no_diagnostics_key_when_clean(tmp_path):
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta())
    sink.consume("final", _final_interval())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    meta = json.loads(conn.execute(
        "SELECT session_meta FROM sessions").fetchone()[0])
    conn.close()
    assert "diagnostics" not in meta


def test_scan_db_sink_keeps_packet_integrity_evidence(tmp_path):
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta())
    sink.consume("final", _final_interval())
    for fid, reason in ((42, "counter_clock_mismatch"), (43, "gap_too_large")):
        sink.consume("diagnostics", FrameQuarantined(
            side=0, cam_id=2, packet_id=17 + fid,
            timestamp_s=fid * 0.025, wire_frame_id=fid,
            previous_abs_frame_id=fid - 2,
            clock_anchor_abs_frame_id=fid - 2,
            clock_anchor_timestamp_s=(fid - 2) * 0.025,
            step=2, elapsed_s=0.025, reason=reason,
        ))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    meta = json.loads(conn.execute(
        "SELECT session_meta FROM sessions").fetchone()[0])
    conn.close()

    rec = meta["diagnostics"]["FrameQuarantined"]
    assert rec["count"] == 2
    assert rec["reasons"] == {
        "counter_clock_mismatch": 1,
        "gap_too_large": 1,
    }
    assert rec["first_detail"]["packet_id"] == 59
    assert rec["first_detail"]["previous_abs_frame_id"] == 40
    assert rec["last_detail"]["packet_id"] == 60


def test_scan_db_sink_keeps_camera_gap_and_recovery_evidence(tmp_path):
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta())
    sink.consume("final", _final_interval())
    sink.consume("diagnostics", CameraStreamGap(
        side=0, cam_id=1, state="missing", missing_frames=9,
        packet_id=109, timestamp_s=2.7,
        first_missing_packet_id=101, first_missing_timestamp_s=2.5,
    ))
    sink.consume("diagnostics", CameraStreamGap(
        side=0, cam_id=1, state="resumed", missing_frames=12,
        packet_id=113, timestamp_s=2.8,
        first_missing_packet_id=101, first_missing_timestamp_s=2.5,
    ))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    meta = json.loads(conn.execute(
        "SELECT session_meta FROM sessions").fetchone()[0])
    conn.close()

    rec = meta["diagnostics"]["CameraStreamGap"]
    assert rec["count"] == 2
    assert rec["first_detail"]["state"] == "missing"
    assert rec["first_detail"]["first_missing_packet_id"] == 101
    assert rec["last_detail"]["state"] == "resumed"
    assert rec["last_detail"]["missing_frames"] == 12
