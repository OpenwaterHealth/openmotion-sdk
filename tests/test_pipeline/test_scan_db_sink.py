"""ScanDBSink (pipeline) — channel-based, same raw-gating contract as CsvSink."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.sinks import ScanDBSink, ScanMetadata


def _meta_with_raw(write_raw, duration=None):
    return ScanMetadata(
        scan_id="abc", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=300,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
        write_raw_csv=write_raw, raw_csv_duration_sec=duration,
    )


def _dummy_raw_batch():
    n = 1
    raw = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw[0, 0, 0, 5] = 42
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


def test_scan_db_sink_creates_session_at_scan_start(tmp_path):
    db_path = tmp_path / "scan.db"
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta_with_raw(write_raw=False))
    sink.on_complete()
    assert db_path.exists()


def test_scan_db_sink_channels_attribute():
    sink = ScanDBSink(db_path=":memory:")
    assert "raw" in sink.channels
    assert "final" in sink.channels


def test_scan_db_sink_raw_gated_by_write_raw_csv(tmp_path):
    """When write_raw_csv=False, raw channel consume should not insert rows."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_with_raw(write_raw=False))
    sink.consume("raw", _dummy_raw_batch())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    count = conn.execute("SELECT COUNT(*) FROM session_raw").fetchone()[0]
    conn.close()
    assert count == 0


def test_scan_db_sink_raw_writes_when_enabled(tmp_path):
    """When write_raw_csv=True, raw channel consume should insert rows."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_with_raw(write_raw=True))
    sink.consume("raw", _dummy_raw_batch())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    count = conn.execute("SELECT COUNT(*) FROM session_raw").fetchone()[0]
    conn.close()
    assert count >= 1
