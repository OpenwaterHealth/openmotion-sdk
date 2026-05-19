"""Tests for omotion.ScanDBSink."""

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDatabase, ScanDBSink


def test_sink_opens_and_closes_session(tmp_path: Path) -> None:
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(str(db_path))
    sid = sink.open(
        label="20260414_120000_owTEST",
        start_ts=1744632000.0,
        notes="pytest run",
        meta={"subject_id": "owTEST"},
    )
    assert sid > 0

    sink.close(end_ts=1744632010.0)

    # Verify row was persisted and end time was written.
    db = ScanDatabase(db_path=str(db_path))
    try:
        session = db.get_session(sid)
        assert session["session_label"] == "20260414_120000_owTEST"
        assert session["session_start"] == 1744632000.0
        assert session["session_end"] == 1744632010.0
        assert session["session_notes"] == "pytest run"
        assert session["session_meta"] == {"subject_id": "owTEST"}
    finally:
        db.close()


def test_sink_close_is_idempotent(tmp_path: Path) -> None:
    sink = ScanDBSink(str(tmp_path / "scans.db"))
    sid = sink.open(label="x", start_ts=0.0, notes="", meta={})
    sink.close(end_ts=1.0)
    # Second close must not raise and must not bump session_end.
    sink.close(end_ts=2.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        assert db.get_session(sid)["session_end"] == 1.0
    finally:
        db.close()


def test_sink_raises_if_callbacks_called_before_open(tmp_path: Path) -> None:
    from omotion.MotionProcessing import CorrectedBatch

    sink = ScanDBSink(str(tmp_path / "scans.db"))
    batch = CorrectedBatch(dark_frame_start=0, dark_frame_end=0, samples=[])
    with pytest.raises(RuntimeError):
        sink.on_corrected_batch(batch)
    with pytest.raises(RuntimeError):
        sink.on_raw_frame(
            "left", 0, 1, 0.0, b"\x00" * 4096, 25.0, 0, 0.0, 0.0, 0.0,
        )
