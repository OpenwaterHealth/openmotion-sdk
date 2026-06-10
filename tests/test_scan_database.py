"""Tests for omotion.ScanDatabase (the re-homed scan_db.py)."""

import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion import ScanDatabase


@pytest.fixture
def tmp_db(tmp_path: Path) -> ScanDatabase:
    db = ScanDatabase(db_path=str(tmp_path / "test.db"))
    yield db
    db.close()


def test_create_and_get_session(tmp_db: ScanDatabase) -> None:
    sid = tmp_db.create_session(
        session_label="20260414_000000_owTEST",
        session_start=1744600000.0,
        session_notes="unit test",
        session_meta={"subject_id": "owTEST", "fps": 40},
    )
    assert sid > 0

    session = tmp_db.get_session(sid)
    assert session is not None
    assert session["session_label"] == "20260414_000000_owTEST"
    assert session["session_start"] == 1744600000.0
    assert session["session_notes"] == "unit test"
    assert session["session_meta"] == {"subject_id": "owTEST", "fps": 40}


def test_no_raw_frame_api_or_table(tmp_path: Path) -> None:
    """Raw histograms are not stored in the DB (raw CSVs are the only raw
    record). New databases have no session_raw table and the class exposes
    no raw-frame API."""
    db = ScanDatabase(db_path=str(tmp_path / "noraw.db"))
    try:
        tables = {
            r[0] for r in db._connection().execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            ).fetchall()
        }
        assert "session_raw" not in tables
        assert not hasattr(db, "insert_raw_frame")
        assert not hasattr(db, "insert_raw_frames")
    finally:
        db.close()


def test_insert_session_data_and_stream(tmp_db: ScanDatabase) -> None:
    sid = tmp_db.create_session(session_label="s", session_start=0.0)
    tmp_db.insert_session_data(
        sid, cam_id=3, side=0, timestamp_s=1.0,
        bfi=0.25, bvi=0.5, contrast=0.1, mean=511.0,
    )
    rows = [r for batch in tmp_db.stream_session_data(sid) for r in batch]
    assert len(rows) == 1
    assert rows[0]["cam_id"] == 3
    assert rows[0]["side"] == 0
    assert rows[0]["bfi"] == 0.25


def test_session_meta_survives_unicode_and_none(tmp_db: ScanDatabase) -> None:
    meta = {"subject": "öwtëst", "ops": None, "nested": {"a": [1, 2, 3]}}
    sid = tmp_db.create_session(
        session_label="u", session_start=0.0, session_meta=meta,
    )
    session = tmp_db.get_session(sid)
    assert session["session_meta"] == meta
