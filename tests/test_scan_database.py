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


def test_insert_and_read_raw_frame_uncompressed(tmp_path: Path) -> None:
    db = ScanDatabase(db_path=str(tmp_path / "raw.db"), compress_raw_hist=False)
    try:
        sid = db.create_session(
            session_label="label",
            session_start=0.0,
        )
        hist = bytes(range(256)) * 16  # 4096-byte blob
        rid = db.insert_raw_frame(
            sid, "left", 0, 1, 1.25, hist,
            temp=27.0, sum_counts=100, tcm=1.0, tcl=2.0, pdc=3.0,
        )
        assert rid > 0

        row = db.get_raw_frame(rid)
        assert row is not None
        assert row["side"] == "left"
        assert row["cam_id"] == 0
        assert row["frame_id"] == 1
        assert row["hist"] == hist
        assert row["temp"] == 27.0
        assert row["sum"] == 100
    finally:
        db.close()


def test_raw_frame_roundtrip_compressed(tmp_path: Path) -> None:
    db = ScanDatabase(db_path=str(tmp_path / "zraw.db"), compress_raw_hist=True)
    try:
        sid = db.create_session(session_label="z", session_start=0.0)
        hist = bytes([0, 1, 2, 3]) * 1024
        rid = db.insert_raw_frame(sid, "right", 2, 5, 0.1, hist)
        row = db.get_raw_frame(rid)
        assert row["hist"] == hist  # transparently decompressed on read
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
