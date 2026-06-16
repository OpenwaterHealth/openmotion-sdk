# tests/test_scan_db_sink_empty.py
from types import SimpleNamespace

import pytest

from omotion.pipeline.sinks import ScanDBSink
from omotion.ScanDatabase import ScanDatabase

pytestmark = pytest.mark.unit


def _meta():
    return SimpleNamespace(
        scan_id="20260615_000000", subject_id="subjX", operator="test",
        started_at_iso="2026-06-15T00:00:00+00:00", duration_sec=10,
        reduced_mode=True, left_camera_mask=195, right_camera_mask=195,
    )


def test_empty_scan_session_is_deleted(tmp_path):
    db_path = str(tmp_path / "scans.db")
    sink = ScanDBSink(db_path)
    sink.on_scan_start(_meta())
    sink.on_complete()  # no "final" frames fed
    db = ScanDatabase(db_path)
    try:
        assert db.get_session_by_label("20260615_000000_subjX") is None
    finally:
        db.close()


def test_nonempty_scan_session_is_retained(tmp_path):
    db_path = str(tmp_path / "scans.db")
    sink = ScanDBSink(db_path)
    sink.on_scan_start(_meta())
    frame = SimpleNamespace(
        cam_id=-1, side="left", abs_frame_id=1, t=0.1,
        bfi=1.0, bvi=2.0, mean=3.0, contrast=0.4, quality="ok",
    )
    sink.consume("final", SimpleNamespace(frames=[frame]))
    sink.on_complete()
    db = ScanDatabase(db_path)
    try:
        sess = db.get_session_by_label("20260615_000000_subjX")
        assert sess is not None
        assert next(db.iter_session_data(int(sess["id"])), None) is not None
    finally:
        db.close()


def test_rows_written_reset_between_scans(tmp_path):
    db_path = str(tmp_path / "scans.db")
    sink = ScanDBSink(db_path)
    # First scan persists a row.
    sink.on_scan_start(_meta())
    frame = SimpleNamespace(
        cam_id=-1, side="left", abs_frame_id=1, t=0.1,
        bfi=1.0, bvi=2.0, mean=3.0, contrast=0.4, quality="ok",
    )
    sink.consume("final", SimpleNamespace(frames=[frame]))
    sink.on_complete()
    # Reuse the same instance for an empty second scan.
    sink.on_scan_start(SimpleNamespace(
        scan_id="20260615_111111", subject_id="subjY", operator="test",
        started_at_iso="2026-06-15T11:11:11+00:00", duration_sec=10,
        reduced_mode=True, left_camera_mask=195, right_camera_mask=195,
    ))
    sink.on_complete()
    db = ScanDatabase(db_path)
    try:
        # Second (empty) session must be deleted despite the first scan's rows.
        assert db.get_session_by_label("20260615_111111_subjY") is None
        # First (non-empty) session is still present.
        assert db.get_session_by_label("20260615_000000_subjX") is not None
    finally:
        db.close()
