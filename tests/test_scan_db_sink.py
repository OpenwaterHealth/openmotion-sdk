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


# ----- Task 4: on_raw_frame ----------------------------------------------

import threading

from omotion.MotionProcessing import CorrectedBatch


def _make_sink(tmp_path, **kwargs):
    sink = ScanDBSink(str(tmp_path / "scans.db"), **kwargs)
    sid = sink.open(label="lbl", start_ts=0.0, notes="", meta={})
    return sink, sid


def test_on_raw_frame_is_noop_when_write_raw_false(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path, write_raw=False)
    sink.on_raw_frame("left", 0, 1, 0.0, b"\x00" * 4096, 25.0, 10, 0.0, 0.0, 0.0)
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert rows == []
    finally:
        db.close()


def test_on_raw_frame_writes_one_row_per_call(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=2)
    hist = b"\xab" * 4096
    for fid in range(5):
        sink.on_raw_frame("left", 0, fid, fid * 0.025, hist, 25.0, 100, 0.0, 0.0, 0.0)
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 5
        assert [r["frame_id"] for r in rows] == [0, 1, 2, 3, 4]
        assert rows[0]["hist"] == hist  # transparently decompressed by default
    finally:
        db.close()


def test_on_raw_frame_flushes_on_batch_size(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=3)
    for fid in range(3):
        sink.on_raw_frame("right", 1, fid, 0.0, b"\x00" * 4096, 0.0, 0, 0.0, 0.0, 0.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        # All three should already be persisted — the 3rd call hit batch_size.
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 3
    finally:
        db.close()
    sink.close(end_ts=1.0)


def test_on_raw_frame_concurrent_writers(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=50)

    def _writer(side, n):
        hist = b"\x01" * 4096
        for fid in range(n):
            sink.on_raw_frame(side, 0, fid, fid * 0.025, hist, 25.0, 10, 0.0, 0.0, 0.0)

    threads = [
        threading.Thread(target=_writer, args=("left", 200)),
        threading.Thread(target=_writer, args=("right", 200)),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 400
    finally:
        db.close()


# ----- Task 5: on_corrected_batch ---------------------------------------

from omotion.MotionProcessing import Sample


def _mk_sample(side, cam_id, frame_id, ts, bfi, bvi, contrast, mean):
    return Sample(
        side=side,
        cam_id=cam_id,
        frame_id=frame_id,
        absolute_frame_id=frame_id,
        timestamp_s=ts,
        row_sum=0,
        temperature_c=25.0,
        mean=mean,
        std_dev=0.0,
        contrast=contrast,
        bfi=bfi,
        bvi=bvi,
        is_corrected=True,
    )


def test_on_corrected_batch_writes_one_row_per_sample(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path)
    batch = CorrectedBatch(
        dark_frame_start=0,
        dark_frame_end=600,
        samples=[
            _mk_sample("left", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),
            _mk_sample("left", 1, 1, 0.025, 0.11, 0.21, 0.31, 501.0),
            _mk_sample("right", 0, 1, 0.025, 0.12, 0.22, 0.32, 502.0),
        ],
    )
    sink.on_corrected_batch(batch)
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch_ in db.stream_session_data(sid) for r in batch_]
        assert len(rows) == 3
        left_rows = [r for r in rows if r["side"] == 0]
        right_rows = [r for r in rows if r["side"] == 1]
        assert len(left_rows) == 2
        assert len(right_rows) == 1
        lr = next(r for r in left_rows if r["cam_id"] == 0)
        assert lr["bfi"] == 0.1
        assert lr["bvi"] == 0.2
        assert lr["contrast"] == 0.3
        assert lr["mean"] == 500.0
        assert lr["timestamp_s"] == 0.025
    finally:
        db.close()


def test_on_corrected_batch_flushes_pending_raw_frames(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=100)
    # Enqueue 5 raw frames (below batch_size, so not yet flushed).
    for fid in range(5):
        sink.on_raw_frame("left", 0, fid, 0.0, b"\x00" * 4096, 25.0, 0, 0.0, 0.0, 0.0)

    batch = CorrectedBatch(
        dark_frame_start=0,
        dark_frame_end=600,
        samples=[_mk_sample("left", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0)],
    )
    sink.on_corrected_batch(batch)

    # Before close, raw rows should already be visible.
    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for b in db.stream_raw_frames(sid) for r in b]
        assert len(rows) == 5
    finally:
        db.close()
    sink.close(end_ts=1.0)


def test_on_corrected_batch_empty_is_noop(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path)
    sink.on_corrected_batch(
        CorrectedBatch(dark_frame_start=0, dark_frame_end=0, samples=[])
    )
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for b in db.stream_session_data(sid) for r in b]
        assert rows == []
    finally:
        db.close()


def test_on_corrected_batch_rounds_floats_to_6_decimals(tmp_path: Path) -> None:
    sink, sid = _make_sink(tmp_path)
    sink.on_corrected_batch(
        CorrectedBatch(
            dark_frame_start=0,
            dark_frame_end=600,
            samples=[
                _mk_sample(
                    "left", 0, 1,
                    0.0250001234567,  # timestamp_s → 0.025000
                    0.1234567891,     # bfi        → 0.123457
                    0.2345678912,     # bvi        → 0.234568
                    0.3456789123,     # contrast   → 0.345679
                    500.1234567891,   # mean       → 500.123457
                ),
            ],
        )
    )
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for b in db.stream_session_data(sid) for r in b]
        assert len(rows) == 1
        r = rows[0]
        assert r["timestamp_s"] == 0.025000
        assert r["bfi"] == 0.123457
        assert r["bvi"] == 0.234568
        assert r["contrast"] == 0.345679
        assert r["mean"] == 500.123457
    finally:
        db.close()


def test_on_corrected_batch_skips_unknown_side(tmp_path: Path) -> None:
    """Defensive: a sample with an unexpected ``side`` value (not
    'left'/'right') is counted as an insert error and skipped, not
    written as a NULL side row."""
    sink, sid = _make_sink(tmp_path)
    samples = [
        _mk_sample("left", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),
        _mk_sample("middle", 0, 1, 0.025, 0.1, 0.2, 0.3, 500.0),  # bogus
    ]
    sink.on_corrected_batch(
        CorrectedBatch(dark_frame_start=0, dark_frame_end=600, samples=samples)
    )
    sink.close(end_ts=1.0)
    assert sink.insert_errors >= 1

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for b in db.stream_session_data(sid) for r in b]
        assert len(rows) == 1
        assert rows[0]["side"] == 0  # the valid 'left' sample
    finally:
        db.close()


def test_on_raw_frame_rounds_floats_to_6_decimals(tmp_path: Path) -> None:
    """Per project policy, floats are stored to 6 decimals to match the
    corrected CSV writer's precision (and to keep DB size in check)."""
    sink, sid = _make_sink(tmp_path, write_raw=True, raw_batch_size=1)
    sink.on_raw_frame(
        "left", 0, 1,
        0.0250001234567,    # timestamp_s — should round to 0.025000
        b"\x00" * 4096,
        25.1234567,         # temp
        42,
        1.23456789, 2.34567891, 3.45678912,  # tcm/tcl/pdc
    )
    sink.close(end_ts=1.0)

    db = ScanDatabase(db_path=str(tmp_path / "scans.db"))
    try:
        rows = [r for batch in db.stream_raw_frames(sid) for r in batch]
        assert len(rows) == 1
        r = rows[0]
        assert r["timestamp_s"] == 0.025000
        assert r["temp"] == 25.123457
        assert r["tcm"] == 1.234568
        assert r["tcl"] == 2.345679
        assert r["pdc"] == 3.456789
    finally:
        db.close()
