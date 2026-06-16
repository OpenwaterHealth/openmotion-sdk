"""ScanDBSink (pipeline) — final-channel-only SQLite sink.

The DB persists the corrected (final-branch) record exclusively:
per-camera EnrichedCorrectedFrames in normal mode, cam_id=-1 side
averages in reduced mode. Live/realtime values and raw histograms are
never written (raw lives in the CSVs from Tee("raw")).
"""

import json
import logging
import sqlite3

import pytest
from omotion.pipeline.sinks import ScanDBSink, ScanMetadata
from omotion.pipeline.stages.dark import (
    EnrichedCorrectedFrame,
    EnrichedCorrectedInterval,
)


def _meta_simple():
    return ScanMetadata(
        scan_id="abc", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=300,
        left_camera_mask=0x01, right_camera_mask=0, reduced_mode=False,
    )


def _meta_reduced():
    return ScanMetadata(
        scan_id="r", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=300,
        left_camera_mask=0x03, right_camera_mask=0x03, reduced_mode=True,
    )


def _frame(abs_id, *, side="left", cam_id=0, t=None, mean=100.0, std=2.0,
           contrast=0.02, bfi=4.0, bvi=6.0, quality="ok"):
    return EnrichedCorrectedFrame(
        abs_frame_id=abs_id, t=(t if t is not None else abs_id * 0.025),
        side=side, cam_id=cam_id, mean=mean, std=std,
        contrast=contrast, bfi=bfi, bvi=bvi, quality=quality,
    )


def _interval(frames, left_abs=10, right_abs=610):
    return EnrichedCorrectedInterval(
        left_abs=left_abs, right_abs=right_abs, frames=frames,
    )


def test_scan_db_sink_creates_session_at_scan_start(tmp_path):
    db_path = tmp_path / "scan.db"
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta_simple())
    sink.on_complete()
    assert db_path.exists()


def test_scan_db_sink_channels_attribute():
    sink = ScanDBSink(db_path=":memory:")
    # Final-branch record + integrity-diagnostics summary. Live values reach
    # the GUI via channels; raw histograms live in the CSVs from Tee("raw").
    assert sink.channels == {"final", "diagnostics"}


def test_scan_db_sink_stamps_session_meta(tmp_path):
    """session_meta carries sdk_flags (reduced_mode, masks) — required by
    SessionPlayback — plus a data_semantics marker distinguishing
    final-branch sessions from legacy live-valued ones."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_reduced())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    meta_json = conn.execute("SELECT session_meta FROM sessions").fetchone()[0]
    conn.close()
    meta = json.loads(meta_json)
    assert meta["data_semantics"] == "final"
    assert meta["sdk_flags"]["reduced_mode"] is True
    assert meta["sdk_flags"]["left_camera_mask"] == 0x03
    assert meta["sdk_flags"]["right_camera_mask"] == 0x03
    assert meta["scan_id"] == "r"
    assert meta["subject_id"] == "subj"


def test_scan_db_sink_final_writes_per_cam_rows(tmp_path):
    """Normal mode: one session_data row per per-camera corrected frame,
    with real cam_id and side."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.consume("final", _interval([
        _frame(10, side="left", cam_id=0, bfi=0.42, bvi=5.1),
        _frame(11, side="right", cam_id=2, bfi=0.31, bvi=4.9),
        _frame(12, side="left", cam_id=7, bfi=0.50, bvi=5.3),
    ]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT side, cam_id, frame_id, bfi, bvi, mean, contrast "
        "FROM session_data ORDER BY frame_id"
    ).fetchall()
    conn.close()
    assert len(rows) == 3
    assert rows[0] == (0, 0, 10, pytest.approx(0.42), pytest.approx(5.1),
                       pytest.approx(100.0), pytest.approx(0.02))
    assert rows[1] == (1, 2, 11, pytest.approx(0.31), pytest.approx(4.9),
                       pytest.approx(100.0), pytest.approx(0.02))
    assert rows[2] == (0, 7, 12, pytest.approx(0.50), pytest.approx(5.3),
                       pytest.approx(100.0), pytest.approx(0.02))


def test_scan_db_sink_final_includes_stencilled_dark_row(tmp_path):
    """The interval's leading dark frame (stencil-interpolated by
    DarkFrameHoldStage) is persisted like any other frame — the DB record
    is gapless at 40 Hz."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.consume("final", _interval([
        _frame(10, cam_id=0, bfi=0.40),   # D_prev, stencilled
        _frame(11, cam_id=0, bfi=0.42),
        _frame(12, cam_id=0, bfi=0.44),
    ]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    fids = [r[0] for r in conn.execute(
        "SELECT frame_id FROM session_data ORDER BY frame_id").fetchall()]
    conn.close()
    assert fids == [10, 11, 12]


def test_scan_db_sink_normal_mode_skips_side_average_rows(tmp_path):
    """cam_id=-1 side-average frames are a reduced-mode concept; in normal
    mode (where SideAverageStage is disabled anyway) they are skipped
    defensively."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.consume("final", _interval([
        _frame(10, cam_id=-1, bfi=0.40),
        _frame(10, cam_id=0, bfi=0.42),
    ]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute("SELECT cam_id FROM session_data").fetchall()
    conn.close()
    assert rows == [(0,)]


def test_scan_db_sink_reduced_mode_persists_only_side_averages(tmp_path):
    """Reduced mode: per-camera frames are skipped; only the cam_id=-1
    side-average frames emitted by SideAverageStage land in session_data."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_reduced())
    # Per-camera interval — must NOT be persisted in reduced mode.
    sink.consume("final", _interval([
        _frame(10, cam_id=0, bfi=0.42),
        _frame(11, cam_id=1, bfi=0.44),
    ]))
    # Side-average interval from SideAverageStage (cam_id=-1).
    sink.consume("final", _interval([
        _frame(42, side="right", cam_id=-1, bfi=3.5, bvi=7.5,
               mean=120.0, contrast=0.25, t=1.5),
    ]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT cam_id, side, frame_id, bfi, bvi, mean, contrast FROM session_data"
    ).fetchall()
    conn.close()
    assert rows == [(-1, 1, 42, pytest.approx(3.5), pytest.approx(7.5),
                     pytest.approx(120.0), pytest.approx(0.25))]


def test_scan_db_sink_nan_values_stored_as_null(tmp_path):
    """NaN metrics (e.g. the side average's undefined fields, degenerate
    calibration) are stored as NULL, not as NaN floats; rows with no finite
    metric at all are skipped."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    nan = float("nan")
    sink.consume("final", _interval([
        _frame(10, cam_id=0, bfi=0.42, bvi=nan, mean=nan, contrast=nan),
        _frame(11, cam_id=0, bfi=nan, bvi=nan, mean=nan, contrast=nan),
    ]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT frame_id, bfi, bvi, mean, contrast FROM session_data"
    ).fetchall()
    conn.close()
    assert rows == [(10, pytest.approx(0.42), None, None, None)]


def test_scan_db_sink_flushes_buffer_below_batch_size_on_complete(tmp_path):
    """Rows buffered below batch_size still land via the on_complete flush."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path, batch_size=1000)
    sink.on_scan_start(_meta_simple())
    sink.consume("final", _interval([_frame(10, cam_id=0)]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    n = conn.execute("SELECT COUNT(*) FROM session_data").fetchone()[0]
    conn.close()
    assert n == 1


def test_scan_db_sink_quality_persisted(tmp_path):
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.consume("final", _interval([
        _frame(10, cam_id=0, quality="ts_corrected"),
    ]))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute("SELECT quality FROM session_data").fetchall()
    conn.close()
    assert rows == [("ts_corrected",)]


def test_scan_db_sink_ignores_other_channels(tmp_path):
    """Payloads on channels the sink doesn't subscribe to are ignored even
    if delivered directly (the runner wouldn't, but be defensive)."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.consume("live", object())
    sink.consume("raw", object())
    sink.consume("final_side", object())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    n = conn.execute("SELECT COUNT(*) FROM session_data").fetchone()[0]
    conn.close()
    assert n == 0


def test_scan_db_sink_no_session_raw_table_in_new_dbs(tmp_path):
    """New databases are created without the session_raw table — raw
    histograms are persisted only via the raw CSVs (Tee("raw") → CsvSink)."""
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    tables = {r[0] for r in conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table'").fetchall()}
    conn.close()
    assert "session_raw" not in tables
    assert {"sessions", "session_data"} <= tables


def test_session_data_has_quality_column(tmp_path):
    """session_data table must include a quality column."""
    from omotion.ScanDatabase import ScanDatabase
    db = ScanDatabase(db_path=str(tmp_path / "test.db"))
    conn = db._connection()
    cursor = conn.execute("PRAGMA table_info(session_data)")
    columns = {row[1] for row in cursor.fetchall()}
    assert "quality" in columns
    db.close()
