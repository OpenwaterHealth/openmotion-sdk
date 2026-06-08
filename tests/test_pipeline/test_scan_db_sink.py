"""ScanDBSink (pipeline) — channel-based, same raw-gating contract as CsvSink."""

import logging

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, SideAverageSample
from omotion.pipeline.sinks import ScanDBSink, ScanMetadata


def _meta_simple():
    """Simple ScanMetadata for testing — no raw CSV gate fields."""
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
    sink.on_scan_start(_meta_simple())
    sink.on_complete()
    assert db_path.exists()


def test_scan_db_sink_channels_attribute():
    sink = ScanDBSink(db_path=":memory:")
    assert "raw" in sink.channels
    assert "live" in sink.channels
    assert "final_side" in sink.channels
    # The per-cam corrected interval ("final") is no longer persisted by the
    # DB sink — the corrected side average comes via "final_side".
    assert "final" not in sink.channels


def test_scan_db_sink_reduced_mode_live_writes_no_per_cam_rows(tmp_path):
    """Reduced mode persists only the corrected side average (cam_id=-1) — the
    per-camera realtime 'live' rows are not written."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_reduced())
    batch = _dummy_live_batch(
        frame_types=["light", "light"], side_ids=[0, 0], cam_ids=[0, 1],
        bfi=[0.4, 0.5], bvi=[5.0, 5.1],
    )
    sink.consume("live", batch)
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    n = conn.execute("SELECT COUNT(*) FROM session_data").fetchone()[0]
    conn.close()
    assert n == 0


def test_scan_db_sink_final_side_writes_cam_id_minus_1(tmp_path):
    """The corrected side average lands as a cam_id=-1 row carrying real
    bfi/bvi/mean/contrast (superseding the old NULL-bfi placeholder)."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_reduced())
    sink.consume("final_side", SideAverageSample(
        t=1.5, frame_id=42, side=1, bfi=3.5, bvi=7.5, mean=120.0, contrast=0.25))
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT cam_id, side, frame_id, bfi, bvi, mean, contrast FROM session_data"
    ).fetchall()
    conn.close()
    assert rows == [(-1, 1, 42, pytest.approx(3.5), pytest.approx(7.5),
                     pytest.approx(120.0), pytest.approx(0.25))]


def _dummy_live_batch(*, frame_types=None, side_ids=None, cam_ids=None,
                     bfi=None, bvi=None):
    """A post-BfiBvi batch: bfi_live/bvi_live/mean_dc_rt/contrast_sn_rt
    set per-cam, side_ids+cam_ids pointing to the source camera for
    each frame. n_frames inferred from the longest input."""
    n = max(len(x) for x in (frame_types or ["light"],
                              side_ids if side_ids is not None else [0],
                              cam_ids if cam_ids is not None else [0]))
    bfi_arr = np.zeros((n, 2, 8), dtype=np.float32)
    bvi_arr = np.zeros((n, 2, 8), dtype=np.float32)
    mean_arr = np.full((n, 2, 8), 100.0, dtype=np.float32)
    contrast_arr = np.full((n, 2, 8), 0.3, dtype=np.float32)
    if bfi is not None:
        for i, val in enumerate(bfi):
            bfi_arr[i, int(side_ids[i]), int(cam_ids[i])] = val
    if bvi is not None:
        for i, val in enumerate(bvi):
            bvi_arr[i, int(side_ids[i]), int(cam_ids[i])] = val
    return FrameBatch(
        cam_ids=np.array(cam_ids if cam_ids is not None else [0] * n, dtype=np.int8),
        frame_ids=np.array([10 + i for i in range(n)], dtype=np.uint8),
        raw_histograms=None,
        temperature_c=np.full((n, 2, 8), 35.0, dtype=np.float32),
        timestamp_s=np.array([0.025 * (10 + i) for i in range(n)], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([100 + i for i in range(n)], dtype=np.int64),
        frame_type=np.array(frame_types or ["light"] * n, dtype="<U8"),
        side_ids=np.array(side_ids if side_ids is not None else [0] * n, dtype=np.int8),
        bfi_live=bfi_arr, bvi_live=bvi_arr,
        mean_dc_rt=mean_arr, contrast_sn_rt=contrast_arr,
    )


def test_scan_db_sink_live_writes_per_cam_rows(tmp_path):
    """Phase 1: the 'live' channel writes per-frame per-cam rows with
    BFI/BVI/mean/contrast — the foundation for past-scan replay from
    the DB. One row per frame, side+cam from side_ids/cam_ids."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    batch = _dummy_live_batch(
        frame_types=["light", "light", "light"],
        side_ids=[0, 1, 0],
        cam_ids=[0, 2, 7],
        bfi=[0.42, 0.31, 0.50],
        bvi=[5.1, 4.9, 5.3],
    )
    sink.consume("live", batch)
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute(
        "SELECT side, cam_id, bfi, bvi FROM session_data ORDER BY frame_id"
    ).fetchall()
    conn.close()
    assert len(rows) == 3
    assert rows[0] == (0, 0, pytest.approx(0.42), pytest.approx(5.1))
    assert rows[1] == (1, 2, pytest.approx(0.31), pytest.approx(4.9))
    assert rows[2] == (0, 7, pytest.approx(0.50), pytest.approx(5.3))


def test_scan_db_sink_live_skips_dark_and_warmup_frames(tmp_path):
    """Dark frames have no useful display BFI/BVI; warmup/stale frames
    are already filtered by the pipeline's Tee but the sink double-
    checks. Only light frames should land in session_data."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    batch = _dummy_live_batch(
        frame_types=["warmup", "dark", "light", "stale"],
        side_ids=[0, 0, 0, 0],
        cam_ids=[0, 0, 0, 0],
        bfi=[0.10, 0.20, 0.30, 0.40],
        bvi=[1.0, 2.0, 3.0, 4.0],
    )
    sink.consume("live", batch)
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute("SELECT bfi FROM session_data").fetchall()
    conn.close()
    assert rows == [(pytest.approx(0.30),)]


def test_scan_db_sink_live_skips_nan_bfi_or_bvi(tmp_path):
    """Early frames before the first dark observation can emit NaN for
    BFI or BVI — skip them so the DB doesn't carry display noise."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    batch = _dummy_live_batch(
        frame_types=["light", "light"],
        side_ids=[0, 0],
        cam_ids=[0, 0],
        bfi=[float("nan"), 0.45],
        bvi=[5.0, 5.5],
    )
    sink.consume("live", batch)
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute("SELECT bfi FROM session_data").fetchall()
    conn.close()
    assert rows == [(pytest.approx(0.45),)]


def test_scan_db_sink_raw_always_writes_when_consume_called(tmp_path):
    """Sink always writes raw when consume('raw', ...) is called.
    Duration/enable gating happens upstream at the Tee layer, not in the sink."""
    import sqlite3
    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())
    sink.consume("raw", _dummy_raw_batch())
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    count = conn.execute("SELECT COUNT(*) FROM session_raw").fetchone()[0]
    conn.close()
    assert count >= 1


def test_scan_db_sink_skips_stale_raw_rows_and_logs(tmp_path, caplog):
    import sqlite3

    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    sink.on_scan_start(_meta_simple())

    batch = _dummy_raw_batch()
    batch.frame_type = np.array(["stale"], dtype="<U8")
    with caplog.at_level(logging.WARNING, logger="omotion.pipeline.sinks"):
        sink.consume("raw", batch)
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    count = conn.execute("SELECT COUNT(*) FROM session_raw").fetchone()[0]
    conn.close()
    assert count == 0
    assert "stale raw frame skipped" in caplog.text


def test_session_data_has_quality_column(tmp_path):
    """session_data table must include a quality column."""
    from omotion.ScanDatabase import ScanDatabase
    db = ScanDatabase(db_path=str(tmp_path / "test.db"))
    conn = db._connection()
    cursor = conn.execute("PRAGMA table_info(session_data)")
    columns = {row[1] for row in cursor.fetchall()}
    assert "quality" in columns
    db.close()


def test_scan_db_sink_uses_source_side_ids_for_raw_rows(tmp_path):
    import sqlite3

    db_path = str(tmp_path / "scan.db")
    sink = ScanDBSink(db_path=db_path)
    meta = ScanMetadata(
        scan_id="side_test", subject_id="subj", operator="op",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=300,
        left_camera_mask=0x01,
        right_camera_mask=0x01,
        reduced_mode=False,
    )
    sink.on_scan_start(meta)

    batch = _dummy_raw_batch()
    batch.side_ids = np.array([1], dtype=np.int8)
    batch.raw_histograms[0, 0, 0, :] = 0
    batch.raw_histograms[0, 1, 0, 5] = 42
    sink.consume("raw", batch)
    sink.on_complete()

    conn = sqlite3.connect(db_path)
    rows = conn.execute("SELECT side, sum FROM session_raw").fetchall()
    conn.close()
    assert rows == [("right", 42)]
