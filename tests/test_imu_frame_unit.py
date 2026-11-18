"""Per-frame IMU motion (bloodflow-app#783) — no hardware.

Covers the pure magnitude function, ImuFrameStage's join of IMU samples to
histogram frames (sample windows, 0/2-sample frames, waiting for the IMU,
the lag and final flushes, skipped frame types, sides without an IMU), the
runner's "imu_frame" routing, ScanDBSink persistence + ScanDatabase reads
(plain and SQLCipher), the schema migration of older databases, and the
ScanWorkflow wiring of ScanRequest.imu_motion.
"""
import shutil
import sqlite3
import threading
import time
from pathlib import Path
from unittest import mock

import numpy as np
import pytest

from omotion import db_key, db_schema
from omotion.imu_csv import ImuSample, motion_magnitude, raw_motion_magnitude
from omotion.pipeline.batch import IMU_FRAME_CHANNEL, FrameBatch, ImuFrameMotion
from omotion.pipeline.stages.imu_frame import ImuFrameStage, average_imu_samples
from omotion.ScanDatabase import ScanDatabase

FIXTURE = Path(__file__).parent / "fixtures" / "legacy_scans_v0.db"
FIXED_KEY = "a" * 64


# ── magnitude ─────────────────────────────────────────────────────────────

def test_motion_magnitude_at_rest_in_any_orientation_is_zero():
    assert motion_magnitude((0, 0, 1), (0, 0, 0)) == (0.0, 0.0)
    acc, _ = motion_magnitude((0.6, 0.0, 0.8), (0, 0, 0))
    assert acc == pytest.approx(0.0, abs=1e-12)


def test_motion_magnitude_values():
    acc, gyr = motion_magnitude((0, 0, 2.0), (3.0, 4.0, 0.0))
    assert acc == pytest.approx(1.0) and gyr == pytest.approx(5.0)
    assert raw_motion_magnitude((0, 0, 3072), (164, 0, 0)) == \
        (pytest.approx(0.5), pytest.approx(10.0))


# ── ImuFrameStage ─────────────────────────────────────────────────────────

def _batch(rows, frame_type=None):
    """rows: (side_id, abs_frame_id, timestamp_s, cam_id)."""
    n = len(rows)
    b = FrameBatch(
        cam_ids=np.array([r[3] for r in rows], dtype=np.int8),
        frame_ids=np.array([r[1] % 256 for r in rows], dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([r[2] for r in rows], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        side_ids=np.array([r[0] for r in rows], dtype=np.int8),
    )
    b.abs_frame_ids = np.array([r[1] for r in rows], dtype=np.int64)
    b.frame_type = np.array(frame_type or ["light"] * n, dtype=object)
    return b


def _frames(side_id, ids, t0=0.0, cams=(1, 2)):
    return [(side_id, f, t0 + 0.025 * f, c) for f in ids for c in cams]


def _sample(ax=0, ay=0, az=2048, gx=0, gy=0, gz=0):
    return ImuSample(frame=0, accel=(ax, ay, az), gyro=(gx, gy, gz),
                     mag=(0, 0, 0), temp_c=25.0)


def _events(batch):
    return [e for e in batch.events if isinstance(e, ImuFrameMotion)]


def test_frame_averages_samples_since_previous_frame():
    st = ImuFrameStage()
    # frames 1..3 at 0.025, 0.050, 0.075; samples between them
    st.add_sample("left", 0.010, _sample(az=2048 + 205))       # f1 window (0.0, 0.025]
    st.add_sample("left", 0.030, _sample(az=2048 + 410, gx=164))
    st.add_sample("left", 0.045, _sample(az=2048, gx=328))
    st.add_sample("left", 0.080, _sample())                     # past f3: watermark
    out = _events(st.process(_batch(_frames(0, [1, 2, 3]))))
    assert [(e.side, e.frame_id, e.n_samples) for e in out] == \
        [("left", 1, 1), ("left", 2, 2), ("left", 3, 0)]
    assert [e.timestamp_s for e in out] == pytest.approx([0.025, 0.05, 0.075])
    f2 = out[1]
    assert f2.accel_dev_g == pytest.approx((410 / 2048 + 0.0) / 2)  # mean of magnitudes
    assert f2.gyro_dps == pytest.approx((10 + 20) / 2)
    assert f2.az_g == pytest.approx((1 + 410 / 2048 + 1.0) / 2)
    assert f2.gx_dps == pytest.approx(15.0) and f2.gy_dps == 0.0
    f3 = out[2]
    assert f3.accel_dev_g is None and f3.ax_g is None and f3.gz_dps is None


def test_first_frame_takes_one_nominal_period_and_drops_older_samples():
    st = ImuFrameStage()
    st.add_sample("left", -0.5, _sample(az=4096))               # pre-scan: dropped
    st.add_sample("left", 0.990, _sample(az=2048 + 1024))
    st.add_sample("left", 1.2, _sample())
    (e,) = _events(st.process(_batch([(0, 40, 1.0, 1)])))
    assert e.n_samples == 1 and e.accel_dev_g == pytest.approx(0.5)


def test_frames_wait_for_the_imu_then_emit_in_order():
    st = ImuFrameStage(lag_s=1.0)
    st.add_sample("left", 0.020, _sample())
    out = _events(st.process(_batch(_frames(0, [1, 2, 3]))))
    assert out == []                                            # IMU behind frame 1
    st.add_sample("left", 0.060, _sample())
    out = _events(st.process(_batch([])))
    assert [e.frame_id for e in out] == [1, 2]                  # frame 3 still waits
    assert out[0].n_samples == 1 and out[1].n_samples == 0
    out = _events(st.process(_batch(_frames(0, [4]))))
    assert out == []


def test_lag_emits_frames_when_the_imu_falls_behind():
    st = ImuFrameStage(lag_s=0.1)
    st.add_sample("left", 0.0, _sample())
    out = _events(st.process(_batch(_frames(0, range(1, 9)))))  # 0.025 .. 0.2
    assert [e.frame_id for e in out] == [1, 2, 3]               # > lag behind 0.2 s
    assert all(e.n_samples == 0 for e in out)


def test_scan_stop_flushes_every_pending_frame():
    st = ImuFrameStage()
    st.add_sample("right", 0.010, _sample(gx=164))
    st.process(_batch(_frames(1, [1, 2, 3])))
    flush = _batch([])
    st.on_scan_stop(flush)
    out = _events(flush)
    assert [(e.side, e.frame_id, e.n_samples) for e in out] == \
        [("right", 1, 1), ("right", 2, 0), ("right", 3, 0)]
    assert out[0].gyro_dps == pytest.approx(10.0)
    assert st.emitted == 3


def test_side_without_imu_emits_nothing_and_late_rows_are_ignored():
    st = ImuFrameStage(lag_s=0.05)
    st.add_sample("left", 1.0, _sample())
    out = _events(st.process(_batch(_frames(0, [1, 2]) + _frames(1, [1, 2]))))
    assert {e.side for e in out} == {"left"}
    out = _events(st.process(_batch(_frames(0, [1]))))          # frame 1 again
    assert out == []
    flush = _batch([])
    st.on_scan_stop(flush)
    assert _events(flush) == []                                  # right dropped


def test_warmup_and_stale_frames_are_skipped():
    st = ImuFrameStage()
    st.add_sample("left", 1.0, _sample())
    rows = [(0, 1, 0.025, 1), (0, 2, 0.05, 1), (0, 3, 0.075, 1)]
    out = _events(st.process(_batch(rows, ["warmup", "stale", "dark"])))
    assert [e.frame_id for e in out] == [3]


def test_add_sample_is_thread_safe_with_process():
    st = ImuFrameStage(lag_s=10.0)
    stop = threading.Event()

    def feed():
        t = 0.0
        while not stop.is_set() and t < 5.0:
            st.add_sample("left", t, _sample(gx=16))
            t += 0.025

    th = threading.Thread(target=feed)
    th.start()
    got = []
    for k in range(0, 200, 10):
        got += _events(st.process(_batch(_frames(0, range(k + 1, k + 11)))))
    stop.set()
    th.join()
    flush = _batch([])
    st.on_scan_stop(flush)
    got += _events(flush)
    ids = [e.frame_id for e in got]
    assert ids == sorted(ids) and len(ids) == len(set(ids)) == 200


def test_average_helper():
    e = average_imu_samples("left", 7, 0.175, [])
    assert e.n_samples == 0 and e.gyro_dps is None
    assert IMU_FRAME_CHANNEL == "imu_frame"


# ── runner routing ────────────────────────────────────────────────────────

class _Rec:
    def __init__(self, channels):
        self.channels = set(channels)
        self.got = []

    def on_scan_start(self, meta):
        pass

    def consume(self, channel, payload):
        self.got.append((channel, payload))

    def on_complete(self):
        pass


def _ev(side="left", fid=1, t=0.025, n=1, acc=0.1, gyr=2.0):
    v = None if n == 0 else 0.0
    return ImuFrameMotion(side, fid, t, n, None if n == 0 else acc,
                          None if n == 0 else gyr, v, v, None if n == 0 else 1.0,
                          v, v, v)


def test_runner_routes_frames_to_imu_frame_channel_only():
    from omotion.pipeline.runner import ScanRunner
    sink, diag = _Rec({"imu_frame"}), _Rec({"diagnostics"})
    runner = ScanRunner(source=mock.Mock(), pipeline=mock.Mock(), sinks=[sink, diag])
    e = _ev()
    runner.dispatch_event(e)
    assert sink.got == [("imu_frame", e)] and diag.got == []


# ── ScanDBSink + ScanDatabase ─────────────────────────────────────────────

def _meta():
    from omotion.pipeline.sinks import ScanMetadata
    return ScanMetadata(scan_id="s", subject_id="subj", operator="op",
                        started_at_iso="2026-11-18T00:00:00Z", duration_sec=10,
                        left_camera_mask=1, right_camera_mask=1, reduced_mode=False)


def _final_interval():
    from omotion.pipeline.stages.dark import (
        EnrichedCorrectedFrame, EnrichedCorrectedInterval,
    )
    f = EnrichedCorrectedFrame(abs_frame_id=1, t=0.025, side="left", cam_id=0,
                               mean=100.0, std=2.0, contrast=0.02, bfi=4.0,
                               bvi=6.0, quality="ok", temp_c=None)
    return EnrichedCorrectedInterval(left_abs=0, right_abs=600, frames=[f])


def _record(db_path, events):
    from omotion.pipeline.sinks import ScanDBSink
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta())
    for e in events:
        sink.consume("imu_frame", e)
    sink.consume("final", _final_interval())
    sink.on_complete()


_KEYS = {"session_id", "side", "frame_id", "timestamp_s", "n_samples",
         "accel_dev_g", "gyro_dps", "ax_g", "ay_g", "az_g",
         "gx_dps", "gy_dps", "gz_dps"}


def test_db_sink_persists_frames_and_getter_filters(tmp_path):
    db_path = tmp_path / "scans.db"
    evs = ([_ev("left", f, 0.025 * f, n=f % 3, acc=0.01 * f) for f in range(1, 9)]
           + [_ev("right", 1, 0.025, gyr=30.0)])
    _record(db_path, evs)
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        rows = db.get_imu_frames(sid)
        assert len(rows) == len(evs) and set(rows[0]) == _KEYS
        left = db.get_imu_frames(sid, side=0)
        assert [r["frame_id"] for r in left] == list(range(1, 9))
        zero = [r for r in left if r["n_samples"] == 0]
        assert zero and all(r["accel_dev_g"] is None and r["az_g"] is None for r in zero)
        assert db.get_imu_frames(sid, side=1)[0]["gyro_dps"] == 30.0
        win = db.get_imu_frames(sid, side=0, t_lo=0.05, t_hi=0.1)   # inclusive
        assert [r["frame_id"] for r in win] == [2, 3, 4]
        assert list(db.iter_imu_frames(sid, t_lo=10.0)) == []


def test_db_sink_drops_rows_with_an_empty_session(tmp_path):
    from omotion.pipeline.sinks import ScanDBSink
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta())
    sink.consume("imu_frame", _ev())
    sink.on_complete()
    con = sqlite3.connect(db_path)
    try:
        assert con.execute("SELECT count(*) FROM sessions").fetchone()[0] == 0
        assert con.execute("SELECT count(*) FROM imu_frame").fetchone()[0] == 0
    finally:
        con.close()


def test_deleting_a_session_cascades_to_its_frames(tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_ev()])
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        assert db.delete_session(sid)
        assert db.get_imu_frames(sid) == []


_COLS = {"id"} | _KEYS


def test_v0_database_migrates_to_imu_frame(tmp_path):
    dest = tmp_path / "scans.db"
    shutil.copy2(FIXTURE, dest)
    with ScanDatabase(db_path=str(dest)) as db:
        conn = db._connection()
        assert db_schema.current_version(conn) == db_schema.SCHEMA_VERSION == 3
        cols = {r[1] for r in conn.execute("PRAGMA table_info('imu_frame')")}
        assert cols == _COLS
        assert conn.execute("SELECT count(*) FROM session_data").fetchone()[0] == 200


def test_v2_database_gains_imu_frame(tmp_path):
    path = tmp_path / "scans.db"
    con = sqlite3.connect(path)
    for target, _, fn in db_schema.MIGRATIONS[:2]:
        fn(con)
        con.execute(f"PRAGMA user_version = {target}")
    con.commit()
    con.close()
    with ScanDatabase(db_path=str(path)) as db:
        assert db_schema.current_version(db._connection()) == 3
        sid = db.create_session(session_label="x", session_start=0.0)
        db.insert_imu_frame_rows([{"session_id": sid, "side": 1, "frame_id": 5,
                                   "timestamp_s": 0.125, "n_samples": 0}])
        (row,) = db.get_imu_frames(sid)
        assert row["n_samples"] == 0 and row["gyro_dps"] is None


@pytest.fixture
def clinical(monkeypatch):
    pytest.importorskip("sqlcipher3")
    monkeypatch.setattr(db_key, "require_encryption", lambda: True)
    monkeypatch.setattr(db_key, "get_key", lambda *, create=False: FIXED_KEY)


def test_frames_round_trip_on_an_encrypted_db(clinical, tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_ev("left", 3, 0.075, acc=0.3)])
    with open(db_path, "rb") as fh:
        assert fh.read(16) != b"SQLite format 3\x00"
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        (row,) = db.get_imu_frames(sid, side=0)
        assert (row["frame_id"], row["accel_dev_g"]) == (3, 0.3)


# ── ScanWorkflow wiring ───────────────────────────────────────────────────

class _Src:
    def __init__(self, *, metadata, **_):
        self.metadata = metadata
        self._stop = threading.Event()
        self._packet_queues = {}

    def __iter__(self):
        while not self._stop.is_set():
            time.sleep(0.02)
        return iter(())

    def close(self):
        self._stop.set()

    def side_t0(self, side):
        return None


def _run(tmp_path, **req_kw):
    from omotion.MotionInterface import MotionInterface
    from omotion.ScanWorkflow import ScanRequest

    motion = MotionInterface(demo_mode=True, data_dir=str(tmp_path),
                             operator_id="test")
    made = []

    class _FakeLogger:
        def __init__(self, sensors, path, anchor_fn=None,
                     fallback_t0_monotonic=None, sample_fn=None):
            self.path, self.sample_fn, self.init_sample_fn = path, sample_fn, sample_fn
            self.events = []
            made.append(self)

        def set_sample_fn(self, fn):
            self.sample_fn = fn

        def start(self):
            self.events.append("start")

        def stop(self):
            self.events.append("stop")

    request = ScanRequest(subject_id="subj", duration_sec=30,
                          left_camera_mask=0xFF, right_camera_mask=0, **req_kw)
    wf = motion.scan_workflow
    started = threading.Event()
    with mock.patch("omotion.pipeline.sources.LiveUsbSource", _Src), \
         mock.patch("omotion.imu_csv.ImuStreamLogger", _FakeLogger), \
         mock.patch.object(motion.left, "is_connected", return_value=True), \
         mock.patch.object(motion.right, "is_connected", return_value=False), \
         mock.patch.object(wf, "_resolve_active_sides",
                           return_value=[("left", 0xFF, mock.MagicMock())]), \
         mock.patch.object(wf, "_scan_subscribe_state", lambda handles: None), \
         mock.patch.object(motion, "run_on_sensors", return_value=True), \
         mock.patch.object(motion.console, "set_trigger_json",
                           side_effect=lambda **kw: kw.get("data")), \
         mock.patch.object(motion.console, "start_trigger",
                           side_effect=lambda *a, **k: started.set() or True):
        assert wf.start_scan(request)
        started.wait(2.0)
        stages = list(wf._runner.pipeline.stages)
        wf.cancel_scan(join_timeout=5.0)
    return made, stages, wf


def test_imu_motion_adds_the_stage_and_feeds_it(tmp_path):
    made, stages, wf = _run(tmp_path, imu_motion=True)
    (lg,) = made
    assert lg.path is None and lg.events[0] == "start" and "stop" in lg.events
    assert isinstance(stages[-1], ImuFrameStage)
    assert lg.init_sample_fn == stages[-1].add_sample
    assert wf.last_imu_csv_path is None


def test_csv_and_motion_share_one_stream(tmp_path):
    made, stages, _ = _run(tmp_path, imu_motion=True, write_imu_csv=True)
    (lg,) = made
    assert lg.path.endswith("_subj_imu.csv") and lg.init_sample_fn is not None


def test_csv_only_has_no_stage(tmp_path):
    made, stages, _ = _run(tmp_path, write_imu_csv=True)
    (lg,) = made
    assert lg.init_sample_fn is None
    assert not any(isinstance(s, ImuFrameStage) for s in stages)


def test_neither_flag_starts_no_imu(tmp_path):
    made, stages, _ = _run(tmp_path)
    assert made == []
    assert not any(isinstance(s, ImuFrameStage) for s in stages)
