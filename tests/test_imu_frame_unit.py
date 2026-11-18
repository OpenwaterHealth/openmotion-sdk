"""Per-frame IMU motion (bloodflow-app#783) — no hardware.

Covers the pure magnitude function, ImuFrameStage's join of IMU samples to
histogram frames (sample windows, 0/2-sample frames, waiting for the IMU,
the lag and final flushes, skipped frame types, sides without an IMU), the
stamping of closing intervals, the runner's "imu_frame" routing, the IMU
columns on session_data (ScanDBSink writes, get_imu_frames reads; plain and
SQLCipher), the schema migration of older databases, and the ScanWorkflow
wiring of ScanRequest.imu_motion.
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
import math

from omotion.imu_csv import (
    ACCEL_BASELINE_WINDOW_S, AccelBaseline, ImuSample, accel_deviation,
    gyro_magnitude,
)
from omotion.pipeline.batch import IMU_FRAME_CHANNEL, FrameBatch, ImuFrameMotion
from omotion.pipeline.stages.imu_frame import ImuFrameStage, average_imu_samples
from omotion.ScanDatabase import ScanDatabase

FIXTURE = Path(__file__).parent / "fixtures" / "legacy_scans_v0.db"
FIXED_KEY = "a" * 64


# ── motion measures ───────────────────────────────────────────────────────

def _feed(base, t0, t1, fn, rate=40.0):
    """Feed base with fn(t) -> accel vector at rate Hz over [t0, t1);
    returns [(t, deviation)]."""
    out, t = [], t0
    while t < t1 - 1e-9:
        out.append((t, base.update(t, fn(t))))
        t += 1.0 / rate
    return out


def test_tilted_gravity_plus_axis_bias_reads_zero_once_the_window_fills():
    # Gravity 30 deg off z, plus a +15/-20/+25 mg per-axis offset: |a| != 1 g,
    # which the old | |a| - 1 | read as ~12 mg of "motion" forever.
    g = (0.0, math.sin(math.radians(30)), math.cos(math.radians(30)))
    bias = (0.015, -0.020, 0.025)
    a = tuple(x + b for x, b in zip(g, bias))
    assert abs(math.sqrt(sum(v * v for v in a)) - 1.0) > 0.01
    out = _feed(AccelBaseline(), 0.0, 5.0, lambda t: a)
    assert max(d for t, d in out) == pytest.approx(0.0, abs=1e-12)


def test_a_step_in_pose_reads_as_motion_then_decays_within_the_window():
    base = AccelBaseline()
    flat = (0.0, 0.0, 1.0)
    tilted = (0.0, math.sin(math.radians(20)), math.cos(math.radians(20)))
    _feed(base, 0.0, 3.0, lambda t: flat)
    after = _feed(base, 3.0, 6.0, lambda t: tilted)
    assert after[0][1] > 0.3                                # the move itself
    assert max(d for t, d in after if t >= 3.0 + ACCEL_BASELINE_WINDOW_S)         == pytest.approx(0.0, abs=1e-9)                     # settled after 2 s


def test_a_small_1hz_oscillation_is_detected():
    amp = 0.02                                              # 20 mg sway along x
    out = _feed(AccelBaseline(), 0.0, 6.0,
                lambda t: (amp * math.sin(2 * math.pi * t), 0.0, 1.0))
    steady = [d for t, d in out if t >= 2.0]
    assert max(steady) == pytest.approx(amp, rel=0.1)
    assert sum(steady) / len(steady) > amp * 0.5


def test_window_partial_fill_and_reset():
    base = AccelBaseline(window_s=1.0)
    assert base.update(0.0, (0, 0, 1)) == 0.0               # first sample: own mean
    assert base.update(0.5, (0, 0, 2)) == pytest.approx(0.5)
    assert base.update(1.0, (0, 0, 2)) == pytest.approx(0.0)  # 0.0 dropped: (0.5, 1.0]
    base.reset()
    assert base.update(5.0, (1, 0, 0)) == 0.0


def test_pure_helpers():
    assert accel_deviation((0, 0, 1.2), (0, 0, 1.0)) == pytest.approx(0.2)
    assert gyro_magnitude((3.0, 4.0, 0.0)) == pytest.approx(5.0)


def _bl(samples):
    """Deviation of each (t, az_g) through a fresh baseline, in order."""
    base = AccelBaseline()
    return [base.update(t, (0.0, 0.0, az)) for t, az in samples]


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
    d = _bl([(0.010, 1 + 205 / 2048), (0.030, 1 + 410 / 2048), (0.045, 1.0)])
    assert f2.accel_dev_g == pytest.approx((d[1] + d[2]) / 2)  # mean per sample
    assert f2.gyro_dps == pytest.approx((10 + 20) / 2)
    assert not hasattr(f2, "ax_g")                              # magnitudes only
    f3 = out[2]
    assert f3.accel_dev_g is None and f3.gyro_dps is None


def test_first_frame_takes_one_nominal_period_and_drops_older_samples():
    st = ImuFrameStage()
    st.add_sample("left", -0.5, _sample(az=4096))               # pre-scan: dropped
    st.add_sample("left", 0.990, _sample(az=2048 + 1024))
    st.add_sample("left", 1.2, _sample())
    (e,) = _events(st.process(_batch([(0, 40, 1.0, 1)])))
    d = _bl([(-0.5, 2.0), (0.990, 1.5)])                    # baseline still sees -0.5
    assert e.n_samples == 1 and e.accel_dev_g == pytest.approx(d[1])


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
    return ImuFrameMotion(side, fid, t, n, None if n == 0 else acc,
                          None if n == 0 else gyr)


def test_runner_routes_frames_to_imu_frame_channel_only():
    from omotion.pipeline.runner import ScanRunner
    sink, diag = _Rec({"imu_frame"}), _Rec({"diagnostics"})
    runner = ScanRunner(source=mock.Mock(), pipeline=mock.Mock(), sinks=[sink, diag])
    e = _ev()
    runner.dispatch_event(e)
    assert sink.got == [("imu_frame", e)] and diag.got == []


# ── stamping the corrected record ─────────────────────────────────────────

def _cframe(side, fid, cam, t=None):
    from omotion.pipeline.stages.dark import EnrichedCorrectedFrame
    return EnrichedCorrectedFrame(abs_frame_id=fid, t=(0.025 * fid if t is None else t),
                                  side=side, cam_id=cam, mean=100.0, std=2.0,
                                  contrast=0.02, bfi=4.0, bvi=6.0, quality="ok")


def _closing(frames):
    from omotion.pipeline.batch import IntervalClosed
    from omotion.pipeline.stages.dark import EnrichedCorrectedInterval
    return IntervalClosed(corrected_batch=EnrichedCorrectedInterval(
        left_abs=0, right_abs=600, frames=frames))


def test_closing_interval_frames_are_stamped_per_side_and_frame():
    st = ImuFrameStage()
    st.add_sample("left", 0.010, _sample(gx=164))            # frame 1: 10 dps
    st.add_sample("left", 0.030, _sample(az=2048 + 1024))     # frame 2: 0.5 g
    st.add_sample("left", 0.100, _sample())
    st.process(_batch(_frames(0, [1, 2, 3]) + _frames(1, [1, 2, 3])))
    frames = [_cframe(s, f, c) for s in ("left", "right") for f in (1, 2, 3) for c in (1, 2)]
    b = _batch([])
    b.events.append(_closing(frames))
    st.process(b)
    left = {(f.abs_frame_id, f.cam_id): f for f in frames if f.side == "left"}
    for cam in (1, 2):                                          # cameras share values
        assert (left[(1, cam)].imu_n_samples, left[(1, cam)].imu_gyro_dps) == \
            (1, pytest.approx(10.0))
        assert left[(2, cam)].imu_accel_dev_g == pytest.approx(0.25)  # 1.5 g vs mean 1.25
        assert (left[(3, cam)].imu_n_samples, left[(3, cam)].imu_accel_dev_g) == (0, None)
    right = [f for f in frames if f.side == "right"]
    assert all(f.imu_n_samples is None for f in right)          # no IMU on that side
    assert st.stamped == 6


def test_frames_still_waiting_for_the_imu_are_averaged_when_their_interval_closes():
    st = ImuFrameStage(lag_s=10.0)
    st.add_sample("left", 0.020, _sample(gx=328))
    b = _batch(_frames(0, [1, 2]))
    assert _events(st.process(b)) == []                         # IMU behind frame 1
    frames = [_cframe("left", 1, 1), _cframe("left", 2, 1)]
    b2 = _batch([])
    b2.events.append(_closing(frames))
    out = _events(st.process(b2))
    assert [e.frame_id for e in out] == [1, 2]                  # forced, still sent live
    assert frames[0].imu_gyro_dps == pytest.approx(20.0) and frames[1].imu_n_samples == 0


def test_scan_stop_stamps_the_terminal_interval():
    st = ImuFrameStage()
    st.add_sample("right", 0.010, _sample(gx=164))
    st.process(_batch(_frames(1, [1])))
    f = _cframe("right", 1, -1)                                 # reduced-mode side row
    flush = _batch([])
    flush.events.append(_closing([f]))
    st.on_scan_stop(flush)
    assert (f.imu_n_samples, f.imu_gyro_dps) == (1, pytest.approx(10.0))


# ── ScanDBSink + ScanDatabase ─────────────────────────────────────────────

def _meta(reduced=False):
    from omotion.pipeline.sinks import ScanMetadata
    return ScanMetadata(scan_id="s", subject_id="subj", operator="op",
                        started_at_iso="2026-11-18T00:00:00Z", duration_sec=10,
                        left_camera_mask=1, right_camera_mask=1, reduced_mode=reduced)


def _stamped(side, fid, cam, n=1, acc=0.1, gyr=2.0):
    f = _cframe(side, fid, cam)
    if n is not None:
        f.imu_n_samples = n
        f.imu_accel_dev_g = None if n == 0 else acc
        f.imu_gyro_dps = None if n == 0 else gyr
    return f


def _record(db_path, frames, reduced=False):
    from omotion.pipeline.sinks import ScanDBSink
    from omotion.pipeline.stages.dark import EnrichedCorrectedInterval
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta(reduced))
    sink.consume("final", EnrichedCorrectedInterval(left_abs=0, right_abs=600,
                                                    frames=frames))
    sink.on_complete()


_KEYS = {"session_id", "side", "frame_id", "timestamp_s", "n_samples",
         "accel_dev_g", "gyro_dps"}


def test_imu_values_are_stored_on_the_session_data_rows(tmp_path):
    db_path = tmp_path / "scans.db"
    frames = ([_stamped("left", f, c, n=f % 3, acc=0.01 * f) for f in range(1, 9)
               for c in (0, 1)]
              + [_stamped("right", 1, 0, gyr=30.0), _stamped("right", 2, 0, n=None)])
    _record(db_path, frames)
    con = sqlite3.connect(db_path)
    try:
        rows = con.execute(
            "SELECT side, frame_id, cam_id, imu_n_samples, imu_accel_dev_g, imu_gyro_dps "
            "FROM session_data ORDER BY side, frame_id, cam_id").fetchall()
    finally:
        con.close()
    assert len(rows) == 18                                      # IMU on the scan rows
    by = {(r[0], r[1], r[2]): r[3:] for r in rows}
    assert by[(0, 4, 0)] == by[(0, 4, 1)] == (1, 0.04, 2.0)     # cameras share values
    assert by[(0, 3, 0)] == (0, None, None)                     # no sample in frame
    assert by[(1, 2, 0)] == (None, None, None)                  # no IMU at all
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        out = db.get_imu_frames(sid)
        assert set(out[0]) == _KEYS
        assert [(r["side"], r["frame_id"]) for r in out] == \
            [(0, f) for f in range(1, 9)] + [(1, 1)]            # one per side/frame
        assert db.get_imu_frames(sid, side=1)[0]["gyro_dps"] == 30.0
        zero = [r for r in out if r["n_samples"] == 0]
        assert zero and all(r["accel_dev_g"] is None and r["gyro_dps"] is None for r in zero)
        win = db.get_imu_frames(sid, side=0, t_lo=0.05, t_hi=0.1)   # inclusive
        assert [r["frame_id"] for r in win] == [2, 3, 4]
        assert list(db.iter_imu_frames(sid, t_lo=10.0)) == []


def test_reduced_mode_side_rows_carry_the_imu(tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_stamped("left", 1, -1, gyr=12.5)], reduced=True)
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        (r,) = db.get_imu_frames(sid)
        assert (r["side"], r["frame_id"], r["gyro_dps"]) == (0, 1, 12.5)


def test_deleting_a_session_deletes_its_imu_values(tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_stamped("left", 1, 0)])
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        assert db.delete_session(sid)
        assert db.get_imu_frames(sid) == []


_IMU_COLS = {"imu_n_samples", "imu_accel_dev_g", "imu_gyro_dps"}


def test_v0_database_gains_the_imu_columns(tmp_path):
    dest = tmp_path / "scans.db"
    shutil.copy2(FIXTURE, dest)
    with ScanDatabase(db_path=str(dest)) as db:
        conn = db._connection()
        assert db_schema.current_version(conn) == db_schema.SCHEMA_VERSION == 3
        cols = {r[1] for r in conn.execute("PRAGMA table_info('session_data')")}
        assert _IMU_COLS <= cols
        tables = {r[0] for r in conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        assert not {"imu_frame", "imu_motion"} & tables
        assert conn.execute("SELECT count(*) FROM session_data").fetchone()[0] == 200
        sid = conn.execute("SELECT session_id FROM session_data LIMIT 1").fetchone()[0]
        assert db.get_imu_frames(sid) == []                     # old rows: no IMU


def test_v2_database_gains_the_imu_columns(tmp_path):
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
        db.insert_session_data(sid, 0, 1, 0.125, frame_id=5, bfi=1.0,
                               imu_n_samples=2, imu_accel_dev_g=0.03,
                               imu_gyro_dps=4.5)
        (row,) = db.get_imu_frames(sid)
        assert (row["side"], row["frame_id"], row["n_samples"], row["gyro_dps"]) == \
            (1, 5, 2, 4.5)


@pytest.fixture
def clinical(monkeypatch):
    pytest.importorskip("sqlcipher3")
    monkeypatch.setattr(db_key, "require_encryption", lambda: True)
    monkeypatch.setattr(db_key, "get_key", lambda *, create=False: FIXED_KEY)


def test_imu_values_round_trip_on_an_encrypted_db(clinical, tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_stamped("left", 3, 0, acc=0.3)])
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
