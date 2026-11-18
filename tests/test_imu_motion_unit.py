"""IMU motion magnitude in 1/8 s bins (bloodflow-app#783) — no hardware.

Covers the pure magnitude function, bin edges/alignment, held (pre-anchor)
samples, the flush at stop, runner event delivery on the "imu_motion"
channel, ScanDBSink persistence + the ScanDatabase read API (plain and
SQLCipher), the schema migration of an older database, and the ScanWorkflow
wiring of ScanRequest.imu_motion.
"""
import shutil
import sqlite3
import threading
import time
from pathlib import Path
from unittest import mock

import pytest

from omotion import db_key, db_schema
from omotion.imu_csv import (
    MOTION_BIN_S,
    ImuStreamLogger,
    MotionBinner,
    motion_bin_index,
    motion_bins_from_imu_csv,
    motion_magnitude,
    raw_motion_magnitude,
)
from omotion.pipeline.batch import IMU_MOTION_CHANNEL, ImuMotionBin, ImuMotionEvent
from omotion.ScanDatabase import ScanDatabase

FIXTURE = Path(__file__).parent / "fixtures" / "legacy_scans_v0.db"
FIXED_KEY = "a" * 64


def _line(frame, ms=None, a=(0, 0, 2048), g=(0, 0, 0)):
    body = (f'{{"F":{frame},"G":[{g[0]},{g[1]},{g[2]}],"M":[0,0,0],'
            f'"A":[{a[0]},{a[1]},{a[2]}],"T":25.00')
    if ms is not None:
        body += f',"ms":{ms}'
    return (body + "}\r\n").encode("ascii")


# ── magnitude ─────────────────────────────────────────────────────────────

def test_motion_magnitude_at_rest_in_any_orientation_is_zero():
    assert motion_magnitude((0, 0, 1), (0, 0, 0)) == (0.0, 0.0)
    acc, _ = motion_magnitude((0.6, 0.0, 0.8), (0, 0, 0))      # tilted, still 1 g
    assert acc == pytest.approx(0.0, abs=1e-12)


def test_motion_magnitude_values():
    acc, gyr = motion_magnitude((0, 0, 2.0), (3.0, 4.0, 0.0))
    assert acc == pytest.approx(1.0) and gyr == pytest.approx(5.0)
    acc, _ = motion_magnitude((0, 0, 0), (0, 0, 0))           # free fall
    assert acc == pytest.approx(1.0)


def test_raw_motion_magnitude_scales_counts():
    acc, gyr = raw_motion_magnitude((0, 0, 3072), (164, 0, 0))  # 1.5 g, 10 dps
    assert acc == pytest.approx(0.5) and gyr == pytest.approx(10.0)


# ── bins ──────────────────────────────────────────────────────────────────

@pytest.mark.parametrize("t,idx", [
    (0.0, 0), (0.1249, 0), (0.125, 1), (0.375, 3), (1.0, 8),
    (-0.025, -1), (-0.125, -1), (-0.1251, -2),
])
def test_bins_align_to_multiples_of_an_eighth_second(t, idx):
    assert MOTION_BIN_S == 0.125
    assert motion_bin_index(t) == idx


def test_binner_emits_a_bin_when_a_sample_passes_its_end():
    b = MotionBinner("left")
    out = []
    for i, ts in enumerate([0.0, 0.025, 0.05, 0.075, 0.1]):
        out += b.add(ts, "device", (0, 0, 2048 + 100 * i), (0, 0, 0))
    assert out == []                                       # bin 0 still open
    out += b.add(0.125, "device", (0, 0, 2048), (164, 0, 0))
    (bin0,) = out
    assert (bin0.side, bin0.t_start_s, bin0.t_end_s, bin0.n_samples) == \
        ("left", 0.0, 0.125, 5)
    assert bin0.accel_dev_g == pytest.approx(400 / 2048)   # peak, not mean
    assert bin0.gyro_dps == 0.0 and bin0.time_source == "device"
    (last,) = b.flush()                                     # partial last bin
    assert (last.t_start_s, last.n_samples) == (0.125, 1)
    assert last.gyro_dps == pytest.approx(10.0)
    assert b.flush() == []


def test_binner_skips_empty_bins_and_marks_host_time():
    b = MotionBinner("right")
    b.add(0.0, "host", (0, 0, 2048), (0, 0, 0))
    out = b.add(0.6, "device", (0, 0, 2048), (0, 0, 0))   # bins 1-3 empty
    assert [x.t_start_s for x in out] == [0.0]
    assert out[0].time_source == "host"
    assert b.flush()[0].t_start_s == 0.5


def test_event_alias_and_channel():
    assert ImuMotionEvent is ImuMotionBin
    assert IMU_MOTION_CHANNEL == "imu_motion"


# ── logger (fake sensor) ──────────────────────────────────────────────────

class _Stream:
    def __init__(self):
        self.q = None

    def flush_stale_data(self, **_):
        return 0

    def start_streaming(self, q, _size):
        self.q = q

    def stop_streaming(self):
        pass


class _Sensor:
    _version = "v1.8.3"

    def __init__(self):
        self.uart = mock.Mock()
        self.uart.imu = _Stream()

    def imu_init(self):
        return True

    def imu_on(self):
        return True

    def imu_off(self):
        return True


def _wait(pred, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not pred():
        time.sleep(0.01)


def test_logger_bins_held_samples_once_anchored_in_time_order():
    sensor, anchor, got = _Sensor(), {}, []
    log = ImuStreamLogger([("left", sensor)], None,
                          anchor_fn=lambda s: anchor.get(s), motion_fn=got.append)
    log.start()
    # 9.850 .. 10.200 s device time; the first histogram is at 10.000 s.
    for i, ms in enumerate(range(9_850, 10_225, 25)):
        sensor.uart.imu.q.put(_line(i, ms=ms, g=(164 * (i % 3), 0, 0)))
    time.sleep(0.2)
    assert got == []                                       # no anchor: held
    anchor["left"] = (10.0, time.monotonic())
    _wait(lambda: len(got) >= 3)
    log.stop()
    starts = [b.t_start_s for b in got]
    assert starts == sorted(starts)
    assert starts[0] == -0.25 and starts[-1] == 0.125     # flushed partial last
    assert all(b.time_source == "device" for b in got)
    assert sum(b.n_samples for b in got) == 15


def test_logger_flushes_last_bin_at_stop_without_new_data():
    sensor, got = _Sensor(), []
    log = ImuStreamLogger([("right", sensor)], None,
                          anchor_fn=lambda s: (0.0, time.monotonic()),
                          motion_fn=got.append)
    log.start()
    sensor.uart.imu.q.put(_line(1, ms=10) + _line(2, ms=35))
    time.sleep(0.2)
    assert got == []
    log.stop()
    assert [(b.side, b.t_start_s, b.n_samples) for b in got] == [("right", 0.0, 2)]


def test_logger_motion_fn_failure_is_contained():
    sensor = _Sensor()

    def boom(_b):
        raise RuntimeError("sink down")

    log = ImuStreamLogger([("left", sensor)], None,
                          anchor_fn=lambda s: (0.0, time.monotonic()), motion_fn=boom)
    log.start()
    sensor.uart.imu.q.put(_line(1, ms=0) + _line(2, ms=200))
    time.sleep(0.2)
    log.stop()                                             # must not raise


def test_live_bins_match_bins_rebuilt_from_the_imu_csv(tmp_path):
    sensor, got = _Sensor(), []
    path = tmp_path / "x_imu.csv"
    log = ImuStreamLogger([("left", sensor)], str(path),
                          anchor_fn=lambda s: (1.0, time.monotonic()),
                          motion_fn=got.append)
    log.start()
    for i, ms in enumerate(range(1_000, 1_600, 25)):
        sensor.uart.imu.q.put(_line(i, ms=ms, a=(0, 0, 2048 + 7 * i), g=(i, 0, 0)))
    time.sleep(0.3)
    log.stop()
    rebuilt = motion_bins_from_imu_csv(str(path))
    assert [(b.t_start_s, b.n_samples) for b in rebuilt] == \
        [(b.t_start_s, b.n_samples) for b in got]
    for a, b in zip(rebuilt, got):
        assert a.accel_dev_g == pytest.approx(b.accel_dev_g)
        assert a.gyro_dps == pytest.approx(b.gyro_dps)


# ── runner delivery ───────────────────────────────────────────────────────

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


def _bin(side="left", t=0.0, acc=0.1, gyr=2.0):
    return ImuMotionBin(side=side, t_start_s=t, t_end_s=t + 0.125,
                        accel_dev_g=acc, gyro_dps=gyr, n_samples=5,
                        time_source="host")


def test_runner_routes_motion_bins_to_imu_motion_channel_only():
    from omotion.pipeline.runner import ScanRunner
    motion, diag = _Rec({"imu_motion"}), _Rec({"diagnostics"})
    runner = ScanRunner(source=mock.Mock(), pipeline=mock.Mock(), sinks=[motion, diag])
    b = _bin()
    runner.dispatch_event(b)
    assert motion.got == [("imu_motion", b)]
    assert diag.got == []


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


def _record(db_path, bins):
    from omotion.pipeline.sinks import ScanDBSink
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta())
    # Bins arrive on another thread, like the IMU consumer.
    t = threading.Thread(target=lambda: [sink.consume("imu_motion", b) for b in bins])
    t.start()
    t.join()
    sink.consume("final", _final_interval())
    sink.on_complete()


def test_db_sink_persists_bins_and_read_api_filters(tmp_path):
    db_path = tmp_path / "scans.db"
    bins = ([_bin("left", t=0.125 * k, acc=0.01 * k) for k in range(-2, 6)]
            + [_bin("right", t=0.0, gyr=30.0)])
    _record(db_path, bins)
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        rows = db.get_imu_motion(sid)
        assert len(rows) == len(bins)
        assert set(rows[0]) == {"session_id", "side", "t_start_s", "t_end_s",
                                "accel_dev_g", "gyro_dps", "n_samples", "time_source"}
        left = db.get_imu_motion(sid, side=0)
        assert [r["t_start_s"] for r in left] == [0.125 * k for k in range(-2, 6)]
        assert db.get_imu_motion(sid, side=1)[0]["gyro_dps"] == 30.0
        win = db.get_imu_motion(sid, side=0, t_lo=0.2, t_hi=0.4)  # overlapping bins
        assert [r["t_start_s"] for r in win] == [0.125, 0.25, 0.375]
        assert list(db.iter_imu_motion(sid, side=0, t_lo=10.0)) == []


def test_db_sink_drops_bins_with_an_empty_session(tmp_path):
    from omotion.pipeline.sinks import ScanDBSink
    db_path = tmp_path / "scans.db"
    sink = ScanDBSink(db_path=str(db_path))
    sink.on_scan_start(_meta())
    sink.consume("imu_motion", _bin())
    sink.on_complete()                                      # no corrected rows
    con = sqlite3.connect(db_path)
    try:
        assert con.execute("SELECT count(*) FROM sessions").fetchone()[0] == 0
        assert con.execute("SELECT count(*) FROM imu_motion").fetchone()[0] == 0
    finally:
        con.close()


def test_deleting_a_session_cascades_to_its_bins(tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_bin()])
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        assert db.delete_session(sid)
        assert db.get_imu_motion(sid) == []


def test_older_database_migrates_to_imu_motion(tmp_path):
    dest = tmp_path / "scans.db"
    shutil.copy2(FIXTURE, dest)                             # v0 field database
    with ScanDatabase(db_path=str(dest)) as db:
        conn = db._connection()
        assert db_schema.current_version(conn) == db_schema.SCHEMA_VERSION == 3
        cols = {r[1] for r in conn.execute("PRAGMA table_info('imu_motion')")}
        assert cols == {"id", "session_id", "side", "t_start_s", "t_end_s",
                        "accel_dev_g", "gyro_dps", "n_samples", "time_source"}
        assert conn.execute("SELECT count(*) FROM session_data").fetchone()[0] == 200


def test_v2_database_gains_imu_motion(tmp_path):
    path = tmp_path / "scans.db"
    con = sqlite3.connect(path)
    for target, _, fn in db_schema.MIGRATIONS[:2]:          # build a v2 DB
        fn(con)
        con.execute(f"PRAGMA user_version = {target}")
    con.commit()
    con.close()
    with ScanDatabase(db_path=str(path)) as db:
        assert db_schema.current_version(db._connection()) == 3
        sid = db.create_session(session_label="x", session_start=0.0)
        db.insert_imu_motion_rows([{
            "session_id": sid, "side": 1, "t_start_s": 0.0, "t_end_s": 0.125,
            "accel_dev_g": 0.2, "gyro_dps": 12.5, "n_samples": 5,
            "time_source": "device"}])
        assert db.get_imu_motion(sid)[0]["gyro_dps"] == 12.5


@pytest.fixture
def clinical(monkeypatch):
    pytest.importorskip("sqlcipher3")
    monkeypatch.setattr(db_key, "require_encryption", lambda: True)
    monkeypatch.setattr(db_key, "get_key", lambda *, create=False: FIXED_KEY)


def test_bins_round_trip_on_an_encrypted_db(clinical, tmp_path):
    db_path = tmp_path / "scans.db"
    _record(db_path, [_bin("left", t=0.25, acc=0.3)])
    with open(db_path, "rb") as fh:
        assert fh.read(16) != b"SQLite format 3\x00"         # really encrypted
    with ScanDatabase(db_path=str(db_path)) as db:
        sid = next(db.iter_sessions())["id"]
        (row,) = db.get_imu_motion(sid, side=0)
        assert (row["t_start_s"], row["accel_dev_g"]) == (0.25, 0.3)


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
    rec = _Rec({"imu_motion"})

    class _FakeLogger:
        def __init__(self, sensors, path, anchor_fn=None,
                     fallback_t0_monotonic=None, motion_fn=None):
            self.path, self.motion_fn, self.events = path, motion_fn, []
            self.init_motion_fn = motion_fn
            made.append(self)

        def set_motion_fn(self, fn):
            self.motion_fn = fn

        def start(self):
            self.events.append("start")
            if self.motion_fn is not None:
                self.motion_fn(_bin(t=-0.125))

        def stop(self):
            self.events.append("stop")

    request = ScanRequest(subject_id="subj", duration_sec=30,
                          left_camera_mask=0xFF, right_camera_mask=0,
                          sinks=[rec], **req_kw)
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
        wf.cancel_scan(join_timeout=5.0)
    return made, rec, wf


def test_imu_motion_alone_streams_without_a_csv_and_reaches_sinks(tmp_path):
    made, rec, wf = _run(tmp_path, imu_motion=True)
    (lg,) = made
    assert lg.path is None                                  # no CSV requested
    assert lg.events[0] == "start" and "stop" in lg.events   # stop is idempotent
    assert [c for c, _ in rec.got] == ["imu_motion"]
    assert rec.got[0][1].t_start_s == -0.125
    assert wf.last_imu_csv_path is None


def test_csv_and_motion_share_one_stream(tmp_path):
    made, rec, _wf = _run(tmp_path, imu_motion=True, write_imu_csv=True)
    (lg,) = made
    assert lg.path.endswith("_subj_imu.csv") and lg.init_motion_fn is not None
    assert [c for c, _ in rec.got] == ["imu_motion"]


def test_csv_only_streams_without_motion_events(tmp_path):
    made, rec, _wf = _run(tmp_path, write_imu_csv=True)
    (lg,) = made
    assert lg.init_motion_fn is None and rec.got == []


def test_neither_flag_starts_no_imu(tmp_path):
    made, rec, _wf = _run(tmp_path)
    assert made == [] and rec.got == []
