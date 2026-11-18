"""Unit tests for the per-scan IMU CSV logger (bloodflow-app#779) — no hardware.

Covers the line parser (partial lines, bad JSON, with/without ``ms``), the
timestamp math (device vs host, pre-anchor buffering, rollover, fallback),
the CSV columns/conversions, the logger lifecycle against a fake sensor, the
LiveUsbSource anchor accessor, and the ScanWorkflow wiring.
"""
import csv
import json
import threading
import time
from unittest import mock

import pytest

from omotion import imu_csv
from omotion.imu_csv import (
    IMU_CSV_HEADERS,
    ImuCsvLogger,
    ImuLineParser,
    ImuSample,
    device_timestamp_s,
    imu_row,
    parse_imu_line,
    resolve_timestamp,
    sensor_fw_supports_imu_stream,
)


def _line(frame=1, ms=None, a=(0, 0, 2048), g=(164, -164, 0), m=(1, 2, 3),
          t="25.50"):
    body = (f'{{"F":{frame},"G":[{g[0]},{g[1]},{g[2]}],'
            f'"M":[{m[0]},{m[1]},{m[2]}],"A":[{a[0]},{a[1]},{a[2]}],"T":{t}')
    if ms is not None:
        body += f',"ms":{ms}'
    return (body + "}\r\n").encode("ascii")


# ── parser ────────────────────────────────────────────────────────────────

def test_parse_line_without_ms():
    s = parse_imu_line(_line(frame=7))
    assert s == ImuSample(frame=7, accel=(0, 0, 2048), gyro=(164, -164, 0),
                          mag=(1, 2, 3), temp_c=25.5, device_ms=None)


def test_parse_line_with_ms():
    s = parse_imu_line(_line(frame=7, ms=123456))
    assert s.device_ms == 123456


@pytest.mark.parametrize("bad", [
    b"", b"   \r\n", b"not json", b'{"F":1,"G":[1,2,3]',          # truncated
    b'{"F":1,"G":[1,2],"M":[1,2,3],"A":[1,2,3],"T":1}',          # short triplet
    b'{"G":[1,2,3],"M":[1,2,3],"A":[1,2,3],"T":1}',              # no F
    b'{"F":"x","G":[1,2,3],"M":[1,2,3],"A":[1,2,3],"T":1}',      # bad F
    b"[1,2,3]", b"\xff\xfe{}",                                    # non-dict / non-ascii
])
def test_parse_line_rejects_malformed(bad):
    assert parse_imu_line(bad) is None


def test_line_parser_reassembles_split_lines():
    p = ImuLineParser()
    raw = _line(frame=1) + _line(frame=2, ms=50)
    out = []
    for i in range(0, len(raw), 13):                 # arbitrary USB chunking
        out += p.feed(raw[i:i + 13])
    assert [s.frame for s in out] == [1, 2]
    assert out[1].device_ms == 50
    assert p.bad_lines == 0 and p.good_lines == 2


def test_line_parser_multiple_lines_one_chunk_and_nul_padding():
    p = ImuLineParser()
    out = p.feed(_line(frame=1) + b"\x00\x00" + _line(frame=2) + b"\x00" * 40)
    assert [s.frame for s in out] == [1, 2]


def test_line_parser_skips_bad_line_and_keeps_going():
    p = ImuLineParser()
    out = p.feed(_line(frame=1) + b'{"F":2,"G":[garbage\r\n' + _line(frame=3))
    assert [s.frame for s in out] == [1, 3]
    assert p.bad_lines == 1


def test_line_parser_resyncs_after_runaway_garbage():
    p = ImuLineParser()
    assert p.feed(b"x" * 2000) == []
    assert p.bad_lines == 1
    assert [s.frame for s in p.feed(_line(frame=9))] == [9]


def test_line_parser_flush_parses_unterminated_tail():
    p = ImuLineParser()
    assert p.feed(_line(frame=4).rstrip(b"\r\n")) == []
    assert [s.frame for s in p.flush()] == [4]
    assert p.flush() == []


# ── timestamp math ────────────────────────────────────────────────────────

def test_device_timestamp_relative_to_histogram_t0():
    assert device_timestamp_s(10_500, 10.0) == pytest.approx(0.5)
    # Before the first histogram sample -> negative.
    assert device_timestamp_s(9_975, 10.0) == pytest.approx(-0.025)


def test_device_timestamp_unwraps_tim5_rollover():
    roll = imu_csv._DEVICE_ROLLOVER_S
    t0 = roll - 0.1                                   # anchor just before wrap
    assert device_timestamp_s(100, t0) == pytest.approx(0.2, abs=1e-6)


def _sample(ms=None):
    return ImuSample(frame=1, accel=(0, 0, 0), gyro=(0, 0, 0), mag=(0, 0, 0),
                     temp_c=0.0, device_ms=ms)


def test_resolve_uses_device_clock_when_ms_present():
    assert resolve_timestamp(_sample(ms=12_040), host_monotonic=999.0,
                             anchor=(12.0, 500.0)) == (pytest.approx(0.04), "device")


def test_resolve_uses_host_clock_without_ms():
    ts, src = resolve_timestamp(_sample(), host_monotonic=500.25,
                                anchor=(12.0, 500.0))
    assert src == "host" and ts == pytest.approx(0.25)


def test_resolve_waits_for_anchor_unless_fallback_given():
    assert resolve_timestamp(_sample(ms=5), 100.0, None) is None
    ts, src = resolve_timestamp(_sample(ms=5), 100.5, None,
                                fallback_t0_monotonic=100.0)
    assert src == "host" and ts == pytest.approx(0.5)


# ── CSV columns ───────────────────────────────────────────────────────────

def test_csv_headers_exact():
    assert IMU_CSV_HEADERS == [
        "side", "imu_frame", "timestamp_s", "time_source", "device_ms",
        "host_time_unix", "ax", "ay", "az", "gx", "gy", "gz",
        "mx", "my", "mz", "accel_x_g", "accel_y_g", "accel_z_g",
        "gyro_x_dps", "gyro_y_dps", "gyro_z_dps", "temp_c",
    ]


def test_imu_row_converts_units_and_blanks_missing_ms():
    s = parse_imu_line(_line(frame=3, a=(-2048, 1024, 2048), g=(164, -1640, 0)))
    row = dict(zip(IMU_CSV_HEADERS, imu_row("left", s, -0.0251234567, "host", 1.7e9)))
    assert row["side"] == "left" and row["imu_frame"] == 3
    assert row["device_ms"] == ""
    assert row["timestamp_s"] == pytest.approx(-0.025123)
    assert (row["accel_x_g"], row["accel_y_g"], row["accel_z_g"]) == (-1.0, 0.5, 1.0)
    assert (row["gyro_x_dps"], row["gyro_y_dps"], row["gyro_z_dps"]) == (10.0, -100.0, 0.0)
    assert row["temp_c"] == 25.5
    assert len(row) == len(IMU_CSV_HEADERS)


@pytest.mark.parametrize("ver,ok", [
    ("v1.6.2", True), ("1.8.3-dev.3", True), ("v2.0.0", True),
    ("v1.6.1", False), ("v0.0.0", False), (None, False), ("garbage", False),
])
def test_firmware_gate(ver, ok):
    assert sensor_fw_supports_imu_stream(ver) is ok


# ── logger lifecycle (fake sensor) ────────────────────────────────────────

class FakeStream:
    def __init__(self, log):
        self.log = log
        self.q = None

    def flush_stale_data(self, expected_size, max_total_ms=1500, **_):
        self.log.append("flush")
        return 0

    def start_streaming(self, q, size):
        self.log.append("start_streaming")
        self.q = q

    def stop_streaming(self):
        self.log.append("stop_streaming")

    def push(self, data):
        self.q.put(data)


class FakeSensor:
    def __init__(self, version="v1.8.3", on_ok=True, on_raises=False):
        self._version = version
        self.log = []
        self.uart = mock.Mock()
        self.uart.imu = FakeStream(self.log)
        self._on_ok = on_ok
        self._on_raises = on_raises

    def imu_init(self):
        self.log.append("imu_init")
        return True

    def imu_on(self):
        self.log.append("imu_on")
        if self._on_raises:
            raise RuntimeError("comm timeout")
        return self._on_ok

    def imu_off(self):
        self.log.append("imu_off")
        return True


def _read(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def _wait_rows(path, n, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if len(_read(path)) >= n:
            return
        time.sleep(0.02)


def test_logger_buffers_until_anchor_then_writes_negative_times(tmp_path):
    path = tmp_path / "s_imu.csv"
    sensor = FakeSensor()
    anchor = {}
    log = ImuCsvLogger([("left", sensor)], str(path),
                       anchor_fn=lambda side: anchor.get(side))
    log.start()
    assert sensor.log == ["imu_init", "flush", "start_streaming", "imu_on"]

    sensor.uart.imu.push(_line(frame=1, ms=9_975))   # before first histogram
    time.sleep(0.15)
    assert _read(path) == []                          # held: no anchor yet
    anchor["left"] = (10.0, time.monotonic())         # first histogram at 10.000 s
    sensor.uart.imu.push(_line(frame=2, ms=10_000)[:20])
    sensor.uart.imu.push(_line(frame=2, ms=10_000)[20:])
    _wait_rows(path, 2)
    log.stop()

    rows = _read(path)
    assert [r["imu_frame"] for r in rows] == ["1", "2"]
    assert float(rows[0]["timestamp_s"]) == pytest.approx(-0.025)
    assert float(rows[1]["timestamp_s"]) == pytest.approx(0.0)
    assert {r["time_source"] for r in rows} == {"device"}
    assert rows[0]["device_ms"] == "9975"
    assert float(rows[0]["host_time_unix"]) > 1e9
    # imu_off before the reader stops (endpoint must go quiet first).
    assert sensor.log[-2:] == ["imu_off", "stop_streaming"]


def test_logger_host_timestamps_without_ms(tmp_path):
    path = tmp_path / "s_imu.csv"
    sensor = FakeSensor()
    t0_host = time.monotonic()
    log = ImuCsvLogger([("right", sensor)], str(path),
                       anchor_fn=lambda side: (123.0, t0_host))
    log.start()
    sensor.uart.imu.push(_line(frame=5))
    _wait_rows(path, 1)
    log.stop()
    (row,) = _read(path)
    assert row["side"] == "right" and row["time_source"] == "host"
    assert row["device_ms"] == ""
    assert 0.0 <= float(row["timestamp_s"]) < 1.0


def test_logger_falls_back_to_scan_start_when_no_histogram_anchor(tmp_path):
    path = tmp_path / "s_imu.csv"
    sensor = FakeSensor()
    log = ImuCsvLogger([("left", sensor)], str(path), anchor_fn=lambda s: None,
                       fallback_t0_monotonic=time.monotonic())
    log.start()
    sensor.uart.imu.push(_line(frame=1, ms=4242))
    time.sleep(0.15)
    log.stop()
    (row,) = _read(path)
    assert row["time_source"] == "host" and row["device_ms"] == "4242"
    assert float(row["timestamp_s"]) >= 0.0


def test_logger_skips_old_firmware(tmp_path):
    sensor = FakeSensor(version="v1.6.1")
    log = ImuCsvLogger([("left", sensor)], str(tmp_path / "x.csv"))
    log.start()
    log.stop()
    assert sensor.log == []
    assert _read(tmp_path / "x.csv") == []           # header only


def test_logger_imu_on_failure_never_raises_and_cleans_up(tmp_path):
    sensor = FakeSensor(on_raises=True)
    log = ImuCsvLogger([("left", sensor)], str(tmp_path / "x.csv"))
    log.start()                                       # must not raise
    assert sensor.log[-2:] == ["imu_off", "stop_streaming"]
    log.stop()                                        # idempotent cleanup
    assert sensor.log.count("imu_off") == 1


def test_logger_stop_is_idempotent_and_tolerates_bad_lines(tmp_path):
    path = tmp_path / "x.csv"
    sensor = FakeSensor()
    log = ImuCsvLogger([("left", sensor)], str(path),
                       anchor_fn=lambda s: (0.0, time.monotonic()))
    log.start()
    sensor.uart.imu.push(b"garbage\r\n" + _line(frame=1, ms=1))
    _wait_rows(path, 1)
    log.stop()
    log.stop()
    assert len(_read(path)) == 1
    assert sensor.log.count("stop_streaming") == 1


# ── LiveUsbSource anchor ──────────────────────────────────────────────────

def test_source_side_t0_reports_device_and_host_anchor():
    from omotion.pipeline.sources import _BaseSource
    src = _BaseSource(metadata=None)
    assert src.side_t0("left") is None
    before = time.monotonic()
    assert src._t0_normalize("left", 42.5) == 0.0
    t0_dev, t0_host = src.side_t0("left")
    assert t0_dev == 42.5 and t0_host >= before
    assert src.side_t0("right") is None
    assert src._t0_normalize("left", 43.0) == pytest.approx(0.5)
    assert src.side_t0("left")[1] == t0_host          # latched once


# ── ScanWorkflow wiring ───────────────────────────────────────────────────

def test_scan_request_imu_csv_defaults_off():
    from omotion.ScanWorkflow import ScanRequest
    req = ScanRequest(subject_id="s", duration_sec=1,
                      left_camera_mask=0xFF, right_camera_mask=0)
    assert req.write_imu_csv is False


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


def test_scan_workflow_starts_imu_before_trigger_and_stops_on_cancel(tmp_path):
    from omotion.MotionInterface import MotionInterface
    from omotion.ScanWorkflow import ScanRequest

    motion = MotionInterface(demo_mode=True, data_dir=str(tmp_path),
                             operator_id="test")
    events = []

    class _RecLogger:
        def __init__(self, sensors, path, anchor_fn=None, fallback_t0_monotonic=None,
                     motion_fn=None):
            events.append(("init", [s for s, _ in sensors], path, anchor_fn))

        def set_motion_fn(self, fn):
            pass

        def start(self):
            events.append(("imu_start",))

        def stop(self):
            events.append(("imu_stop",))

    fake_side = ("left", 0xFF, mock.MagicMock())
    request = ScanRequest(subject_id="subj", duration_sec=30,
                          left_camera_mask=0xFF, right_camera_mask=0,
                          write_imu_csv=True)
    wf = motion.scan_workflow
    with mock.patch("omotion.pipeline.sources.LiveUsbSource", _Src), \
         mock.patch("omotion.imu_csv.ImuStreamLogger", _RecLogger), \
         mock.patch.object(motion.left, "is_connected", return_value=True), \
         mock.patch.object(motion.right, "is_connected", return_value=False), \
         mock.patch.object(wf, "_resolve_active_sides", return_value=[fake_side]), \
         mock.patch.object(wf, "_scan_subscribe_state", lambda handles: None), \
         mock.patch.object(motion, "run_on_sensors", return_value=True), \
         mock.patch.object(motion.console, "set_trigger_json",
                           side_effect=lambda **kw: kw.get("data")), \
         mock.patch.object(motion.console, "start_trigger",
                           side_effect=lambda *a, **k: events.append(("trigger",)) or True):
        assert wf.start_scan(request)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and ("trigger",) not in events:
            time.sleep(0.01)
        wf.cancel_scan(join_timeout=5.0)

    names = [e[0] for e in events]
    assert names[0] == "init" and events[0][1] == ["left"]
    assert events[0][2].endswith("_subj_imu.csv")
    assert events[0][3] is not None                   # anchored to the source
    assert names.index("imu_start") < names.index("trigger")
    assert names[-1] == "imu_stop"
    assert wf.last_imu_csv_path == events[0][2]
