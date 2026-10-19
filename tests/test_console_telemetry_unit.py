import logging
from types import SimpleNamespace

from omotion.ConsoleTelemetry import ConsoleTelemetry, ConsoleTelemetryPoller


class _FakeConsole:
    def __init__(self, se_raw: int, so_raw: int) -> None:
        self._se_raw = se_raw
        self._so_raw = so_raw

    def read_i2c_packet(self, mux_index, channel, device_addr, reg_addr, read_len):
        assert mux_index == 1
        assert device_addr == 0x41
        assert reg_addr == 0x24
        assert read_len == 1
        if channel == 6:
            return bytes([self._se_raw]), 1
        if channel == 7:
            return bytes([self._so_raw]), 1
        raise AssertionError(f"Unexpected channel {channel}")


def test_read_safety_logs_named_se_so_faults(caplog):
    poller = ConsoleTelemetryPoller(_FakeConsole(se_raw=0x01, so_raw=0x06))
    snap = ConsoleTelemetry()

    with caplog.at_level(logging.ERROR):
        poller._read_safety(snap)

    assert snap.safety_ok is False
    assert "Safety interlock SE faults" in caplog.text
    assert "POWER_PEAK_CURRENT_LIMIT_FAIL" in caplog.text
    assert "Safety interlock SO faults" in caplog.text
    assert "PULSE_UPPER_LIMIT_FAIL_OR_PULSE_LOWER_LIMIT_FAIL" in caplog.text
    assert "RATE_LOWER_LIMIT_FAIL" in caplog.text
    # Decoded labels are also surfaced on the snapshot for consumers (issue #56):
    # SE=0x01 (peak) + SO=0x06 (pulse|rate), combined in first-seen order.
    assert snap.safety_faults == [
        "POWER_PEAK_CURRENT_LIMIT_FAIL",
        "PULSE_UPPER_LIMIT_FAIL_OR_PULSE_LOWER_LIMIT_FAIL",
        "RATE_LOWER_LIMIT_FAIL",
    ]


def test_read_safety_clear_has_empty_faults():
    poller = ConsoleTelemetryPoller(_FakeConsole(se_raw=0x00, so_raw=0x00))
    snap = ConsoleTelemetry()
    poller._read_safety(snap)
    assert snap.safety_ok is True
    assert snap.safety_known is True
    assert snap.safety_faults == []


def test_read_safety_dedups_faults_across_channels():
    # Both channels report the same peak-current fault -> one label, not two.
    poller = ConsoleTelemetryPoller(_FakeConsole(se_raw=0x01, so_raw=0x01))
    snap = ConsoleTelemetry()
    poller._read_safety(snap)
    assert snap.safety_faults == ["POWER_PEAK_CURRENT_LIMIT_FAIL"]


def test_read_safety_unknown_leaves_faults_empty():
    # Chip not responding (empty read) -> safety_known False, no faults asserted.
    class _NoData:
        def read_i2c_packet(self, *a, **k):
            return None, None

    poller = ConsoleTelemetryPoller(_NoData())
    snap = ConsoleTelemetry()
    poller._read_safety(snap)
    assert snap.safety_known is False
    assert snap.safety_faults == []


class _FakeConsoleForTec:
    """Console stub whose tec_status() either answers or raises."""

    def __init__(self, result=None, exc: Exception | None = None) -> None:
        self._result = result
        self._exc = exc
        self.calls = 0

    def is_connected(self):
        return True

    def tec_status(self):
        self.calls += 1
        if self._exc is not None:
            raise self._exc
        return self._result

    def read_pdu_mon(self):
        return SimpleNamespace(raws=[], volts=[])

    def read_i2c_packet(self, mux_index, channel, device_addr, reg_addr, read_len):
        return b"\x00" * read_len, read_len

    def get_lsync_pulsecount(self):
        return 0


def test_tec_known_defaults_false_on_fresh_snapshot():
    # tec_good defaults False, which is indistinguishable from "tripped" --
    # tec_known is what tells a consumer the value was never measured (#206).
    snap = ConsoleTelemetry()
    assert snap.tec_known is False
    assert snap.tec_good is False


def test_read_tec_sets_known_on_successful_read():
    console = _FakeConsoleForTec(result=("1.0", "0.5", "0.5", "25.0", True))
    poller = ConsoleTelemetryPoller(console)
    snap = ConsoleTelemetry()

    poller._read_tec(snap)

    assert snap.tec_known is True
    assert snap.tec_good is True
    assert snap.tec_v_raw == 1.0
    assert snap.tec_set_raw == 0.5


def test_read_tec_reports_trip_as_known_false_good():
    # A real trip: firmware answered and cleared the bit.
    console = _FakeConsoleForTec(result=("1.0", "0.5", "0.5", "25.0", False))
    poller = ConsoleTelemetryPoller(console)
    snap = ConsoleTelemetry()

    poller._read_tec(snap)

    assert snap.tec_known is True
    assert snap.tec_good is False


def test_failed_tec_read_leaves_known_false():
    # tec_status() raises -> _read_all catches, read_ok goes False, and the
    # snapshot must NOT look like a trip the console never reported.
    console = _FakeConsoleForTec(exc=RuntimeError("UART timeout"))
    poller = ConsoleTelemetryPoller(console)

    snap = poller._read_all()

    assert snap.read_ok is False
    assert "UART timeout" in (snap.error or "")
    assert snap.tec_known is False


from omotion.ConsoleTelemetry import PdcSample, PDC_MA_PER_LSB


def test_pdc_sample_scales_raw_to_mA():
    s = PdcSample.from_raw(frame_idx=42, pdc_raw=100, flags=0x01, host_recv_timestamp=1.23)
    assert s.frame_idx == 42
    assert s.pdc_mA == 100 * PDC_MA_PER_LSB
    assert s.dark_slot is True
    assert s.host_recv_timestamp == 1.23
    assert s.dropped_delta == 0


def test_pdc_sample_dark_slot_false_when_flags_clear():
    s = PdcSample.from_raw(frame_idx=43, pdc_raw=200, flags=0x00, host_recv_timestamp=0.0)
    assert s.dark_slot is False


from unittest.mock import MagicMock
import time as _time

class _FakeConsoleForDrain:
    """Mock console that returns scripted drain responses on each call."""
    def __init__(self, drain_responses):
        self._drain_responses = list(drain_responses)
        self.drain_calls = 0
        self.tec_calls = 0
        self.pdu_calls = 0
        self.safety_calls = 0
        self.analog_calls = 0

    def is_connected(self):
        return True

    def get_pdc_buffer(self, max_samples=64):
        self.drain_calls += 1
        if self._drain_responses:
            return self._drain_responses.pop(0)
        return 0, []

    # Stubs used by the slow tick — set as MagicMocks externally if needed
    def tec_status(self):
        self.tec_calls += 1
        return 0.0, 0.0, 0.0, 0.0, False

    def read_pdu_mon(self):
        self.pdu_calls += 1
        m = MagicMock(); m.raws = []; m.volts = []
        return m

    def read_i2c_packet(self, mux_index, channel, device_addr, reg_addr, read_len):
        self.safety_calls += 1
        return b"\x00" * read_len, read_len

    def get_lsync_pulsecount(self):
        self.analog_calls += 1
        return 0


def test_poller_drains_pdc_each_tick_and_fires_listeners():
    drain_responses = [
        (0, [(1, 100, 0x00), (2, 200, 0x01)]),
        (0, [(3, 150, 0x00)]),
    ]
    console = _FakeConsoleForDrain(drain_responses)
    poller = ConsoleTelemetryPoller(console)
    received = []
    poller.add_pdc_listener(received.append)

    # Run two ticks synchronously by calling the inner method that processes
    # one drain pass + (optionally) a slow refresh.
    poller._tick_once()
    poller._tick_once()

    assert console.drain_calls == 2
    assert len(received) == 3
    assert received[0].frame_idx == 1 and received[0].dark_slot is False
    assert received[1].dark_slot is True
    assert received[2].frame_idx == 3
    assert poller.get_last_pdc_sample().frame_idx == 3


def test_poller_runs_slow_refresh_every_10th_tick():
    console = _FakeConsoleForDrain([(0, [])] * 25)
    poller = ConsoleTelemetryPoller(console)
    for _ in range(20):
        poller._tick_once()
    # Slow refresh fires on tick 0 and tick 10 (and not in between).
    assert console.tec_calls == 2
    assert console.pdu_calls == 2


def test_poller_attaches_dropped_delta_to_first_sample_only():
    drain_responses = [(7, [(1, 100, 0), (2, 110, 0)])]
    console = _FakeConsoleForDrain(drain_responses)
    poller = ConsoleTelemetryPoller(console)
    received = []
    poller.add_pdc_listener(received.append)
    poller._tick_once()
    assert received[0].dropped_delta == 7
    assert received[1].dropped_delta == 0


def test_read_analog_tolerates_lsync_failure():
    """A transient lsync failure must not propagate out of _read_analog —
    one bad read shouldn't fail the whole telemetry snapshot (tcm just
    defaults to 0 for that poll)."""
    console = _FakeConsoleForDrain([])
    console.get_lsync_pulsecount = MagicMock(side_effect=RuntimeError("uart busy"))
    poller = ConsoleTelemetryPoller(console)
    snap = ConsoleTelemetry()

    poller._read_analog(snap)

    assert snap.tcm == 0
