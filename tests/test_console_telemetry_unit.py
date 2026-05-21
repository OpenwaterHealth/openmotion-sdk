import logging

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
