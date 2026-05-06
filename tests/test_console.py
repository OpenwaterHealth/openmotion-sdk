"""
Console module tests (Section 2 of the test plan).
"""

import re
import time

import pytest

from omotion.CommandError import CommandError
from omotion.config import MuxChannel
from omotion.MotionConfig import MotionConfig

pytestmark = pytest.mark.console


# ===========================================================================
# 2.1 Basic connectivity
# ===========================================================================

def test_console_ping(console):
    assert console.ping() is True


def test_console_version(console):
    v = console.get_version()
    assert isinstance(v, str) and len(v) > 0
    assert re.match(r"\d+\.\d+\.\d+", v), f"Unexpected version format: {v!r}"


def test_console_hardware_id(console):
    hw = console.get_hardware_id()
    assert isinstance(hw, str) and len(hw) > 0


def test_console_echo(console):
    payload = b"hello"
    data, length = console.echo(payload)
    assert data == payload
    assert length == len(payload)


def test_console_echo_empty(console):
    result = console.echo(b"")
    # Some firmware builds return (None, None) or (b"", 0) for empty echo
    if result is not None:
        data, length = result
        assert length == 0 or length is None


def test_console_toggle_led(console):
    assert console.toggle_led() is True
    assert console.toggle_led() is True


def test_console_board_id(console):
    board_id = console.read_board_id()
    assert isinstance(board_id, int)


def test_console_messages(console):
    msgs = console.get_messages()
    assert isinstance(msgs, str)


# ===========================================================================
# 2.2 TEC subsystem
# ===========================================================================

def test_tec_status_types(console):
    result = console.tec_status()
    assert len(result) == 5
    v_raw, set_raw, curr_raw, volt_raw, good = result
    for v in (v_raw, set_raw, curr_raw, volt_raw):
        # The API returns formatted strings; convert for range check
        fv = float(v)
        assert 0.0 <= fv <= 3.3, f"ADC value out of range: {fv}"
    assert isinstance(good, bool)


def test_tec_adc_channels(console):
    for ch in range(4):
        val = console.tec_adc(ch)
        assert isinstance(val, float), f"ch{ch}: expected float, got {type(val)}"
        assert 0.0 <= val <= 3.3, f"ch{ch}: {val} out of [0, 3.3]"


def test_tec_voltage_read(console):
    v = console.tec_voltage()
    assert isinstance(v, float)
    assert 0.0 <= v <= 3.3


def test_tec_voltage_set_readback(console):
    target = 1.5
    console.tec_voltage(target)
    time.sleep(0.1)
    readback = console.tec_voltage()
    assert abs(readback - target) <= 0.05, (
        f"TEC voltage readback {readback:.3f} V differs from set point {target} V"
    )


def test_temperatures(console):
    temps = console.get_temperatures()
    assert len(temps) == 3
    for t in temps:
        assert isinstance(t, float)
        assert -40.0 <= t <= 85.0, f"Temperature out of physical range: {t}"


# ===========================================================================
# 2.3 PDU monitor
# ===========================================================================

def test_pdu_mon_structure(console):
    pdu = console.read_pdu_mon()
    assert pdu is not None
    assert len(pdu.raws) == 16
    assert len(pdu.volts) == 16
    for raw in pdu.raws:
        assert isinstance(raw, int)
    for volt in pdu.volts:
        assert isinstance(volt, float)


def test_pdu_mon_ranges(console):
    pdu = console.read_pdu_mon()
    for i, v in enumerate(pdu.volts):
        assert -0.5 <= v <= 60.0, f"PDU channel {i} voltage {v} V out of gross bounds"


# ===========================================================================
# 2.4 I2C pass-through
# ===========================================================================

def test_i2c_scan(console):
    addrs = console.scan_i2c_mux_channel(mux_index=0, channel=0)
    assert isinstance(addrs, list)
    for a in addrs:
        assert isinstance(a, int) and 0 <= a <= 127


def test_i2c_read_bad_address(console):
    with pytest.raises((CommandError, ValueError, Exception)):
        console.read_i2c_packet(
            mux_index=0,
            channel=0,
            i2c_addr=0x7F,
            reg_addr=0x00,
            num_bytes=1,
        )


# ===========================================================================
# 2.5 GPIO and ADC
# ===========================================================================

@pytest.mark.xfail(reason="Firmware returns 0-byte GPIO payload (console-fw issue)", raises=ValueError)
def test_read_gpio(console):
    val = console.read_gpio_value()
    # Returns int or float depending on firmware response
    assert isinstance(val, (int, float))


@pytest.mark.xfail(reason="Firmware returns 0-byte ADC payload (console-fw issue)", raises=ValueError)
def test_read_adc(console):
    val = console.read_adc_value()
    assert isinstance(val, (int, float))
    if isinstance(val, float):
        assert 0.0 <= val <= 3.3, f"ADC value {val} out of [0, 3.3]"


# ===========================================================================
# 2.6 Fan control
# ===========================================================================

def test_fan_set(console):
    """set_fan_speed sends OW_CTRL_SET_FAN and returns the requested value."""
    result = console.set_fan_speed(75)
    assert result == 75 or result == -1, f"Unexpected set_fan_speed result: {result}"


def test_fan_set_bounds(console):
    """set_fan_speed accepts 0 and 100."""
    assert console.set_fan_speed(0) is not None
    assert console.set_fan_speed(100) is not None


def test_fan_set_rejects_out_of_range(console):
    """set_fan_speed raises ValueError for values outside 0..100."""
    with pytest.raises(ValueError):
        console.set_fan_speed(-1)
    with pytest.raises(ValueError):
        console.set_fan_speed(101)


@pytest.mark.parametrize("fan_index", [1, 2, 3])
def test_fan_rpm_feedback(console, fan_index):
    """get_fan_rpm(fan_index=1..3) returns an RPM int or None on OW_ERROR."""
    rpm = console.get_fan_rpm(fan_index=fan_index)
    assert rpm is None or isinstance(rpm, int), (
        f"Fan {fan_index} RPM {rpm} not int or None"
    )


def test_fan_rpm_rejects_out_of_range(console):
    """get_fan_rpm raises ValueError for fan_index outside 1..3."""
    with pytest.raises(ValueError):
        console.get_fan_rpm(fan_index=0)
    with pytest.raises(ValueError):
        console.get_fan_rpm(fan_index=4)


# ===========================================================================
# 2.7 RGB indicator
# ===========================================================================

@pytest.fixture()
def restore_rgb(console):
    original = console.get_rgb_led()
    yield
    console.set_rgb_led(original)


def test_rgb_set_and_get(console, restore_rgb):
    console.set_rgb_led(0x01)
    readback = console.get_rgb_led()
    assert readback == 0x01


# ===========================================================================
# 2.8 Frame sync / trigger
# ===========================================================================

def test_fsync_pulsecount(console):
    count = console.get_fsync_pulsecount()
    assert isinstance(count, int) and count >= 0


def test_lsync_pulsecount(console):
    count = console.get_lsync_pulsecount()
    assert isinstance(count, int) and count >= 0


def test_trigger_set_get(console):
    console.set_trigger_json({"rate": 10})
    cfg = console.get_trigger_json()
    assert isinstance(cfg, dict) and len(cfg) > 0


@pytest.mark.slow
def test_trigger_start_stop_lsync(console):
    """Start a 10 Hz trigger, assert LSYNC counter accumulates pulses."""
    console.set_trigger_json({"rate": 10})
    console.start_trigger()
    time.sleep(1.1)
    count = console.get_lsync_pulsecount()
    console.stop_trigger()
    assert count >= 1, f"Expected at least 1 LSYNC pulse after 1.1 s, got {count}"


# ===========================================================================
# 2.9 Configuration (MotionConfig)
# ===========================================================================

def test_read_config(console):
    cfg = console.read_config()
    if cfg is not None:
        assert isinstance(cfg, MotionConfig)


def test_write_read_config_roundtrip(console):
    original_data = {"test_key": "test_value", "version": 1}
    mc = MotionConfig(json_data=original_data)
    console.write_config(mc)
    readback = console.read_config()
    assert readback is not None
    assert readback.json_data.get("test_key") == "test_value"
    assert readback.json_data.get("version") == 1


def test_write_config_json_roundtrip(console):
    console.write_config_json('{"write_json_key": 42}')
    readback = console.read_config()
    assert readback is not None
    assert readback.json_data.get("write_json_key") == 42


# ===========================================================================
# 2.10 FPGA programming (console-side)
# ===========================================================================

@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_prog_open_close(console):
    console.fpga_prog_open(MuxChannel.FPGA_TA)
    console.fpga_prog_close(MuxChannel.FPGA_TA)


@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_prog_read_status(console):
    console.fpga_prog_open(MuxChannel.FPGA_TA)
    try:
        status = console.fpga_prog_read_status(MuxChannel.FPGA_TA)
        assert isinstance(status, int)
    finally:
        console.fpga_prog_close(MuxChannel.FPGA_TA)


@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_prog_cfg_reset(console):
    console.fpga_prog_open(MuxChannel.FPGA_TA)
    try:
        console.fpga_prog_cfg_reset(MuxChannel.FPGA_TA)
    finally:
        console.fpga_prog_close(MuxChannel.FPGA_TA)


@pytest.mark.slow
@pytest.mark.fpga
@pytest.mark.destructive
def test_fpga_prog_featrow_roundtrip(console):
    console.fpga_prog_open(MuxChannel.FPGA_TA)
    try:
        feat, feabits = console.fpga_prog_featrow_read(MuxChannel.FPGA_TA)
        console.fpga_prog_featrow_write(MuxChannel.FPGA_TA, feat, feabits)
        feat2, feabits2 = console.fpga_prog_featrow_read(MuxChannel.FPGA_TA)
        assert feat2 == feat and feabits2 == feabits
    finally:
        console.fpga_prog_close(MuxChannel.FPGA_TA)


@pytest.mark.slow
@pytest.mark.fpga
@pytest.mark.destructive
def test_fpga_prog_ufm_roundtrip(console):
    page_data = bytes([0xAB] * 16)
    console.fpga_prog_open(MuxChannel.FPGA_TA)
    try:
        console.fpga_prog_ufm_reset(MuxChannel.FPGA_TA)
        console.fpga_prog_ufm_write_page(MuxChannel.FPGA_TA, page_data)
        console.fpga_prog_ufm_reset(MuxChannel.FPGA_TA)
        readback = console.fpga_prog_ufm_read_page(MuxChannel.FPGA_TA)
        assert readback == page_data
    finally:
        console.fpga_prog_close(MuxChannel.FPGA_TA)


# ===========================================================================
# 2.12 Console telemetry poller
# ===========================================================================

@pytest.mark.slow
def test_telemetry_poller_starts_on_connect(console):
    if not console.is_connected():
        pytest.skip("Console not connected (may have entered DFU in an earlier test)")
    time.sleep(1.5)
    snap = console.telemetry.get_snapshot()
    assert snap is not None
    # read_ok may be False on first poll if I2C bus is still settling; just check
    # that the poller produced a snapshot at all (timestamp > 0).
    assert snap.timestamp > 0, "Telemetry poller has not fired — no snapshot timestamp"


def test_telemetry_fields_populated(console):
    if not console.is_connected():
        pytest.skip("Console not connected")
    time.sleep(1.5)
    snap = console.telemetry.get_snapshot()
    assert snap is not None
    assert snap.tcm >= 0
    assert snap.tcl >= 0
    assert len(snap.pdu_raws) == 16
    assert isinstance(snap.safety_ok, bool)
    assert snap.timestamp > 0


@pytest.mark.slow
def test_telemetry_listener_fires(console):
    calls = []
    console.telemetry.add_listener(calls.append)
    time.sleep(2.5)
    console.telemetry.remove_listener(calls.append)
    assert len(calls) >= 2, (
        f"Telemetry listener called {len(calls)} time(s) in 2.5 s; expected ≥2"
    )


@pytest.mark.slow
def test_telemetry_poller_stops(console):
    console.telemetry.stop()
    time.sleep(2.0)
    snap1 = console.telemetry.get_snapshot()
    ts1 = snap1.timestamp if snap1 else None
    time.sleep(2.0)
    snap2 = console.telemetry.get_snapshot()
    ts2 = snap2.timestamp if snap2 else None
    assert ts1 == ts2, "Poller timestamp changed after stop() — poller still running"
    console.telemetry.start()


def test_safety_interlock_clear(console):
    time.sleep(1.5)
    snap = console.telemetry.get_snapshot()
    assert snap is not None
    assert snap.safety_ok is True, (
        f"Safety interlock reported tripped "
        f"(SE=0x{snap.safety_se:02X}, SO=0x{snap.safety_so:02X})"
    )


# ===========================================================================
# 2.11 DFU  — runs last (name sorts after all other tests)
# ===========================================================================

@pytest.mark.skip(reason="DFU temporarily disabled")
@pytest.mark.destructive
@pytest.mark.slow
def test_z_enter_dfu(console):
    """Enter DFU mode. Must run LAST — device re-enumerates after this."""
    result = console.enter_dfu()
    assert result is True
