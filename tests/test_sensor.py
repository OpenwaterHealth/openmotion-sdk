"""
Sensor module tests (Section 3 of the test plan).

Tests are parametrised over the 'any_sensor' fixture so they run
against both left and right sensors automatically.  Side-specific
tests use the 'sensor_left' / 'sensor_right' fixtures directly.
"""

import math
import struct
import time

import numpy as np
import pytest

pytestmark = pytest.mark.sensor



# ===========================================================================
# 3.1 Basic connectivity
# ===========================================================================

def test_sensor_ping(any_sensor):
    assert any_sensor.ping() is True


def test_sensor_version(any_sensor):
    import re
    v = any_sensor.get_version()
    assert isinstance(v, str) and len(v) > 0
    assert re.match(r"\d+\.\d+\.\d+", v), f"Unexpected version format: {v!r}"


def test_sensor_hardware_id(any_sensor):
    hw = any_sensor.get_hardware_id()
    assert isinstance(hw, str) and len(hw) > 0


def test_sensor_echo(any_sensor):
    payload = b"test"
    data, length = any_sensor.echo(payload)
    assert data == payload
    assert length == len(payload)


def test_sensor_toggle_led(any_sensor):
    assert any_sensor.toggle_led() is True
    assert any_sensor.toggle_led() is True


# ===========================================================================
# 3.2 IMU
# ===========================================================================

@pytest.fixture(scope="function", autouse=False)
def imu_enabled(any_sensor):
    """Power the IMU on for the duration of one test, then turn it off.

    Function-scoped so the IMU is explicitly disabled after each test that
    needs it, preventing the enabled state from leaking into unrelated tests.
    """
    any_sensor.imu_init()
    any_sensor.imu_on()
    yield
    try:
        any_sensor.imu_off()
    except Exception:
        pass


@pytest.mark.imu
def test_imu_temperature(any_sensor):
    t = any_sensor.imu_get_temperature()
    assert isinstance(t, float)
    assert -40.0 <= t <= 85.0, f"IMU temperature {t} °C out of physical range"


@pytest.mark.imu
def test_imu_accelerometer(any_sensor, imu_enabled):
    accel = any_sensor.imu_get_accelerometer()
    assert isinstance(accel, list) and len(accel) == 3
    for v in accel:
        assert isinstance(v, int)
    magnitude = math.sqrt(sum(v ** 2 for v in accel))
    assert magnitude > 0, "Accelerometer magnitude is zero — sensor may be unresponsive"


@pytest.mark.imu
def test_imu_gyroscope(any_sensor, imu_enabled):
    gyro = any_sensor.imu_get_gyroscope()
    assert isinstance(gyro, list) and len(gyro) == 3
    for v in gyro:
        assert isinstance(v, int)
        # Raw LSB values depend on firmware full-scale range (~16000 for this hardware)
        assert -32768 <= v <= 32767, f"Gyro axis {v} out of signed 16-bit range"


# ===========================================================================
# 3.3 Fan control
# ===========================================================================

def test_sensor_fan_on(any_sensor):
    assert any_sensor.set_fan_control(True) is True


def test_sensor_fan_off(any_sensor):
    assert any_sensor.set_fan_control(False) is True


def test_sensor_fan_status_roundtrip(any_sensor):
    any_sensor.set_fan_control(True)
    assert any_sensor.get_fan_control_status() is True
    any_sensor.set_fan_control(False)
    assert any_sensor.get_fan_control_status() is False


# ===========================================================================
# 3.4 Debug flags
# ===========================================================================

from omotion.config import (
    DEBUG_FLAG_USB_PRINTF,
    DEBUG_FLAG_HISTO_THROTTLE,
    DEBUG_FLAG_FAKE_DATA,
    DEBUG_FLAG_COMM_VERBOSE,
    DEBUG_FLAG_CMD_VERBOSE,
)

_ALL_DEBUG_FLAGS = (
    DEBUG_FLAG_USB_PRINTF
    | DEBUG_FLAG_HISTO_THROTTLE
    | DEBUG_FLAG_FAKE_DATA
    | DEBUG_FLAG_COMM_VERBOSE
    | DEBUG_FLAG_CMD_VERBOSE
)


def test_debug_flags_roundtrip(any_sensor):
    """Baseline: write 0x03 and read it back (pre-existing sanity check)."""
    original = any_sensor.get_debug_flags()
    try:
        any_sensor.set_debug_flags(0x03)
        readback = any_sensor.get_debug_flags()
        assert readback == 0x03, f"Debug flags readback {readback:#04x}, expected 0x03"
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flag_usb_printf(any_sensor):
    """Bit 0 — DEBUG_FLAG_USB_PRINTF: enables firmware printf output over USB."""
    original = any_sensor.get_debug_flags()
    try:
        assert any_sensor.set_debug_flags(DEBUG_FLAG_USB_PRINTF) is True
        readback = any_sensor.get_debug_flags()
        assert readback & DEBUG_FLAG_USB_PRINTF, (
            f"USB_PRINTF bit not set; readback=0x{readback:08X}"
        )
        assert any_sensor.set_debug_flags(0x00) is True
        readback = any_sensor.get_debug_flags()
        assert not (readback & DEBUG_FLAG_USB_PRINTF), (
            f"USB_PRINTF bit still set after clear; readback=0x{readback:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flag_histo_throttle(any_sensor):
    """Bit 1 — DEBUG_FLAG_HISTO_THROTTLE: firmware sends real histogram data every 5 s only.

    While throttled, the sensor must still acknowledge commands promptly — it
    claims success for intermediate requests rather than blocking.
    """
    original = any_sensor.get_debug_flags()
    try:
        assert any_sensor.set_debug_flags(DEBUG_FLAG_HISTO_THROTTLE) is True
        readback = any_sensor.get_debug_flags()
        assert readback & DEBUG_FLAG_HISTO_THROTTLE, (
            f"HISTO_THROTTLE bit not set; readback=0x{readback:08X}"
        )
        # Behavioral check: the command/response path must remain live while
        # throttled.  ping() is a fast NOP-style command that should not hang.
        assert any_sensor.ping() is True, (
            "Sensor stopped responding to ping with HISTO_THROTTLE active"
        )
        assert any_sensor.set_debug_flags(0x00) is True
        readback = any_sensor.get_debug_flags()
        assert not (readback & DEBUG_FLAG_HISTO_THROTTLE), (
            f"HISTO_THROTTLE bit still set after clear; readback=0x{readback:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flag_fake_data(any_sensor):
    """Bit 2 — DEBUG_FLAG_FAKE_DATA: firmware turns off cameras and streams synthetic data."""
    original = any_sensor.get_debug_flags()
    try:
        assert any_sensor.set_debug_flags(DEBUG_FLAG_FAKE_DATA) is True
        readback = any_sensor.get_debug_flags()
        assert readback & DEBUG_FLAG_FAKE_DATA, (
            f"FAKE_DATA bit not set; readback=0x{readback:08X}"
        )
        # Behavioral check: regular commands must still succeed while fake data
        # is active (firmware is not wedged).
        assert any_sensor.ping() is True, (
            "Sensor stopped responding to ping with FAKE_DATA active"
        )
        assert any_sensor.set_debug_flags(0x00) is True
        readback = any_sensor.get_debug_flags()
        assert not (readback & DEBUG_FLAG_FAKE_DATA), (
            f"FAKE_DATA bit still set after clear; readback=0x{readback:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flag_comm_verbose(any_sensor):
    """Bit 4 — DEBUG_FLAG_COMM_VERBOSE: firmware logs each command ID and response."""
    original = any_sensor.get_debug_flags()
    try:
        assert any_sensor.set_debug_flags(DEBUG_FLAG_COMM_VERBOSE) is True
        readback = any_sensor.get_debug_flags()
        assert readback & DEBUG_FLAG_COMM_VERBOSE, (
            f"COMM_VERBOSE bit not set; readback=0x{readback:08X}"
        )
        assert any_sensor.set_debug_flags(0x00) is True
        readback = any_sensor.get_debug_flags()
        assert not (readback & DEBUG_FLAG_COMM_VERBOSE), (
            f"COMM_VERBOSE bit still set after clear; readback=0x{readback:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flag_cmd_verbose(any_sensor):
    """Bit 5 — DEBUG_FLAG_CMD_VERBOSE: firmware prints inside command handlers."""
    original = any_sensor.get_debug_flags()
    try:
        assert any_sensor.set_debug_flags(DEBUG_FLAG_CMD_VERBOSE) is True
        readback = any_sensor.get_debug_flags()
        assert readback & DEBUG_FLAG_CMD_VERBOSE, (
            f"CMD_VERBOSE bit not set; readback=0x{readback:08X}"
        )
        assert any_sensor.set_debug_flags(0x00) is True
        readback = any_sensor.get_debug_flags()
        assert not (readback & DEBUG_FLAG_CMD_VERBOSE), (
            f"CMD_VERBOSE bit still set after clear; readback=0x{readback:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flags_each_independent(any_sensor):
    """Each flag can be set and cleared in isolation without contaminating other bits."""
    individual_flags = [
        ("USB_PRINTF",     DEBUG_FLAG_USB_PRINTF),
        ("HISTO_THROTTLE", DEBUG_FLAG_HISTO_THROTTLE),
        ("FAKE_DATA",      DEBUG_FLAG_FAKE_DATA),
        ("COMM_VERBOSE",   DEBUG_FLAG_COMM_VERBOSE),
        ("CMD_VERBOSE",    DEBUG_FLAG_CMD_VERBOSE),
    ]
    original = any_sensor.get_debug_flags()
    try:
        for name, flag in individual_flags:
            # Start from a clean zero baseline.
            any_sensor.set_debug_flags(0x00)
            any_sensor.set_debug_flags(flag)
            readback = any_sensor.get_debug_flags()
            assert readback & flag, (
                f"{name} (0x{flag:02X}) not set in readback 0x{readback:08X}"
            )
            # Clear and confirm gone.
            any_sensor.set_debug_flags(0x00)
            readback = any_sensor.get_debug_flags()
            assert not (readback & flag), (
                f"{name} (0x{flag:02X}) still present after clear: 0x{readback:08X}"
            )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flags_combined_all(any_sensor):
    """All defined debug flags can be set simultaneously and each bit survives."""
    original = any_sensor.get_debug_flags()
    try:
        assert any_sensor.set_debug_flags(_ALL_DEBUG_FLAGS) is True
        readback = any_sensor.get_debug_flags()
        assert readback == _ALL_DEBUG_FLAGS, (
            f"Combined flags readback 0x{readback:08X}, expected 0x{_ALL_DEBUG_FLAGS:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


def test_debug_flags_cleared_by_zero(any_sensor):
    """Writing 0x00 clears every debug bit the SDK knows about."""
    original = any_sensor.get_debug_flags()
    try:
        any_sensor.set_debug_flags(_ALL_DEBUG_FLAGS)
        assert any_sensor.set_debug_flags(0x00) is True
        readback = any_sensor.get_debug_flags()
        assert readback & _ALL_DEBUG_FLAGS == 0, (
            f"Bits still set after writing 0x00: 0x{readback:08X}"
        )
    finally:
        any_sensor.set_debug_flags(original)


# ===========================================================================
# 3.5 Camera power
# ===========================================================================

def test_camera_power_on_off(any_sensor):
    assert any_sensor.enable_camera_power(0xFF) is True
    time.sleep(0.1)
    assert any_sensor.disable_camera_power(0xFF) is True


def test_camera_power_status(any_sensor):
    any_sensor.enable_camera_power(0x01)
    try:
        status = any_sensor.get_camera_power_status()
        assert isinstance(status, list) and len(status) > 0
        assert status[0], "Camera 0 should be powered on"
    finally:
        any_sensor.disable_camera_power(0x01)


def test_camera_power_selective(any_sensor):
    any_sensor.enable_camera_power(0x01)
    try:
        status = any_sensor.get_camera_power_status()
        assert status[0], "Camera 0 should be on"
        if len(status) > 1:
            assert not status[1], "Camera 1 should be off"
    finally:
        any_sensor.disable_camera_power(0x01)


# ===========================================================================
# 3.6 FPGA control
# ===========================================================================

def _power_up(sensor, mask=0x01):
    """Power on cameras and fail fast if enable_camera_power returns False."""
    ok = sensor.enable_camera_power(mask)
    if ok is False:
        pytest.fail(f"enable_camera_power(0x{mask:02X}) returned False")
    time.sleep(0.5)  # match ScanWorkflow settle time


@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_check_after_program(any_sensor):
    _bring_up_camera(any_sensor)  # power → program_fpga → configure_registers
    try:
        assert any_sensor.check_camera_fpga(0x01) is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_status(any_sensor):
    _bring_up_camera(any_sensor)  # power → program_fpga → configure_registers
    try:
        result = any_sensor.get_status_fpga(0x01)
        assert result is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_usercode(any_sensor):
    _bring_up_camera(any_sensor)  # power → program_fpga → configure_registers
    try:
        result = any_sensor.get_usercode_fpga(0x01)
        assert result is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
@pytest.mark.fpga
def test_fpga_activate(any_sensor):
    _bring_up_camera(any_sensor)  # power → program_fpga → configure_registers
    try:
        assert any_sensor.activate_camera_fpga(0x01) is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_fpga_reset(any_sensor):
    """Reset is called after full bring-up; power-cycle wipes FPGA + registers."""
    _bring_up_camera(any_sensor)
    try:
        assert any_sensor.reset_camera_sensor(0x01) is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
@pytest.mark.fpga
def test_program_fpga_success(any_sensor):
    """program_fpga on its own returns True — firmware loads bitstream into SRAM."""
    ok = any_sensor.enable_camera_power(0x01)
    if ok is False:
        pytest.fail("enable_camera_power returned False")
    time.sleep(0.5)
    try:
        result = any_sensor.program_fpga(camera_position=0x01, manual_process=False)
        assert result is True, "program_fpga returned False — bitstream load failed"
    finally:
        any_sensor.disable_camera_power(0x01)


@pytest.mark.slow
@pytest.mark.fpga
def test_bloodflow_app_camera_fpga_sequence(any_sensor):
    """Exact production sequence from ScanWorkflow / motion_connector:
    enable_camera_power → program_fpga → camera_configure_registers.
    All three must succeed; any failure here explains flakiness in the app."""
    ok = any_sensor.enable_camera_power(0x01)
    assert ok is not False, "enable_camera_power failed"
    time.sleep(0.5)
    try:
        ok = any_sensor.program_fpga(camera_position=0x01, manual_process=False)
        assert ok is True, "program_fpga failed — FPGA will be unconfigured"
        time.sleep(0.1)
        ok = any_sensor.camera_configure_registers(0x01)
        assert ok is True, "camera_configure_registers failed after FPGA load"
    finally:
        any_sensor.disable_camera_power(0x01)


@pytest.mark.slow
@pytest.mark.fpga
def test_camera_status_bits_after_bringup(any_sensor):
    """After full bring-up the status word must have READY + FPGA_DONE + REGS_DONE set."""
    _bring_up_camera(any_sensor)
    try:
        status_map = any_sensor.get_camera_status(0x01)
        assert status_map is not None, "get_camera_status returned None"
        status = status_map.get(0)
        assert status is not None, "No status entry for camera 0"
        assert bool(status & (1 << 0)), f"READY bit not set (status=0x{status:02X})"
        assert bool(status & (1 << 1)), f"FPGA_DONE bit not set (status=0x{status:02X})"
        assert bool(status & (1 << 2)), f"REGS_DONE bit not set (status=0x{status:02X})"
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
@pytest.mark.fpga
def test_sram_prog_enter_erase_exit(any_sensor):
    """Manual SRAM programming primitives (used by flash_sensors.py manual flow):
    power → reset → activate → check ID → enter SRAM prog → erase → exit."""
    ok = any_sensor.enable_camera_power(0x01)
    if ok is False:
        pytest.fail("enable_camera_power returned False")
    time.sleep(0.5)
    try:
        assert any_sensor.reset_camera_sensor(0x01) is True
        time.sleep(0.1)
        assert any_sensor.activate_camera_fpga(0x01) is True
        assert any_sensor.check_camera_fpga(0x01) is True
        assert any_sensor.enter_sram_prog_fpga(0x01) is True
        assert any_sensor.erase_sram_fpga(0x01) is True
        assert any_sensor.exit_sram_prog_fpga(0x01) is True
    finally:
        any_sensor.disable_camera_power(0x01)


# ===========================================================================
# 3.7 Camera configuration
# ===========================================================================

def _decode_raw_histogram(raw):
    """Decode the 4100-byte payload returned by camera_get_histogram.

    Layout: 4096 bytes of histogram (1024 × uint32 little-endian)
            followed by 4 bytes of float32 temperature.

    Returns:
        (histogram, temperature_c) — numpy uint32 array of length 1024
        and a float temperature in degrees Celsius.
    """
    assert len(raw) == 4100, f"Expected 4100 bytes from camera_get_histogram, got {len(raw)}"
    histogram = np.frombuffer(bytes(raw[:4096]), dtype=np.uint32)
    temperature_c = struct.unpack_from("<f", bytes(raw), 4096)[0]
    return histogram, temperature_c


def _bring_up_camera(sensor, mask=0x01, configure=True):
    """Full camera bring-up matching the production ScanWorkflow sequence:
      1. enable_camera_power  →  500 ms settle (rails + FPGA supply)
      2. program_fpga          →  loads bitstream into SRAM  (blocks up to ~16 s)
                               →  100 ms settle after completion
      3. camera_configure_registers  →  writes camera sensor registers

    NOTE: program_fpga can take up to 16 seconds; any test calling this
    helper will be inherently slow and should be marked @pytest.mark.slow.
    Power cycling wipes both the FPGA bitstream and camera register state,
    so all three steps are required before any camera-level operation.
    """
    ok = sensor.enable_camera_power(mask)
    if ok is False:
        pytest.fail(f"enable_camera_power(0x{mask:02X}) returned False")
    time.sleep(0.5)  # match ScanWorkflow settle time

    ok = sensor.program_fpga(camera_position=mask, manual_process=False)
    if ok is False:
        pytest.fail(f"program_fpga(0x{mask:02X}) returned False")
    time.sleep(0.1)  # settle after bitstream load completes

    if configure:
        ok = sensor.camera_configure_registers(mask)
        if ok is False:
            pytest.fail(f"camera_configure_registers(0x{mask:02X}) returned False")


def _tear_down_camera(sensor, cam=0, mask=0x01):
    sensor.disable_camera_power(mask)


@pytest.mark.slow
def test_camera_configure_registers(any_sensor):
    _bring_up_camera(any_sensor)
    try:
        assert any_sensor.camera_configure_registers(0x01) is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_camera_configure_test_pattern(any_sensor):
    """Normal configure first, then overlay test pattern — FPGA must be loaded."""
    _bring_up_camera(any_sensor)
    try:
        assert any_sensor.camera_configure_test_pattern(camera_position=0x01, test_pattern=1) is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_camera_status(any_sensor):
    _bring_up_camera(any_sensor)
    try:
        status = any_sensor.get_camera_status(0x01)
        assert status is not None
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_camera_security_uid(any_sensor):
    _bring_up_camera(any_sensor)
    try:
        uid = any_sensor.read_camera_security_uid(0)
        assert isinstance(uid, (bytes, bytearray)) and len(uid) > 0
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_cached_security_uid(any_sensor):
    _bring_up_camera(any_sensor)
    try:
        any_sensor.refresh_id_cache()
        uid_str = any_sensor.get_cached_camera_security_uid(0)
        assert isinstance(uid_str, str) and len(uid_str) > 0
    finally:
        any_sensor.clear_id_cache()
        _tear_down_camera(any_sensor)


def test_camera_switch(any_sensor):
    any_sensor.switch_camera(1)
    any_sensor.switch_camera(0)


# ===========================================================================
# 3.8 Frame sync
# ===========================================================================

@pytest.mark.slow
def test_fsin_enable_disable(any_sensor):
    """FSIN aggregator requires full bring-up: power → FPGA → configure."""
    _bring_up_camera(any_sensor)
    try:
        assert any_sensor.enable_aggregator_fsin() is True
        assert any_sensor.disable_aggregator_fsin() is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_fsin_external_enable_disable(any_sensor):
    """External FSIN requires full bring-up: power → FPGA → configure."""
    _bring_up_camera(any_sensor)
    try:
        assert any_sensor.enable_camera_fsin_ext() is True
        assert any_sensor.disable_camera_fsin_ext() is True
    finally:
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_camera_stream_enable_disable(any_sensor):
    """Streaming requires full bring-up: power → FPGA → configure."""
    _bring_up_camera(any_sensor)
    try:
        assert any_sensor.enable_camera(0x01) is True
        assert any_sensor.disable_camera(0x01) is True
    finally:
        _tear_down_camera(any_sensor)


# ===========================================================================
# 3.9 Single-frame histogram capture
# ===========================================================================

@pytest.mark.slow
def test_single_histogram_raw_bytes(any_sensor):
    """Full bring-up: power → program_fpga → configure → FSIN → capture → get histogram."""
    _bring_up_camera(any_sensor)
    any_sensor.enable_aggregator_fsin()
    try:
        assert any_sensor.camera_capture_histogram(0x01) is True
        raw = any_sensor.camera_get_histogram(0x01)
        assert isinstance(raw, (bytes, bytearray))
        assert len(raw) == 4100, f"Expected 4100 bytes, got {len(raw)}"
    finally:
        any_sensor.disable_aggregator_fsin()
        _tear_down_camera(any_sensor)


@pytest.mark.slow
def test_single_histogram_parsed(any_sensor):
    _bring_up_camera(any_sensor)
    any_sensor.enable_aggregator_fsin()
    try:
        any_sensor.camera_capture_histogram(0x01)
        raw = any_sensor.camera_get_histogram(0x01)
        assert raw is not None and len(raw) == 4100, (
            f"camera_get_histogram returned {len(raw) if raw else 0} bytes "
            "(expected 4100) — check FSIN / FPGA state"
        )
        histogram, temperature_c = _decode_raw_histogram(raw)
        assert len(histogram) == 1024
        assert histogram.dtype == np.uint32
        assert isinstance(temperature_c, float)
    finally:
        any_sensor.disable_aggregator_fsin()
        _tear_down_camera(any_sensor)


def test_sensor_serial_roundtrip(any_sensor):
    original = any_sensor.read_serial_number()  # may be None on a fresh board
    try:
        assert any_sensor.write_serial_number("QWW04Q10003", force=True) is True
        assert any_sensor.read_serial_number() == "QWW04Q10003"

        # Guarded write must be refused now that a serial exists.
        assert any_sensor.write_serial_number("ZZZ99Z99999", force=False) is False
        assert any_sensor.read_serial_number() == "QWW04Q10003"

        # Force overwrite succeeds.
        assert any_sensor.write_serial_number("ZZZ99Z99999", force=True) is True
        assert any_sensor.read_serial_number() == "ZZZ99Z99999"
    finally:
        if original:
            any_sensor.write_serial_number(original, force=True)


def test_sensor_serial_rejects_bad_input(any_sensor):
    before = any_sensor.read_serial_number()
    assert any_sensor.write_serial_number("bad-serial!", force=True) is False
    assert any_sensor.read_serial_number() == before


