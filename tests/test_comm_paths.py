"""
Communication path tests (Section 4 of the test plan).

Verifies that packets travel over the correct transport, that
no commands are silently dropped, and that the dual-sensor USB
topology maps devices to the expected side.
"""

import struct
import threading
import time

import numpy as np
import pytest

from omotion.CommandError import CommandError

pytestmark = pytest.mark.sensor

# Minimal BFI calibration arrays for SciencePipeline (shape: modules × cameras)
_BFI_ZEROS = np.zeros((2, 8), dtype=np.float32)
_BFI_ONES = np.ones((2, 8), dtype=np.float32) * 10.0


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


# ===========================================================================
# 4.1 UART framing (console path)
# ===========================================================================

@pytest.mark.console
def test_uart_sync_mode_blocking(console):
    """A synchronous ping must return True and not hang."""
    assert console.ping() is True


# ===========================================================================
# 4.2 USB bulk command path (sensor)
# ===========================================================================

def test_comm_interface_response_routing(any_sensor):
    """Back-to-back pings should both return True without cross-contamination."""
    assert any_sensor.ping() is True
    assert any_sensor.ping() is True


@pytest.mark.slow
def test_comm_interface_no_missed_acks(any_sensor):
    """
    100 echo commands with unique payloads — every response must arrive
    and match its request.  This is the primary 'no missed comms' check
    for the USB command endpoint.
    """
    n = 100
    results = []

    for i in range(n):
        payload = bytes([i & 0xFF, (i >> 8) & 0xFF])
        data, length = any_sensor.echo(payload)
        results.append((payload, data, length))

    for i, (sent, received, length) in enumerate(results):
        assert length == len(sent), f"Echo {i}: length mismatch ({length} != {len(sent)})"
        assert received == sent, f"Echo {i}: payload mismatch"

    assert len(results) == n


def test_comm_interface_concurrent_pings(any_sensor):
    """
    Two threads each send 10 pings concurrently.  All must succeed,
    verifying the per-request queue routing inside MotionComposite.
    """
    errors = []

    def ping_worker():
        for _ in range(10):
            try:
                result = any_sensor.ping()
                if result is not True:
                    errors.append(f"ping returned {result!r}")
            except Exception as exc:
                errors.append(str(exc))

    threads = [threading.Thread(target=ping_worker) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=15)

    assert not errors, f"Concurrent ping failures: {errors}"


# ===========================================================================
# 4.3 Stream endpoint isolation
# ===========================================================================

@pytest.mark.slow
def test_stream_histogram_data_returned(any_sensor):
    """
    After a full camera bring-up and histogram capture, the raw bytes
    returned by camera_get_histogram must be 4100 bytes and decode to
    1024 histogram bins + float32 temperature.
    """
    ok = any_sensor.enable_camera_power(0x01)
    if ok is False:
        pytest.fail(
            "enable_camera_power returned False — "
            "TCA9548A I2C mux may be stuck (err: HAL_ERROR); power cycle the sensor"
        )
    time.sleep(0.5)
    ok = any_sensor.program_fpga(camera_position=0x01, manual_process=False)
    if ok is False:
        any_sensor.disable_camera_power(0x01)
        pytest.fail("program_fpga returned False — FPGA bitstream load failed")
    time.sleep(0.1)
    ok = any_sensor.camera_configure_registers(0x01)
    if ok is False:
        any_sensor.disable_camera_power(0x01)
        pytest.fail("camera_configure_registers returned False")

    try:
        any_sensor.camera_capture_histogram(0x01)
        raw = any_sensor.camera_get_histogram(0x01)
        assert isinstance(raw, (bytes, bytearray)) and len(raw) == 4100, (
            f"camera_get_histogram returned {len(raw) if raw else 0} bytes (expected 4100)"
        )
        histogram, temperature_c = _decode_raw_histogram(raw)
        assert len(histogram) == 1024, "Expected 1024 histogram bins"
        assert isinstance(temperature_c, float)
    finally:
        any_sensor.disable_camera_power(0x01)


@pytest.mark.slow
def test_dual_sensor_independent_pings(sensor_left, sensor_right):
    """Both sensors must respond to ping simultaneously without interference."""
    results = {}
    errors = {}

    def do_ping(side, sensor):
        try:
            results[side] = sensor.ping()
        except Exception as exc:
            errors[side] = str(exc)

    threads = [
        threading.Thread(target=do_ping, args=("left", sensor_left)),
        threading.Thread(target=do_ping, args=("right", sensor_right)),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10)

    assert not errors, f"Ping errors: {errors}"
    assert results.get("left") is True, "Left sensor ping failed"
    assert results.get("right") is True, "Right sensor ping failed"
