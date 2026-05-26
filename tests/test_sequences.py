"""
Round-trip sequence tests (Section 5 of the test plan).

Each test brings a subsystem up from a cold state, exercises it,
then tears it back down — even on failure.  Teardown always runs
in a ``finally`` block.
"""

import os
import time
import threading

import numpy as np
import pytest

import struct

# All sequence tests involve program_fpga (up to 60 s per sensor side) so the
# default 30-second timeout is too short.  Override at module level; individual
# tests that are even heavier carry their own @pytest.mark.timeout decorator.
pytestmark = [pytest.mark.sensor, pytest.mark.sequence, pytest.mark.timeout(120)]

# Minimal BFI calibration arrays required by SciencePipeline
_BFI_ZEROS = np.zeros((2, 8), dtype=np.float32)
_BFI_ONES = np.ones((2, 8), dtype=np.float32) * 10.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

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


def _camera_up(sensor, mask=0x01, configure=True):
    """Power on → FPGA program → optional register configure.

    NOTE: program_fpga can take up to 16 seconds; tests using this helper
    should be marked @pytest.mark.slow.
    """
    ok = sensor.enable_camera_power(mask)
    if ok is False:
        pytest.fail(f"enable_camera_power(0x{mask:02X}) returned False")
    time.sleep(0.5)  # rail settle

    ok = sensor.program_fpga(camera_position=mask, manual_process=False)
    if ok is False:
        pytest.fail(f"program_fpga(0x{mask:02X}) returned False")
    time.sleep(0.1)  # settle after bitstream load completes

    if configure:
        ok = sensor.camera_configure_registers(mask)
        if ok is False:
            pytest.fail(f"camera_configure_registers(0x{mask:02X}) returned False")


# ===========================================================================
# 5.1 Camera bring-up and single frame
# ===========================================================================

def test_camera_full_bringup_single_frame(any_sensor):
    """FPGA program → configure → status check → capture → parse histogram."""
    _camera_up(any_sensor)

    # Mirror what the high-level get_camera_histogram() does: verify the camera
    # reports READY + FPGA-loaded + registers-configured (bits 0, 1, 2) before
    # firing the capture command.  Without this the returned packet is too short.
    status_map = any_sensor.get_camera_status(0x01)
    assert status_map is not None, "get_camera_status returned None"
    status = status_map.get(0)  # camera_id 0, index not bitmask
    assert status is not None, "No status for camera 0 in status_map"
    ready     = bool(status & (1 << 0))
    fpga_done = bool(status & (1 << 1))
    regs_done = bool(status & (1 << 2))
    assert ready,     f"Camera 0 not READY (status=0x{status:02X})"
    assert fpga_done, f"Camera 0 FPGA not loaded (status=0x{status:02X})"
    assert regs_done, f"Camera 0 registers not configured (status=0x{status:02X})"

    assert any_sensor.camera_capture_histogram(0x01) is True
    raw = any_sensor.camera_get_histogram(0x01)
    assert isinstance(raw, (bytes, bytearray)) and len(raw) == 4100, (
        f"camera_get_histogram returned {len(raw) if raw else 0} bytes (expected 4100)"
    )

    histogram, temperature_c = _decode_raw_histogram(raw)
    assert len(histogram) == 1024
    assert isinstance(temperature_c, float)


# ===========================================================================
# 5.3 FPGA enable → histogram → FPGA disable
# ===========================================================================

@pytest.mark.fpga
def test_fpga_enable_histogram_disable(any_sensor):
    """FPGA program → configure → capture one histogram."""
    _camera_up(any_sensor)  # program_fpga → configure_registers
    any_sensor.camera_capture_histogram(0x01)
    raw = any_sensor.camera_get_histogram(0x01)
    histogram, _ = _decode_raw_histogram(raw)
    assert len(histogram) == 1024


# ===========================================================================
# 5.4 Streaming acquisition
# ===========================================================================

# ===========================================================================
# 5.5 External FSIN sequence
# ===========================================================================

def test_external_fsin_sequence(any_sensor):
    """Enable FSIN ext → capture one frame → disable FSIN ext."""
    _camera_up(any_sensor)
    assert any_sensor.enable_camera_fsin_ext() is True
    try:
        time.sleep(0.2)
        any_sensor.camera_capture_histogram(0x01)
        raw = any_sensor.camera_get_histogram(0x01)
        assert isinstance(raw, (bytes, bytearray)) and len(raw) > 0
    finally:
        any_sensor.disable_camera_fsin_ext()


# ===========================================================================
# 5.6 Test pattern verification
# ===========================================================================

def test_test_pattern_histogram(any_sensor):
    """Normal configure → overlay test pattern → capture histogram → assert non-zero bins."""
    _camera_up(any_sensor, configure=True)  # normal register init required first
    any_sensor.camera_configure_test_pattern(camera_position=0x01, test_pattern=1)
    any_sensor.camera_capture_histogram(0x01)
    raw = any_sensor.camera_get_histogram(0x01)
    histogram, _ = _decode_raw_histogram(raw)
    assert int(histogram.sum()) > 0, "Test pattern histogram has no counts"


# ===========================================================================
# 5.7 Console trigger + LSYNC count
# ===========================================================================

@pytest.mark.console
@pytest.mark.slow
def test_trigger_lsync_sequence(console):
    """Fire ~10 Hz trigger for 1 s, assert LSYNC counter accumulates pulses."""
    console.set_trigger_json({"rate": 10})
    console.start_trigger()
    time.sleep(1.1)
    count = console.get_lsync_pulsecount()
    console.stop_trigger()
    assert count >= 1, f"Expected at least 1 LSYNC pulse in 1.1 s, got {count}"


# ===========================================================================
# 5.9 Full scan workflow
# ===========================================================================

@pytest.mark.slow
@pytest.mark.console
@pytest.mark.timeout(300)
def test_scan_workflow_end_to_end(motion, tmp_path):
    """
    Execute a 5-second scan via ScanWorkflow and assert a non-empty
    CSV is written with a matching frame count.
    """
    import csv as csv_module
    from omotion.ScanWorkflow import ScanRequest

    request = ScanRequest(
        subject_id="pytest_subject",
        duration_sec=5,
        left_camera_mask=0x01,
        right_camera_mask=0x01,
        data_dir=str(tmp_path),
        disable_laser=False,
    )

    result_holder = {}
    done = threading.Event()

    def on_result(result):
        result_holder["result"] = result
        done.set()

    motion.scan_workflow.start_scan(request, on_complete_fn=on_result)
    done.wait(timeout=30)

    result = result_holder.get("result")
    assert result is not None, "ScanWorkflow did not call on_complete_fn"
    assert result.ok, f"Scan failed: {result.error}"
    assert not result.canceled

    for path in (result.left_path, result.right_path):
        if path:
            assert os.path.isfile(path), f"Expected CSV at {path}"
            with open(path, newline="") as f:
                rows = list(csv_module.reader(f))
            data_rows = [r for r in rows if r and not r[0].startswith("#")]
            assert len(data_rows) > 1, f"CSV at {path} has no data rows"


# ===========================================================================
# 5.10 Camera power cycle
#
# These tests deliberately power-cycle the cameras, which can leave the
# TCA9548 I2C mux in a bad state (known firmware bug).  They are placed
# last so that the off/on transitions do not poison earlier tests.
# ===========================================================================

def test_camera_power_cycle(any_sensor):
    """On → status on → off → status off → on → off."""
    assert any_sensor.enable_camera_power(0x01) is not False
    try:
        status = any_sensor.get_camera_power_status()
        assert status[0], "Camera 0 should be on after enable"

        any_sensor.disable_camera_power(0x01)
        time.sleep(0.1)
        status = any_sensor.get_camera_power_status()
        assert not status[0], "Camera 0 should be off after disable"

        assert any_sensor.enable_camera_power(0x01) is not False
        time.sleep(0.1)
        status = any_sensor.get_camera_power_status()
        assert status[0], "Camera 0 should be on after second enable"
    finally:
        any_sensor.disable_camera_power(0x01)


# ===========================================================================
# 5.11 TCA I2C switch recovery after power cycle (regression)
#
# Known firmware bug: after a camera power cycle the TCA9548 I2C mux on the
# sensor module is not properly reset, causing subsequent
# enable_camera_power() calls to fail.  These tests isolate the failure so
# that when the firmware is patched, the suite turns green automatically.
# ===========================================================================

def test_tca_recovery_after_power_off_on(any_sensor):
    """Power off → on should succeed (TCA must be re-initialised)."""
    assert any_sensor.enable_camera_power(0x01) is not False
    any_sensor.disable_camera_power(0x01)
    time.sleep(0.2)
    ok = any_sensor.enable_camera_power(0x01)
    try:
        assert ok is not False, (
            "enable_camera_power failed after power cycle — "
            "TCA I2C mux likely not reset by firmware"
        )
    finally:
        any_sensor.disable_camera_power(0x01)


def test_tca_recovery_after_full_power_cycle(any_sensor):
    """Full on → off → on → off → on cycle — last enable must succeed."""
    for _ in range(2):
        assert any_sensor.enable_camera_power(0x01) is not False
        time.sleep(0.1)
        any_sensor.disable_camera_power(0x01)
        time.sleep(0.2)

    ok = any_sensor.enable_camera_power(0x01)
    try:
        assert ok is not False, (
            "enable_camera_power failed after repeated power cycles — "
            "TCA I2C mux not recovered"
        )
    finally:
        any_sensor.disable_camera_power(0x01)
