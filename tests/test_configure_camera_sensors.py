"""Tests for ScanWorkflow.start_configure_camera_sensors skip-if-programmed logic.

Pure-software: sensors are mocks, no hardware needed. Covers the per-scan
FPGA re-flash redundancy fix — cameras whose status reports firmware
programmed (bit 1) and configured (bit 2) are skipped unless the request
sets force_program.
"""

import dataclasses
import threading
from unittest import mock

from omotion.ScanWorkflow import ConfigureRequest, ScanWorkflow

# Camera status byte bits (see MotionSensor.get_camera_status):
#   0 — Peripheral READY, 1 — Firmware programmed, 2 — Configured
STATUS_READY = 0x01
STATUS_PROGRAMMED = 0x07


def _make_sensor(status_by_pos):
    """Mock MotionSensor whose get_camera_status reports the given statuses.

    status_by_pos: either a single status byte applied to every queried
    position, or a dict {position: status_byte}.
    """
    sensor = mock.Mock()
    sensor.is_connected.return_value = True
    sensor.enable_camera_power.return_value = True
    sensor.disable_camera_power.return_value = True
    sensor.get_camera_power_status.return_value = [True] * 8

    def _status(mask):
        return {
            i: (status_by_pos[i] if isinstance(status_by_pos, dict)
                else status_by_pos)
            for i in range(8) if (mask >> i) & 1
        }

    sensor.get_camera_status.side_effect = _status
    sensor.program_fpga.return_value = True
    sensor.camera_configure_registers.return_value = True
    return sensor


def _run_configure(sensor, request):
    """Run the configure worker to completion against a single mocked left sensor."""
    workflow = ScanWorkflow(mock.Mock())
    done = threading.Event()
    results = []
    progress = []

    def _on_complete(res):
        results.append(res)
        done.set()

    with mock.patch.object(
        ScanWorkflow, "_resolve_active_sides",
        return_value=[("left", request.left_camera_mask, sensor)],
    ), mock.patch("omotion.ScanWorkflow.time.sleep"):
        assert workflow.start_configure_camera_sensors(
            request,
            on_progress_fn=progress.append,
            on_complete_fn=_on_complete,
        )
        assert done.wait(timeout=10), "configure worker did not finish"
    return results[0], progress


def test_configure_request_has_force_program_field():
    field_names = {f.name for f in dataclasses.fields(ConfigureRequest)}
    assert "force_program" in field_names
    req = ConfigureRequest(left_camera_mask=0x01, right_camera_mask=0)
    assert req.force_program is False


def test_skips_already_programmed_cameras():
    """Cameras with status bits 1 and 2 set are not re-flashed or re-configured."""
    sensor = _make_sensor(STATUS_PROGRAMMED)
    req = ConfigureRequest(left_camera_mask=0x03, right_camera_mask=0)
    result, progress = _run_configure(sensor, req)
    assert result.ok, result.error
    sensor.program_fpga.assert_not_called()
    sensor.camera_configure_registers.assert_not_called()
    # Skipped cameras still count toward progress so the bar reaches 100.
    assert progress[-1] == 100


def test_programs_unprogrammed_cameras():
    sensor = _make_sensor(STATUS_READY)
    req = ConfigureRequest(left_camera_mask=0x01, right_camera_mask=0)
    result, progress = _run_configure(sensor, req)
    assert result.ok, result.error
    sensor.program_fpga.assert_called_once_with(
        camera_position=0x01, manual_process=False
    )
    sensor.camera_configure_registers.assert_called_once_with(
        camera_position=0x01
    )
    assert progress[-1] == 100


def test_force_program_reflashes_programmed_cameras():
    sensor = _make_sensor(STATUS_PROGRAMMED)
    req = ConfigureRequest(
        left_camera_mask=0x01, right_camera_mask=0, force_program=True
    )
    result, _ = _run_configure(sensor, req)
    assert result.ok, result.error
    sensor.program_fpga.assert_called_once()
    sensor.camera_configure_registers.assert_called_once()


def test_partially_configured_camera_is_fully_programmed():
    """Programmed-but-not-configured (bit 1 without bit 2) gets the full
    program + configure sequence, matching get_camera_histogram's pattern."""
    sensor = _make_sensor(0x03)  # READY + programmed, not configured
    req = ConfigureRequest(left_camera_mask=0x01, right_camera_mask=0)
    result, _ = _run_configure(sensor, req)
    assert result.ok, result.error
    sensor.program_fpga.assert_called_once()
    sensor.camera_configure_registers.assert_called_once()


def test_mixed_mask_programs_only_unprogrammed_positions():
    sensor = _make_sensor({0: STATUS_PROGRAMMED, 1: STATUS_READY})
    req = ConfigureRequest(left_camera_mask=0x03, right_camera_mask=0)
    result, progress = _run_configure(sensor, req)
    assert result.ok, result.error
    sensor.program_fpga.assert_called_once_with(
        camera_position=0x02, manual_process=False
    )
    sensor.camera_configure_registers.assert_called_once_with(
        camera_position=0x02
    )
    assert progress[-1] == 100
