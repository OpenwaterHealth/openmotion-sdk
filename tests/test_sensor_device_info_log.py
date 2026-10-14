"""Unit tests for MotionSensor.log_device_info block formatting.

These construct a MotionSensor without USB enumeration and mock the
identity reads, so they run with no hardware. The point of the block is
that every piece of hardware info we need (firmware, hw_id, serial, the
last shutdown reason, and all 8 camera UIDs) is logged, inside a
``====``-guarded, side-labeled region emitted as a single log record.
"""
import logging
import struct

from unittest.mock import MagicMock

from omotion.MotionSensor import MotionSensor
from omotion.MotionSensor import logger as sensor_logger
from omotion.reset_history import parse_reset_history


def _make_sensor(uids, history=None):
    s = MotionSensor.__new__(MotionSensor)
    s.get_version = MagicMock(return_value="1.7.0")
    s.get_cached_hardware_id = MagicMock(return_value="DEADBEEF")
    s.get_hardware_id = MagicMock(return_value="DEADBEEF")
    s.read_serial_number = MagicMock(return_value="OW-SN-001")
    s.get_reset_history = MagicMock(return_value=history)
    s._cached_camera_uids = uids
    return s


def _history(last_shutdown, boot_count, prev_alive_ms, counts):
    """A parsed sysmon_reset_history_t v1 (sensor-fw#137)."""
    payload = struct.pack(
        "<BB2x3I8I7II4III", 1, last_shutdown, boot_count, prev_alive_ms, 9000,
        *counts, *([0] * 7), 0, 0, 0, 0, 0, 0, 0,
    )
    return parse_reset_history(payload)


_UIDS = {i: f"0x{i:012X}" for i in range(8)}


def _block(caplog):
    """The single log record emitted by log_device_info."""
    assert len(caplog.records) == 1, [r.getMessage() for r in caplog.records]
    return caplog.records[0].getMessage()


def test_block_is_single_guarded_record(caplog):
    sensor = _make_sensor({i: f"0x{i:012X}" for i in range(8)})
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="left")
    msg = _block(caplog)
    lines = msg.splitlines()
    # Guarded top and bottom with a ==== rule.
    assert set(lines[0]) == {"="}
    assert set(lines[-1]) == {"="}
    assert len(lines[0]) >= 4


def test_block_carries_label_and_all_identity_fields(caplog):
    sensor = _make_sensor({i: f"0x{i:012X}" for i in range(8)})
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="right")
    msg = _block(caplog)
    assert "RIGHT" in msg                # side label, uppercased
    assert "1.7.0" in msg                # firmware version
    assert "DEADBEEF" in msg             # hardware id
    assert "OW-SN-001" in msg            # serial number


def test_block_logs_all_eight_camera_uids(caplog):
    uids = {i: f"0x{i:012X}" for i in range(8)}
    sensor = _make_sensor(uids)
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="left")
    msg = _block(caplog)
    for i in range(8):
        assert f"cam{i}" in msg
        assert uids[i] in msg


def test_missing_camera_uid_is_shown_not_dropped(caplog):
    # Camera 3 failed to read (empty string); it must still appear so a
    # dead camera is visible in the log rather than silently omitted.
    uids = {i: (f"0x{i:012X}" if i != 3 else "") for i in range(8)}
    sensor = _make_sensor(uids)
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="left")
    msg = _block(caplog)
    assert "cam3" in msg
    assert "<none>" in msg


def test_firmware_without_reset_history_says_so(caplog):
    sensor = _make_sensor(_UIDS, history=None)
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="left")
    msg = _block(caplog)
    assert "shutdown = unavailable" in msg


def test_block_carries_the_last_shutdown_reason(caplog):
    history = _history(2, 2, 61_000, (1, 0, 1, 0, 0, 0, 0, 0))
    sensor = _make_sensor(_UIDS, history=history)
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="left")
    # A host reset is not a failure: still exactly one record, no WARNING.
    msg = _block(caplog)
    lines = msg.splitlines()
    assert "  shutdown = host reset (OW_CMD_RESET); the previous session ran 1m01s" in lines
    assert "  boots    = 2 since power-on (power_off 1, host_reset 1)" in lines
    # Shutdown lines sit between the identity fields and the camera UIDs.
    assert lines.index("  serial   = OW-SN-001") < lines.index("  camera UIDs:")
    assert any(line.startswith("  shutdown = ") for line in
               lines[lines.index("  serial   = OW-SN-001"):lines.index("  camera UIDs:")])


def test_abnormal_shutdown_is_repeated_as_a_warning(caplog):
    # Reset loop: three watchdog resets, none of which reached the main loop.
    history = _history(1, 4, 0, (1, 3, 0, 0, 0, 0, 0, 0))
    sensor = _make_sensor(_UIDS, history=history)
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="right")
    assert [r.levelno for r in caplog.records] == [logging.INFO, logging.WARNING]
    warning = caplog.records[1].getMessage()
    assert warning.startswith("RIGHT sensor: last shutdown was unexpected reset")
    assert "never reached the main loop" in warning
    assert "boot 4 since power-on" in warning


def test_power_off_is_not_a_warning(caplog):
    history = _history(0, 1, 0, (1, 0, 0, 0, 0, 0, 0, 0))
    sensor = _make_sensor(_UIDS, history=history)
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info(label="left")
    assert "shutdown = power off" in _block(caplog)


def test_no_label_defaults_to_sensor(caplog):
    sensor = _make_sensor({i: f"0x{i:012X}" for i in range(8)})
    with caplog.at_level(logging.INFO, logger=sensor_logger.name):
        sensor.log_device_info()
    msg = _block(caplog)
    assert "SENSOR" in msg
