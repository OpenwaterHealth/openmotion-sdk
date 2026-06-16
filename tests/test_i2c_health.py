"""Tests for the connection-time I2C health check.

Pure-unit tests cover the shared logging helper. The fixture-based tests
confirm that connecting a device auto-populates its cached health snapshot
(runs against hardware, or in demo mode via OPENMOTION_DEMO=1; skips
otherwise).
"""

from unittest.mock import Mock

import pytest

from omotion.utils import log_i2c_health


# ---------------------------------------------------------------------------
# log_i2c_health — pure unit, no hardware
# ---------------------------------------------------------------------------

@pytest.mark.unit
def test_log_i2c_health_none_is_debug():
    lg = Mock()
    log_i2c_health("dev", None, lg)
    lg.debug.assert_called_once()
    lg.info.assert_not_called()
    lg.warning.assert_not_called()


@pytest.mark.unit
def test_log_i2c_health_all_present_is_info():
    lg = Mock()
    log_i2c_health("dev", {"all_present": True}, lg)
    lg.info.assert_called_once()
    lg.warning.assert_not_called()


@pytest.mark.unit
def test_log_i2c_health_degraded_is_warning():
    lg = Mock()
    log_i2c_health("dev", {"all_present": False, "imu": False}, lg)
    lg.warning.assert_called_once()
    lg.info.assert_not_called()


# ---------------------------------------------------------------------------
# Connection-time population — requires a connected device (or demo mode)
# ---------------------------------------------------------------------------

@pytest.mark.console
def test_console_i2c_health_populated_on_connect(console):
    # The handle reads and caches its boot-time snapshot during connection.
    snap = console.i2c_health
    assert snap is not None, "i2c_health should be populated at connection"
    assert console.is_i2c_healthy() == bool(snap.get("all_present"))


@pytest.mark.sensor
def test_sensor_i2c_health_populated_on_connect(sensor_left):
    snap = sensor_left.i2c_health
    assert snap is not None, "i2c_health should be populated at connection"
    assert sensor_left.is_i2c_healthy() == bool(snap.get("all_present"))
