"""
Shared fixtures and configuration for the OpenMotion SDK hardware test suite.

All session-scoped fixtures skip gracefully when the required hardware is
not present, so a partial rig (console-only, sensor-only, etc.) still
produces a meaningful test run.
"""

import os

import pytest

from omotion import MotionInterface


# ---------------------------------------------------------------------------
# Session-level interface fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def motion():
    """Initialise MotionInterface and yield for the whole session."""
    demo = os.getenv("OPENMOTION_DEMO", "0") == "1"
    iface = MotionInterface(demo_mode=demo)
    iface.start(wait=True, wait_timeout=3.0)
    yield iface
    iface.stop()


# ---------------------------------------------------------------------------
# Console fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def console(motion):
    if not motion.console.is_connected():
        pytest.skip("Console module not connected")
    return motion.console


# ---------------------------------------------------------------------------
# Sensor fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def sensor_left(motion):
    if not motion.left.is_connected():
        pytest.skip("Left sensor not connected")
    return motion.left


@pytest.fixture(scope="session")
def sensor_right(motion):
    if not motion.right.is_connected():
        pytest.skip("Right sensor not connected")
    return motion.right


@pytest.fixture(
    scope="session",
    params=["left", "right"],
    ids=["sensor_left", "sensor_right"],
)
def any_sensor(request, motion):
    """Parametrised fixture — each sensor test runs against both sides."""
    side = request.param
    sensor = motion.left if side == "left" else motion.right
    if not sensor.is_connected():
        pytest.skip(f"{side} sensor not connected")
    return sensor
