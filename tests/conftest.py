"""
Shared fixtures and configuration for the OpenMotion SDK hardware test suite.

All session-scoped fixtures skip gracefully when the required hardware is
not present, so a partial rig (console-only, sensor-only, etc.) still
produces a meaningful test run.
"""

import os
import time

import pytest

from omotion import MotionInterface


# How long a device fixture waits for its handle to reach CONNECTED before
# skipping. Hotplug enumeration is asynchronous, so the rig can take several
# seconds past start() to settle — a short wait silently skips the whole HIL
# tier on a cold rig. Override with OPENMOTION_CONNECT_TIMEOUT for slow boxes.
_CONNECT_TIMEOUT = float(os.getenv("OPENMOTION_CONNECT_TIMEOUT", "12"))


def _await_connected(handle, label):
    """Poll until ``handle`` is connected or the timeout elapses; skip if not.

    Session-scoped, so the first device fixture absorbs the enumeration wait
    once and the rest see an already-connected rig."""
    deadline = time.monotonic() + _CONNECT_TIMEOUT
    while time.monotonic() < deadline:
        if handle.is_connected():
            return handle
        time.sleep(0.1)
    pytest.skip(f"{label} not connected (after {_CONNECT_TIMEOUT:.0f}s)")


# ---------------------------------------------------------------------------
# Session-level interface fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def motion():
    """Initialise MotionInterface and yield for the whole session."""
    demo = os.getenv("OPENMOTION_DEMO", "0") == "1"
    iface = MotionInterface(demo_mode=demo)
    # start(wait=...) only blocks until attached devices leave CONNECTING; the
    # CONNECTED transition can lag behind. The device fixtures poll for that.
    iface.start(wait=True, wait_timeout=_CONNECT_TIMEOUT)
    yield iface
    iface.stop()


# ---------------------------------------------------------------------------
# Console fixture
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def console(motion):
    return _await_connected(motion.console, "Console module")


# ---------------------------------------------------------------------------
# Sensor fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="session")
def sensor_left(motion):
    return _await_connected(motion.left, "Left sensor")


@pytest.fixture(scope="session")
def sensor_right(motion):
    return _await_connected(motion.right, "Right sensor")


@pytest.fixture(
    scope="session",
    params=["left", "right"],
    ids=["sensor_left", "sensor_right"],
)
def any_sensor(request, motion):
    """Parametrised fixture — each sensor test runs against both sides."""
    side = request.param
    sensor = motion.left if side == "left" else motion.right
    return _await_connected(sensor, f"{side} sensor")
