"""Unit tests for stream data-flow stall detection (#192).

The histogram stream had no failure-reporting path at all — not a threshold
set too permissively, no wire. ``MotionComposite`` wires ``on_io_error`` for
the command interface only, and ``StreamInterface``'s read-timeout branch
reads, verbatim, ``# Otherwise keep waiting — scan is still running.`` with no
counter and no ceiling. A side that stopped delivering frames mid-scan was
indistinguishable from an idle one, so a scan could run its full duration
producing nothing (bloodflow-app#390: an entire sensor side dark for ~21
minutes while the firmware logged 23,133 "HISTO enqueue fail: queue full").

These pin detection only. Deciding what to *do* about a stalled side (abort
the scan, warn, keep going) is policy and deliberately lives above this layer
— the callback is the hook.

No hardware: the device is a MagicMock whose read raises a USB timeout, which
is exactly what a silent-but-enumerated IN endpoint produces.
"""

import queue
import threading
import time
from unittest.mock import MagicMock

import pytest
import usb.core

from omotion.StreamInterface import StreamInterface

pytestmark = pytest.mark.unit

# Comfortably above the loop's own iteration cost, far below test timeouts.
_STALL_SEC = 0.3
_SETTLE = 2.0


def _silent_device():
    """A device that stays enumerated but never delivers a frame."""
    dev = MagicMock(spec=usb.core.Device)

    def _timeout(*_a, **_kw):
        time.sleep(0.01)  # emulate the read window rather than spinning hot
        raise usb.core.USBTimeoutError("timed out", -7, 110)

    dev.read.side_effect = _timeout
    return dev


def _make_stream(dev, stall_timeout_sec=_STALL_SEC):
    si = StreamInterface(dev, interface_index=1, desc="TEST-HISTO")
    si.ep_in = MagicMock(bEndpointAddress=0x81, wMaxPacketSize=512)
    si.stall_timeout_sec = stall_timeout_sec
    return si


class _Recorder:
    """Captures stall callbacks from the stream thread."""

    def __init__(self):
        self.calls = []
        self.fired = threading.Event()

    def __call__(self, stalled_for_sec):
        self.calls.append(stalled_for_sec)
        self.fired.set()


def _run_briefly(si, seconds):
    si.start_streaming(queue.Queue(maxsize=16), expected_size=4105)
    try:
        time.sleep(seconds)
    finally:
        si.stop_streaming()


def test_silent_endpoint_reports_a_stall():
    """The regression: a side that stops delivering must be noticed."""
    si = _make_stream(_silent_device())
    rec = _Recorder()
    si.on_stream_stall = rec

    si.start_streaming(queue.Queue(maxsize=16), expected_size=4105)
    try:
        assert rec.fired.wait(timeout=_SETTLE), (
            "a totally silent IN endpoint produced no stall report"
        )
    finally:
        si.stop_streaming()

    assert rec.calls[0] >= _STALL_SEC


def test_stall_is_reported_once_per_streaming_session():
    """One report, not one per read window — the point is to escalate, not
    to fill the log at 2 Hz for the rest of a 12-hour scan."""
    si = _make_stream(_silent_device())
    rec = _Recorder()
    si.on_stream_stall = rec

    si.start_streaming(queue.Queue(maxsize=16), expected_size=4105)
    try:
        assert rec.fired.wait(timeout=_SETTLE)
        time.sleep(_STALL_SEC * 3)
    finally:
        si.stop_streaming()

    assert len(rec.calls) == 1, f"stall reported {len(rec.calls)} times"


def test_no_stall_before_the_threshold():
    si = _make_stream(_silent_device(), stall_timeout_sec=10.0)
    rec = _Recorder()
    si.on_stream_stall = rec

    _run_briefly(si, 0.4)

    assert rec.calls == []


def test_flowing_data_never_stalls():
    """Regression fence: a healthy stream must stay silent."""
    dev = MagicMock(spec=usb.core.Device)

    def _frame(*_a, **_kw):
        time.sleep(0.01)
        return bytearray(4105)

    dev.read.side_effect = _frame
    si = _make_stream(dev)
    rec = _Recorder()
    si.on_stream_stall = rec

    _run_briefly(si, _STALL_SEC * 3)

    assert rec.calls == []
    assert si.packets_received > 0


def test_data_resets_the_stall_clock():
    """Frames arriving just under the threshold must keep it from firing —
    otherwise a merely slow stream is reported as a dead one."""
    dev = MagicMock(spec=usb.core.Device)
    state = {"n": 0}

    def _slow(*_a, **_kw):
        # One frame every ~0.2 s against a 0.3 s threshold.
        state["n"] += 1
        time.sleep(0.2)
        return bytearray(4105)

    dev.read.side_effect = _slow
    si = _make_stream(dev)
    rec = _Recorder()
    si.on_stream_stall = rec

    _run_briefly(si, 1.2)

    assert rec.calls == [], "a slow-but-alive stream was reported as stalled"


def test_zero_timeout_disables_detection():
    si = _make_stream(_silent_device(), stall_timeout_sec=0)
    rec = _Recorder()
    si.on_stream_stall = rec

    _run_briefly(si, _STALL_SEC * 3)

    assert rec.calls == []


def test_stall_clock_restarts_with_each_streaming_session():
    """A stall in scan N must not immediately re-fire at the start of N+1."""
    si = _make_stream(_silent_device())
    rec = _Recorder()
    si.on_stream_stall = rec

    si.start_streaming(queue.Queue(maxsize=16), expected_size=4105)
    try:
        assert rec.fired.wait(timeout=_SETTLE)
    finally:
        si.stop_streaming()

    rec.fired.clear()
    si.start_streaming(queue.Queue(maxsize=16), expected_size=4105)
    try:
        # Immediately after restart the clock is fresh, so nothing yet.
        assert not rec.fired.wait(timeout=_STALL_SEC / 3)
    finally:
        si.stop_streaming()


def test_a_raising_callback_cannot_kill_the_stream_thread():
    """The callback is foreign code. A scan must not die because a consumer
    threw — the same discipline CommInterface._notify_io_error already uses."""
    si = _make_stream(_silent_device())
    si.on_stream_stall = MagicMock(side_effect=RuntimeError("boom"))

    si.start_streaming(queue.Queue(maxsize=16), expected_size=4105)
    try:
        time.sleep(_STALL_SEC * 2)
        assert si.thread.is_alive(), "stream thread died on a raising callback"
    finally:
        si.stop_streaming()


# --- the missing wire ----------------------------------------------------

def test_composite_wires_the_histo_stall_callback():
    """The regression this ticket is named for: MotionComposite wired
    on_io_error for the command interface and nothing at all for the
    streams, so a stalled side had no route to any consumer."""
    from omotion.MotionComposite import MotionComposite

    seen = []
    mc = MotionComposite(
        MagicMock(spec=usb.core.Device), desc="LEFT",
        on_stream_stall=lambda desc, secs: seen.append((desc, secs)),
    )

    assert mc.histo.on_stream_stall is not None, "histo stream has no error wire"

    mc.histo.on_stream_stall(7.5)

    assert seen == [("LEFT-HISTO", 7.5)], (
        "a stalled side must identify itself to the consumer"
    )


def test_composite_does_not_arm_the_watchdog_on_the_imu_stream():
    """The IMU stream can be legitimately idle; only the histogram stream
    has a guaranteed cadence to measure against."""
    from omotion.MotionComposite import MotionComposite

    mc = MotionComposite(MagicMock(spec=usb.core.Device), desc="LEFT")

    assert mc.histo.stall_timeout_sec > 0
    assert mc.imu.stall_timeout_sec == 0
