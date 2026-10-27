"""Thread-lifecycle regression tests for the USB transport layer.

A packaged-app fault dump (ow-testapp 2026-08-17) showed ~52 live
``CommInterface._process_responses`` threads after an afternoon of sensor
release/reacquire cycles: the async response thread was started in the
``CommInterface`` constructor, so any ``MotionComposite`` that was built
but whose ``open()`` raised (the normal case during the post-enumeration
"resource busy" retry window in ``MotionSensor._drive_connecting``) left a
thread parked on ``_buffer_condition.wait(timeout=0.1)`` forever — nothing
ever set its ``stop_event``, and nothing joined it.

These tests pin the fixed contract, by counting ``threading.enumerate()``
across simulated open/close cycles:

* constructing a transport starts no threads;
* start/stop (open/close) cycles return the process to its thread
  baseline — every transport thread has exited and been joined;
* a fatal USB error in the read loop shuts the response thread down even
  if nobody calls ``stop_read_thread`` afterwards;
* ``MotionSensor._drive_connecting`` closes a composite whose ``open()``
  raised instead of orphaning it.

No hardware: devices are fakes. pyusb's ``usb.util`` claim/release/dispose
helpers all delegate to ``device._ctx``, so a fake with a MagicMock ``_ctx``
drives the real claim/release code paths.
"""

import queue
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import usb.core

from omotion.CommInterface import CommInterface
from omotion.MotionComposite import MotionComposite

pytestmark = pytest.mark.unit


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


def _usb_timeout(*_args, **_kwargs):
    """Behave like an idle endpoint: brief block, then a read timeout.

    The 1 ms sleep keeps fake read loops from spinning a core flat out
    while a test lets them run.
    """
    time.sleep(0.001)
    raise usb.core.USBTimeoutError("timeout", -7, 110)


class _FakeEndpoint:
    def __init__(self, addr):
        self.bEndpointAddress = addr
        self.wMaxPacketSize = 64


class _FakeInterfaceSetting:
    """Iterable of endpoints — the shape usb.util.find_descriptor expects."""

    def __init__(self):
        self._endpoints = [_FakeEndpoint(0x81), _FakeEndpoint(0x01)]

    def __iter__(self):
        return iter(self._endpoints)


class _FakeConfiguration:
    def __getitem__(self, key):  # cfg[(interface_index, alt_setting)]
        return _FakeInterfaceSetting()


class _FakeDevice:
    """Just enough usb.core.Device surface for MotionComposite open/close."""

    def __init__(self, fail_open=False):
        self._ctx = MagicMock()  # claim/release/dispose land here (usb.util)
        self.fail_open = fail_open

    def set_configuration(self):
        if self.fail_open:
            # The classic post-enumeration failure the connect retry loop
            # exists for: the interface is still held elsewhere.
            raise usb.core.USBError("Resource busy", -6, 16)

    def get_active_configuration(self):
        return _FakeConfiguration()

    def read(self, endpoint, size, timeout=None):
        _usb_timeout()

    def write(self, endpoint, data, timeout=None):
        return len(data)

    def clear_halt(self, endpoint):
        pass


def _make_comm(read_side_effect=_usb_timeout) -> CommInterface:
    """An async CommInterface over a MagicMock dev, bypassing claim()."""
    dev = MagicMock()
    dev.read.side_effect = read_side_effect
    ci = CommInterface(dev, interface_index=0, desc="TEST", async_mode=True)
    ci.ep_in = MagicMock(bEndpointAddress=0x81, wMaxPacketSize=64)
    ci.ep_out = MagicMock(bEndpointAddress=0x01)
    return ci


# ---------------------------------------------------------------------------
# Thread accounting
# ---------------------------------------------------------------------------


def _live_threads() -> set:
    return {t for t in threading.enumerate() if t.is_alive()}


def _extra_threads(baseline: set, grace: float = 3.0) -> set:
    """Threads beyond ``baseline`` still alive after up to ``grace`` seconds.

    Empty means every thread the test started has exited. The stop paths
    join, so in practice this returns on the first check; the grace window
    only matters for the self-termination tests, where the exit is signalled
    rather than joined.
    """
    deadline = time.monotonic() + grace
    while True:
        extra = _live_threads() - baseline
        if not extra or time.monotonic() >= deadline:
            return extra
        time.sleep(0.01)


def _names(threads: set) -> list:
    return sorted(t.name for t in threads)


# ---------------------------------------------------------------------------
# CommInterface lifecycle
# ---------------------------------------------------------------------------


def test_async_constructor_starts_no_threads():
    """Building the transport must not start it: an orphaned instance whose
    open() never happened (or failed) has no thread to leak."""
    baseline = _live_threads()
    ci = _make_comm()
    assert not (_live_threads() - baseline), (
        "CommInterface(async_mode=True) started a thread in the constructor"
    )
    assert ci.response_thread is None


def test_comm_start_stop_cycles_return_to_thread_baseline():
    """Simulated connect/disconnect cycles: stop_read_thread must stop AND
    join both the read loop and the async response parser, every cycle."""
    ci = _make_comm()
    baseline = _live_threads()

    for cycle in range(5):
        ci.start_read_thread()
        assert ci.read_thread.is_alive(), f"cycle {cycle}: read thread not running"
        assert ci.response_thread is not None and ci.response_thread.is_alive(), (
            f"cycle {cycle}: response thread not running"
        )

        ci.stop_read_thread()
        # stop joins, so the exit is synchronous — no grace window needed.
        assert not ci.read_thread.is_alive(), (
            f"cycle {cycle}: read thread survived stop_read_thread"
        )
        assert not ci.response_thread.is_alive(), (
            f"cycle {cycle}: response thread survived stop_read_thread "
            "(the fault-dump leak: 52 of these after a day of cycles)"
        )
        extra = _live_threads() - baseline
        assert not extra, f"cycle {cycle} leaked threads: {_names(extra)}"


def test_fatal_read_error_stops_response_thread_without_close():
    """When the read loop dies on a fatal USB error (unplug), the response
    thread must follow on its own — even if no one ever calls
    stop_read_thread on this instance."""

    def _enodev(*_a, **_k):
        raise usb.core.USBError("No such device", -4, 19)

    ci = _make_comm(read_side_effect=_enodev)
    io_errors = []
    ci.on_io_error = lambda errno, msg: io_errors.append(errno)

    baseline = _live_threads()
    ci.start_read_thread()

    extra = _extra_threads(baseline)
    assert not extra, (
        f"transport threads survived fatal read error: {_names(extra)}"
    )
    assert ci._transport_down_evt.is_set()
    assert ci.stop_event.is_set()
    assert io_errors == [19]


def test_restart_after_fatal_error_brings_both_threads_back():
    """Reconnect after a transport death must restart the full thread pair
    and clear the latched stop/transport-down state."""
    calls = {"n": 0}

    def _read(*_a, **_k):
        calls["n"] += 1
        if calls["n"] == 1:
            raise usb.core.USBError("No such device", -4, 19)
        _usb_timeout()

    ci = _make_comm(read_side_effect=_read)
    baseline = _live_threads()

    ci.start_read_thread()
    assert not _extra_threads(baseline), "thread pair did not die on fatal error"

    ci.start_read_thread()  # reconnect
    assert ci.read_thread.is_alive()
    assert ci.response_thread.is_alive()
    assert not ci.stop_event.is_set()
    assert not ci._transport_down_evt.is_set()

    ci.stop_read_thread()
    assert not (_live_threads() - baseline)


# ---------------------------------------------------------------------------
# MotionComposite lifecycle
# ---------------------------------------------------------------------------


def test_composite_open_close_cycles_return_to_thread_baseline():
    """Full composite open/close cycles (with a streaming session in the
    middle, like a scan) must return the process to its thread baseline."""
    dev = _FakeDevice()
    baseline = _live_threads()

    for cycle in range(5):
        comp = MotionComposite(dev, desc="TESTSIDE", async_mode=True)
        assert not (_live_threads() - baseline), (
            f"cycle {cycle}: constructing a composite started threads"
        )

        comp.open()
        assert comp.comm.read_thread.is_alive()
        assert comp.comm.response_thread.is_alive()

        comp.histo.start_streaming(queue.Queue(), expected_size=512)
        assert comp.histo.thread.is_alive()

        comp.close()
        extra = _live_threads() - baseline
        assert not extra, f"cycle {cycle} leaked threads: {_names(extra)}"


def test_composite_close_without_open_is_safe_and_threadless():
    """The connect retry loop closes composites that never opened; that
    close must neither raise nor leave threads behind."""
    dev = _FakeDevice()
    baseline = _live_threads()
    comp = MotionComposite(dev, desc="TESTSIDE", async_mode=True)
    comp.close()
    assert not (_live_threads() - baseline)


# ---------------------------------------------------------------------------
# MotionSensor connect retry loop
# ---------------------------------------------------------------------------


def test_drive_connecting_open_failure_closes_composite_and_leaks_nothing(
    monkeypatch,
):
    """The original leak: every failed open() attempt in the connect retry
    loop orphaned a constructed composite. Each attempt must now close it —
    releasing interfaces, disposing USB resources, leaving no threads."""
    import sys

    from omotion.MotionSensor import MotionSensor
    from omotion.connection_state import ConnectionState

    dev = _FakeDevice(fail_open=True)
    sensor = MotionSensor("left", vid=0x0483, pid=0x5750)
    monkeypatch.setattr(sensor, "_find_dev", lambda: dev)
    # Collapse the ~1.9 s retry backoff without touching the global time
    # module (this file's own helpers use it). Resolve the module through
    # sys.modules: the package re-exports the class under the same name,
    # so the string form "omotion.MotionSensor.time" resolves wrongly.
    ms_module = sys.modules[MotionSensor.__module__]
    monkeypatch.setattr(
        ms_module,
        "time",
        SimpleNamespace(sleep=lambda _s: None, monotonic=time.monotonic),
    )

    baseline = _live_threads()
    sensor._drive_connecting(reason="test")

    assert sensor.state == ConnectionState.DISCONNECTED
    assert sensor.uart is None
    # One close per failed backoff attempt: dispose_resources reaches
    # dev._ctx.dispose each time.
    assert dev._ctx.dispose.call_count == 5, (
        f"expected every failed connect attempt to close its composite, "
        f"got {dev._ctx.dispose.call_count} dispose call(s) for 5 attempts"
    )
    extra = _extra_threads(baseline)
    assert not extra, f"_drive_connecting leaked threads: {_names(extra)}"
