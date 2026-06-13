# tests/test_monitor_nonblocking.py
"""Failure-mode-A regression: a blocked connect on one handle must not stall
another handle's connect. Uses real ConnectWorker threads with fake handles."""
import threading

from omotion.connection_monitor import PollArrived
from omotion.connection_state import ConnectionState
from omotion.connect_worker import ConnectWorker


class _FakeHandle:
    def __init__(self, name, connect_fn):
        self.name = name
        self.state = ConnectionState.DISCONNECTED
        self._worker = ConnectWorker(
            name=name, do_connect=connect_fn, do_disconnect=lambda r: None,
            cooldown=0.0,
        )
        self._worker.start()

    def is_connected(self):
        return self.state == ConnectionState.CONNECTED

    def _handle_event(self, event):
        if isinstance(event, PollArrived) and self.state == ConnectionState.DISCONNECTED:
            self._worker.post("connect", "poll")

    def _shutdown(self):
        self._worker.stop()


def test_blocked_console_connect_does_not_starve_sensor():
    release = threading.Event()
    sensor_connected = threading.Event()

    def slow(reason, abort):
        release.wait(3.0)              # console blocks
        return True

    def fast(reason, abort):
        sensor_connected.set()
        return True

    console = _FakeHandle("console", slow)
    left = _FakeHandle("left", fast)

    # Directly dispatch PollArrived to both (bypassing USB enumeration).
    console._handle_event(PollArrived(handle_name="console"))
    left._handle_event(PollArrived(handle_name="left"))

    try:
        assert sensor_connected.wait(1.0), "sensor starved by blocked console"
    finally:
        release.set()
        console._shutdown()
        left._shutdown()
