"""Console teardown must not log a spurious ERROR from the telemetry poller.

Every ordinary ``MotionInterface.stop()`` used to print
``ERROR openmotion.sdk.Console - ValueError: Console controller not connected``:
``_drive_disconnecting`` flipped the state to DISCONNECTING *before* stopping
the telemetry poller, so a poll that was mid-command tripped the
``is_connected()`` guard in ``read_pdu_mon`` / ``get_lsync_pulsecount``, whose
``except ValueError`` logged at ERROR unconditionally (#128). No hardware needed.
"""

import importlib
import logging

import pytest

from omotion.MotionConsole import MotionConsole
from omotion.connection_state import ConnectionState

console_module = importlib.import_module("omotion.MotionConsole")


class _StubUart:
    """A UART that is never talked to: the guard must fire first."""
    demo_mode = False

    def send_packet(self, **_kwargs):
        raise AssertionError("send_packet must not be reached")

    def clear_buffer(self):
        pass

    def close(self):
        pass


def _console(state: ConnectionState) -> MotionConsole:
    console = MotionConsole(vid=0x0483, pid=0xA53E)
    console.uart = _StubUart()
    console._set_state(state, reason="test")
    return console


def test_drive_disconnecting_stops_telemetry_before_leaving_connected():
    console = MotionConsole(vid=0, pid=0, demo_mode=True)
    console._set_state(ConnectionState.CONNECTED, reason="test")
    seen: dict = {}

    def _stop():
        seen["state_at_stop"] = console.state
        seen["connected_at_stop"] = console.is_connected()

    console.telemetry.stop = _stop

    console._drive_disconnecting("user_stop")

    # The poller is joined while the console still reports CONNECTED, so a
    # poll that is mid-command finishes normally instead of hitting the
    # not-connected guard.
    assert seen["state_at_stop"] is ConnectionState.CONNECTED
    assert seen["connected_at_stop"] is True
    assert console.state is ConnectionState.DISCONNECTED


@pytest.mark.parametrize("method", ["read_pdu_mon", "get_lsync_pulsecount"])
def test_not_connected_guard_is_debug_while_disconnecting(caplog, method):
    console = _console(ConnectionState.DISCONNECTING)

    with caplog.at_level(logging.DEBUG, logger=console_module.logger.name):
        with pytest.raises(ValueError, match="not connected"):
            getattr(console, method)()

    records = [r for r in caplog.records if r.name == console_module.logger.name]
    assert records, "expected the guard to be logged"
    assert all(r.levelno < logging.ERROR for r in records), [
        r.getMessage() for r in records if r.levelno >= logging.ERROR
    ]
    assert any("during disconnect" in r.getMessage() for r in records)


@pytest.mark.parametrize("method", ["read_pdu_mon", "get_lsync_pulsecount"])
def test_not_connected_guard_stays_error_when_not_tearing_down(caplog, method):
    """Severity is only lowered for a device on its way out; a call while
    CONNECTING (or never connected) is still an unexpected error."""
    console = _console(ConnectionState.CONNECTING)

    with caplog.at_level(logging.DEBUG, logger=console_module.logger.name):
        with pytest.raises(ValueError, match="not connected"):
            getattr(console, method)()

    assert any(
        r.levelno == logging.ERROR and "not connected" in r.getMessage()
        for r in caplog.records
    )
