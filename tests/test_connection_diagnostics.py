"""Connection-state evidence for readiness timeouts (#263). No hardware."""

from omotion.MotionInterface import MotionInterface
from omotion.connection_state import ConnectionState

EXHAUSTED = (
    "connect_retry_exhausted:could not open port 'COM5': "
    "PermissionError(13, 'Access is denied.', None, 5)"
)


def test_handles_remember_the_reason_for_their_last_transition():
    interface = MotionInterface(demo_mode=True)
    assert interface.console.state_reason == ""
    assert interface.left.state_reason == ""

    interface.console._set_state(ConnectionState.CONNECTING, reason="poll_arrived")
    interface.console._set_state(ConnectionState.DISCONNECTED, reason=EXHAUSTED)
    interface.right._set_state(ConnectionState.CONNECTING, reason="poll_arrived")

    assert interface.console.state_reason == EXHAUSTED
    assert interface.right.state_reason == "poll_arrived"


def test_describe_connections_reports_state_reason_and_console_port(monkeypatch):
    interface = MotionInterface(demo_mode=True)
    monkeypatch.setattr(interface.console.uart, "find_port", lambda: "COM5")
    interface.console._set_state(ConnectionState.CONNECTING, reason="poll_arrived")
    interface.console._set_state(ConnectionState.DISCONNECTED, reason=EXHAUSTED)

    assert interface.describe_connections() == (
        f"console=DISCONNECTED ({EXHAUSTED}); "
        "left=DISCONNECTED (no transition yet); "
        "right=DISCONNECTED (no transition yet); "
        "console COM port: COM5"
    )


def test_describe_connections_never_raises_when_enumeration_fails(monkeypatch):
    interface = MotionInterface(demo_mode=True)

    def boom():
        raise RuntimeError("no serial backend")

    monkeypatch.setattr(interface.console.uart, "find_port", boom)

    assert "console COM port: enumeration failed (no serial backend)" in (
        interface.describe_connections()
    )
