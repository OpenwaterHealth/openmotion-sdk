"""Unit tests for MotionConsole command-method return/raise contracts.

Constructs MotionConsole without UART enumeration (``__new__``) and stubs
``self.uart`` — the same no-hardware pattern as test_i2c_read_register.py.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from omotion.CommandError import CommandError
from omotion.MotionConsole import MotionConsole
from omotion.config import OW_RESP
from omotion.connection_state import ConnectionState


def _console(uart) -> MotionConsole:
    c = MotionConsole.__new__(MotionConsole)
    c.uart = uart
    c._state = ConnectionState.CONNECTED
    c.is_connected = lambda: True
    return c


def test_tec_status_demo_mode_returns_strings():
    """Demo mode must return the same shape as the real path, which
    formats the four readings as f"{v:.6f}" strings."""
    c = _console(SimpleNamespace(demo_mode=True))
    volt, temp_set, tec_curr, tec_volt, tec_good = c.tec_status()
    for value in (volt, temp_set, tec_curr, tec_volt):
        assert isinstance(value, str)
    assert isinstance(tec_good, bool)


def test_fpga_prog_read_status_demo_mode_returns_int():
    """Callers bit-shift the returned status; demo mode returning None
    breaks every (sr >> n) & 1 check."""
    c = _console(SimpleNamespace(demo_mode=True))
    sr = c.fpga_prog_read_status(0)
    assert isinstance(sr, int)


def test_fpga_prog_read_status_short_response_raises_command_error():
    """A wrong-length response must surface as CommandError, not the
    TypeError from ValueError's unsupported response= kwarg."""
    r = SimpleNamespace(packetType=OW_RESP, data=b"\x01\x02")
    uart = SimpleNamespace(
        demo_mode=False,
        send_packet=MagicMock(return_value=r),
        clear_buffer=lambda: None,
    )
    c = _console(uart)
    with pytest.raises(CommandError):
        c.fpga_prog_read_status(0)


def test_get_lsync_pulsecount_reraises_transport_errors():
    """Must match get_fsync_pulsecount: log then re-raise, never swallow
    the exception and fall through to an implicit None."""
    uart = SimpleNamespace(
        demo_mode=False,
        send_packet=MagicMock(side_effect=RuntimeError("uart torn down")),
        clear_buffer=lambda: None,
    )
    c = _console(uart)
    with pytest.raises(RuntimeError):
        c.get_lsync_pulsecount()


def test_get_fsync_pulsecount_reraises_transport_errors():
    """Behavior pin for the sibling method lsync must match."""
    uart = SimpleNamespace(
        demo_mode=False,
        send_packet=MagicMock(side_effect=RuntimeError("uart torn down")),
        clear_buffer=lambda: None,
    )
    c = _console(uart)
    with pytest.raises(RuntimeError):
        c.get_fsync_pulsecount()


def test_read_pdu_mon_does_not_print_packet():
    """Every sibling command method has its r.print_packet() commented
    out; read_pdu_mon must not spam stdout on every poll."""
    r = SimpleNamespace(
        packetType=OW_RESP,
        data=bytes(96),
        data_len=96,
        print_packet=MagicMock(),
    )
    uart = SimpleNamespace(
        demo_mode=False,
        send_packet=MagicMock(return_value=r),
        clear_buffer=lambda: None,
    )
    c = _console(uart)
    pdu = c.read_pdu_mon()
    r.print_packet.assert_not_called()
    assert pdu is not None
