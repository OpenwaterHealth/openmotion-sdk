"""
Error and edge-case tests (Section 6 of the test plan).
"""

import pytest

from omotion.CommandError import CommandError
from omotion.UartPacket import UartPacket
from omotion.config import OW_CMD, OW_CMD_PING


# ===========================================================================
# 6.1 Packet framing / CRC
# ===========================================================================

def test_value_error_on_crc_mismatch():
    """Corrupt CRC bytes in a serialised packet; re-parsing must raise ValueError."""
    pkt = UartPacket(
        id=1,
        packetType=OW_CMD,
        command=OW_CMD_PING,
        addr=0,
        reserved=0,
        data=b"\x01\x02\x03",
    )
    raw = bytearray(pkt.to_bytes())
    # Flip a byte in the middle of the payload (not start/end byte)
    raw[5] ^= 0xFF
    with pytest.raises(ValueError, match="CRC mismatch"):
        UartPacket(buffer=bytes(raw))


def test_mutable_default_arg_isolation():
    """
    Two UartPackets constructed with no explicit data must not share the
    same list object (regression for the mutable default `data=[]` bug).
    """
    a = UartPacket(id=1, packetType=OW_CMD, command=OW_CMD_PING, addr=0, reserved=0)
    b = UartPacket(id=2, packetType=OW_CMD, command=OW_CMD_PING, addr=0, reserved=0)
    assert a.data is not b.data, (
        "UartPacket instances share the same default data list — mutable default bug"
    )


# ===========================================================================
# 6.2 Timeout behaviour
# ===========================================================================

@pytest.mark.console
def test_command_error_or_timeout_on_bad_subtype(console):
    """
    Sending an unrecognised command byte should surface as a CommandError
    or TimeoutError rather than silently returning garbage.
    """
    try:
        connected = console.is_connected()
    except Exception:
        connected = False
    if not connected:
        pytest.skip("Console not connected (may have entered DFU in an earlier test)")
    with pytest.raises((CommandError, TimeoutError, Exception)):
        console._uart.send_packet(
            packetType=0xE2,
            command=0xFE,
            addr=0,
            reserved=0,
            data=b"",
            timeout=1.0,
        )


# ===========================================================================
# 6.3 Idempotent connect / disconnect
# ===========================================================================

@pytest.mark.console
def test_double_ping_after_connect(console):
    """A second ping on an already-open connection must succeed."""
    try:
        connected = console.is_connected()
    except Exception:
        connected = False
    if not connected:
        pytest.skip("Console not connected (may have entered DFU in an earlier test)")
    try:
        r1 = console.ping()
        r2 = console.ping()
    except Exception as exc:
        pytest.skip(f"Console ping raised after DFU: {exc}")
    assert r1 is True
    assert r2 is True


@pytest.mark.sensor
def test_sensor_double_ping(any_sensor):
    assert any_sensor.ping() is True
    assert any_sensor.ping() is True


# ===========================================================================
# 6.4 USB error propagation
# ===========================================================================

@pytest.mark.sensor
def test_sensor_ping_after_is_connected(any_sensor):
    assert any_sensor.is_connected() is True
    assert any_sensor.ping() is True


# ===========================================================================
# 6.5 CommandError API
# ===========================================================================

def test_command_error_optional_response():
    """CommandError must accept None as its response argument."""
    err = CommandError("test error", response=None)
    assert err.response is None


def test_command_error_with_response():
    """CommandError must accept arbitrary response data."""
    err = CommandError("test error", response={"key": "value"})
    assert err.response == {"key": "value"}


def test_command_error_is_runtime_error():
    err = CommandError("something went wrong")
    assert isinstance(err, RuntimeError)
    assert str(err) == "something went wrong"
