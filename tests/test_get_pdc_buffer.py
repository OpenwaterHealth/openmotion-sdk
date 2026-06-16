import struct
from unittest.mock import MagicMock

from omotion.MotionConsole import MotionConsole
from omotion.config import (
    OW_CONTROLLER, OW_CTRL_GET_PDC_BUFFER, OW_ERROR,
)


def _make_console_with_uart_response(payload: bytes):
    console = MotionConsole.__new__(MotionConsole)
    console.uart = MagicMock()

    response = MagicMock()
    response.packetType = 0  # OW_RESP
    response.data = payload
    response.data_len = len(payload)
    console.uart.send_packet.return_value = response
    console.uart.clear_buffer = MagicMock()
    return console


def test_get_pdc_buffer_parses_empty_response():
    # dropped=0, count=0, no tuples
    payload = struct.pack("<HB", 0, 0)
    console = _make_console_with_uart_response(payload)
    dropped, samples = console.get_pdc_buffer(max_samples=64)
    assert dropped == 0
    assert samples == []
    console.uart.send_packet.assert_called_once()
    _, kwargs = console.uart.send_packet.call_args
    assert kwargs["packetType"] == OW_CONTROLLER
    assert kwargs["command"] == OW_CTRL_GET_PDC_BUFFER
    assert kwargs["data"] == bytes([64])


def test_get_pdc_buffer_parses_three_samples():
    samples_raw = b"".join(
        struct.pack("<IHB", fid, raw, flags)
        for fid, raw, flags in [(100, 0x0123, 0x00), (101, 0x0456, 0x01), (102, 0x07AB, 0x00)]
    )
    payload = struct.pack("<HB", 0, 3) + samples_raw
    console = _make_console_with_uart_response(payload)
    dropped, samples = console.get_pdc_buffer(max_samples=64)
    assert dropped == 0
    assert samples == [
        (100, 0x0123, 0x00),
        (101, 0x0456, 0x01),
        (102, 0x07AB, 0x00),
    ]


def test_get_pdc_buffer_returns_empty_on_error():
    console = _make_console_with_uart_response(b"")
    console.uart.send_packet.return_value.packetType = OW_ERROR
    dropped, samples = console.get_pdc_buffer(max_samples=8)
    assert dropped == 0
    assert samples == []
