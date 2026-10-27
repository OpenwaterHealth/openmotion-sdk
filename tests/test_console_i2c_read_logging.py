"""MotionConsole.read_i2c_packet must say which failure it was and which device.

Every failure returns the same ``(None, None)`` (kept for API compatibility),
so the log line is the only place the distinction survives - and it is
forwarded into the WI-00015 Procedures pane audit log (#263).
"""

import importlib
import logging
from types import SimpleNamespace

import pytest

from omotion.MotionConsole import OW_ERROR, MotionConsole

# The package re-exports the class under the module's name; go via importlib
# to reach the module (and its logger) rather than the class.
console_module = importlib.import_module("omotion.MotionConsole")


class _StubUart:
    demo_mode = False

    def __init__(self, response):
        self._response = response

    def send_packet(self, **_kwargs):
        return self._response

    def clear_buffer(self):
        pass


def _console(response):
    console = MotionConsole(vid=0x0483, pid=0xA53E)
    console.uart = _StubUart(response)
    return console


@pytest.mark.parametrize(
    ("response", "fragment"),
    [
        (None, "UART not open"),
        (SimpleNamespace(packetType=OW_ERROR, data=b"", data_len=0), "OW_ERROR"),
        (SimpleNamespace(packetType=0, data=b"", data_len=0), "no data"),
    ],
)
def test_read_i2c_packet_logs_the_failure_kind_with_the_device_location(
    caplog, response, fragment
):
    console = _console(response)

    with caplog.at_level(logging.ERROR, logger=console_module.logger.name):
        result = console.read_i2c_packet(
            mux_index=1, channel=4, device_addr=0x41, reg_addr=0x16, read_len=1
        )

    assert result == (None, None)
    assert any(
        fragment in record.getMessage()
        and "mux=1 ch=4 addr=0x41 reg=0x16 len=1" in record.getMessage()
        for record in caplog.records
    ), caplog.text


def test_read_i2c_packet_returns_data_unchanged_on_success():
    console = _console(SimpleNamespace(packetType=0, data=b"\x01", data_len=1))

    assert console.read_i2c_packet(
        mux_index=1, channel=4, device_addr=0x41, reg_addr=0x16, read_len=1
    ) == (b"\x01", 1)
