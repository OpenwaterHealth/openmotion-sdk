"""Unit tests for MotionSensor.i2c_read_register payload encoding.

These construct a MotionSensor without USB enumeration and mock ``_send`` so
they run with no hardware.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from omotion.MotionSensor import MotionSensor
from omotion.config import OW_CMD, OW_CMD_I2C_REG_READ, OW_RESP, OW_ERROR


def _make_sensor(response):
    s = MotionSensor.__new__(MotionSensor)
    s._send = MagicMock(return_value=response)
    return s


def _resp(data=b"", packet_type=OW_RESP):
    data = bytes(data)
    return SimpleNamespace(packetType=packet_type, data=data, data_len=len(data))


def test_read_8bit_no_mux_encoding():
    sensor = _make_sensor(_resp(b"\xEA"))
    result = sensor.i2c_read_register(0x68, 0x00)
    assert result == b"\xEA"
    kwargs = sensor._send.call_args.kwargs
    assert kwargs["packetType"] == OW_CMD
    assert kwargs["command"] == OW_CMD_I2C_REG_READ
    assert bytes(kwargs["data"]) == bytes([0x68, 0x01, 0xFF, 0x00, 0x00, 0x00, 0x01])


def test_read_16bit_addr_encoding():
    sensor = _make_sensor(_resp(b"\x12\x34"))
    sensor.i2c_read_register(0x40, 0x1234, read_len=2, reg_addr_size=2)
    data = bytes(sensor._send.call_args.kwargs["data"])
    assert data == bytes([0x40, 0x02, 0xFF, 0x12, 0x34, 0x00, 0x02])


def test_mux_channel_encoded_in_byte2():
    sensor = _make_sensor(_resp(b"\x00"))
    sensor.i2c_read_register(0x50, 0x05, mux_channel=3)
    data = bytes(sensor._send.call_args.kwargs["data"])
    # Full-payload assertion: mux byte plus all other fields, so a byte
    # transpose elsewhere is also caught.
    assert data == bytes([0x50, 0x01, 0x03, 0x00, 0x05, 0x00, 0x01])


def test_read_len_256_high_byte():
    """The boundary value 256 must encode big-endian as [0x01, 0x00]."""
    sensor = _make_sensor(_resp(b"\x00" * 256))
    sensor.i2c_read_register(0x40, 0x00, read_len=256)
    data = bytes(sensor._send.call_args.kwargs["data"])
    assert data[5] == 0x01 and data[6] == 0x00


def test_error_response_returns_false():
    sensor = _make_sensor(_resp(b"", packet_type=OW_ERROR))
    assert sensor.i2c_read_register(0x68, 0x00) is False


@pytest.mark.parametrize("kwargs", [
    {"dev_addr": 0x80, "reg_addr": 0x00},                       # dev_addr too big
    {"dev_addr": 0x68, "reg_addr": 0x00, "reg_addr_size": 3},   # bad size
    {"dev_addr": 0x68, "reg_addr": 0x100, "reg_addr_size": 1},  # reg too big for 8-bit
    {"dev_addr": 0x68, "reg_addr": 0x00, "read_len": 0},        # read_len 0
    {"dev_addr": 0x68, "reg_addr": 0x00, "read_len": 257},      # read_len too big
    {"dev_addr": 0x68, "reg_addr": 0x00, "mux_channel": 8},     # mux out of range
])
def test_validation_raises(kwargs):
    sensor = _make_sensor(_resp(b"\x00"))
    with pytest.raises(ValueError):
        sensor.i2c_read_register(**kwargs)
