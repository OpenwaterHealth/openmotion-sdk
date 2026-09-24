"""Unit tests for MotionSensor factory-command error contracts.

Pins the established False-on-error convention that NvcmProgrammer's
_SensorI2CDriver depends on (``is False`` identity checks), and guards
against the phantom OW*Error exception types reappearing in docstrings —
those names were never defined anywhere in the package.
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from omotion.MotionSensor import MotionSensor
from omotion.config import OW_ERROR, OW_RESP

PHANTOM_EXCEPTIONS = ("OWNotConnectedError", "OWCommunicationError", "OWDeviceError")

FACTORY_METHODS = [
    "i2c_scan",
    "creset",
    "i2c_write",
    "i2c_read",
    "i2c_write_read",
    "i2c_read_register",
]


def _make_sensor(response):
    s = MotionSensor.__new__(MotionSensor)
    s._send = MagicMock(return_value=response)
    return s


def _resp(data=b"", packet_type=OW_RESP):
    data = bytes(data)
    return SimpleNamespace(packetType=packet_type, data=data, data_len=len(data))


def test_i2c_scan_success_returns_address_list():
    sensor = _make_sensor(_resp(b"\x29\x68"))
    assert sensor.i2c_scan() == [0x29, 0x68]


def test_i2c_scan_error_returns_false_identity():
    sensor = _make_sensor(_resp(packet_type=OW_ERROR))
    assert sensor.i2c_scan() is False


def test_creset_error_returns_false_identity():
    sensor = _make_sensor(_resp(packet_type=OW_ERROR))
    assert sensor.creset(True) is False


def test_i2c_write_error_returns_false_identity():
    sensor = _make_sensor(_resp(packet_type=OW_ERROR))
    assert sensor.i2c_write(0x40, b"\x01") is False


def test_i2c_read_error_returns_false_identity():
    sensor = _make_sensor(_resp(packet_type=OW_ERROR))
    assert sensor.i2c_read(0x40, 4) is False


@pytest.mark.parametrize("method_name", FACTORY_METHODS)
def test_docstrings_do_not_promise_undefined_exceptions(method_name):
    doc = getattr(MotionSensor, method_name).__doc__ or ""
    for phantom in PHANTOM_EXCEPTIONS:
        assert phantom not in doc, (
            f"{method_name} docstring promises {phantom}, which is not "
            "defined anywhere in the package"
        )
