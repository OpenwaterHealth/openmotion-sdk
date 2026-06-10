"""Unit tests for omotion.NvcmProgrammer (no hardware).

Uses hand-assembled micro .iea/.ied files (same technique as
test_i2c_parser_verify.py) against a scripted mock sensor.
"""

import pytest

from omotion.NvcmProgrammer import (
    NvcmProgrammer,
    NvcmResult,
    DEFAULT_ALGO_PATH,
    DEFAULT_DATA_PATH,
)
from omotion.i2c_parser import (
    I2C_STARTTRAN, I2C_RESTARTTRAN, I2C_ENDTRAN, I2C_TRANSOUT, I2C_TRANSIN,
    I2C_TDI, I2C_TDO, I2C_CONTINUE, I2C_TRST, I2C_ENDVME,
)

VERSION = b"_I2C1.0"


class MockSensor:
    """Records every call; serves scripted bytes for write-reads."""

    def __init__(self, read_results=None):
        self.calls = []
        self._reads = list(read_results or [])

    def enable_camera_power(self, mask):
        self.calls.append(("enable_camera_power", mask))
        return True

    def switch_camera(self, camera_id):
        self.calls.append(("switch_camera", camera_id))
        return True

    def creset(self, state):
        self.calls.append(("creset", state))
        return 1 if state else 0

    def i2c_write(self, addr, data):
        self.calls.append(("i2c_write", addr, bytes(data)))

    def i2c_write_read(self, addr, data, read_len):
        self.calls.append(("i2c_write_read", addr, bytes(data), read_len))
        if self._reads:
            return self._reads.pop(0)
        return bytes([0xFF] * read_len)

    def i2c_read(self, addr, read_len):
        self.calls.append(("i2c_read", addr, read_len))
        if self._reads:
            return self._reads.pop(0)
        return bytes([0xFF] * read_len)


def transout(payload: bytes) -> bytes:
    return bytes([I2C_TRANSOUT, len(payload) * 8, I2C_TDI]) + payload + bytes([I2C_CONTINUE])


def transin(expected: bytes) -> bytes:
    return bytes([I2C_TRANSIN, len(expected) * 8, I2C_TDO]) + expected + bytes([I2C_CONTINUE])


def micro_files(tmp_path, expected_idcode=b"\x01\x2c\x00\x43"):
    """Algo: creset low, one pure write, one write+read of 4 bytes."""
    algo = bytearray()
    algo += VERSION
    algo += bytes([I2C_TRST, 0x00])                       # creset low
    # pure write: START, addr 0x80, payload C6 02 00 00, STOP
    algo += bytes([I2C_STARTTRAN])
    algo += transout(b"\x80")
    algo += transout(b"\xc6\x02\x00\x00")
    algo += bytes([I2C_ENDTRAN])
    # write-then-read: START, addr, cmd E0, RESTART, read-addr, read 4 expect idcode
    algo += bytes([I2C_STARTTRAN])
    algo += transout(b"\x80")
    algo += transout(b"\xe0\x00\x00\x00")
    algo += bytes([I2C_RESTARTTRAN])
    algo += transout(b"\x81")
    algo += transin(expected_idcode)
    algo += bytes([I2C_ENDTRAN])
    algo += bytes([I2C_ENDVME])
    algo_p = tmp_path / "t.iea"
    data_p = tmp_path / "t.ied"
    algo_p.write_bytes(bytes(algo))
    data_p.write_bytes(b"\x00")  # compress flag only
    return str(algo_p), str(data_p)


def test_default_files_exist_and_parse():
    assert DEFAULT_ALGO_PATH.is_file()
    assert DEFAULT_DATA_PATH.is_file()
    assert DEFAULT_ALGO_PATH.read_bytes()[:7] == VERSION


def test_burn_happy_path_dispatches_and_succeeds(tmp_path):
    algo, data = micro_files(tmp_path)
    sensor = MockSensor(read_results=[b"\x01\x2c\x00\x43"])
    res = NvcmProgrammer(sensor).burn(3, algo_path=algo, data_path=data)
    assert isinstance(res, NvcmResult)
    assert res.success is True
    assert res.error is None
    assert ("enable_camera_power", 0x04) in sensor.calls
    assert ("switch_camera", 2) in sensor.calls          # camera 3 -> id 2
    assert ("i2c_write", 0x40, b"\xc6\x02\x00\x00") in sensor.calls
    assert ("i2c_write_read", 0x40, b"\xe0\x00\x00\x00", 4) in sensor.calls


def test_burn_verify_failure_maps_error(tmp_path):
    algo, data = micro_files(tmp_path)
    sensor = MockSensor(read_results=[b"\x82\x91\x15\xf8"])  # wrong idcode
    res = NvcmProgrammer(sensor).burn(3, algo_path=algo, data_path=data)
    assert res.success is False
    assert "VERIFY FAIL" in res.error


def test_progress_callback_monotonic_and_complete(tmp_path):
    algo, data = micro_files(tmp_path)
    sensor = MockSensor(read_results=[b"\x01\x2c\x00\x43"])
    seen = []
    res = NvcmProgrammer(sensor).burn(
        3, algo_path=algo, data_path=data,
        progress_cb=lambda done, total: seen.append((done, total)))
    assert res.success
    assert seen, "progress callback never fired"
    totals = {t for _, t in seen}
    assert len(totals) == 1
    dones = [d for d, _ in seen]
    assert dones == sorted(dones)
    assert seen[-1][0] == seen[-1][1]                    # ends at 100%


def test_invalid_camera_rejected():
    with pytest.raises(ValueError):
        NvcmProgrammer(MockSensor()).burn(0)
    with pytest.raises(ValueError):
        NvcmProgrammer(MockSensor()).burn(9)
