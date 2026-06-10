"""Unit tests for i2c_parser readback verification and polling loops.

Regression tests for the NVCM dropped-row incident (2026-06-09): the
Python port of the Lattice ispVME I2C parser originally skipped all
TDO/DTDO comparison, which (a) made VERIFY phases meaningless and
(b) made LSC_CHECK_BUSY polling loops exit after one ignored read,
violating the NVCM row-program busy window and silently corrupting
one-time-programmable memory.

The micro-algorithms below are hand-assembled .iea byte streams using
the opcode constants from the parser itself.
"""

import pytest

from omotion.i2c_parser import (
    I2CDriver,
    I2CParser,
    ERR_VERIFY_FAIL,
    I2C_STARTTRAN,
    I2C_ENDTRAN,
    I2C_TRANSIN,
    I2C_TDO,
    I2C_MASK,
    I2C_CONTINUE,
    I2C_LOOP,
    I2C_ENDLOOP,
    I2C_ENDVME,
)


class ScriptedDriver(I2CDriver):
    """Hardware-style driver that returns scripted bytes for reads."""

    def __init__(self, reads):
        self._reads = list(reads)
        self.read_count = 0

    def is_simulation(self) -> bool:
        return False

    def start(self):
        pass

    def restart(self):
        pass

    def stop(self):
        pass

    def write(self, data: bytes):
        pass

    def read(self, num_bytes: int) -> bytes:
        self.read_count += 1
        if self._reads:
            data = self._reads.pop(0)
        else:
            data = bytes([0xFF] * num_bytes)
        return data[:num_bytes]


def transin_expecting(expected: bytes, mask: bytes | None = None) -> bytes:
    """Assemble a TRANSIN opcode block expecting `expected` (8*N bits)."""
    body = bytes([I2C_TRANSIN, len(expected) * 8, I2C_TDO]) + expected
    if mask is not None:
        body += bytes([I2C_MASK]) + mask
    body += bytes([I2C_CONTINUE])
    return body


def run_algo(algo_body: bytes, driver: I2CDriver) -> int:
    parser = I2CParser(algo_body + bytes([I2C_ENDVME]), b"\x00", driver=driver)
    return parser.ispProcessI2C()


def test_transin_match_passes():
    algo = transin_expecting(bytes([0x01, 0x2C, 0x00, 0x43]))
    drv = ScriptedDriver([bytes([0x01, 0x2C, 0x00, 0x43])])
    assert run_algo(algo, drv) == 0


def test_transin_mismatch_fails():
    algo = transin_expecting(bytes([0x01, 0x2C, 0x00, 0x43]))
    drv = ScriptedDriver([bytes([0x82, 0x91, 0x15, 0xF8])])
    assert run_algo(algo, drv) == ERR_VERIFY_FAIL


def test_transin_mask_ignores_masked_bits():
    # Expect 0x00 in bit7 only; actual has other bits set but bit7 clear.
    algo = transin_expecting(bytes([0x00]), mask=bytes([0x80]))
    drv = ScriptedDriver([bytes([0x7F])])
    assert run_algo(algo, drv) == 0


def test_transin_mask_catches_masked_mismatch():
    algo = transin_expecting(bytes([0x00]), mask=bytes([0x80]))
    drv = ScriptedDriver([bytes([0x80])])  # busy bit still set
    assert run_algo(algo, drv) == ERR_VERIFY_FAIL


def test_polling_loop_retries_until_match():
    """Busy-poll: device busy (0x80) twice, then ready (0x00)."""
    body = (bytes([I2C_STARTTRAN])
            + transin_expecting(bytes([0x00]), mask=bytes([0x80]))
            + bytes([I2C_ENDTRAN]))
    algo = bytes([I2C_LOOP, 10]) + body + bytes([I2C_ENDLOOP])
    drv = ScriptedDriver([bytes([0x80]), bytes([0x80]), bytes([0x00])])
    assert run_algo(algo, drv) == 0
    assert drv.read_count == 3  # polled until not-busy


def test_polling_loop_exhaustion_fails():
    """Device never ready: loop must report failure, not fall through."""
    body = (bytes([I2C_STARTTRAN])
            + transin_expecting(bytes([0x00]), mask=bytes([0x80]))
            + bytes([I2C_ENDTRAN]))
    algo = bytes([I2C_LOOP, 3]) + body + bytes([I2C_ENDLOOP])
    drv = ScriptedDriver([bytes([0x80])] * 5)
    assert run_algo(algo, drv) == ERR_VERIFY_FAIL
    assert drv.read_count == 3  # bounded by loop count


def test_simulation_driver_skips_verification():
    """Default driver (simulation) must keep passing regardless of data."""
    algo = transin_expecting(bytes([0x01, 0x2C, 0x00, 0x43]))
    parser = I2CParser(algo + bytes([I2C_ENDVME]), b"\x00")  # default driver
    assert parser.ispProcessI2C() == 0
