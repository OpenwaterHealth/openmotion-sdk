# tests/test_uart_cancel.py
import threading
import time

import pytest

from omotion.MotionUart import MotionUart
from omotion.CommandError import CommandError


class _FakeSerial:
    """A serial port that is open but never returns data."""
    is_open = True

    def read_all(self):
        return b""

    def close(self):
        self.is_open = False


def _uart_with_fake():
    u = MotionUart(vid=0x0483, pid=0xA53E, timeout=20)
    u.serial = _FakeSerial()
    return u


def test_read_packet_aborts_on_cancel_event():
    u = _uart_with_fake()
    cancel = threading.Event()

    # Cancel after 200 ms from another thread.
    threading.Timer(0.2, cancel.set).start()

    start = time.monotonic()
    with pytest.raises(CommandError):
        u.read_packet(timeout=20, cancel_evt=cancel)
    elapsed = time.monotonic() - start
    # Must abort within ~1 s, NOT wait the full 20 s timeout.
    assert elapsed < 1.0, f"read_packet did not abort promptly ({elapsed:.1f}s)"


def test_read_packet_still_times_out_without_cancel():
    u = _uart_with_fake()
    start = time.monotonic()
    with pytest.raises(ValueError):
        u.read_packet(timeout=0.5)
    assert 0.4 < time.monotonic() - start < 2.0
