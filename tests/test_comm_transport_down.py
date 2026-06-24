"""Unit tests for CommInterface transport-down cancellation.

Verifies that when the USB read loop signals the transport is down
(via the new ``_transport_down_evt``), any in-flight ``send_packet``
call unblocks immediately instead of spinning for the full ``timeout``
(which is 60 s for ``program_fpga`` / ``camera_configure_registers``
and was the root cause of bloodflow-app#130 — power-cycle during a
scan left the SDK's configure worker hung for 60 s, blocking the next
scan attempt).
"""

import threading
import time
from unittest.mock import MagicMock

import pytest

from omotion.CommInterface import CommInterface
from omotion.config import OW_CMD, OW_CMD_NOP


def _make_comm(*, async_mode: bool) -> CommInterface:
    """Build a CommInterface bypassing claim() — dev is a MagicMock so
    write() succeeds silently and the response stays absent."""
    dev = MagicMock()
    dev.write.return_value = 12
    ci = CommInterface(dev, interface_index=0, desc="TEST", async_mode=async_mode)
    ci.ep_out = MagicMock(bEndpointAddress=0x01)
    ci.ep_in = MagicMock(bEndpointAddress=0x81, wMaxPacketSize=64)
    return ci


def test_async_send_unblocks_on_transport_down():
    """A pending async send must raise within a few ms of the transport
    flag being set, not wait for the full 60 s timeout."""
    ci = _make_comm(async_mode=True)

    box: dict = {}

    def _do_send():
        t0 = time.monotonic()
        try:
            ci.send_packet(
                packetType=OW_CMD, command=OW_CMD_NOP, timeout=60,
            )
            box["result"] = "returned"
        except ConnectionError as e:
            box["err"] = e
        box["elapsed"] = time.monotonic() - t0

    t = threading.Thread(target=_do_send, daemon=True)
    t.start()

    # Let send_packet enter its wait loop.
    time.sleep(0.05)
    ci._transport_down_evt.set()

    t.join(timeout=2.0)
    assert not t.is_alive(), "send_packet did not unblock on transport_down"
    assert "err" in box, f"expected ConnectionError, got: {box}"
    assert box["elapsed"] < 1.0, (
        f"send_packet took {box['elapsed']:.2f}s — should have unblocked "
        "promptly on transport_down"
    )


def test_sync_send_unblocks_on_transport_down():
    """Same guarantee for sync (non-async) mode."""
    ci = _make_comm(async_mode=False)
    ci.receive = MagicMock(return_value=b"")  # no data arriving

    box: dict = {}

    def _do_send():
        t0 = time.monotonic()
        try:
            ci.send_packet(
                packetType=OW_CMD, command=OW_CMD_NOP, timeout=60,
            )
            box["result"] = "returned"
        except ConnectionError as e:
            box["err"] = e
        box["elapsed"] = time.monotonic() - t0

    t = threading.Thread(target=_do_send, daemon=True)
    t.start()

    time.sleep(0.05)
    ci._transport_down_evt.set()

    t.join(timeout=2.0)
    assert not t.is_alive(), "sync send_packet did not unblock on transport_down"
    assert "err" in box, f"expected ConnectionError, got: {box}"
    assert box["elapsed"] < 1.0, (
        f"sync send_packet took {box['elapsed']:.2f}s — should have unblocked "
        "promptly on transport_down"
    )


def test_start_read_thread_clears_transport_down():
    """Restarting the read thread (e.g. after reconnect) must clear the
    transport-down flag so subsequent sends are not poisoned."""
    ci = _make_comm(async_mode=True)
    ci._transport_down_evt.set()

    # Avoid spawning a real read thread (no real dev to read from); patch
    # the loop to exit immediately so start_read_thread just runs the
    # bookkeeping (clearing stop_event + transport_down_evt).
    ci._read_loop = lambda: None
    ci.start_read_thread()

    # Let the (no-op) read thread finish before we assert.
    if ci.read_thread is not None:
        ci.read_thread.join(timeout=1.0)
    assert not ci._transport_down_evt.is_set(), (
        "_transport_down_evt should be cleared when read thread (re)starts"
    )
