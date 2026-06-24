"""Pure-software tests for omotion.framing and the unified async transport
(MotionUart over a fake serial). No hardware: runs under
``pytest -m "not console and not sensor and not destructive"``.
"""
import logging
import threading
import time

import pytest

from omotion.UartPacket import UartPacket
from omotion.MotionUart import MotionUart
from omotion.framing import (
    MAX_FRAME_DATA_LEN,
    emit_log_packet,
    extract_frame,
    is_log_packet,
)
from omotion.config import (
    OW_CMD,
    OW_CMD_ECHO,
    OW_CMD_PING,
    OW_DATA,
    OW_RESP,
    OW_START_BYTE,
)


def make_frame(data=b"", id=1, ptype=OW_RESP, cmd=OW_CMD_PING, addr=0, reserved=0):
    return UartPacket(
        id=id, packetType=ptype, command=cmd, addr=addr, reserved=reserved, data=data
    ).to_bytes()


def make_log_frame(text=b"hello\n"):
    """A firmware printf packet: id=0, OW_DATA, OW_CMD_ECHO."""
    return make_frame(data=text, id=0, ptype=OW_DATA, cmd=OW_CMD_ECHO)


# --------------------------------------------------------------------------- #
# extract_frame
# --------------------------------------------------------------------------- #

def test_extract_single_frame():
    frame = make_frame(b"abc")
    buf = bytearray(frame)
    assert extract_frame(buf) == frame
    assert len(buf) == 0


def test_extract_multiple_frames_back_to_back():
    f1 = make_frame(b"one", id=1)
    f2 = make_frame(b"two", id=2)
    buf = bytearray(f1 + f2)
    assert extract_frame(buf) == f1
    assert extract_frame(buf) == f2
    assert extract_frame(buf) is None
    assert len(buf) == 0


def test_extract_partial_frame_returns_none_then_completes():
    frame = make_frame(b"payload")
    buf = bytearray(frame[:6])
    assert extract_frame(buf) is None
    assert bytes(buf) == frame[:6]
    buf.extend(frame[6:])
    assert extract_frame(buf) == frame


def test_extract_skips_leading_garbage():
    frame = make_frame(b"x")
    buf = bytearray(b"\x00\x11\x22" + frame)
    assert extract_frame(buf) == frame
    assert len(buf) == 0


def test_extract_resyncs_past_false_start():
    false_start = bytes([OW_START_BYTE, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF])
    frame = make_frame(b"real")
    buf = bytearray(false_start + frame)
    assert extract_frame(buf) == frame
    assert len(buf) == 0


def test_extract_empty_buffer():
    assert extract_frame(bytearray()) is None


def test_extract_all_garbage_is_dropped():
    buf = bytearray(b"\x01\x02\x03\x04")
    assert extract_frame(buf) is None
    assert len(buf) == 0


def test_max_frame_data_len_is_generous():
    assert MAX_FRAME_DATA_LEN >= 8192


# --------------------------------------------------------------------------- #
# is_log_packet / emit_log_packet
# --------------------------------------------------------------------------- #

def test_is_log_packet_true_for_printf():
    assert is_log_packet(UartPacket(buffer=make_log_frame(b"hi"))) is True


@pytest.mark.parametrize(
    "frame",
    [
        make_frame(b"hi", id=1, ptype=OW_DATA, cmd=OW_CMD_ECHO),
        make_frame(b"hi", id=0, ptype=OW_RESP, cmd=OW_CMD_ECHO),
        make_frame(b"hi", id=0, ptype=OW_DATA, cmd=OW_CMD_PING),
    ],
)
def test_is_log_packet_false_for_responses(frame):
    assert is_log_packet(UartPacket(buffer=frame)) is False


def test_emit_log_packet_logs_text(caplog):
    pkt = UartPacket(buffer=make_log_frame(b"boot complete\r\n"))
    with caplog.at_level(logging.WARNING):
        emit_log_packet(pkt, "console")
    assert any(
        "PRINTF" in r.getMessage() and "boot complete" in r.getMessage()
        for r in caplog.records
    )


# --------------------------------------------------------------------------- #
# Unified async transport (MotionUart) over a fake serial
# --------------------------------------------------------------------------- #

class FakeSerial:
    """Threadsafe serial stand-in: read(1)/read_all() drain queued chunks."""

    def __init__(self, chunks=()):
        self._chunks = [bytes(c) for c in chunks]
        self._lock = threading.Lock()
        self.is_open = True

    def feed(self, data):
        with self._lock:
            self._chunks.append(bytes(data))

    def read(self, n=1):
        with self._lock:
            if self._chunks:
                return self._chunks.pop(0)
        time.sleep(0.005)
        return b""

    def read_all(self):
        with self._lock:
            if not self._chunks:
                return b""
            data = b"".join(self._chunks)
            self._chunks.clear()
            return data

    def write(self, data):
        return len(data)

    def reset_input_buffer(self):
        with self._lock:
            self._chunks.clear()

    def close(self):
        self.is_open = False


def make_async_uart(chunks=()):
    u = MotionUart(vid=0, pid=0, demo_mode=False)
    u.serial = FakeSerial(chunks)
    u.port = "FAKE"
    u.start_read_thread()
    return u


def test_transport_demuxes_log_and_returns_response(caplog):
    resp = make_frame(b"OK", id=42, cmd=OW_CMD_PING)
    log = make_log_frame(b"hello from fw\n")
    u = make_async_uart([log + resp])
    try:
        with caplog.at_level(logging.WARNING):
            r = u.send_packet(id=42, packetType=OW_CMD, command=OW_CMD_PING, timeout=2)
        assert r.id == 42
        assert bytes(r.data) == b"OK"
        assert any("PRINTF" in rec.getMessage() for rec in caplog.records)
    finally:
        u.stop_read_thread()


def test_transport_reassembles_fragments():
    resp = make_frame(b"payload-across-reads", id=7)
    u = make_async_uart([resp[:5]])
    try:
        u.serial.feed(resp[5:])
        r = u.send_packet(id=7, packetType=OW_CMD, command=OW_CMD_PING, timeout=2)
        assert r.id == 7
        assert bytes(r.data) == b"payload-across-reads"
    finally:
        u.stop_read_thread()


def test_transport_drops_bad_crc_then_returns_good():
    bad = bytearray(make_frame(b"bad", id=1))
    bad[10] ^= 0xFF  # corrupt the payload so CRC fails
    good = make_frame(b"good", id=2)
    u = make_async_uart([bytes(bad) + good])
    try:
        r = u.send_packet(id=2, packetType=OW_CMD, command=OW_CMD_PING, timeout=2)
        assert r.id == 2
        assert bytes(r.data) == b"good"
    finally:
        u.stop_read_thread()


def test_transport_ignores_stale_id_and_times_out():
    # A response with the wrong id must not satisfy a different request.
    other = make_frame(b"x", id=99)
    u = make_async_uart([other])
    try:
        with pytest.raises(TimeoutError):
            u.send_packet(id=1, packetType=OW_CMD, command=OW_CMD_PING, timeout=0.4)
    finally:
        u.stop_read_thread()


def test_transport_send_unblocks_on_transport_down():
    u = make_async_uart([])
    try:
        def trip():
            time.sleep(0.05)
            u._transport_down_evt.set()

        threading.Thread(target=trip, daemon=True).start()
        t0 = time.monotonic()
        with pytest.raises(ConnectionError):
            u.send_packet(id=1, packetType=OW_CMD, command=OW_CMD_PING, timeout=5)
        assert time.monotonic() - t0 < 1.0
    finally:
        u.stop_read_thread()


def test_demo_mode_send_returns_without_serial():
    u = MotionUart(vid=0, pid=0, demo_mode=True)
    r = u.send_packet(packetType=OW_CMD, command=OW_CMD_PING)
    assert r is not None  # demo short-circuit, never touches a port
