"""Pure-software tests for omotion.framing and the MotionUart printf demux.

No hardware: runs under ``pytest -m "not console and not sensor and not destructive"``.
"""
import logging

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
    out = extract_frame(buf)
    assert out == frame
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
    assert extract_frame(buf) is None       # not enough bytes yet
    assert bytes(buf) == frame[:6]          # held intact for next read
    buf.extend(frame[6:])
    assert extract_frame(buf) == frame


def test_extract_skips_leading_garbage():
    frame = make_frame(b"x")
    buf = bytearray(b"\x00\x11\x22" + frame)
    assert extract_frame(buf) == frame
    assert len(buf) == 0


def test_extract_resyncs_past_false_start():
    # A 0xAA whose declared length is implausible (0xFFFF > MAX) must be
    # discarded so the real frame that follows is still found.
    false_start = bytes([OW_START_BYTE, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF])
    frame = make_frame(b"real")
    buf = bytearray(false_start + frame)
    assert extract_frame(buf) == frame
    assert len(buf) == 0


def test_extract_empty_buffer():
    buf = bytearray()
    assert extract_frame(buf) is None


def test_extract_all_garbage_is_dropped():
    buf = bytearray(b"\x01\x02\x03\x04")
    assert extract_frame(buf) is None
    assert len(buf) == 0  # nothing salvageable — buffer cleared


def test_max_frame_data_len_is_generous():
    # Must cover both firmwares (sensor up to 8192) so the shared extractor is
    # safe for the async path too.
    assert MAX_FRAME_DATA_LEN >= 8192


# --------------------------------------------------------------------------- #
# is_log_packet / emit_log_packet
# --------------------------------------------------------------------------- #

def test_is_log_packet_true_for_printf():
    pkt = UartPacket(buffer=make_log_frame(b"hi"))
    assert is_log_packet(pkt) is True


@pytest.mark.parametrize(
    "frame",
    [
        make_frame(b"hi", id=1, ptype=OW_DATA, cmd=OW_CMD_ECHO),   # id != 0
        make_frame(b"hi", id=0, ptype=OW_RESP, cmd=OW_CMD_ECHO),   # wrong type
        make_frame(b"hi", id=0, ptype=OW_DATA, cmd=OW_CMD_PING),   # wrong cmd
    ],
)
def test_is_log_packet_false_for_responses(frame):
    assert is_log_packet(UartPacket(buffer=frame)) is False


def test_emit_log_packet_logs_text(caplog):
    pkt = UartPacket(buffer=make_log_frame(b"boot complete\r\n"))
    with caplog.at_level(logging.WARNING):
        emit_log_packet(pkt, "console")
    msgs = [r.getMessage() for r in caplog.records]
    assert any("PRINTF" in m and "boot complete" in m for m in msgs)


# --------------------------------------------------------------------------- #
# MotionUart.read_packet — demuxes log packets out of the response stream
# --------------------------------------------------------------------------- #

class FakeSerial:
    """Minimal serial stand-in: read_all() drains queued chunks."""

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.is_open = True

    def read_all(self):
        return self._chunks.pop(0) if self._chunks else b""

    def reset_input_buffer(self):
        pass


def make_uart(chunks):
    u = MotionUart(vid=0, pid=0, demo_mode=False)
    u.serial = FakeSerial(chunks)
    return u


def test_read_packet_skips_log_and_returns_response(caplog):
    log = make_log_frame(b"some printf\n")
    resp = make_frame(b"OK", id=7, cmd=OW_CMD_PING)
    uart = make_uart([log + resp])
    with caplog.at_level(logging.WARNING):
        pkt = uart.read_packet(timeout=2)
    assert pkt.id == 7
    assert bytes(pkt.data) == b"OK"
    assert any("PRINTF" in r.getMessage() for r in caplog.records)


def test_read_packet_reassembles_fragments():
    resp = make_frame(b"payload-across-reads", id=3)
    uart = make_uart([resp[:5], resp[5:]])
    pkt = uart.read_packet(timeout=2)
    assert pkt.id == 3
    assert bytes(pkt.data) == b"payload-across-reads"


def test_read_packet_times_out_with_no_data():
    uart = make_uart([])  # read_all always returns b""
    with pytest.raises(ValueError):
        uart.read_packet(timeout=0.2)


def test_read_packet_drops_bad_crc_then_returns_good():
    bad = bytearray(make_frame(b"bad", id=1))
    bad[10] ^= 0xFF  # corrupt a CRC byte
    good = make_frame(b"good", id=2)
    uart = make_uart([bytes(bad) + good])
    pkt = uart.read_packet(timeout=2)
    assert pkt.id == 2
    assert bytes(pkt.data) == b"good"
