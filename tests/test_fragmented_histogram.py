"""Preserve complete histogram packets across arbitrary USB read boundaries."""

import queue
import struct
import threading

import numpy as np
import pytest

from omotion import MotionProcessing as wire


def packet() -> bytes:
    """Build a checksum-valid histogram whose payload also resembles a packet header."""
    payload = bytearray(wire.HISTOGRAM_BYTES)
    payload[100:106] = struct.pack('<BBI', wire.SOF, wire.TYPE_HISTO_CMP, 12)
    histogram = np.frombuffer(payload, dtype='<u4').copy()
    histogram[0] = wire.EXPECTED_HISTOGRAM_SUM - int(histogram.sum())
    histogram[-1] = 1 << 24
    body = (struct.pack('<I', 1000) + bytes([wire.SOH, 0]) + histogram.tobytes()
            + struct.pack('<f', 25.0) + bytes([wire.EOH]))
    prefix = struct.pack('<BBI', wire.SOF, wire.TYPE_HISTO, len(body) + 9) + body
    return prefix + struct.pack('<H', wire._crc16(prefix[:-1])) + bytes([wire.EOF])


def parse(chunks: list[bytes]) -> list[wire.HistogramPacket]:
    """Drain queued USB fragments with the real parser, without hardware."""
    pending = queue.Queue()
    for chunk in chunks:
        pending.put_nowait(chunk)
    stopped = threading.Event()
    stopped.set()
    packets = []
    wire.parse_histogram_stream(pending, stopped, bytearray(), on_packet_fn=packets.append)
    return packets


@pytest.mark.parametrize('split', [1, 5, 6, 9, 112, 512, -1])
def test_fragmented_packet_never_resyncs_into_its_payload(split: int) -> None:
    """A bounded valid header waits for its payload instead of discarding it."""
    complete = packet()
    expected = wire.parse_histogram_packet_structured(memoryview(complete))
    packets = parse([complete[:split], complete[split:]])
    assert len(packets) == 1
    assert packets[0].bytes_consumed == len(complete)
    assert packets[0].samples[0].frame_id == 1
    np.testing.assert_array_equal(packets[0].samples[0].histogram, expected.samples[0].histogram)


def test_final_flush_recovers_after_a_truncated_corrupt_header() -> None:
    """At EOF, recover a complete later packet rather than waiting for impossible bytes."""
    truncated = struct.pack('<BBI', wire.SOF, wire.TYPE_HISTO, wire.MAX_PACKET_SIZE) + bytes(20)
    packets = parse([truncated + packet()])
    assert len(packets) == 1
    assert packets[0].samples[0].frame_id == 1


def test_oversized_corrupt_header_does_not_block_the_next_packet() -> None:
    """An impossible declared packet size must not defer resynchronization."""
    corrupt = struct.pack('<BBI', wire.SOF, wire.TYPE_HISTO, wire.MAX_PACKET_SIZE + 1) + bytes(20)
    packets = parse([corrupt + packet()])
    assert len(packets) == 1
    assert packets[0].samples[0].frame_id == 1
