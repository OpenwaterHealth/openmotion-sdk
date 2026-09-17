"""Unit tests for StreamInterface's TYPE_HISTO_CMP decompression helpers.

Pure software — synthesizes a compressed packet and round-trips it through
``_decompress_histo_cmp``. Regression for the missing ``import struct``:
every TYPE_HISTO_CMP packet raised NameError, which ``_process_packet``
logged as a generic "decompression FAILED" and silently dropped.
"""
import struct

from omotion.StreamInterface import (
    _decompress_histo_cmp,
    _rle_decompress,
    _util_crc16,
)
from omotion.config import TYPE_HISTO, TYPE_HISTO_CMP


def _rle_literal(data: bytes) -> bytes:
    """PackBits-style literal encoding: runs of <=128 literal bytes."""
    out = bytearray()
    for i in range(0, len(data), 128):
        chunk = data[i : i + 128]
        out.append(len(chunk) - 1)
        out += chunk
    return bytes(out)


def _build_cmp_packet(payload: bytes) -> bytes:
    """Build a TYPE_HISTO_CMP packet the way the firmware frames one.

    Layout: [SOF, type, size:4][compressed][UNCMP_CRC16][PKT_CRC16][0xDD],
    with both CRCs computed over range-minus-last-byte to match the
    firmware's (and _decompress_histo_cmp's) off-by-one convention.
    """
    compressed = _rle_literal(payload)
    uncmp_crc = struct.pack("<H", _util_crc16(payload[:-1]))
    total = 6 + len(compressed) + 2 + 3
    header = struct.pack("<BBI", 0xAA, TYPE_HISTO_CMP, total)
    body = header + compressed + uncmp_crc
    pkt_crc = struct.pack("<H", _util_crc16(body[:-1]))
    return body + pkt_crc + b"\xDD"


def test_rle_literal_roundtrip():
    data = bytes(range(64)) * 3
    assert _rle_decompress(_rle_literal(data)) == data


def test_decompress_histo_cmp_rebuilds_type_histo_packet():
    payload = bytes((i * 7) % 251 for i in range(300))
    rebuilt = _decompress_histo_cmp(_build_cmp_packet(payload))
    assert rebuilt[0] == 0xAA
    assert rebuilt[1] == TYPE_HISTO
    assert rebuilt[-1] == 0xDD
    assert rebuilt[6:-3] == payload
