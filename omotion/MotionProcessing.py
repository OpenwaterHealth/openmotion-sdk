"""
omotion.MotionProcessing — packet parsing shim (post-Phase-F).

This module is now a **thin shim**: it provides the wire-level histogram
packet parser, the public dataclasses (Sample, CorrectedBatch,
HistogramSample, HistogramPacket), and the shared constants.

All science-pipeline logic (BFI/BVI computation, dark-frame correction,
SciencePipeline class, FrameIdUnwrapper) has moved to the
``omotion.pipeline`` package (Phase E/F, commit 0ac12f1 and later).

What lives here
---------------
- Wire-level constants: HISTO_SIZE_WORDS, HISTOGRAM_BYTES, PACKET_*,
  MIN_*_PACKET_SIZE, MAX_PACKET_SIZE, SOF/SOH/EOH/EOF, FRAME_ID_MODULUS,
  EXPECTED_HISTOGRAM_SUM, PEDESTAL_HEIGHT
- Dataclasses: HistogramSample, HistogramPacket, Sample, CorrectedBatch
- Parsing helpers: parse_histogram_stream, parse_histogram_packet_structured,
  _rle_decompress (re-exported from omotion.utils), _util_crc16
- Legacy byte-level helper: bytes_to_integers (used by MotionSensor)
- File conversion helper: process_bin_file

Camera-array constants (CAMERA_GAIN_MAP, HISTO_BINS, HISTO_BINS_SQ) live in
omotion/config.py. ADC_GAIN is pedestal-derived — see
omotion.pipeline.pedestal.adc_gain_for_pedestal.
"""

import csv
import logging
import struct
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np
import queue
import threading
from omotion import _log_root
from omotion.config import TYPE_HISTO, TYPE_HISTO_CMP, CMP_UNCMP_CRC_SIZE
from omotion.utils import rle_decompress as _rle_decompress

import binascii as _binascii


def _crc16(buf) -> int:
    """CRC-CCITT (polynomial 0x1021, init 0xFFFF) via the C implementation in binascii."""
    return _binascii.crc_hqx(buf, 0xFFFF)


# Public alias used by the shim test (and any external code).
_util_crc16 = _crc16


# ---------------------------------------------------------------------------
# Histogram payload constants
# ---------------------------------------------------------------------------

HISTO_SIZE_WORDS = 1024
HISTOGRAM_BYTES = HISTO_SIZE_WORDS * 4  # 4096
PACKET_HEADER_SIZE = 6
PACKET_FOOTER_SIZE = 3
HISTO_BLOCK_SIZE = 1 + 1 + HISTOGRAM_BYTES + 4 + 1  # SOH + cam + histo + temp + EOH
TIMESTAMP_SIZE = 4
MIN_PACKET_ENVELOPE_SIZE = PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE
MIN_HISTO_PACKET_SIZE = PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE + HISTO_BLOCK_SIZE
MIN_PACKET_SIZE = MIN_HISTO_PACKET_SIZE

# TIM5 on the sensor MCU runs at 100 kHz; get_timestamp_ms() returns TIM5->CNT/100
# so the 32-bit counter wraps every 2^32 / 100 / 1000 ≈ 42949.67 seconds (~11.9 hours).
_TIMESTAMP_ROLLOVER_S: float = (2**32) / 100.0 / 1000.0
# TYPE_HISTO_CMP has: header + compressed_payload(>=1) + uncmp_crc16(2) + footer(3)
MIN_HISTO_CMP_PACKET_SIZE = PACKET_HEADER_SIZE + 1 + CMP_UNCMP_CRC_SIZE + PACKET_FOOTER_SIZE
MAX_PACKET_SIZE = 32837

SOF, SOH, EOH, EOF = 0xAA, 0xFF, 0xEE, 0xDD

# Frame ID rollover constants. The firmware packs frame_id into the high byte
# of the last histogram word, so it is an 8-bit counter (0–255).  We detect
# a forward wrap whenever the apparent backward delta would exceed this limit.
FRAME_ID_MODULUS = 256
FRAME_ROLLOVER_THRESHOLD = 128

# Expected sum of all histogram bins for a valid frame.
# When this is not None, any parsed histogram whose bin sum differs from this
# value is treated as corrupt and silently dropped from the sample list.
# Set to the integer value confirmed during calibration; leave as None to
# disable the check (e.g. during development before the expected value is
# known).
EXPECTED_HISTOGRAM_SUM: int | None = 2_457_606

# Camera sensor pedestal height.
# When no light reaches the sensor the pixel ADC output settles at this
# offset rather than zero.  The pedestal is a fixed DC bias present in every
# frame — bright frames and dark frames alike — so it cancels automatically
# in the dark-corrected stream (corrected_mean = fm.u1 − dark_u1 removes it
# from both terms).  For the uncorrected real-time stream we subtract it
# explicitly before emitting the mean so that downstream consumers see a
# zero-referenced signal.
PEDESTAL_HEIGHT: float = 64.0

logger = logging.getLogger(
    f"{_log_root}.MotionProcessing" if _log_root else "MotionProcessing"
)

# Struct formats
_U16 = struct.Struct("<H")
_U32 = struct.Struct("<I")
_F32 = struct.Struct("<f")
_HDR = struct.Struct("<BBI")
_BLK_HEAD = struct.Struct("<BB")


# ---------------------------------------------------------------------------
# Wire-level data structures
# ---------------------------------------------------------------------------

@dataclass
class HistogramSample:
    cam_id: int
    frame_id: int          # raw u8 from the wire (0–255)
    timestamp_s: float
    histogram: np.ndarray
    temperature_c: float
    row_sum: int

    def to_csv_row(self, extra_cols: list | None = None) -> list:
        return [
            self.cam_id,
            self.frame_id,
            self.timestamp_s,
            *self.histogram.tolist(),
            self.temperature_c,
            self.row_sum,
            *(extra_cols or []),
        ]


@dataclass
class HistogramPacket:
    samples: list[HistogramSample]
    bytes_consumed: int
    timestamp_s: float | None


# ---------------------------------------------------------------------------
# Science-level data structures
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    side: str
    cam_id: int
    frame_id: int           # raw u8 from the wire
    absolute_frame_id: int  # monotonic counter with rollover handled
    timestamp_s: float
    row_sum: int
    temperature_c: float
    mean: float
    std_dev: float
    contrast: float
    bfi: float
    bvi: float
    is_corrected: bool = False  # True when dark-frame interpolation has been applied
    is_dark: bool = False       # True when this sample represents a laser-off (dark) frame


@dataclass
class CorrectedBatch:
    """
    Batch of dark-frame-corrected samples for one interval between two
    consecutive dark frames.  Emitted once per dark-frame interval (e.g.
    every 600 frames / 15 seconds at 40 Hz) after the linear interpolation
    of the dark baseline has been computed.

    ``dark_frame_start`` / ``dark_frame_end`` are the absolute frame IDs of
    the two bounding dark frames used for interpolation.
    ``samples`` contains one ``Sample`` per (side, cam_id) per
    non-dark frame in the interval, with ``is_corrected=True``.
    """
    dark_frame_start: int
    dark_frame_end: int
    samples: list[Sample]


# ---------------------------------------------------------------------------
# Binary packet parsing
# ---------------------------------------------------------------------------

def bytes_to_integers(byte_array: bytes | bytearray) -> tuple[list[int], list[int]]:
    """
    Convert 4096 histogram bytes into packed integer bins and hidden figures.

    Input is expected as 1024 chunks of 4 bytes each:
    - first 3 bytes: little-endian 24-bit histogram bin value
    - last byte: hidden figure metadata (e.g. frame-id carrier)
    """
    if len(byte_array) != HISTOGRAM_BYTES:
        raise ValueError("Input byte array must be exactly 4096 bytes.")

    integers: list[int] = []
    hidden_figures: list[int] = []
    for i in range(0, len(byte_array), 4):
        chunk = byte_array[i : i + 4]
        hidden_figures.append(chunk[3])
        integers.append(int.from_bytes(chunk[0:3], byteorder="little"))
    return integers, hidden_figures


def _parse_histo_payload(
    payload: bytes,
    expected_row_sum: int | None,
    original_pkt_len: int,
) -> "HistogramPacket":
    """
    Parse the raw (already-verified) decompressed payload bytes of a histogram
    packet into a HistogramPacket.  No header, footer, or CRC processing is
    performed — the caller is responsible for those checks before calling this.

    Used by the TYPE_HISTO_CMP path to avoid rebuilding a fake TYPE_HISTO
    packet and paying for a redundant CRC pass over the full decompressed data.
    """
    payload_len = len(payload)
    if payload_len < HISTO_BLOCK_SIZE:
        raise ValueError("Decompressed payload too small")

    has_timestamp = (payload_len % HISTO_BLOCK_SIZE) == TIMESTAMP_SIZE
    if not has_timestamp and (payload_len % HISTO_BLOCK_SIZE) != 0:
        raise ValueError("Decompressed payload length mismatch")

    mv = memoryview(payload)
    off = 0
    timestamp_sec: Optional[float] = None
    samples: list[HistogramSample] = []

    if has_timestamp:
        timestamp_ms = _U32.unpack_from(mv, off)[0]
        timestamp_sec = timestamp_ms / 1000.0
        off += TIMESTAMP_SIZE

    while off < payload_len:
        soh, cam_id = _BLK_HEAD.unpack_from(mv, off)
        if soh != SOH:
            raise ValueError("Missing SOH")
        off += _BLK_HEAD.size

        hist = np.frombuffer(mv, dtype=np.uint32, count=HISTO_SIZE_WORDS, offset=off)
        off += HISTOGRAM_BYTES

        temp = _F32.unpack_from(mv, off)[0]
        off += 4

        if mv[off] != EOH:
            raise ValueError("Missing EOH")
        off += 1

        last_word = hist[-1]
        frame_id = (last_word >> 24) & 0xFF
        hist = hist.copy()
        hist[-1] = last_word & 0x00_FF_FF_FF

        ts_val = timestamp_sec if timestamp_sec is not None else 0.0
        row_sum = int(hist.sum(dtype=np.uint64))

        _expected = expected_row_sum if expected_row_sum is not None else EXPECTED_HISTOGRAM_SUM
        if _expected is not None and row_sum != _expected:
            logger.warning(
                "Histogram sum mismatch for cam %d frame %d: "
                "got %d, expected %d — dropping sample",
                int(cam_id), int(frame_id), row_sum, _expected,
            )
            continue

        samples.append(
            HistogramSample(
                cam_id=int(cam_id),
                frame_id=int(frame_id),
                timestamp_s=float(ts_val),
                histogram=hist,
                temperature_c=float(temp),
                row_sum=row_sum,
            )
        )

    return HistogramPacket(
        samples=samples,
        bytes_consumed=original_pkt_len,
        timestamp_s=timestamp_sec,
    )


def _candidate_packet_size_ok(pkt_type_byte: int, candidate_size: int) -> bool:
    if pkt_type_byte == TYPE_HISTO:
        return MIN_HISTO_PACKET_SIZE <= candidate_size <= MAX_PACKET_SIZE
    if pkt_type_byte == TYPE_HISTO_CMP:
        return MIN_HISTO_CMP_PACKET_SIZE <= candidate_size <= MAX_PACKET_SIZE
    return False


def parse_histogram_packet_structured(
    pkt: memoryview,
    expected_row_sum: int | None = None,
) -> HistogramPacket:
    """
    Parse a binary histogram packet into normalized packet/sample dataclasses.

    Parameters
    ----------
    pkt
        Raw bytes of a single histogram packet.
    expected_row_sum
        When not None, each parsed sample's bin sum is compared against this
        value.  Samples whose sum does not match are logged as warnings and
        excluded from the returned ``HistogramPacket.samples`` list — they are
        treated as if the frame never arrived (will not be written to CSV and
        will not be fed into the science pipeline).  Pass ``None`` (default) to
        disable the check.  The module-level ``EXPECTED_HISTOGRAM_SUM``
        constant is a convenient global override point.

    Returns
    -------
    HistogramPacket
        Packet with parsed samples and metadata.
    """
    if len(pkt) < MIN_PACKET_ENVELOPE_SIZE:
        raise ValueError("Packet too small")

    sof, pkt_type, pkt_len = _HDR.unpack_from(pkt, 0)
    if sof != SOF or pkt_type not in (TYPE_HISTO, TYPE_HISTO_CMP):
        raise ValueError("Bad header")

    if pkt_len > len(pkt):
        raise ValueError("Truncated packet")

    # If compressed, verify both CRCs, decompress, and parse payload directly.
    # Packet layout: [Header 6B][Compressed N B][UNCMP_CRC16 2B][PKT_CRC16 2B][EOF 1B]
    if pkt_type == TYPE_HISTO_CMP:
        if pkt_len < MIN_HISTO_CMP_PACKET_SIZE:
            raise ValueError("TYPE_HISTO_CMP packet too small")
        footer_off = pkt_len - PACKET_FOOTER_SIZE            # offset of PKT_CRC16
        uncmp_crc_off = footer_off - CMP_UNCMP_CRC_SIZE      # offset of UNCMP_CRC16

        # 1. Verify transport CRC
        pkt_crc_expected = struct.unpack_from("<H", pkt, footer_off)[0]
        pkt_crc_actual = _crc16(memoryview(pkt[: footer_off - 1]))
        if pkt_crc_actual != pkt_crc_expected:
            raise ValueError(
                f"TYPE_HISTO_CMP transport CRC mismatch "
                f"(got 0x{pkt_crc_actual:04X}, expected 0x{pkt_crc_expected:04X})"
            )

        # 2. Decompress
        uncmp_crc_expected = struct.unpack_from("<H", pkt, uncmp_crc_off)[0]
        compressed_payload = bytes(pkt[PACKET_HEADER_SIZE : uncmp_crc_off])
        decompressed = _rle_decompress(compressed_payload)

        # 3. Verify decompressed payload CRC
        uncmp_crc_actual = _crc16(memoryview(decompressed[:-1]))
        if uncmp_crc_actual != uncmp_crc_expected:
            raise ValueError(
                f"TYPE_HISTO_CMP decompressed CRC mismatch "
                f"(got 0x{uncmp_crc_actual:04X}, expected 0x{uncmp_crc_expected:04X}) "
                f"— decompressor produced wrong output"
            )

        # 4. Parse the verified decompressed payload directly.
        # Both CRCs have already passed so there is no need to rebuild a fake
        # TYPE_HISTO packet and pay for a third CRC pass over the same data.
        return _parse_histo_payload(decompressed, expected_row_sum, pkt_len)

    payload_len = pkt_len - PACKET_HEADER_SIZE - PACKET_FOOTER_SIZE
    if payload_len < HISTO_BLOCK_SIZE:
        raise ValueError("Packet payload too small")

    has_timestamp = (payload_len % HISTO_BLOCK_SIZE) == TIMESTAMP_SIZE
    if not has_timestamp and (payload_len % HISTO_BLOCK_SIZE) != 0:
        raise ValueError("Packet length mismatch")

    payload_end = pkt_len - PACKET_FOOTER_SIZE
    off = PACKET_HEADER_SIZE

    samples: list[HistogramSample] = []
    timestamp_sec: Optional[float] = None

    if has_timestamp:
        timestamp_ms = _U32.unpack_from(pkt, off)[0]
        timestamp_sec = timestamp_ms / 1000.0
        off += TIMESTAMP_SIZE

    while off < payload_end:
        soh, cam_id = _BLK_HEAD.unpack_from(pkt, off)
        if soh != SOH:
            raise ValueError("Missing SOH")
        off += _BLK_HEAD.size

        hist = np.frombuffer(pkt, dtype=np.uint32, count=HISTO_SIZE_WORDS, offset=off)
        off += HISTOGRAM_BYTES

        temp = _F32.unpack_from(pkt, off)[0]
        off += 4

        if pkt[off] != EOH:
            raise ValueError("Missing EOH")
        off += 1

        # Strip frame-id from high byte of last word.
        last_word = hist[-1]
        frame_id = (last_word >> 24) & 0xFF
        hist = hist.copy()
        hist[-1] = last_word & 0x00_FF_FF_FF

        ts_val = timestamp_sec if timestamp_sec is not None else 0.0
        row_sum = int(hist.sum(dtype=np.uint64))

        # Sum validation — drop corrupt/doubled frames before they reach the
        # pipeline.  The expected value is the invariant photon-count total
        # that every valid frame must satisfy.
        _expected = expected_row_sum if expected_row_sum is not None else EXPECTED_HISTOGRAM_SUM
        if _expected is not None and row_sum != _expected:
            logger.warning(
                "Histogram sum mismatch for cam %d frame %d: "
                "got %d, expected %d — dropping sample",
                int(cam_id), int(frame_id), row_sum, _expected,
            )
            continue

        samples.append(
            HistogramSample(
                cam_id=int(cam_id),
                frame_id=int(frame_id),
                timestamp_s=float(ts_val),
                histogram=hist,
                temperature_c=float(temp),
                row_sum=row_sum,
            )
        )

    crc_expected = _U16.unpack_from(pkt, off)[0]
    off += 2
    if pkt[off] != EOF:
        raise ValueError("Missing EOF")

    if _crc16(pkt[: off - 3]) != crc_expected:
        raise ValueError("CRC mismatch")

    return HistogramPacket(
        samples=samples,
        bytes_consumed=pkt_len,
        timestamp_s=timestamp_sec,
    )


# ---------------------------------------------------------------------------
# File-oriented helpers (CSV)
# ---------------------------------------------------------------------------

def process_bin_file(
    src_bin: str, dst_csv: str, start_offset: int = 0, batch_rows: int = 4096
) -> None:
    """
    Convert raw histogram binary stream to CSV rows.
    """
    with open(src_bin, "rb") as f:
        data = memoryview(f.read())

    off = start_offset
    packet_ok = packet_fail = crc_failure = other_fail = bad_header_fail = 0
    out_buf: List[List] = []

    with open(dst_csv, "w", newline="") as fcsv:
        wr = csv.writer(fcsv)
        wr.writerow(
            [
                "cam_id",
                "frame_id",
                "timestamp_s",
                *range(HISTO_SIZE_WORDS),
                "temperature",
                "sum",
            ]
        )

        while off + MIN_PACKET_ENVELOPE_SIZE <= len(data):
            try:
                packet = parse_histogram_packet_structured(data[off:])
                off += packet.bytes_consumed
                packet_ok += 1

                for sample in packet.samples:
                    out_buf.append(sample.to_csv_row())

                if len(out_buf) >= batch_rows:
                    wr.writerows(out_buf)
                    out_buf.clear()
            except Exception as exc:
                if exc.args and exc.args[0] == "CRC mismatch":
                    crc_failure += 1
                elif exc.args and exc.args[0] == "Missing SOH":
                    packet_fail += 1
                elif exc.args and exc.args[0] == "Bad header":
                    bad_header_fail += 1
                else:
                    other_fail += 1

                # Resync: search for next valid packet header (SOF byte)
                old_off = off
                search_from = off + 1
                found_sync = False
                while search_from + PACKET_HEADER_SIZE <= len(data):
                    nxt = data.obj.find(b"\xAA", search_from)
                    if nxt == -1 or nxt + PACKET_HEADER_SIZE > len(data):
                        break
                    # Verify type byte is a known histogram type
                    pkt_type_byte = data[nxt + 1]
                    if pkt_type_byte not in (TYPE_HISTO, TYPE_HISTO_CMP):
                        search_from = nxt + 1
                        continue
                    candidate_size = _U32.unpack_from(data, nxt + 2)[0]
                    if _candidate_packet_size_ok(int(pkt_type_byte), int(candidate_size)):
                        off = nxt
                        found_sync = True
                        break
                    search_from = nxt + 1
                if found_sync:
                    continue
                break

        if out_buf:
            wr.writerows(out_buf)

    total_packets = packet_ok + packet_fail + crc_failure + other_fail + bad_header_fail
    logger.info("Parsed %d packets, %d OK", total_packets, packet_ok)


def parse_histogram_stream(
    q: queue.Queue,
    stop_evt: threading.Event,
    buffer_accumulator: bytearray,
    on_row_fn: Callable[[int, int, float, np.ndarray, int, float], None] | None = None,
    expected_row_sum: int | None = None,
    t0_normalizer: Callable[[float], float] | None = None,
) -> int:
    """
    Parse a histogram USB stream queue and fire ``on_row_fn`` for every
    valid sample.

    Parameters
    ----------
    expected_row_sum
        Forwarded to ``parse_histogram_packet_structured``.  When not None,
        samples whose histogram bin sum does not match are silently dropped
        from the ``on_row_fn`` callback.
    t0_normalizer
        Optional callback that converts an absolute firmware timestamp
        into a per-scan-zero timestamp; invoked once per sample before
        ``on_row_fn`` fires.

    Returns
    -------
    int
        Number of valid samples processed.
    """
    rows_processed = 0

    # Monotonic timestamp unwrapping: the firmware's 32-bit millisecond counter
    # rolls over every ~42949.67 s.  Track an offset so timestamps never go backwards.
    _ts_last: float | None = None
    _ts_offset: float = 0.0

    while not stop_evt.is_set() or not q.empty():
        try:
            data = q.get(timeout=0.300)
            if data:
                buffer_accumulator.extend(data)
            q.task_done()
        except queue.Empty:
            continue

        offset = 0
        while offset + MIN_PACKET_ENVELOPE_SIZE <= len(buffer_accumulator):
            try:
                pkt_view = memoryview(buffer_accumulator[offset:])
                packet = parse_histogram_packet_structured(
                    pkt_view, expected_row_sum=expected_row_sum
                )
                offset += packet.bytes_consumed

                for sample in packet.samples:
                    # Unwrap the firmware's 32-bit millisecond timestamp so it
                    # increases monotonically across the ~42949 s rollover boundary.
                    raw_ts = sample.timestamp_s
                    if _ts_last is not None and (raw_ts + _ts_offset) < (_ts_last - _TIMESTAMP_ROLLOVER_S / 2):
                        _ts_offset += _TIMESTAMP_ROLLOVER_S
                    sample.timestamp_s = raw_ts + _ts_offset
                    _ts_last = sample.timestamp_s
                    # Normalize to per-scan t0 if a normalizer was supplied
                    # (typically by ScanWorkflow). After this, sample.timestamp_s
                    # is seconds since the first sample emitted in this scan,
                    # so every downstream consumer — raw CSV, row handler /
                    # on_raw_frame_fn callback, the science pipeline, and the
                    # corrected outputs that flow from it — sees the same
                    # 0-based per-scan time origin.
                    if t0_normalizer is not None:
                        sample.timestamp_s = t0_normalizer(sample.timestamp_s)

                    if on_row_fn:
                        on_row_fn(
                            sample.cam_id,
                            sample.frame_id,
                            sample.timestamp_s,
                            sample.histogram,
                            sample.row_sum,
                            sample.temperature_c,
                        )
                        rows_processed += 1

            except ValueError as e:
                old_off = offset
                search_from = offset + 1
                found_sync = False
                while search_from + PACKET_HEADER_SIZE <= len(buffer_accumulator):
                    nxt = buffer_accumulator.find(b"\xaa", search_from)
                    if nxt == -1 or nxt + PACKET_HEADER_SIZE > len(buffer_accumulator):
                        break
                    # Verify type byte is a known histogram type
                    pkt_type_byte = buffer_accumulator[nxt + 1]
                    if pkt_type_byte not in (TYPE_HISTO, TYPE_HISTO_CMP):
                        search_from = nxt + 1
                        continue
                    candidate_size = struct.unpack_from(
                        "<I", buffer_accumulator, nxt + 2
                    )[0]
                    if _candidate_packet_size_ok(int(pkt_type_byte), int(candidate_size)):
                        offset = nxt
                        found_sync = True
                        logger.warning(
                            "Parser error at offset %d, resynced to %d "
                            "(skipped %d bytes): %s",
                            old_off, nxt, nxt - old_off, e,
                        )
                        break
                    search_from = nxt + 1
                if found_sync:
                    continue
                break

        if offset > 0:
            del buffer_accumulator[:offset]

    # --- Final accumulator flush ------------------------------------------------
    # The main loop exits as soon as stop_evt is set and the queue is empty, but
    # bytes may still be sitting in buffer_accumulator from the last dequeue —
    # in particular the final frame, which the USB layer can deliver up to 250 ms
    # after the trigger stops.  Attempt one more full parse pass and log anything
    # that was recovered (or couldn't be parsed) so frame loss is visible in logs.
    if buffer_accumulator:
        logger.warning(
            "parse_histogram_stream: %d bytes remain in accumulator after "
            "stream end — attempting final parse pass",
            len(buffer_accumulator),
        )
        rows_before_final_flush = rows_processed
        offset = 0
        while offset + MIN_PACKET_ENVELOPE_SIZE <= len(buffer_accumulator):
            try:
                pkt_view = memoryview(buffer_accumulator[offset:])
                packet = parse_histogram_packet_structured(
                    pkt_view, expected_row_sum=expected_row_sum
                )
                offset += packet.bytes_consumed
                for sample in packet.samples:
                    raw_ts = sample.timestamp_s
                    if _ts_last is not None and (raw_ts + _ts_offset) < (_ts_last - _TIMESTAMP_ROLLOVER_S / 2):
                        _ts_offset += _TIMESTAMP_ROLLOVER_S
                    sample.timestamp_s = raw_ts + _ts_offset
                    _ts_last = sample.timestamp_s
                    if t0_normalizer is not None:
                        sample.timestamp_s = t0_normalizer(sample.timestamp_s)

                    if on_row_fn:
                        on_row_fn(
                            sample.cam_id,
                            sample.frame_id,
                            sample.timestamp_s,
                            sample.histogram,
                            sample.row_sum,
                            sample.temperature_c,
                        )
                        rows_processed += 1
            except ValueError as e:
                old_off = offset
                search_from = offset + 1
                found_sync = False
                while search_from + PACKET_HEADER_SIZE <= len(buffer_accumulator):
                    nxt = buffer_accumulator.find(b"\xaa", search_from)
                    if nxt == -1 or nxt + PACKET_HEADER_SIZE > len(buffer_accumulator):
                        break
                    pkt_type_byte = buffer_accumulator[nxt + 1]
                    if pkt_type_byte not in (TYPE_HISTO, TYPE_HISTO_CMP):
                        search_from = nxt + 1
                        continue
                    candidate_size = struct.unpack_from(
                        "<I", buffer_accumulator, nxt + 2
                    )[0]
                    if _candidate_packet_size_ok(int(pkt_type_byte), int(candidate_size)):
                        offset = nxt
                        found_sync = True
                        logger.warning(
                            "parse_histogram_stream: final flush parser error at "
                            "offset %d, resynced to %d (skipped %d bytes): %s",
                            old_off, nxt, nxt - old_off, e,
                        )
                        break
                    search_from = nxt + 1
                if found_sync:
                    continue
                break
        if offset > 0:
            logger.info(
                "parse_histogram_stream: final flush recovered %d additional row(s)",
                rows_processed - rows_before_final_flush,
            )
            del buffer_accumulator[:offset]
        if buffer_accumulator:
            logger.warning(
                "parse_histogram_stream: %d bytes could not be parsed and were "
                "discarded — likely an incomplete final packet",
                len(buffer_accumulator),
            )

    return rows_processed
