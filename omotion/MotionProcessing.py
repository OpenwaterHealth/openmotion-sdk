import csv
import logging
import struct
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

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


# Histogram payload constants
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
HISTO_BINS = np.arange(HISTO_SIZE_WORDS, dtype=np.float64)
HISTO_BINS_SQ = HISTO_BINS * HISTO_BINS

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

# Shot-noise correction constants.
#
# Photon shot noise follows Poisson statistics: variance (in electrons) equals
# mean (in electrons).  Converting to digital units:
#
#   shot_noise_var_DN = ADC_GAIN · g · mean_electrons
#                     = ADC_GAIN · mean_DN
#
# where g is the per-camera analog gain and mean_DN is the dark-corrected mean
# in digital units.  This expected shot-noise contribution is subtracted from
# the dark-corrected variance in the corrected path so that the residual
# variance (and therefore the speckle contrast) reflects only laser speckle
# fluctuations, not photon counting noise.
#
# ADC_GAIN: (full-scale DN range above pedestal) / (electrons at full scale)
#   = (1024 − 64) / 11 000 ≈ 0.0873 DN/e⁻
#
# CAMERA_GAIN_MAP: analog gain for each of the 8 camera positions within a
#   sensor module.  The outer cameras use higher gain to compensate for the
#   reduced illumination at the array periphery.
ADC_GAIN: float = (1024 - 64) / 11_000          # DN per electron ≈ 0.0873
CAMERA_GAIN_MAP: np.ndarray = np.array(
    [16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float64
)  # index 0 = cam position 0 (outermost), index 7 = cam position 7 (outermost)

logger = logging.getLogger(
    f"{_log_root}.MotionProcessing" if _log_root else "MotionProcessing"
)

# Struct formats
_U16 = struct.Struct("<H")
_U32 = struct.Struct("<I")
_F32 = struct.Struct("<f")
_HDR = struct.Struct("<BBI")
_BLK_HEAD = struct.Struct("<BB")


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


# ---------------------------------------------------------------------------
# Frame ID unwrapping
# ---------------------------------------------------------------------------

class FrameIdUnwrapper:
    """
    Converts a raw u8 frame ID (0–255) into a monotonically increasing
    absolute frame number by detecting rollover events.

    The firmware frame counter wraps from 255 back to 0.  We detect the
    crossing by watching whether the new raw ID is numerically smaller than
    the previous one while the unsigned forward delta is still within the
    normal range (≤ FRAME_ROLLOVER_THRESHOLD).  A delta larger than the
    threshold indicates an anomalous backward jump (retransmit / corruption)
    rather than a genuine rollover, so we leave the epoch untouched.

    One unwrapper instance must be kept per (side, cam_id) pair so that
    independent per-camera counters do not interfere with one another.

    **Sensor firmware quirk handling:** the camera's *very first* frame
    after a fresh scan start often has raw_id=1, then the cycle starts
    properly from raw_id=0 on the second frame. Naively this produces
    a non-monotonic absolute_frame_id sequence (1, 0, 1, 2, 3, ...).
    The unwrapper detects this exact pattern on the second call and
    re-aligns: the spurious first frame keeps abs_frame_id=0 (a
    "preamble" slot that the discard_count warmup will drop anyway),
    and the second frame's raw=0 becomes abs=1.
    """

    def __init__(self) -> None:
        self._last_raw: int | None = None
        self._epoch: int = 0
        # Set to True after the first call. Lets unwrap() detect the
        # camera's spurious-first-frame pattern on the second call.
        self._first_seen_raw: int | None = None
        self._second_call_realigned: bool = False
        # Offset added to the raw cycle position to derive abs. Starts
        # at 0 and gets bumped to 1 if the spurious-first-frame pattern
        # is detected.
        self._cycle_offset: int = 0

    def unwrap(self, raw_frame_id: int) -> int:
        if self._last_raw is None:
            # First frame ever. Tentatively treat it as the start of
            # the cycle (return 0 as a "preamble" slot).
            self._last_raw = raw_frame_id
            self._first_seen_raw = raw_frame_id
            return 0 if raw_frame_id == 1 else raw_frame_id

        # On the second call, decide whether the camera quirk is in
        # play. The signature is: first raw=1, second raw=0. If we see
        # this, the cycle's "true" first frame is the SECOND frame, so
        # we re-anchor _last_raw to raw=0 and bump cycle_offset so abs
        # comes out as 1 for that frame instead of 0.
        if not self._second_call_realigned:
            self._second_call_realigned = True
            if self._first_seen_raw == 1 and raw_frame_id == 0:
                # Re-anchor.
                self._last_raw = 0
                self._cycle_offset = 1
                return 1

        delta = (raw_frame_id - self._last_raw) & 0xFF

        if delta <= FRAME_ROLLOVER_THRESHOLD and raw_frame_id < self._last_raw:
            # Normal forward progress that crossed the 0/255 boundary.
            self._epoch += 1
        # delta > FRAME_ROLLOVER_THRESHOLD means apparent backward jump —
        # treat as anomaly and leave epoch unchanged.

        self._last_raw = raw_frame_id
        return self._epoch * FRAME_ID_MODULUS + raw_frame_id + self._cycle_offset

    def reset(self) -> None:
        self._last_raw = None
        self._epoch = 0
        self._first_seen_raw = None
        self._second_call_realigned = False
        self._cycle_offset = 0


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
    csv_writer,
    buffer_accumulator: bytearray,
    extra_cols_fn: Callable[[], list] | None = None,
    on_row_fn: Callable[[int, int, float, np.ndarray, int, float], None] | None = None,
    expected_row_sum: int | None = None,
    csv_deadline: float | None = None,
    on_csv_closed_fn: Callable[[], None] | None = None,
    csv_stop_event: threading.Event | None = None,
) -> int:
    """
    Parse a histogram USB stream queue, feed the science pipeline, and
    optionally write CSV rows.

    ``on_row_fn`` is the primary output and is called for every valid parsed
    sample regardless of CSV state.  CSV writing via ``csv_writer`` is
    secondary and can be disabled entirely (``csv_writer=None``) or
    automatically stopped after a wall-clock deadline (``csv_deadline``).

    Parameters
    ----------
    csv_writer
        A ``csv.writer``-compatible object, or ``None`` to skip CSV output.
    csv_deadline
        ``time.monotonic()``-style deadline after which CSV writing stops but
        ``on_row_fn`` continues.  ``None`` means write for the full duration.
    on_csv_closed_fn
        Called exactly once when ``csv_deadline`` is reached (or
        ``csv_stop_event`` is set) and the writer is deactivated.  Useful
        for emitting a log message to the caller.
    csv_stop_event
        A :class:`threading.Event` shared across all writer threads for the
        same scan.  When any thread's deadline fires it sets this event so
        every other thread stops writing on its next sample check, keeping
        row counts equal across left/right CSVs.  ``None`` disables
        cross-thread synchronisation.
    expected_row_sum
        Forwarded to ``parse_histogram_packet_structured``.  When not None,
        samples whose histogram bin sum does not match are silently dropped
        from both the CSV and the ``on_row_fn`` callback.

    Returns
    -------
    int
        Number of rows written to ``csv_writer``.
    """
    rows_written = 0
    _csv_active = csv_writer is not None

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

                    # Check CSV deadline before every row so the cutoff is
                    # accurate to within one sample period (~25 ms at 40 Hz).
                    if _csv_active and (
                        (csv_deadline is not None and time.monotonic() >= csv_deadline)
                        or (csv_stop_event is not None and csv_stop_event.is_set())
                    ):
                        _csv_active = False
                        # Broadcast to peer writer threads sharing this event
                        # so every side's CSV ends at the same sample boundary.
                        if csv_stop_event is not None and not csv_stop_event.is_set():
                            csv_stop_event.set()
                        if on_csv_closed_fn:
                            try:
                                on_csv_closed_fn()
                            except Exception:
                                pass

                    if _csv_active:
                        extra_cols = extra_cols_fn() if extra_cols_fn else []
                        row = sample.to_csv_row(extra_cols=extra_cols)
                        csv_writer.writerow(row)
                        rows_written += 1

                    if on_row_fn:
                        on_row_fn(
                            sample.cam_id,
                            sample.frame_id,
                            sample.timestamp_s,
                            sample.histogram,
                            sample.row_sum,
                            sample.temperature_c,
                        )

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
            "parse_stream_to_csv: %d bytes remain in accumulator after "
            "stream end — attempting final parse pass",
            len(buffer_accumulator),
        )
        rows_before_final_flush = rows_written
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

                    if _csv_active and (
                        (csv_deadline is not None and time.monotonic() >= csv_deadline)
                        or (csv_stop_event is not None and csv_stop_event.is_set())
                    ):
                        _csv_active = False
                        # Broadcast to peer writer threads sharing this event
                        # so every side's CSV ends at the same sample boundary.
                        if csv_stop_event is not None and not csv_stop_event.is_set():
                            csv_stop_event.set()
                        if on_csv_closed_fn:
                            try:
                                on_csv_closed_fn()
                            except Exception:
                                pass

                    if _csv_active:
                        extra_cols = extra_cols_fn() if extra_cols_fn else []
                        row = sample.to_csv_row(extra_cols=extra_cols)
                        csv_writer.writerow(row)
                        rows_written += 1

                    if on_row_fn:
                        on_row_fn(
                            sample.cam_id,
                            sample.frame_id,
                            sample.timestamp_s,
                            sample.histogram,
                            sample.row_sum,
                            sample.temperature_c,
                        )
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
                            "parse_stream_to_csv: final flush parser error at "
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
                "parse_stream_to_csv: final flush recovered %d additional row(s)",
                rows_written - rows_before_final_flush,
            )
            del buffer_accumulator[:offset]
        if buffer_accumulator:
            logger.warning(
                "parse_stream_to_csv: %d bytes could not be parsed and were "
                "discarded — likely an incomplete final packet",
                len(buffer_accumulator),
            )

    return rows_written


def stream_queue_to_csv_file(
    q: queue.Queue,
    stop_evt: threading.Event,
    filename: str,
    *,
    extra_headers: list[str] | None = None,
    extra_cols_fn: Callable[[], list] | None = None,
    on_row_fn: Callable[[int, int, float, np.ndarray, int, float], None] | None = None,
    on_complete_fn: Callable[[int], None] | None = None,
    on_error_fn: Callable[[Exception], None] | None = None,
    expected_row_sum: int | None = None,
) -> int:
    """
    High-level helper: parse stream queue data and write a CSV file end-to-end.

    This owns file open/header/write/close so applications only pass:
    - destination filename
    - queue + stop event
    - optional callbacks for extra columns and row handling.

    Parameters
    ----------
    expected_row_sum
        Forwarded to ``parse_histogram_stream`` / ``parse_histogram_packet_structured``.
        Samples whose histogram bin sum does not match are dropped from both
        the CSV and the ``on_row_fn`` callback before being written.
    """
    rows_written = 0
    extra_headers = extra_headers or []

    try:
        with open(filename, "w", newline="", encoding="utf-8") as f:
            csv_writer = csv.writer(f)
            csv_writer.writerow(
                [
                    "cam_id",
                    "frame_id",
                    "timestamp_s",
                    *range(HISTO_SIZE_WORDS),
                    "temperature",
                    "sum",
                    *extra_headers,
                ]
            )

            buffer_accumulator = bytearray()
            rows_written = parse_histogram_stream(
                q=q,
                stop_evt=stop_evt,
                csv_writer=csv_writer,
                buffer_accumulator=buffer_accumulator,
                extra_cols_fn=extra_cols_fn,
                on_row_fn=on_row_fn,
                expected_row_sum=expected_row_sum,
            )
    except Exception as e:
        if on_error_fn:
            on_error_fn(e)
        logger.error("Writer error (%s): %s", filename, e, exc_info=True)
        return rows_written

    if on_complete_fn:
        on_complete_fn(rows_written)
    return rows_written


# ---------------------------------------------------------------------------
# Pure science computation functions
# ---------------------------------------------------------------------------

def compute_realtime_metrics(
    *,
    side: str,
    cam_id: int,
    frame_id: int,
    absolute_frame_id: int,
    timestamp_s: float,
    hist: np.ndarray,
    row_sum: int,
    temperature_c: float,
    bfi_c_min,
    bfi_c_max,
    bfi_i_min,
    bfi_i_max,
    pedestal: float | None = None,
) -> Sample:
    """
    Pure metric computation for one histogram row (uncorrected stream).

    The *pedestal* is subtracted from the raw histogram mean before computing
    contrast, BVI, and the emitted ``mean`` field.  This removes the fixed ADC
    DC bias so that downstream consumers see a zero-referenced intensity signal.

    The pedestal does **not** affect the stored ``u1`` values used later for
    dark-frame correction: those raw values are stored separately in the
    pipeline, and the pedestal cancels automatically in the subtraction
    ``corrected_mean = fm.u1 − dark_u1`` because both terms carry it equally.
    """
    if pedestal is None:
        pedestal = PEDESTAL_HEIGHT
    if row_sum > 0:
        raw_mean = float(np.dot(hist, HISTO_BINS) / row_sum)
    else:
        raw_mean = 0.0

    # Subtract the sensor pedestal for display/calibration purposes.
    # Clamp to zero so downstream code never sees a negative mean.
    mean_val = max(0.0, raw_mean - pedestal)

    # Variance is invariant to the pedestal shift (constant subtracted from
    # every bin centre does not change E[X²]−E[X]²), so we compute it from
    # the raw second moment but use the pedestal-adjusted mean for contrast.
    if row_sum > 0 and mean_val > 0:
        mean2 = float(np.dot(hist, HISTO_BINS_SQ) / row_sum)
        var = max(0.0, mean2 - (raw_mean * raw_mean))
        std = np.sqrt(var)
        contrast = float(std / mean_val)
    else:
        std = 0.0
        contrast = 0.0

    module_idx = 0 if side == "left" else 1
    cam_pos = int(cam_id) % 8

    if module_idx >= bfi_c_min.shape[0] or cam_pos >= bfi_c_min.shape[1]:
        bfi_val = contrast * 10.0
    else:
        cmin = float(bfi_c_min[module_idx, cam_pos])
        cmax = float(bfi_c_max[module_idx, cam_pos])
        cden = (cmax - cmin) or 1.0
        bfi_val = (1.0 - ((contrast - cmin) / cden)) * 10.0

    if module_idx >= bfi_i_min.shape[0] or cam_pos >= bfi_i_min.shape[1]:
        bvi_val = mean_val * 10.0
    else:
        imin = float(bfi_i_min[module_idx, cam_pos])
        imax = float(bfi_i_max[module_idx, cam_pos])
        iden = (imax - imin) or 1.0
        bvi_val = (1.0 - ((mean_val - imin) / iden)) * 10.0

    timestamp = float(timestamp_s) if timestamp_s else time.time()
    return Sample(
        side=side,
        cam_id=int(cam_id),
        frame_id=int(frame_id),
        absolute_frame_id=int(absolute_frame_id),
        timestamp_s=timestamp,
        row_sum=int(row_sum),
        temperature_c=float(temperature_c),
        mean=float(mean_val),
        std_dev=float(std),
        contrast=float(contrast),
        bfi=float(bfi_val),
        bvi=float(bvi_val),
    )


def compute_corrected_values(
    # TODO this is just a placeholder for the future corrected algorithm
    # TODO note that this function will need to operate on large numbers of histograms and will only
    # be able to happen once a dark frame has been captured, which may be every 15 seconds
    *,
    mean_val: float,
    bfi_val: float,
    bvi_val: float,
    last_bfi: float | None,
    last_bvi: float | None,
    mean_threshold: float,
) -> tuple[float, float]:
    """
    Pure correction computation from current values and prior state.
    """
    if mean_val < mean_threshold and last_bfi is not None:
        bfi_corr = float(last_bfi)
    else:
        bfi_corr = float(bfi_val)

    if mean_val < mean_threshold and last_bvi is not None:
        bvi_corr = float(last_bvi)
    else:
        bvi_corr = float(bvi_val)

    return bfi_corr, bvi_corr


# ---------------------------------------------------------------------------
# Unified science pipeline
# ---------------------------------------------------------------------------

@dataclass
class _StoredFrameMoments:
    """Per-sample first/second moments stored between dark frames for later correction."""
    side: str
    cam_id: int
    frame_id: int              # raw u8
    absolute_frame_id: int
    timestamp_s: float
    u1: float                  # first moment  (mean of histogram)
    u2: float                  # second moment (mean of bins²)
    temperature_c: float
    row_sum: int


class SciencePipeline:
    """
    Unified single-threaded science computation pipeline for both sensor sides.

    All histogram samples — from left and right sensors alike — are fed in
    through a single ingress queue.  A single worker thread:

      1. Discards the first ``discard_count`` frames (default 9) which are
         noisy camera-warmup frames.
      2. Unwraps the raw u8 frame ID for each (side, cam_id) pair into a
         monotonically increasing ``absolute_frame_id``.
      3. Identifies dark frames by schedule: frame ``discard_count + 1``
         (the 10th frame) is the first dark reference, then every
         ``dark_interval`` frames from frame 1 (i.e. 601, 1201, …).
      4. For **non-dark** frames: computes uncorrected BFI/BVI from the raw
         histogram and fires ``on_uncorrected_fn`` immediately.  Also stores
         the first/second moments for later dark-frame correction.
      5. For **dark** frames: stores the dark baseline statistics.  When two
         consecutive dark frames are available for a camera, linearly
         interpolates the dark baseline across the interval and computes
         corrected BFI/BVI for every stored frame in that interval.  The
         result is emitted via ``on_corrected_batch_fn`` as a
         ``CorrectedBatch``.

    This means consumers receive:
    - A **continuous stream** of uncorrected samples (real-time, every frame)
    - **Periodic batches** of properly corrected samples (every dark interval)

    Parameters
    ----------
    left_camera_mask, right_camera_mask
        Bitmask of active cameras (bit N set → cam_id N is expected).
        Pass 0x00 for a side that is not connected.
    bfi_c_min/max, bfi_i_min/max
        Calibration arrays, shape (2, 8) — module index × camera position.
    on_uncorrected_fn
        Called immediately for each non-dark frame with uncorrected BFI/BVI.
        Receives a ``Sample`` with ``is_corrected=False``.
    on_corrected_batch_fn
        Called once per dark-frame interval with a ``CorrectedBatch``
        containing dark-frame-corrected BFI/BVI for every frame in that
        interval.
    dark_interval
        Number of frames between scheduled dark frames (default 600 = 15 s
        at 40 Hz).
    discard_count
        Number of initial frames to discard (default 9, frames 1–9).
    """

    def __init__(
        self,
        *,
        left_camera_mask: int = 0xFF,
        right_camera_mask: int = 0xFF,
        bfi_c_min,
        bfi_c_max,
        bfi_i_min,
        bfi_i_max,
        on_uncorrected_fn: Callable[[Sample], None] | None = None,
        on_corrected_batch_fn: Callable[[CorrectedBatch], None] | None = None,
        on_dark_frame_fn: Callable[[Sample], None] | None = None,
        on_rolling_avg_fn: Callable[[Sample], None] | None = None,
        rolling_avg_enabled: bool = False,
        rolling_avg_window: int = 10,
        dark_interval: int = 600,
        discard_count: int = 9,
        expected_row_sum: int | None = None,
        noise_floor: int = 10,
        log_dark_endpoints: bool = False,
        dark_integrity_max_u1_above_pedestal: float = 30.0,
        dark_integrity_max_std: float = 20.0,
    ):
        self._bfi_c_min = bfi_c_min
        self._bfi_c_max = bfi_c_max
        self._bfi_i_min = bfi_i_min
        self._bfi_i_max = bfi_i_max
        self._on_uncorrected_fn = on_uncorrected_fn
        self._on_corrected_batch_fn = on_corrected_batch_fn
        self._on_dark_frame_fn = on_dark_frame_fn
        self._on_rolling_avg_fn = on_rolling_avg_fn
        self._rolling_avg_enabled = bool(rolling_avg_enabled)
        self._rolling_avg_window = int(rolling_avg_window)
        self._log_dark_endpoints = bool(log_dark_endpoints)
        # Dark-integrity monitor: every frame stored in _dark_history is
        # cross-checked against these bounds. The defaults catch the
        # firmware off-by-one symptom we've actually observed in the
        # field (frame 10 looks like a light frame: u1≈200, std≈40).
        self._dark_integrity_max_u1_above_pedestal = float(dark_integrity_max_u1_above_pedestal)
        self._dark_integrity_max_std = float(dark_integrity_max_std)
        # Populated by _check_dark_integrity. Callers (CalibrationWorkflow)
        # query this after pipeline.stop() to fail loudly.
        self._dark_integrity_warnings: list[str] = []
        # Per (side, cam_id): deque of the last N uncorrected light Samples.
        # Populated lazily on first light sample for that key; empty when
        # rolling_avg_enabled is False so disabled mode has zero overhead.
        self._rolling_buffers: dict[tuple[str, int], "deque[Sample]"] = {}
        self._dark_interval = dark_interval
        self._discard_count = discard_count
        self._expected_row_sum = expected_row_sum
        self._noise_floor = int(noise_floor)

        # One FrameIdUnwrapper per (side, cam_id) — created lazily on first use.
        self._unwrappers: dict[tuple[str, int], FrameIdUnwrapper] = {}

        # Tracks which (side, cam_id) pairs have received their first frame.
        self._first_frame_seen: set[tuple[str, int]] = set()

        # --- Dark-frame correction state ---
        # Per (side, cam_id): ordered list of
        #   (absolute_frame_id, raw_frame_id, timestamp_s, u1, variance)
        # for each dark frame observed.
        self._dark_history: dict[tuple[str, int], list[tuple[int, int, float, float, float]]] = {}

        # Per (side, cam_id): stored moments for frames between the last two
        # dark frames, awaiting correction.
        self._pending_moments: dict[tuple[str, int], list[_StoredFrameMoments]] = {}

        # Per (side, cam_id): last uncorrected sample emitted for that camera.
        # Used to repeat values on dark frames so the live plot sees no blip.
        self._last_uncorrected: dict[tuple[str, int], Sample] = {}

        # Per (side, cam_id): last corrected sample from the previous batch.
        # Used to interpolate the corrected value for the start dark frame.
        # Stores the two most recent corrected samples per camera so that the
        # quadratic 4-point stencil can be applied at each dark frame position.
        self._last_corrected: dict[tuple[str, int], list[Sample]] = {}

        self._ingress_queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._science_thread = threading.Thread(
            target=self._science_worker, daemon=True, name="SciencePipeline"
        )

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def start(self) -> None:
        self._science_thread.start()

    @property
    def dark_integrity_warnings(self) -> list[str]:
        """Return the list of dark-integrity warnings collected so far.

        A non-empty list means the science pipeline classified one or
        more frames as dark-by-schedule, but the actual measurement
        (u1, std) failed the dark heuristic — i.e. the firmware emitted
        a light frame in a slot the science pipeline expected to be
        dark. Calibration callers should fail loudly when this is
        non-empty since the dark interpolation will be polluted.
        """
        return list(self._dark_integrity_warnings)

    def _check_dark_integrity(
        self,
        side: str,
        cam_id: int,
        absolute_frame: int,
        u1: float,
        variance: float,
    ) -> None:
        """Verify a frame the schedule classified as dark actually
        looks dark. Logs at ERROR level and stores a warning string on
        the pipeline if not."""
        std = float(np.sqrt(max(0.0, variance)))
        max_u1 = PEDESTAL_HEIGHT + self._dark_integrity_max_u1_above_pedestal
        max_std = self._dark_integrity_max_std
        if u1 <= max_u1 and std <= max_std:
            return
        msg = (
            f"DARK INTEGRITY FAILURE: {side} cam {cam_id} frame "
            f"{absolute_frame} was classified as dark by schedule, but "
            f"its measurement looks like a light frame "
            f"(u1={u1:.2f} > pedestal+{self._dark_integrity_max_u1_above_pedestal:.0f}={max_u1:.0f}? "
            f"std={std:.2f} > {max_std:.0f}?). "
            "Probable cause: firmware off-by-one in NUM_DARK_FRAMES_AT_START "
            "or unwrapper alignment quirk. Dark-frame interpolation will be "
            "polluted; corrected stream values are unreliable."
        )
        logger.error(msg)
        self._dark_integrity_warnings.append(msg)

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        self._science_thread.join(timeout=timeout)

    def enqueue(
        self,
        side: str,
        cam_id: int,
        frame_id: int,
        timestamp_s: float,
        hist: np.ndarray,
        row_sum: int,
        temperature_c: float,
    ) -> None:
        """Feed one histogram sample from the named side into the pipeline."""
        self._ingress_queue.put(
            (side, cam_id, frame_id, timestamp_s, hist, row_sum, temperature_c)
        )

    # ------------------------------------------------------------------
    # Worker
    # ------------------------------------------------------------------

    def _science_worker(self) -> None:
        while not self._stop_event.is_set() or not self._ingress_queue.empty():
            try:
                item = self._ingress_queue.get(timeout=0.050)
            except queue.Empty:
                continue

            side, cam_id, raw_frame_id, ts, hist, row_sum, temp = item
            key = (side, cam_id)

            # --- 0a. Sum validation (defense-in-depth) -------------------------
            _expected_sum = (
                self._expected_row_sum
                if self._expected_row_sum is not None
                else EXPECTED_HISTOGRAM_SUM
            )
            if _expected_sum is not None and row_sum != _expected_sum:
                logger.warning(
                    "SciencePipeline: histogram sum mismatch for %s cam %d "
                    "frame %d: got %d, expected %d — dropping sample",
                    side, cam_id, raw_frame_id, row_sum, _expected_sum,
                )
                continue

            # --- 0b. First-frame staleness check --------------------------------
            if key not in self._first_frame_seen:
                self._first_frame_seen.add(key)
                if raw_frame_id != 1:
                    logger.warning(
                        "SciencePipeline: first frame for %s cam %d has "
                        "frame_id=%d (expected 1) — likely stale from previous "
                        "scan; dropping sample",
                        side, cam_id, raw_frame_id,
                    )
                    continue

            # --- 1. Unwrap frame ID -------------------------------------------
            if key not in self._unwrappers:
                self._unwrappers[key] = FrameIdUnwrapper()
            absolute_frame = self._unwrappers[key].unwrap(raw_frame_id)

            # --- 2. Discard early warmup frames (1 through discard_count) ------
            if absolute_frame <= self._discard_count:
                logger.debug(
                    "Discarding warmup frame %d for %s cam %d",
                    absolute_frame, side, cam_id,
                )
                continue

            # --- 3. Noise floor decimation ------------------------------------
            # Zero out any bin whose count is below the noise floor threshold
            # before computing moments.  This suppresses low-level dark noise
            # that would otherwise bias the mean and variance estimates.
            if self._noise_floor > 0:
                below = hist < self._noise_floor
                if below.any():
                    hist = hist.copy()
                    hist[below] = 0
                    row_sum = int(hist.sum(dtype=np.uint64))

            # --- 4. Compute raw moments from histogram -------------------------
            if row_sum > 0:
                u1 = float(np.dot(hist, HISTO_BINS) / row_sum)
                u2 = float(np.dot(hist, HISTO_BINS_SQ) / row_sum)
            else:
                u1 = 0.0
                u2 = 0.0

            # --- 5. Dark frame handling ----------------------------------------
            if self._is_dark_frame(absolute_frame):
                variance = max(0.0, u2 - u1 * u1)

                # Fire on_dark_frame_fn with pedestal-subtracted dark-baseline
                # statistics so callback consumers can follow the same
                # zero-referenced mean convention used by uncorrected light
                # samples. Internal correction state remains raw (u1/u2).
                # BFI/BVI are 0 — not meaningful on a dark frame.
                if self._on_dark_frame_fn is not None:
                    dark_std = float(np.sqrt(variance))
                    dark_mean = float(u1 - PEDESTAL_HEIGHT)
                    dark_contrast = (dark_std / dark_mean) if dark_mean > 0 else 0.0
                    dark_sample = Sample(
                        side=side,
                        cam_id=cam_id,
                        frame_id=raw_frame_id,
                        absolute_frame_id=absolute_frame,
                        timestamp_s=ts,
                        row_sum=row_sum,
                        temperature_c=temp,
                        mean=dark_mean,
                        std_dev=dark_std,
                        contrast=dark_contrast,
                        bfi=0.0,
                        bvi=0.0,
                        is_corrected=False,
                        is_dark=True,
                    )
                    try:
                        self._on_dark_frame_fn(dark_sample)
                    except Exception:
                        logger.exception("Error in on_dark_frame_fn callback")

                dark_list = self._dark_history.setdefault(key, [])
                dark_list.append((absolute_frame, raw_frame_id, ts, u1, variance))
                logger.debug(
                    "Dark frame %d for %s cam %d (dark #%d): "
                    "u1=%.2f var=%.4f",
                    absolute_frame, side, cam_id, len(dark_list),
                    u1, variance,
                )
                # Sanity-check: did this frame *actually* look like a
                # dark? If not, the schedule and the firmware disagree
                # and we want to know about it.
                self._check_dark_integrity(
                    side, cam_id, absolute_frame, u1, variance,
                )

                # With 2+ dark frames we can correct the preceding interval.
                if len(dark_list) >= 2:
                    self._emit_corrected_for_camera(key)

                # Rule 1: emit an uncorrected sample for the dark frame that
                # repeats the last known good (non-dark) values so the live
                # plot sees no blip at the dark-frame position.  The sample
                # carries is_dark=True so consumers can filter it out.
                prev = self._last_uncorrected.get(key)
                if prev is not None:
                    dark_uncorrected = Sample(
                        side=prev.side,
                        cam_id=prev.cam_id,
                        frame_id=raw_frame_id,
                        absolute_frame_id=absolute_frame,
                        timestamp_s=ts,
                        row_sum=prev.row_sum,
                        temperature_c=prev.temperature_c,
                        mean=prev.mean,
                        std_dev=prev.std_dev,
                        contrast=prev.contrast,
                        bfi=prev.bfi,
                        bvi=prev.bvi,
                        is_corrected=False,
                        is_dark=True,
                    )
                    if self._on_uncorrected_fn:
                        try:
                            self._on_uncorrected_fn(dark_uncorrected)
                        except Exception:
                            pass
                continue

            # --- 5. Store moments for later dark-frame correction --------------
            self._pending_moments.setdefault(key, []).append(
                _StoredFrameMoments(
                    side=side,
                    cam_id=cam_id,
                    frame_id=raw_frame_id,
                    absolute_frame_id=absolute_frame,
                    timestamp_s=ts,
                    u1=u1,
                    u2=u2,
                    temperature_c=temp,
                    row_sum=row_sum,
                )
            )

            # --- 6. Compute uncorrected BFI/BVI and emit immediately ----------
            uncorrected = compute_realtime_metrics(
                side=side,
                cam_id=cam_id,
                frame_id=raw_frame_id,
                absolute_frame_id=absolute_frame,
                timestamp_s=ts,
                hist=hist,
                row_sum=row_sum,
                temperature_c=temp,
                bfi_c_min=self._bfi_c_min,
                bfi_c_max=self._bfi_c_max,
                bfi_i_min=self._bfi_i_min,
                bfi_i_max=self._bfi_i_max,
            )  # is_corrected defaults to False

            if self._on_uncorrected_fn:
                try:
                    self._on_uncorrected_fn(uncorrected)
                except Exception:
                    pass

            self._last_uncorrected[key] = uncorrected

            # --- 7. Rolling-average over the last N uncorrected light samples ---
            # Placed in the light branch only, so dark-frame repeat samples
            # never enter the window (the dark branch above continues before
            # reaching this code path).
            if self._rolling_avg_enabled and self._on_rolling_avg_fn is not None:
                buf = self._rolling_buffers.get(key)
                if buf is None:
                    buf = deque(maxlen=self._rolling_avg_window)
                    self._rolling_buffers[key] = buf
                buf.append(uncorrected)

                n = len(buf)
                mean_avg = sum(s.mean for s in buf) / n
                contrast_avg = sum(s.contrast for s in buf) / n

                rolling_sample = Sample(
                    side=uncorrected.side,
                    cam_id=uncorrected.cam_id,
                    frame_id=uncorrected.frame_id,
                    absolute_frame_id=uncorrected.absolute_frame_id,
                    timestamp_s=uncorrected.timestamp_s,
                    row_sum=0,
                    temperature_c=0.0,
                    mean=mean_avg,
                    std_dev=0.0,
                    contrast=contrast_avg,
                    bfi=0.0,
                    bvi=0.0,
                    is_corrected=False,
                    is_dark=False,
                )
                try:
                    self._on_rolling_avg_fn(rolling_sample)
                except Exception:
                    logger.exception("Error in on_rolling_avg_fn callback")

        # After the main loop the queue is fully drained.  The firmware
        # guarantees the very last frame of every scan is a dark (laser-off)
        # frame, but that terminal dark does not fall on a scheduled dark
        # position unless the scan happened to end exactly at the right length.
        # Flush any buffered intervals now so the corrected CSV is always
        # populated even for short scans.
        self._flush_terminal_dark()

    # ------------------------------------------------------------------
    # Dark-frame correction helpers
    # ------------------------------------------------------------------

    def _flush_terminal_dark(self) -> None:
        """Emit corrected batches for any cameras with buffered but un-corrected frames.

        Called once after the ingress queue fully drains.  The firmware
        guarantees the last frame of every scan is a dark (laser-off) frame, so
        the last entry in each camera's ``_pending_moments`` list is that
        terminal dark.  We promote it to a synthetic dark-history entry and
        call ``_emit_corrected_for_camera`` so the corrected CSV is populated
        even when the scan ended before the next scheduled dark position.

        If a camera has no dark history at all (scan stopped before frame 10)
        there is no baseline to correct against, so that camera is skipped.
        """
        for key, pending in list(self._pending_moments.items()):
            if not pending:
                continue
            dark_list = self._dark_history.get(key)
            if not dark_list:
                # No dark reference captured yet — cannot correct.
                logger.warning(
                    "SciencePipeline: scan ended before first dark frame for "
                    "%s cam %d — skipping terminal flush",
                    key[0], key[1],
                )
                continue

            # The last pending moment is the hardware-guaranteed terminal dark.
            terminal = pending[-1]
            terminal_var = max(0.0, terminal.u2 - terminal.u1 * terminal.u1)
            dark_list.append((
                terminal.absolute_frame_id,
                terminal.frame_id,
                terminal.timestamp_s,
                terminal.u1,
                terminal_var,
            ))
            # Remove it from pending so _emit_corrected_for_camera doesn't
            # include it as a corrected bright frame.
            self._pending_moments[key] = pending[:-1]

            logger.debug(
                "SciencePipeline: terminal dark flush for %s cam %d at "
                "absolute frame %d",
                key[0], key[1], terminal.absolute_frame_id,
            )
            # Same sanity check applies to the terminal dark — guards
            # against scans that ended without the firmware-guaranteed
            # laser-off frame.
            self._check_dark_integrity(
                key[0], key[1], terminal.absolute_frame_id,
                terminal.u1, terminal_var,
            )
            self._emit_corrected_for_camera(key)

    def _is_dark_frame(self, absolute_frame: int) -> bool:
        """Return True if *absolute_frame* is a scheduled dark frame.

        The first usable dark is at ``discard_count + 1`` (frame 10 by
        default).  Subsequent darks follow the firmware schedule: frames
        1, 1 + dark_interval, 1 + 2·dark_interval, … — but the very first
        (frame 1) is discarded as warmup noise.
        """
        if absolute_frame == self._discard_count + 1:
            return True
        return (
            absolute_frame > self._discard_count + 1
            and (absolute_frame - 1) % self._dark_interval == 0
        )

    def _calibrate_bfi_bvi(
        self, side: str, cam_id: int, contrast: float, mean_val: float,
    ) -> tuple[float, float]:
        """Compute calibrated BFI/BVI from corrected contrast and mean."""
        module_idx = 0 if side == "left" else 1
        cam_pos = int(cam_id) % 8

        if (
            module_idx >= self._bfi_c_min.shape[0]
            or cam_pos >= self._bfi_c_min.shape[1]
        ):
            return contrast * 10.0, mean_val * 10.0

        cmin = float(self._bfi_c_min[module_idx, cam_pos])
        cmax = float(self._bfi_c_max[module_idx, cam_pos])
        cden = (cmax - cmin) or 1.0
        bfi = (1.0 - ((contrast - cmin) / cden)) * 10.0

        imin = float(self._bfi_i_min[module_idx, cam_pos])
        imax = float(self._bfi_i_max[module_idx, cam_pos])
        iden = (imax - imin) or 1.0
        bvi = (1.0 - ((mean_val - imin) / iden)) * 10.0

        return float(bfi), float(bvi)

    def _emit_corrected_for_camera(self, key: tuple[str, int]) -> None:
        """Compute dark-frame-corrected BFI/BVI for one camera's interval.

        Applies linear interpolation of the dark baseline across the interval
        between the two most recent dark frames, then emits a ``CorrectedBatch``
        that includes:

        * All non-dark frames in the open interval (prev_dark, curr_dark),
          corrected by subtracting the linearly-interpolated dark baseline.
        * The ``prev_dark`` frame itself (Rule 2), whose corrected BFI/BVI
          are linearly interpolated between the last corrected sample of the
          *previous* batch (frame prev_dark − 1) and the first corrected
          sample of this batch (frame prev_dark + 1).  If no previous batch
          exists yet, the first non-dark frame value is repeated.

        The ``curr_dark`` frame is not included here; it becomes ``prev_dark``
        in the next batch and will be handled at that time.
        """
        dark_list = self._dark_history[key]
        prev_abs, prev_raw_fid, prev_ts, prev_u1, prev_var = dark_list[-2]
        curr_abs, _curr_raw_fid, _curr_ts, curr_u1, curr_var = dark_list[-1]

        pending = self._pending_moments.get(key, [])
        if not pending:
            return

        if self._log_dark_endpoints:
            side, cam_id = key
            logger.info(
                "dark endpoints for %s cam %d: "
                "frame %d  u1=%.2f  std=%.2f  ->  "
                "frame %d  u1=%.2f  std=%.2f",
                side, cam_id,
                prev_abs, prev_u1, float(np.sqrt(max(0.0, prev_var))),
                curr_abs, curr_u1, float(np.sqrt(max(0.0, curr_var))),
            )

        interval = curr_abs - prev_abs
        corrected_samples: list[Sample] = []

        for fm in pending:
            if fm.absolute_frame_id <= prev_abs or fm.absolute_frame_id >= curr_abs:
                continue

            # Linear interpolation weight ∈ (0, 1) across the open interval.
            # t = 0 corresponds to prev_dark, t = 1 to curr_dark.
            if interval > 1:
                t = (fm.absolute_frame_id - prev_abs) / interval
            else:
                t = 0.5

            # Interpolated dark baseline at this frame position
            dark_u1 = prev_u1 + (curr_u1 - prev_u1) * t
            dark_var = prev_var + (curr_var - prev_var) * t

            # Dark-corrected moments
            corrected_mean = fm.u1 - dark_u1
            raw_var = fm.u2 - fm.u1 * fm.u1
            corrected_var = raw_var - dark_var

            # Shot-noise correction: subtract the expected Poisson variance.
            # Shot noise variance in DN = ADC_GAIN · analog_gain · mean_DN.
            # Use max(0, corrected_mean) so a slightly negative corrected mean
            # (possible when dark subtraction over-corrects) does not inflate
            # the variance instead of reducing it.
            cam_pos = int(key[1]) % 8
            shot_noise_var = ADC_GAIN * max(0.0, corrected_mean) * CAMERA_GAIN_MAP[cam_pos]
            corrected_var -= shot_noise_var

            corrected_std = float(np.sqrt(max(0.0, corrected_var)))
            corrected_contrast = (
                corrected_std / corrected_mean if corrected_mean > 0 else 0.0
            )

            bfi, bvi = self._calibrate_bfi_bvi(
                key[0], key[1], corrected_contrast, corrected_mean,
            )

            corrected_samples.append(
                Sample(
                    side=key[0],
                    cam_id=key[1],
                    frame_id=fm.frame_id,
                    absolute_frame_id=fm.absolute_frame_id,
                    timestamp_s=fm.timestamp_s,
                    row_sum=fm.row_sum,
                    temperature_c=fm.temperature_c,
                    mean=float(corrected_mean),
                    std_dev=corrected_std,
                    contrast=float(corrected_contrast),
                    bfi=float(bfi),
                    bvi=float(bvi),
                    is_corrected=True,
                )
            )

        # Rule 2: corrected value for the prev_dark frame itself is computed
        # using the same 4-point quadratic stencil as the legacy pipeline:
        #
        #   v[dark] = (-1/6)*v[-2] + (2/3)*v[-1] + (2/3)*v[+1] + (-1/6)*v[+2]
        #
        # where v[-1]/v[-2] are the last two corrected samples of the previous
        # batch and v[+1]/v[+2] are the first two corrected samples of this
        # batch.  Falls back gracefully when fewer neighbours are available:
        #   - Only v[-1] and v[+1] available  → simple average (linear)
        #   - No left neighbours at all         → repeat v[+1]
        if corrected_samples:
            right1 = corrected_samples[0]                          # prev_abs + 1
            right2 = corrected_samples[1] if len(corrected_samples) >= 2 else None
            prev_batch = self._last_corrected.get(key, [])
            left1 = prev_batch[-1] if len(prev_batch) >= 1 else None   # prev_abs - 1
            left2 = prev_batch[-2] if len(prev_batch) >= 2 else None   # prev_abs - 2

            def _quad(attr):
                r1 = getattr(right1, attr)
                r2 = getattr(right2, attr) if right2 is not None else None
                l1 = getattr(left1,  attr) if left1  is not None else None
                l2 = getattr(left2,  attr) if left2  is not None else None
                if l1 is not None and r2 is not None and l2 is not None:
                    # Full 4-point stencil
                    return (-1/6)*l2 + (2/3)*l1 + (2/3)*r1 + (-1/6)*r2
                elif l1 is not None:
                    # Linear fallback: only immediate neighbours
                    return (l1 + r1) / 2.0
                else:
                    # No left history (first interval): repeat right neighbour
                    return r1

            dark_corrected = Sample(
                side=key[0],
                cam_id=key[1],
                frame_id=prev_raw_fid,
                absolute_frame_id=prev_abs,
                timestamp_s=prev_ts,
                row_sum=right1.row_sum,
                temperature_c=right1.temperature_c,
                mean=_quad("mean"),
                std_dev=_quad("std_dev"),
                contrast=_quad("contrast"),
                bfi=_quad("bfi"),
                bvi=_quad("bvi"),
                is_corrected=True,
            )
            # Insert at front so the batch is in chronological order.
            corrected_samples.insert(0, dark_corrected)

            # Keep the two most recent corrected samples for the next interval.
            tail = corrected_samples[-2:] if len(corrected_samples) >= 2 else corrected_samples[-1:]
            self._last_corrected[key] = tail

        # Keep only moments that fall after the current dark (shouldn't
        # normally happen, but guards against edge-case ordering).
        self._pending_moments[key] = [
            fm for fm in pending if fm.absolute_frame_id >= curr_abs
        ]

        if corrected_samples and self._on_corrected_batch_fn:
            batch = CorrectedBatch(
                dark_frame_start=prev_abs,
                dark_frame_end=curr_abs,
                samples=corrected_samples,
            )
            try:
                self._on_corrected_batch_fn(batch)
            except Exception:
                logger.exception("Error in on_corrected_batch_fn callback")

def create_science_pipeline(
    *,
    left_camera_mask: int = 0xFF,
    right_camera_mask: int = 0xFF,
    bfi_c_min,
    bfi_c_max,
    bfi_i_min,
    bfi_i_max,
    on_uncorrected_fn: Callable[[Sample], None] | None = None,
    on_corrected_batch_fn: Callable[[CorrectedBatch], None] | None = None,
    on_dark_frame_fn: Callable[[Sample], None] | None = None,
    on_rolling_avg_fn: Callable[[Sample], None] | None = None,
    rolling_avg_enabled: bool = False,
    rolling_avg_window: int = 10,
    dark_interval: int = 600,
    discard_count: int = 9,
    expected_row_sum: int | None = None,
    noise_floor: int = 10,
    log_dark_endpoints: bool = False,
    dark_integrity_max_u1_above_pedestal: float = 30.0,
    dark_integrity_max_std: float = 20.0,
) -> SciencePipeline:
    """
    Factory for a ready-to-run unified science pipeline.

    Parameters
    ----------
    on_uncorrected_fn
        Fires immediately per non-dark frame with uncorrected BFI/BVI.
    on_corrected_batch_fn
        Fires once per dark-frame interval with a ``CorrectedBatch``
        containing dark-frame-corrected samples for the entire interval.
    on_dark_frame_fn
        Fires once per scheduled dark frame with a ``Sample`` whose
        ``is_dark=True``.  ``mean`` is pedestal-subtracted using
        ``PEDESTAL_HEIGHT``; ``std_dev = sqrt(variance)`` from raw moments;
        ``contrast = std_dev / mean`` when ``mean > 0`` else ``0``;
        ``bfi`` and ``bvi`` are 0 (not meaningful on a dark frame).
        Registration-gated — pass None to disable (default).
    on_rolling_avg_fn
        When ``rolling_avg_enabled`` is True, fires once per uncorrected
        light frame per camera with a ``Sample`` whose ``mean`` and
        ``contrast`` are the arithmetic means over the last
        ``rolling_avg_window`` light samples for that (side, cam_id).
        Other numeric fields are zeroed.  Dark frames never enter the
        window.  Partial windows emit (no wait for N samples to fill).
    rolling_avg_enabled
        When True, activates the rolling-average stage.  Default False —
        no buffer is allocated and ``on_rolling_avg_fn`` is never invoked.
    rolling_avg_window
        Window size N (default 10).  Ignored when
        ``rolling_avg_enabled`` is False.
    dark_interval
        Frames between dark frames (default 600 = 15 s at 40 Hz).
    discard_count
        Number of initial warmup frames to discard (default 9).
    expected_row_sum
        When not None, samples whose histogram bin sum does not match this
        value are discarded.
    noise_floor
        Histogram bins with a count strictly below this value are zeroed
        before moment computation (default 74).  Set to 0 to disable.
    """
    pipeline = SciencePipeline(
        left_camera_mask=left_camera_mask,
        right_camera_mask=right_camera_mask,
        bfi_c_min=bfi_c_min,
        bfi_c_max=bfi_c_max,
        bfi_i_min=bfi_i_min,
        bfi_i_max=bfi_i_max,
        on_uncorrected_fn=on_uncorrected_fn,
        on_corrected_batch_fn=on_corrected_batch_fn,
        on_dark_frame_fn=on_dark_frame_fn,
        on_rolling_avg_fn=on_rolling_avg_fn,
        rolling_avg_enabled=rolling_avg_enabled,
        rolling_avg_window=rolling_avg_window,
        dark_interval=dark_interval,
        discard_count=discard_count,
        expected_row_sum=expected_row_sum,
        noise_floor=noise_floor,
        log_dark_endpoints=log_dark_endpoints,
        dark_integrity_max_u1_above_pedestal=dark_integrity_max_u1_above_pedestal,
        dark_integrity_max_std=dark_integrity_max_std,
    )
    pipeline.start()
    return pipeline


def feed_pipeline_from_csv(
    csv_path: str,
    side: str,
    pipeline: "SciencePipeline",
) -> int:
    """
    Read a raw histogram CSV file and enqueue every row into a SciencePipeline.

    The CSV must have at minimum the columns produced by the raw-histogram
    writer: ``cam_id``, ``frame_id``, ``timestamp_s``, histogram bins labeled
    ``"0"`` through ``"1023"``, ``temperature``, and ``sum``.  Extra columns
    such as ``tcm``, ``tcl``, and ``pdc`` are silently ignored.

    Parameters
    ----------
    csv_path
        Path to the raw histogram CSV file.
    side
        Sensor side — ``"left"`` or ``"right"``.
    pipeline
        A running :class:`SciencePipeline` instance (created via
        :func:`create_science_pipeline`).

    Returns
    -------
    int
        Number of rows enqueued into the pipeline.
    """
    rows_fed = 0
    with open(csv_path, "r", newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            cam_id = int(row["cam_id"])
            frame_id = int(row["frame_id"])
            timestamp_s = float(row["timestamp_s"])
            temperature_c = float(row.get("temperature", 0.0))
            hist = np.array(
                [int(row[str(i)]) for i in range(HISTO_SIZE_WORDS)],
                dtype=np.uint32,
            )
            row_sum = int(row["sum"])
            pipeline.enqueue(
                side, cam_id, frame_id, timestamp_s, hist, row_sum, temperature_c
            )
            rows_fed += 1
    return rows_fed

