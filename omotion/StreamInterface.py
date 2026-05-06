import logging
import time
import usb.core
import usb.util
import threading
from omotion.USBInterfaceBase import USBInterfaceBase
from omotion.config import TYPE_HISTO, TYPE_HISTO_CMP
from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.StreamInterface" if _log_root else "StreamInterface"
)


def _rle_decompress(data: bytes) -> bytes:
    """Decompress PackBits-style byte-level RLE data."""
    result = bytearray()
    i = 0
    n = len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 0x80:
            # Literal run: (ctrl + 1) bytes follow
            count = ctrl + 1
            result.extend(data[i : i + count])
            i += count
        else:
            # Repeat run: next byte repeated (ctrl - 0x80 + 3) times
            count = ctrl - 0x80 + 3
            result.extend(bytes([data[i]]) * count)
            i += 1
    return bytes(result)


_HEADER_SIZE = 6      # SOF(1) + type(1) + size(4)
_FOOTER_SIZE = 3      # CRC(2) + EOF(1)
# TYPE_HISTO_CMP packets have an extra 2-byte CRC-16 of the *uncompressed*
# payload inserted immediately before the normal footer.
_UNCMP_CRC_SIZE = 2


import binascii as _binascii


def _util_crc16(buf) -> int:
    """CRC-CCITT (polynomial 0x1021, init 0xFFFF) via the C implementation in binascii."""
    return _binascii.crc_hqx(buf, 0xFFFF)


def _decompress_histo_cmp(raw: bytes) -> bytes:
    """
    Given a raw TYPE_HISTO_CMP packet, decompress the payload and return
    a reconstructed TYPE_HISTO packet (so downstream consumers are unaffected).

    Packet layout (TYPE_HISTO_CMP):
      [Header 6B][Compressed payload N B][UNCMP_CRC16 2B][PKT_CRC16 2B][EOF 1B]

    Two CRCs are checked:
      1. PKT_CRC16  – covers header + compressed payload + UNCMP_CRC16 (transport integrity)
      2. UNCMP_CRC16 – covers the *decompressed* payload (decompressor correctness)

    Raises ValueError on any integrity failure.
    """
    if len(raw) < _HEADER_SIZE + _UNCMP_CRC_SIZE + _FOOTER_SIZE:
        raise ValueError("Compressed packet too small")

    # ── 1. Verify transport CRC (covers everything before PKT_CRC) ──
    footer_off = len(raw) - _FOOTER_SIZE        # offset of PKT_CRC16
    pkt_crc_expected = struct.unpack_from("<H", raw, footer_off)[0]
    if raw[footer_off + 2] != 0xDD:
        raise ValueError("Compressed packet missing EOF marker")
    pkt_crc_actual = _util_crc16(raw[: footer_off - 1])   # matches firmware range
    if pkt_crc_actual != pkt_crc_expected:
        raise ValueError(
            f"Compressed packet CRC mismatch "
            f"(got 0x{pkt_crc_actual:04X}, expected 0x{pkt_crc_expected:04X})"
        )

    # ── 2. Extract UNCMP_CRC16 (sits just before the footer) ──
    uncmp_crc_off = footer_off - _UNCMP_CRC_SIZE
    uncmp_crc_expected = struct.unpack_from("<H", raw, uncmp_crc_off)[0]

    # ── 3. Decompress (compressed payload is between header and UNCMP_CRC) ──
    compressed_payload = raw[_HEADER_SIZE : uncmp_crc_off]
    decompressed = _rle_decompress(compressed_payload)

    # ── 4. Verify decompressed payload CRC ──
    uncmp_crc_actual = _util_crc16(decompressed[:-1])   # same off-by-one as firmware
    if uncmp_crc_actual != uncmp_crc_expected:
        raise ValueError(
            f"Decompressed payload CRC mismatch "
            f"(got 0x{uncmp_crc_actual:04X}, expected 0x{uncmp_crc_expected:04X}) "
            f"— decompressor produced wrong output"
        )

    # ── 5. Rebuild as a TYPE_HISTO packet ──
    new_total = _HEADER_SIZE + len(decompressed) + _FOOTER_SIZE
    header = struct.pack("<BBI", raw[0], TYPE_HISTO, new_total)

    # Recompute CRC for the reconstructed packet (excluding last byte, matching firmware)
    crc_data = header + decompressed
    crc = _util_crc16(crc_data[: len(crc_data) - 1])
    footer = struct.pack("<HB", crc, 0xDD)

    return header + decompressed + footer


# =========================================
# Stream Interface (IN only + thread + queue)
# =========================================
class StreamInterface(USBInterfaceBase):
    def __init__(self, dev, interface_index, desc="Stream"):
        super().__init__(dev, interface_index, desc)
        self.thread = None
        self.stop_event = threading.Event()
        self.data_queue = None
        self.expected_size = None
        self.isStreaming = False
        self.packets_received: int = 0  # USB transfers queued since last start_streaming

    def start_streaming(self, queue_obj, expected_size):
        if self.thread and self.thread.is_alive():
            logger.info(f"{self.desc}: Stream already running")
            return
        self.data_queue = queue_obj
        self.expected_size = expected_size
        self.packets_received = 0
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()
        self.isStreaming = True
        logger.info(f"{self.desc}: Streaming started")

    def stop_streaming(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        self.isStreaming = False
        self.data_queue = None
        self.expected_size = None
        logger.info(
            f"{self.desc}: Streaming stopped — "
            f"{self.packets_received} USB read chunk(s) received"
        )

    def flush_stale_data(
        self,
        expected_size: int,
        read_timeout_ms: int = 50,
        max_total_ms: int = 1500,
    ) -> int:
        """
        Drain and discard any data already buffered in the USB host-side
        endpoint from a previous streaming session.

        Call this *before* ``start_streaming()`` at scan startup, while the
        MCU trigger is still off, so that leftover USB transfers from the
        prior scan cannot appear at the top of the new scan's CSV.

        The flush works by issuing blocking reads (with a short timeout) until
        the endpoint returns a timeout error — at which point the buffer is
        empty and no more data is expected before the trigger fires.

        Parameters
        ----------
        expected_size
            Read buffer size passed to ``dev.read()``.  Use the same value as
            the upcoming ``start_streaming()`` call (i.e. ``request.expected_size``).
        read_timeout_ms
            Milliseconds to wait per read attempt.  Should be short — just
            long enough for the USB host controller to confirm the endpoint is
            empty.  Default 50 ms is sufficient for all known configurations.
        max_total_ms
            Hard cap on total flush duration.  Prevents startup hangs if the
            backend returns empty reads indefinitely or data keeps arriving.

        Returns
        -------
        int
            Number of bytes discarded.
        """
        if self.isStreaming:
            logger.warning(f"{self.desc}: flush_stale_data called while streaming — skipping")
            return 0

        if self.ep_in is None:
            logger.warning(f"{self.desc}: flush_stale_data called before endpoint claimed — skipping")
            return 0

        bytes_discarded = 0
        reads = 0
        t_start = time.monotonic()
        while True:
            if int((time.monotonic() - t_start) * 1000) >= max_total_ms:
                logger.warning(
                    f"{self.desc}: flush_stale_data reached max duration "
                    f"({max_total_ms} ms), proceeding with stream startup"
                )
                break
            try:
                data = self.dev.read(
                    self.ep_in.bEndpointAddress, expected_size, timeout=read_timeout_ms
                )
                reads += 1
                if data:
                    bytes_discarded += len(data)
                else:
                    # Some backends can return an empty read rather than raising
                    # a timeout USBError. Treat as endpoint-empty and stop flush.
                    break
            except usb.core.USBError as e:
                if e.errno in (110, 10060):
                    # Timeout — endpoint buffer is now empty.
                    break
                elif e.errno in (19, 5, 32):
                    # Device lost during flush — stop silently.
                    logger.warning(f"{self.desc}: device error during flush: {e}")
                    break
                else:
                    logger.warning(f"{self.desc}: USB error during flush: {e}")
                    break

        if bytes_discarded:
            logger.info(
                f"{self.desc}: flushed {bytes_discarded} stale bytes "
                f"({bytes_discarded // expected_size} transfer(s)) from USB endpoint"
            )
        elif reads > 0:
            logger.info(f"{self.desc}: stale-data flush complete ({reads} read attempt(s), no payload)")
        return bytes_discarded

    def drain_final(
        self,
        expected_size: int,
        timeout_ms: int = 750,
    ) -> list[bytes]:
        """
        After ``stop_streaming()`` has returned, attempt one or more reads to
        recover any USB transfers that landed in the host-side endpoint buffer
        after ``_stream_loop`` exited.

        This handles the race where the MCU delivers its final bulk transfer
        significantly later than the normal inter-frame cadence (e.g. > 350 ms
        after trigger-off), causing ``_stream_loop`` to exit on a timeout+stop
        before the transfer arrives.

        Parameters
        ----------
        expected_size
            Read buffer size — same value used for ``start_streaming()``.
        timeout_ms
            How long to wait per read attempt.  750 ms is comfortably beyond
            the ~250 ms worst-case MCU DMA flush latency.

        Returns
        -------
        list[bytes]
            Byte chunks recovered (0 or 1 items in the normal case).
        """
        if self.isStreaming:
            logger.warning(f"{self.desc}: drain_final called while streaming — skipping")
            return []
        if self.ep_in is None:
            logger.warning(f"{self.desc}: drain_final called before endpoint claimed — skipping")
            return []

        chunks: list[bytes] = []
        while True:
            try:
                data = self.dev.read(
                    self.ep_in.bEndpointAddress, expected_size, timeout=timeout_ms
                )
                if data:
                    chunks.append(bytes(data))
                    logger.info(
                        f"{self.desc}: drain_final recovered {len(data)} bytes "
                        f"(chunk {len(chunks)})"
                    )
            except usb.core.USBError as e:
                if e.errno in (110, 10060):
                    # Timeout — endpoint is empty, nothing more to recover.
                    break
                elif e.errno in (19, 5, 32):
                    logger.warning(f"{self.desc}: device error during drain_final: {e}")
                    break
                else:
                    logger.warning(f"{self.desc}: USB error during drain_final: {e}")
                    break

        if chunks:
            logger.info(
                f"{self.desc}: drain_final recovered {len(chunks)} chunk(s) "
                f"({sum(len(c) for c in chunks)} bytes total)"
            )
        return chunks

    def _process_packet(self, raw, pkt_count, cmp_count, cmp_errors):
        """Process a single framed packet: decompress if needed, queue it.
        Returns (cmp_count, cmp_errors)."""
        pkt_type = raw[1] if len(raw) > 1 else -1

        if pkt_type == TYPE_HISTO_CMP:
            cmp_count += 1
            compressed_size = len(raw)
            try:
                raw = _decompress_histo_cmp(raw)
                decompressed_size = len(raw)
                if cmp_count <= 3 or cmp_count % 100 == 0:
                    logger.info(
                        f"{self.desc}: [CMP] pkt#{pkt_count} decompressed "
                        f"{compressed_size} -> {decompressed_size} bytes "
                        f"({cmp_count} compressed so far)"
                    )
            except Exception as exc:
                cmp_errors += 1
                logger.error(
                    f"{self.desc}: [CMP] decompression FAILED pkt#{pkt_count}, "
                    f"compressed_size={compressed_size}, "
                    f"error={exc} (errors: {cmp_errors}/{cmp_count})"
                )
                return cmp_count, cmp_errors  # drop corrupted packet
        elif pkt_type not in (TYPE_HISTO, TYPE_HISTO_CMP) and pkt_count <= 5:
            logger.warning(
                f"{self.desc}: unexpected pkt_type=0x{pkt_type:02X}, "
                f"len={len(raw)}, pkt#{pkt_count}"
            )

        if self.data_queue:
            self.data_queue.put(raw)
        return cmp_count, cmp_errors

    def _stream_loop(self):
        # Read timeout must exceed the worst-case USB transfer latency for the
        # final frame.  Normal cadence is ~25 ms; the last frame of a scan can
        # take up to 250 ms because the MCU flushes its DMA buffer only after
        # the trigger is stopped.  500 ms gives a comfortable 2x margin.
        #
        # Exit condition: stop was requested AND the last read timed out.
        # A timeout while stop is pending means the endpoint is empty — all
        # in-flight transfers have been received.  Exiting on stop_event alone
        # (the old behaviour) caused the final frame to be dropped whenever it
        # arrived after the 100 ms read window had already closed.
        _READ_TIMEOUT_MS = 500

        while True:
            # Check stop FIRST so a stop-then-clear race in stop_streaming
            # (which nulls expected_size/data_queue after join times out)
            # can't drive us back into dev.read with None args.
            if self.stop_event.is_set():
                break
            # Snapshot the read parameters under the assumption stop_streaming
            # may null them out concurrently. If they're already None the
            # streaming session is over — exit instead of crashing inside
            # libusb on `length * b'\x00'`.
            expected_size = self.expected_size
            data_queue = self.data_queue
            if expected_size is None or data_queue is None:
                break
            try:
                data = self.dev.read(
                    self.ep_in.bEndpointAddress, expected_size,
                    timeout=_READ_TIMEOUT_MS,
                )
                if data and data_queue is self.data_queue:
                    data_queue.put(bytes(data))
                    self.packets_received += 1
            except usb.core.USBError as e:
                if e.errno in (110, 10060):
                    # Timeout — no data arrived within the read window.
                    if self.stop_event.is_set():
                        # Stop requested and endpoint is now empty: exit cleanly.
                        break
                    # Otherwise keep waiting — scan is still running.
                elif e.errno in (19, 5, 32):
                    # Fatal device errors: ENODEV, EIO, EPIPE — device is gone.
                    logger.error(f"{self.desc} stream error (device lost): {e}")
                    break
                else:
                    logger.error(f"{self.desc} stream error: {e}")
                    if self.stop_event.is_set():
                        break
