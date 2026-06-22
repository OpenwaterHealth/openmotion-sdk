"""Shared async framed-packet transport for the OpenMotion command links.

Both command transports — the console's serial VCP (:class:`omotion.MotionUart`)
and the sensor's USB-bulk interface 0 (:class:`omotion.CommInterface`) — speak
the same framed protocol and both receive unsolicited firmware log packets
interleaved with command responses. They differ only in the raw byte I/O and
lifecycle, so the threading / framing / dispatch logic lives here once:

  * a **reader thread** continuously drains the link (``_raw_read``),
  * a **dispatch thread** carves frames (:mod:`omotion.framing`) off the stream
    and routes them — firmware log packets are surfaced via the logger, command
    responses go on a queue,
  * :meth:`send_packet` writes a request and waits for the response with the
    matching id, unblocking promptly if the transport drops.

A continuous reader is what lets unsolicited log packets share the command
channel without colliding with responses — the reason the sensor tolerated
firmware printf long before the console did.

Subclasses implement ``_raw_read()`` / ``_raw_write()`` plus their own
open/close, and may override the throttle hooks to pace commands.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable, Optional

import omotion.config as config
from omotion import _log_root
from omotion.UartPacket import UartPacket
from omotion.config import OW_ACK, OW_CMD_NOP
from omotion.framing import emit_log_packet, extract_frame, is_log_packet

logger = logging.getLogger(f"{_log_root}.transport" if _log_root else "transport")


# Pretty names for TX debug logging, derived from the config enums.
_PACKET_TYPE_NAMES = {
    value: name
    for name, value in vars(config).items()
    if name.startswith("OW_") and name.isupper() and isinstance(value, int)
}
_CMD_NAMES = {
    "OW_CMD": {v: n for n, v in vars(config).items() if n.startswith("OW_CMD_")},
    "OW_CONTROLLER": {v: n for n, v in vars(config).items() if n.startswith("OW_CTRL_")},
    "OW_FPGA": {v: n for n, v in vars(config).items() if n.startswith("OW_FPGA_")},
    "OW_CAMERA": {v: n for n, v in vars(config).items() if n.startswith("OW_CAMERA_")},
    "OW_IMU": {v: n for n, v in vars(config).items() if n.startswith("OW_IMU_")},
}


def _format_named(value: int, name_map: dict, width: int = 2) -> str:
    name = name_map.get(value)
    return f"{name}(0x{value:0{width}X})" if name else f"0x{value:0{width}X}"


class PacketTransport:
    """Transport-agnostic async command/response engine. See module docstring."""

    def __init__(
        self,
        desc: str = "XPORT",
        default_timeout: float = 10.0,
        on_io_error: Optional[Callable[[Optional[int], str], None]] = None,
    ):
        self.desc = desc
        self.default_timeout = default_timeout
        self.on_io_error = on_io_error
        self.packet_count = 0

        self.stop_event = threading.Event()
        # Latched when the reader exits on a fatal transport error so an
        # in-flight send_packet unblocks immediately instead of waiting out
        # its full timeout (root cause of bloodflow-app#130 on the sensor).
        self._transport_down_evt = threading.Event()

        # Reader appends raw bytes; the dispatcher carves frames off the front.
        self._read_buffer = bytearray()
        self._buffer_lock = threading.Lock()
        self._buffer_condition = threading.Condition(self._buffer_lock)

        self._io_lock = threading.RLock()
        self._send_lock = threading.Lock()

        self.read_thread: Optional[threading.Thread] = None
        self.response_thread: Optional[threading.Thread] = None
        self.response_queue: queue.Queue = queue.Queue()

    # ────────────────────────────────────────────────────────────────────
    # Raw byte I/O — subclass provides
    # ────────────────────────────────────────────────────────────────────

    def _raw_read(self) -> bytes:
        """Return bytes read from the link (``b""`` on idle/timeout). Raise on a
        fatal/disconnect error — the reader loop treats that as transport-down."""
        raise NotImplementedError

    def _raw_write(self, data: bytes) -> None:
        """Write raw bytes to the link. Raise on a fatal error."""
        raise NotImplementedError

    # ────────────────────────────────────────────────────────────────────
    # Thread lifecycle
    # ────────────────────────────────────────────────────────────────────

    def start_read_thread(self) -> None:
        # A fresh reader implies the transport is up again; clear any stale
        # transport-down latch so subsequent sends aren't poisoned.
        self.stop_event.clear()
        self._transport_down_evt.clear()
        if self.response_thread is None or not self.response_thread.is_alive():
            self.response_thread = threading.Thread(
                target=self._process_responses, daemon=True
            )
            self.response_thread.start()
        if self.read_thread is not None and self.read_thread.is_alive():
            logger.info("%s: read thread already running", self.desc)
            return
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        logger.info("%s: read thread started", self.desc)

    def stop_read_thread(self) -> None:
        """Signal both worker threads to exit and join them (unless called from
        one of them, in which case that thread is already on its way out)."""
        self.stop_event.set()
        with self._buffer_condition:
            self._buffer_condition.notify_all()
        for t in (self.read_thread, self.response_thread):
            if t is not None and threading.current_thread() is not t:
                t.join(timeout=2.0)
        logger.info("%s: read thread stopped", self.desc)

    # ────────────────────────────────────────────────────────────────────
    # Core loops
    # ────────────────────────────────────────────────────────────────────

    def _read_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                data = self._raw_read()
            except Exception as e:  # noqa: BLE001 — any read failure is fatal here
                if self.stop_event.is_set():
                    break
                logger.warning("%s: read error; exiting read loop: %s", self.desc, e)
                # Latch transport-down BEFORE the callback so any send_packet
                # spinning in its wait loop observes the dead transport next tick.
                self._transport_down_evt.set()
                self._notify_io_error(e)
                break
            if data:
                with self._buffer_condition:
                    self._read_buffer.extend(data)
                    self._buffer_condition.notify()

    def _process_responses(self) -> None:
        while not self.stop_event.is_set():
            with self._buffer_condition:
                if not self._read_buffer:
                    self._buffer_condition.wait(timeout=0.1)
                    continue
                # Carve one complete frame off the front (resyncs past garbage).
                frame = extract_frame(self._read_buffer)
            if frame is None:
                continue  # partial frame — wait for the reader to add more
            try:
                pkt = UartPacket(buffer=frame)
            except ValueError:
                continue  # bad CRC / malformed — drop it
            # Unsolicited firmware log packets are surfaced and dropped; only
            # real command responses go to the queue for send_packet to match.
            if is_log_packet(pkt):
                emit_log_packet(pkt, self.desc)
            else:
                self.response_queue.put(pkt)

    # ────────────────────────────────────────────────────────────────────
    # Send
    # ────────────────────────────────────────────────────────────────────

    def _build_packet(self, id, packetType, command, addr, reserved, data):
        """Resolve the id (auto-increment when None), build the frame bytes, and
        log the TX. Call inside ``_send_lock`` (it bumps ``packet_count``)."""
        if id is None:
            self.packet_count = (self.packet_count + 1) & 0xFFFF or 1
            id = self.packet_count
        if data:
            if not isinstance(data, (bytes, bytearray)):
                raise ValueError("Data must be bytes or bytearray")
            payload = data
        else:
            payload = b""
        pkt = UartPacket(
            id=id, packetType=packetType, command=command,
            addr=addr, reserved=reserved, data=payload,
        )
        tx = pkt.to_bytes()
        cmd_names = _CMD_NAMES.get(_PACKET_TYPE_NAMES.get(packetType), {})
        logger.debug(
            "%s: TX id=0x%04X type=%s cmd=%s addr=0x%02X reserved=0x%02X len=%d",
            self.desc, id, _format_named(packetType, _PACKET_TYPE_NAMES),
            _format_named(command, cmd_names), addr, reserved, len(payload),
        )
        return id, tx

    def send_packet(
        self,
        id=None,
        packetType=OW_ACK,
        command=OW_CMD_NOP,
        addr=0,
        reserved=0,
        data=None,
        timeout=None,
        max_retries=0,
    ) -> UartPacket:
        """Write a command and return the response with the matching id.

        Raises ``ConnectionError`` if the transport drops mid-wait, ``TimeoutError``
        if no matching response arrives within ``timeout`` (defaults to
        ``self.default_timeout``), and ``ValueError`` on bad arguments.
        """
        timeout = self.default_timeout if timeout is None else timeout
        with self._send_lock:
            self._pace_before_send()
            try:
                id, tx = self._build_packet(id, packetType, command, addr, reserved, data)
                self._raw_write(tx)
                time.sleep(0.0005)
                start = time.monotonic()
                while time.monotonic() - start < timeout:
                    if self._transport_down_evt.is_set():
                        raise ConnectionError(
                            f"{self.desc}: transport down, packet id 0x{id:04X} "
                            "not deliverable"
                        )
                    try:
                        resp = self.response_queue.get(timeout=0.02)
                    except queue.Empty:
                        continue
                    if resp.id != id:
                        logger.warning(
                            "%s: discarding stale response id=0x%04X (expected 0x%04X)",
                            self.desc, resp.id, id,
                        )
                        continue
                    return resp
                raise TimeoutError(
                    f"{self.desc}: no response for packet id 0x{id:04X}"
                )
            finally:
                self._mark_send_done()

    def _pace_before_send(self) -> None:
        """Hook (called holding the send lock): subclasses may throttle the
        command rate. Default: no throttling."""

    def _mark_send_done(self) -> None:
        """Hook (called holding the send lock): subclasses may record the
        send-completion time for rate throttling. Default: no-op."""

    def clear_buffer(self) -> None:
        """Drop any partially-received frame. Safe to call from any thread —
        the reader thread is the only other toucher and it locks the same
        condition."""
        with self._buffer_condition:
            self._read_buffer.clear()

    def _notify_io_error(self, error) -> None:
        cb = self.on_io_error
        if cb is None:
            return
        errno = getattr(error, "errno", None)
        try:
            cb(errno, str(error))
        except Exception as e:
            logger.warning("on_io_error callback raised: %s", e)
