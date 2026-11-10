"""Pure UART transport (synchronous request/response).

`MotionUart` no longer owns connection lifecycle — the owning handle
(`MotionConsole`) drives `open(port)` / `close()` from its state machine.
On a fatal serial error during reads or writes, the transport invokes the
`on_io_error(errno, message)` callback; the handle's state machine is
expected to react by transitioning out of CONNECTED.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

import serial
import serial.tools.list_ports

from omotion.UartPacket import UartPacket
from omotion.config import (
    OW_ACK,
    OW_CMD_ECHO,
    OW_CMD_NOP,
    CONSOLE_MAX_PAYLOAD,
    OW_DATA,
    OW_END_BYTE,
    OW_ERROR,
    OW_START_BYTE,
)
from omotion.utils import util_crc16
from omotion import _log_root
from omotion.CommandError import CommandError

logger = logging.getLogger(f"{_log_root}.UART" if _log_root else "UART")


class MotionUart:
    def __init__(
        self,
        vid: int,
        pid: int,
        baudrate: int = 921600,
        timeout: int = 10,
        align: int = 0,
        demo_mode: bool = False,
        desc: str = "VCP",
        on_io_error: Optional[Callable[[Optional[int], str], None]] = None,
        max_payload: int = CONSOLE_MAX_PAYLOAD,
    ):
        self.vid = vid
        self.pid = pid
        self.port: Optional[str] = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.align = align
        self.packet_count = 0
        self.demo_mode = demo_mode
        self.descriptor = desc
        self.serial: Optional[serial.Serial] = None
        self._io_lock = threading.RLock()
        self.on_io_error = on_io_error
        # Largest payload the device accepts; a longer one is refused in
        # send_packet rather than NAKed by the firmware.
        self.max_payload = max_payload
        # Minimum spacing between the end of one command and the TX of the
        # next — console firmware drops commands sent back-to-back (see
        # send_packet).
        self._min_cmd_gap = 0.02
        self._last_cmd_ts = 0.0

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle (driven by MotionConsole state machine)
    # ────────────────────────────────────────────────────────────────────

    def open(self, port: str) -> None:
        """Open the serial port. Raises serial.SerialException on failure."""
        if self.demo_mode:
            self.port = port or "DEMO"
            return
        # Short blocking-read timeout: read_packet waits for data with
        # select()-backed read() calls in a deadline loop, so the per-read
        # timeout just sets the poll granularity. (Long blocking reads would
        # pin the io-lock; pure in_waiting polling proved unreliable on
        # macOS CDC drivers.)
        self.serial = serial.Serial(
            port=port, baudrate=self.baudrate, timeout=0.05
        )
        self.port = port
        logger.info("UART %s opened on %s", self.descriptor, port)

    def close(self) -> None:
        """Close the serial port. Idempotent."""
        if self.demo_mode:
            self.port = None
            return
        s = self.serial
        self.serial = None
        if s is not None:
            try:
                if s.is_open:
                    s.close()
            except Exception as e:
                logger.debug("serial.close raised: %s", e)
        if self.port is not None:
            logger.info("UART %s closed (was on %s)", self.descriptor, self.port)
        self.port = None

    def is_open(self) -> bool:
        if self.demo_mode:
            return self.port is not None
        return self.serial is not None and self.serial.is_open

    # ────────────────────────────────────────────────────────────────────
    # VID/PID discovery
    # ────────────────────────────────────────────────────────────────────

    def find_port(self) -> Optional[str]:
        """Return the COM/tty device path that matches our VID/PID, or None."""
        for p in serial.tools.list_ports.comports():
            if (
                getattr(p, "vid", None) == self.vid
                and getattr(p, "pid", None) == self.pid
            ):
                return p.device
        return None

    # ────────────────────────────────────────────────────────────────────
    # I/O
    # ────────────────────────────────────────────────────────────────────

    def _notify_io_error(self, errno: Optional[int], message: str) -> None:
        cb = self.on_io_error
        if cb is None:
            return
        try:
            cb(errno, message)
        except Exception as e:
            logger.warning("on_io_error callback raised: %s", e)

    def _tx(self, data: bytes) -> None:
        if self.demo_mode:
            logger.debug("Demo mode TX: %s", data.hex())
            return
        if self.serial is None or not self.serial.is_open:
            raise CommandError("UART not open")
        try:
            with self._io_lock:
                if self.align > 0:
                    while len(data) % self.align != 0:
                        data += bytes([OW_END_BYTE])
                self.serial.write(data)
        except serial.SerialException as se:
            errno = getattr(se, "errno", None)
            self._notify_io_error(errno, str(se))
            raise

    def read_packet(self, timeout: int = 20) -> UartPacket:
        """Block until a packet arrives or `timeout` seconds elapse."""
        if self.demo_mode:
            return UartPacket(
                id=0, packetType=OW_ERROR, command=0, addr=0, reserved=0, data=[]
            )
        if self.serial is None:
            raise CommandError("UART not open")
        # Frame on packet structure rather than read timing: OS serial
        # drivers chunk CDC data arbitrarily (macOS especially), so "no new
        # bytes for one poll tick" is not a packet boundary. Header is
        # start(1) id(2) type(1) cmd(1) addr(1) reserved(1) len(2); trailer
        # is crc(2) end(1).
        HEADER_LEN = 9
        TRAILER_LEN = 3
        MAX_DATA_LEN = 8192
        with self._io_lock:
            start_time = time.monotonic()
            raw_data = b""
            expected = None

            while timeout == -1 or time.monotonic() - start_time < timeout:
                try:
                    # Blocking single-byte read (select-backed, up to the
                    # port's 50 ms timeout), then drain whatever else has
                    # arrived. read_all()/in_waiting alone can miss arrivals
                    # on macOS CDC drivers.
                    chunk = self.serial.read(1)
                    if chunk:
                        waiting = self.serial.in_waiting
                        if waiting:
                            chunk += self.serial.read(waiting)
                except serial.SerialException as se:
                    self._notify_io_error(getattr(se, "errno", None), str(se))
                    raise
                if chunk:
                    raw_data += chunk
                # Resync: drop any noise ahead of the start byte
                if raw_data and raw_data[0] != OW_START_BYTE:
                    idx = raw_data.find(bytes([OW_START_BYTE]))
                    raw_data = raw_data[idx:] if idx >= 0 else b""
                if len(raw_data) >= HEADER_LEN:
                    data_len = int.from_bytes(raw_data[7:9], "big")
                    if data_len > MAX_DATA_LEN:
                        # Implausible length — we latched onto a stray start
                        # byte. Shift one byte and re-seek.
                        raw_data = raw_data[1:]
                        expected = None
                        continue
                    expected = HEADER_LEN + data_len + TRAILER_LEN
                    if len(raw_data) >= expected:
                        break

        if not raw_data:
            raise ValueError("No data received from UART within timeout")
        if expected is None or len(raw_data) < expected:
            raise ValueError(
                f"Incomplete packet from UART within timeout "
                f"(got {len(raw_data)} bytes, expected {expected})"
            )
        return UartPacket(buffer=raw_data[:expected])

    def send_packet(
        self,
        id=None,
        packetType=OW_ACK,
        command=OW_CMD_NOP,
        addr: int = 0,
        reserved: int = 0,
        data=None,
        timeout: int = 20,
    ) -> Optional[UartPacket]:
        """Send a command packet and return the matching response.

        Returns None only when the transport is closed (caller should treat
        as a connection error). Raises `CommandError` on validation errors
        and re-raises `serial.SerialException` after notifying the I/O
        error callback so the handle can transition out of CONNECTED.
        """
        try:
            if not self.demo_mode and (
                self.serial is None or not self.serial.is_open
            ):
                logger.error("Cannot send packet. UART not open.")
                return None

            if id is None:
                self.packet_count += 1
                if self.packet_count >= 0xFFFF:
                    self.packet_count = 1
                id = self.packet_count

            if data:
                if not isinstance(data, (bytes, bytearray)):
                    raise ValueError("Data must be bytes or bytearray")
                payload = data
                payload_length = len(payload)
                if payload_length > self.max_payload:
                    raise ValueError(
                        f"Payload of {payload_length} bytes exceeds the "
                        f"{self.max_payload}-byte limit"
                    )
            else:
                payload_length = 0
                payload = b""

            packet = bytearray()
            packet.append(OW_START_BYTE)
            packet.extend(id.to_bytes(2, "big"))
            packet.append(packetType)
            packet.append(command)
            packet.append(addr)
            packet.append(reserved)
            packet.extend(payload_length.to_bytes(2, "big"))
            if payload_length > 0:
                packet.extend(payload)

            crc_value = util_crc16(packet[1:])  # exclude start byte
            packet.extend(crc_value.to_bytes(2, "big"))
            packet.append(OW_END_BYTE)

            with self._io_lock:
                # The console firmware drops commands that arrive too soon
                # after it finished the previous response (measured on
                # console fw 1.8.0: back-to-back loses ~50%, 5 ms gap loses
                # ~3%, 10 ms loses none). Enforce a 20 ms floor between the
                # previous command's completion and the next TX. The old
                # polling read loop provided this gap by accident.
                gap = self._min_cmd_gap - (time.monotonic() - self._last_cmd_ts)
                if gap > 0:
                    time.sleep(gap)
                try:
                    self._tx(packet)
                    ret_packet = self._await_response(id, timeout)
                    return ret_packet
                finally:
                    self._last_cmd_ts = time.monotonic()

        except ValueError as ve:
            logger.error("Validation error in send_packet: %s", ve)
            raise CommandError(str(ve)) from ve
        except serial.SerialException as se:
            # Already notified on_io_error from _tx/read_packet — the
            # handle's state machine logs the disconnect at INFO. Logging
            # here at DEBUG keeps in-flight commands from spamming ERROR
            # during the disconnect window. The exception is still
            # re-raised so callers can react.
            logger.debug("Serial error in send_packet: %s", se)
            raise

    def _await_response(self, id: int, timeout: int) -> UartPacket:
        """Read packets until the response matching `id` arrives.

        A response that arrives after its command already timed out sits in
        the OS buffer and would otherwise be returned as the answer to the
        *next* command, leaving the transport permanently off-by-one.
        Discard mismatched ids until the matching response (or the deadline)
        arrives — same policy as CommInterface on the USB path.
        """
        deadline = None if timeout == -1 else time.monotonic() + timeout
        while True:
            remaining = (
                -1 if deadline is None
                else max(0.001, deadline - time.monotonic())
            )
            ret_packet = self.read_packet(timeout=remaining)
            if ret_packet.id != id:
                if (
                    ret_packet.id == 0
                    and ret_packet.packetType == OW_DATA
                    and ret_packet.command == OW_CMD_ECHO
                ):
                    # Unsolicited MCU printf packet — same shape the
                    # sensor comm path logs and skips.
                    text = (
                        bytes(ret_packet.data)
                        .decode("utf-8", errors="replace")
                        .rstrip("\x00")
                        .strip()
                    )
                    logger.info("[console PRINTF] %s", text)
                else:
                    logger.warning(
                        "Discarding stale UART response id=0x%04X "
                        "(expected 0x%04X)",
                        ret_packet.id, id,
                    )
                if deadline is not None and time.monotonic() >= deadline:
                    raise ValueError(
                        f"No matching response for packet id 0x{id:04X} "
                        f"within timeout"
                    )
                continue
            return ret_packet

    def clear_buffer(self) -> None:
        if self.demo_mode or self.serial is None:
            return
        try:
            self.serial.reset_input_buffer()
        except Exception:
            pass

    def print(self) -> None:
        logger.info("    Serial Port: %s", self.port)
        logger.info("    Serial Baud: %s", self.baudrate)
