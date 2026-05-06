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
    OW_CMD_NOP,
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

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle (driven by MotionConsole state machine)
    # ────────────────────────────────────────────────────────────────────

    def open(self, port: str) -> None:
        """Open the serial port. Raises serial.SerialException on failure."""
        if self.demo_mode:
            self.port = port or "DEMO"
            return
        self.serial = serial.Serial(
            port=port, baudrate=self.baudrate, timeout=self.timeout
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
                id=0, packet_type=OW_ERROR, command=0, addr=0, reserved=0, data=[]
            )
        if self.serial is None:
            raise CommandError("UART not open")
        with self._io_lock:
            start_time = time.monotonic()
            raw_data = b""
            count = 0

            while timeout == -1 or time.monotonic() - start_time < timeout:
                time.sleep(0.05)
                try:
                    raw_data += self.serial.read_all()
                except serial.SerialException as se:
                    self._notify_io_error(getattr(se, "errno", None), str(se))
                    raise
                if raw_data:
                    count += 1
                    if count > 1:
                        break

        if not raw_data:
            raise ValueError("No data received from UART within timeout")
        return UartPacket(buffer=raw_data)

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
                self._tx(packet)
                time.sleep(0.0005)
                ret_packet = self.read_packet(timeout=timeout)
                time.sleep(0.0005)
                return ret_packet

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
