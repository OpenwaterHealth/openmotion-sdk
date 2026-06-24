"""Console command/response transport — serial VCP.

Same async engine as the sensor's USB link: :class:`omotion.packet_transport.PacketTransport`
runs the reader/dispatch threads and matches responses by id; this class only
supplies serial-specific raw I/O and lifecycle (open/close/discovery). A
continuous reader is what lets unsolicited firmware printf packets share the
command channel without colliding with responses.

`MotionUart` does not own connection lifecycle — the owning `MotionConsole`
drives `open(port)` / `close()` from its state machine. On a fatal serial error
the transport invokes the `on_io_error(errno, message)` callback so the state
machine can transition out of CONNECTED.
"""
from __future__ import annotations

import logging
from typing import Callable, Optional

import serial
import serial.tools.list_ports

from omotion.UartPacket import UartPacket
from omotion.config import OW_END_BYTE, OW_ERROR
from omotion.packet_transport import PacketTransport
from omotion.CommandError import CommandError
from omotion import _log_root

logger = logging.getLogger(f"{_log_root}.UART" if _log_root else "UART")

# Short blocking read timeout for the reader thread — keeps it responsive to
# incoming bytes and to shutdown without a busy-spin. The per-command timeout is
# separate (PacketTransport.send_packet / default_timeout).
_READER_TIMEOUT_S = 0.1
# Default per-command response timeout for the console (preserves prior behavior).
_CONSOLE_CMD_TIMEOUT_S = 20.0


class MotionUart(PacketTransport):
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
        PacketTransport.__init__(
            self, desc=desc,
            default_timeout=_CONSOLE_CMD_TIMEOUT_S, on_io_error=on_io_error,
        )
        self.vid = vid
        self.pid = pid
        self.port: Optional[str] = None
        self.baudrate = baudrate
        self.timeout = timeout  # retained for API compat; reader uses _READER_TIMEOUT_S
        self.align = align
        self.demo_mode = demo_mode
        self.descriptor = desc
        self.serial: Optional[serial.Serial] = None

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle (driven by MotionConsole state machine)
    # ────────────────────────────────────────────────────────────────────

    def open(self, port: str) -> None:
        """Open the serial port and start the reader. Raises serial.SerialException
        on failure."""
        if self.demo_mode:
            self.port = port or "DEMO"
            return
        self.serial = serial.Serial(
            port=port, baudrate=self.baudrate, timeout=_READER_TIMEOUT_S
        )
        self.port = port
        self.start_read_thread()
        logger.info("UART %s opened on %s", self.descriptor, port)

    def close(self) -> None:
        """Stop the reader and close the serial port. Idempotent."""
        if self.demo_mode:
            self.port = None
            return
        self.stop_read_thread()
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
    # Raw byte I/O for the shared transport engine
    # ────────────────────────────────────────────────────────────────────

    def _raw_read(self) -> bytes:
        s = self.serial
        if s is None:
            raise CommandError("UART not open")
        # read(1) blocks up to the (short) serial timeout, then drain whatever
        # else arrived. Returns b"" on an idle window. SerialException → fatal.
        first = s.read(1)
        if not first:
            return b""
        rest = s.read_all()
        return first + (rest or b"")

    def _raw_write(self, data: bytes) -> None:
        if self.demo_mode:
            logger.debug("Demo mode TX: %s", bytes(data).hex())
            return
        s = self.serial
        if s is None or not s.is_open:
            raise CommandError("UART not open")
        if self.align > 0:
            data = bytes(data)
            while len(data) % self.align != 0:
                data += bytes([OW_END_BYTE])
        try:
            s.write(data)
        except serial.SerialException as se:
            self._notify_io_error(se)
            raise

    def send_packet(self, *args, **kwargs):
        if self.demo_mode:
            return UartPacket(
                id=0, packetType=OW_ERROR, command=0, addr=0, reserved=0, data=[]
            )
        return super().send_packet(*args, **kwargs)

    def print(self) -> None:
        logger.info("    Serial Port: %s", self.port)
        logger.info("    Serial Baud: %s", self.baudrate)
