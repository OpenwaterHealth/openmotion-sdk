"""Sensor command/response transport — USB-bulk interface 0.

The threading, framing and dispatch live in :class:`omotion.packet_transport.PacketTransport`;
this class only supplies the USB-specific raw I/O (bulk read/write with libusb
error handling), interface claim, and a synchronous send path used by tests.
"""
import logging
import time

import usb.core
import usb.util

from omotion.UartPacket import UartPacket
from omotion.config import OW_ACK, OW_CMD_NOP, OW_END_BYTE
from omotion.packet_transport import PacketTransport
from omotion.USBInterfaceBase import USBInterfaceBase
from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.CommInterface" if _log_root else "CommInterface"
)


# =========================================
# Comm Interface (IN + OUT + threads)
# =========================================
class CommInterface(PacketTransport, USBInterfaceBase):
    def __init__(self, dev, interface_index, desc="Comm", async_mode=False):
        USBInterfaceBase.__init__(self, dev, interface_index, desc)
        PacketTransport.__init__(
            self, desc=desc, async_mode=async_mode, default_timeout=10.0
        )

    def claim(self):
        USBInterfaceBase.claim(self)
        intf = self.dev.get_active_configuration()[(self.interface_index, 0)]
        self.ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: (
                usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            ),
        )
        if not self.ep_out:
            raise RuntimeError(f"{self.desc}: No OUT endpoint found")

    # ────────────────────────────────────────────────────────────────────
    # Raw byte I/O for the shared transport engine
    # ────────────────────────────────────────────────────────────────────

    def _raw_read(self) -> bytes:
        try:
            data = self.dev.read(
                self.ep_in.bEndpointAddress, self.ep_in.wMaxPacketSize, timeout=100
            )
        except usb.core.USBError as e:
            if e.errno in (110, 10060):  # ETIMEDOUT / WSAETIMEDOUT — idle window
                return b""
            raise  # ENODEV / EIO / EPIPE / etc. — fatal, reader loop handles it
        return bytes(data) if data else b""

    def _raw_write(self, data) -> None:
        self.write(data)

    def write(self, data, timeout=100, _retries=5):
        with self._io_lock:
            for attempt in range(1 + _retries):
                try:
                    return self.dev.write(self.ep_out.bEndpointAddress, data, timeout=timeout)
                except usb.core.USBError as e:
                    # Firmware back-pressure: the device's OUT FIFO is temporarily
                    # full.  Back off briefly and retry so callers don't have to
                    # care about transient busy periods (e.g. after program_fpga).
                    if e.errno in (110, 10060):  # ETIMEDOUT / WSAETIMEDOUT
                        if attempt < _retries:
                            delay = 0.05 * (attempt + 1)  # 50 ms, 100 ms, 150 ms …
                            logger.warning(
                                "%s: write timeout (attempt %d/%d), retrying in %.0f ms",
                                self.desc, attempt + 1, 1 + _retries, delay * 1000,
                            )
                            time.sleep(delay)
                            continue
                        logger.error("%s: write timed out after %d attempts", self.desc, 1 + _retries)
                        raise
                    # A stalled endpoint (EPIPE / broken-pipe) can be recovered by
                    # issuing a CLEAR_HALT control transfer.  Try once; if it works
                    # re-send the original data.  Any other USB error is re-raised
                    # so callers and the reader loop disconnect logic see it normally.
                    if e.errno in (32, -9):  # EPIPE on Linux; LIBUSB_ERROR_PIPE cross-platform
                        logger.warning("%s: OUT endpoint stalled, attempting clear_halt", self.desc)
                        try:
                            usb.util.clear_halt(self.dev, self.ep_out)
                            return self.dev.write(self.ep_out.bEndpointAddress, data, timeout=timeout)
                        except Exception as recovery_err:
                            logger.error("%s: clear_halt recovery failed: %s", self.desc, recovery_err)
                    raise

    def receive(self, length=512, timeout=100):
        with self._io_lock:
            data = self.dev.read(self.ep_in.bEndpointAddress, length, timeout=timeout)
            logger.debug(f"Received {len(data)} bytes.")
            return data

    # ────────────────────────────────────────────────────────────────────
    # Send: async path is inherited; sync path (legacy / tested) lives here
    # because it reads raw USB chunks until the end byte.
    # ────────────────────────────────────────────────────────────────────

    def send_packet(
        self,
        id=None,
        packetType=OW_ACK,
        command=OW_CMD_NOP,
        addr=0,
        reserved=0,
        data=None,
        timeout=10.0,
        max_retries=0,
    ) -> UartPacket:
        if self.async_mode:
            return super().send_packet(
                id=id, packetType=packetType, command=command, addr=addr,
                reserved=reserved, data=data, timeout=timeout, max_retries=max_retries,
            )
        # Synchronous path: accumulate raw reads until the END byte. Polls
        # _transport_down_evt so an in-flight send unblocks if the link drops.
        with self._send_lock:
            id, tx = self._build_packet(id, packetType, command, addr, reserved, data)
            self.write(tx)
            time.sleep(0.0005)
            start = time.monotonic()
            buf = bytearray()
            with self._io_lock:
                while time.monotonic() - start < timeout:
                    if self._transport_down_evt.is_set():
                        raise ConnectionError(
                            f"{self.desc}: transport down, packet id 0x{id:04X} "
                            "not deliverable"
                        )
                    try:
                        resp = self.receive()
                        time.sleep(0.005)
                        if resp:
                            buf.extend(resp)
                            if buf and buf[-1] == OW_END_BYTE:
                                return UartPacket(buffer=buf)
                    except usb.core.USBError:
                        continue
            raise TimeoutError(
                f"{self.desc}: no response (sync) for packet id 0x{id:04X}"
            )
