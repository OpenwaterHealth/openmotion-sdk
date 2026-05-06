import logging
import omotion.config as config
from omotion.UartPacket import UartPacket
from omotion.config import (
    OW_ACK,
    OW_CMD_NOP,
    OW_END_BYTE,
    OW_START_BYTE,
    OW_DATA,
    OW_CMD_ECHO,
)
import usb.core
import usb.util
import time
import threading
import queue
from omotion.USBInterfaceBase import USBInterfaceBase
from omotion import _log_root

# Max data_len we accept (sanity check to avoid runaway buffer)
OW_MAX_PACKET_DATA_LEN = 4096 * 2

logger = logging.getLogger(
    f"{_log_root}.CommInterface" if _log_root else "CommInterface"
)

# Max data_len we accept (sanity check to avoid runaway buffer)
OW_MAX_PACKET_DATA_LEN = 4096 * 2

_PACKET_TYPE_NAMES = {
    value: name
    for name, value in vars(config).items()
    if name.startswith("OW_") and name.isupper() and isinstance(value, int)
}
_CMD_NAMES = {
    "OW_CMD": {
        value: name
        for name, value in vars(config).items()
        if name.startswith("OW_CMD_")
    },
    "OW_CONTROLLER": {
        value: name
        for name, value in vars(config).items()
        if name.startswith("OW_CTRL_")
    },
    "OW_FPGA": {
        value: name
        for name, value in vars(config).items()
        if name.startswith("OW_FPGA_")
    },
    "OW_CAMERA": {
        value: name
        for name, value in vars(config).items()
        if name.startswith("OW_CAMERA_")
    },
    "OW_IMU": {
        value: name
        for name, value in vars(config).items()
        if name.startswith("OW_IMU_")
    },
}


def _format_named(value: int, name_map: dict[int, str], width: int = 2) -> str:
    name = name_map.get(value)
    if name:
        return f"{name}(0x{value:0{width}X})"
    return f"0x{value:0{width}X}"


# =========================================
# Comm Interface (IN + OUT + threads)
# =========================================
class CommInterface(USBInterfaceBase):
    def __init__(self, dev, interface_index, desc="Comm", async_mode=False):
        super().__init__(dev, interface_index, desc)
        self.read_thread = None
        self.stop_event = threading.Event()
        # Contiguous byte buffer: USB reader extends the end, packet parser chops from the front
        self._read_buffer = bytearray()
        self._buffer_lock = threading.Lock()
        self._buffer_condition = threading.Condition(self._buffer_lock)
        self.packet_count = 0
        self.async_mode = async_mode
        # Callback invoked once when the read loop sees a fatal USB error
        # (e.g. ENODEV, EIO, EPIPE). The owning MotionSensor wires this to
        # submit an EVT_IO_ERROR to the ConnectionMonitor, which then drives
        # the handle's state machine. The single-event-queue design means we
        # don't need the previous _disconnect_notified flag or the daemon-
        # thread dispatch hack that worked around join-self deadlocks.
        self.on_io_error = None
        self._io_lock = threading.RLock()
        self._send_lock = threading.Lock()
        if self.async_mode:
            self.response_queue = queue.Queue()
            self.response_thread = threading.Thread(
                target=self._process_responses, daemon=True
            )
            self.response_thread.start()

    def claim(self):
        super().claim()
        intf = self.dev.get_active_configuration()[(self.interface_index, 0)]
        self.ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: (
                usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
            ),
        )
        if not self.ep_out:
            raise RuntimeError(f"{self.desc}: No OUT endpoint found")

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
        with self._send_lock:
            if id is None:
                self.packet_count = (self.packet_count + 1) & 0xFFFF or 1
                id = self.packet_count

            if data:
                if not isinstance(data, (bytes, bytearray)):
                    raise ValueError("Data must be bytes or bytearray")
                payload = data
            else:
                payload = b""

            uart_packet = UartPacket(
                id=id,
                packet_type=packetType,
                command=command,
                addr=addr,
                reserved=reserved,
                data=payload,
            )

            tx_bytes = uart_packet.to_bytes()
            packet_type_name = _PACKET_TYPE_NAMES.get(packetType)
            cmd_names = _CMD_NAMES.get(packet_type_name, {})
            logger.debug(
                f"{self.desc}: TX id=0x{id:04X} "
                f"type={_format_named(packetType, _PACKET_TYPE_NAMES)} "
                f"cmd={_format_named(command, cmd_names)} "
                f"addr=0x{addr:02X} reserved=0x{reserved:02X} len={len(payload)} data={tx_bytes.hex()}"
            )

            self.write(tx_bytes)
            time.sleep(0.0005)

            if not self.async_mode:
                start = time.monotonic()
                data = bytearray()
                with self._io_lock:
                    while time.monotonic() - start < timeout:
                        try:
                            resp = self.receive()
                            time.sleep(0.005)
                            if resp:
                                data.extend(resp)
                                if data and data[-1] == OW_END_BYTE:
                                    return UartPacket(buffer=data)
                        except usb.core.USBError:
                            continue
                last_error = TimeoutError("No response")
            else:
                start_time = time.monotonic()
                while time.monotonic() - start_time < timeout:
                    if self.response_queue.empty():
                        time.sleep(0.0005)
                    else:
                        time.sleep(0.001)
                        pkt = self.response_queue.get()
                        if pkt.id != id:
                            logger.warning(
                                "%s: discarding stale response id=0x%04X (expected 0x%04X)",
                                self.desc, pkt.id, id,
                            )
                            continue
                        return pkt
                raise TimeoutError(f"No response in async mode, packet id 0x{id:04X}")

    def clear_buffer(self):
        with self._buffer_lock:
            self._read_buffer.clear()

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
                    # so callers and _read_loop disconnect logic see it normally.
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

    def start_read_thread(self):
        if self.read_thread and self.read_thread.is_alive():
            logger.info(f"{self.desc}: Read thread already running")
            return
        self.stop_event.clear()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        logger.info(f"{self.desc}: Read thread started")

    def stop_read_thread(self):
        """Signal the read loop to exit and join it (unless we're being
        called from the read thread itself, in which case the caller is
        already on its way out)."""
        self.stop_event.set()
        rt = self.read_thread
        if rt is not None and threading.current_thread() is not rt:
            rt.join(timeout=2.0)
        logger.info(f"{self.desc}: Read thread stopped")

    def _notify_io_error(self, error):
        """Invoke the io-error callback exactly once per read-loop exit.

        The state-machine on the owning handle dedups further events that
        arrive while the handle is already in DISCONNECTING, so we don't
        need a flag here — the queue serializes everything."""
        cb = self.on_io_error
        if cb is None:
            return
        errno = getattr(error, "errno", None)
        try:
            cb(errno, str(error))
        except Exception as e:
            logger.warning("on_io_error callback raised: %s", e)

    def _read_loop(self):
        while not self.stop_event.is_set():
            try:
                data = self.dev.read(
                    self.ep_in.bEndpointAddress, self.ep_in.wMaxPacketSize, timeout=100
                )
                if data:
                    data_bytes = bytes(data)
                    with self._buffer_condition:
                        self._read_buffer.extend(data_bytes)
                        self._buffer_condition.notify()
                    logger.debug(f"Read {len(data)} bytes.")
                time.sleep(0.001)
            except usb.core.USBError as e:
                # During an intentional shutdown the read loop will see USB
                # errors as the transport is closed; suppress them silently.
                if self.stop_event.is_set():
                    break
                if e.errno in (110, 10060):
                    # Read timeout — no data this window. Keep looping.
                    continue
                # errno 32 = EPIPE (stalled/disconnected endpoint)
                # errno 19 = ENODEV (device unplugged)
                # errno  5 = EIO (device I/O error)
                # Any other USB error we treat the same way: notify the
                # owning handle and exit. The handle's state machine
                # decides how to react (transition to DISCONNECTING,
                # release resources). Logging the disconnect once at the
                # state-machine level avoids the historical duplicate-log
                # spam from multiple disconnect paths racing.
                logger.warning(
                    f"{self.desc}: USB read error (errno={e.errno}); exiting read loop: {e}"
                )
                self._notify_io_error(e)
                break

    def _process_responses(self):
        while not self.stop_event.is_set():
            with self._buffer_condition:
                if not self._read_buffer:
                    self._buffer_condition.wait(timeout=0.1)
                    continue
                buf = self._read_buffer
                # Align to start of packet: discard leading bytes until OW_START_BYTE
                if buf[0] != OW_START_BYTE:
                    try:
                        start_idx = buf.index(OW_START_BYTE)
                    except ValueError:
                        start_idx = len(buf)
                    del self._read_buffer[:start_idx]
                    if start_idx == len(buf):
                        continue
                    buf = self._read_buffer
                # Need at least 9 bytes to read data_len (bytes 7:9)
                if len(buf) < 9:
                    continue
                data_len = int.from_bytes(buf[7:9], "big")
                if data_len > OW_MAX_PACKET_DATA_LEN:
                    del self._read_buffer[:1]
                    continue
                packet_len = 12 + data_len  # header(11) + data + crc(2) + end(1)
                if len(buf) < packet_len:
                    continue
                if buf[packet_len - 1] != OW_END_BYTE:
                    del self._read_buffer[:1]
                    continue
                packet_bytes = bytes(buf[:packet_len])
                del self._read_buffer[:packet_len]
            try:
                uart_packet = UartPacket(buffer=packet_bytes)
            except ValueError:
                continue
            if (
                uart_packet.id == 0
                and uart_packet.packet_type == OW_DATA
                and uart_packet.command == OW_CMD_ECHO
            ):
                _raw = bytes(uart_packet.data[:uart_packet.data_len]) if uart_packet.data_len > 0 else b""
                try:
                    _text = _raw.decode("utf-8", errors="replace").rstrip("\x00").strip()
                except Exception:
                    _text = ""
                if _text:
                    logger.warning("[%s PRINTF] %s", self.desc, _text)
                else:
                    logger.warning("[%s] MCU echo: data=%s", self.desc, _raw.hex() if _raw else "")
            else:
                self.response_queue.put(uart_packet)
