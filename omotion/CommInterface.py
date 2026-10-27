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
from omotion.USBInterfaceBase import USBInterfaceBase, is_usb_timeout
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
        # Set by the read loop when it exits on a fatal USB error
        # (ENODEV / EIO / EPIPE / other non-timeout failures). send_packet
        # polls it inside the wait loops so an in-flight command unblocks
        # the moment the transport drops, instead of spinning for the full
        # caller-supplied timeout (60 s for program_fpga, etc.). See
        # bloodflow-app#130: power-cycle mid-configure left the SDK's
        # configure worker hung for 60 s, blocking the next scan attempt.
        self._transport_down_evt = threading.Event()
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
        # The async response-parser thread is created by start_read_thread()
        # alongside the read thread — never here. Starting it in the
        # constructor meant a CommInterface that was built but never opened
        # (e.g. MotionComposite.open() failed in the post-enumeration
        # "resource busy" window) had a _process_responses thread that
        # nothing could ever stop: stop_event is only set on the stop path,
        # which such orphans never reach. A packaged-app fault dump
        # (2026-08-17) showed ~52 of these threads accumulated across a day
        # of sensor connect/disconnect cycles.
        self.response_thread = None
        if self.async_mode:
            self.response_queue = queue.Queue()

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
                packetType=packetType,
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

            if not self.async_mode:
                start = time.monotonic()
                data = bytearray()
                with self._io_lock:
                    while time.monotonic() - start < timeout:
                        if self._transport_down_evt.is_set():
                            raise ConnectionError(
                                f"{self.desc}: transport down, packet id "
                                f"0x{id:04X} not deliverable"
                            )
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
                # Block on the queue instead of sleep-polling: get(timeout=)
                # wakes the instant _process_responses enqueues the packet.
                # The 50 ms chunks are only so a dead transport still cancels
                # an in-flight send promptly (bloodflow-app#130); on the happy
                # path they add no latency. The old poll loop paid a fixed
                # ~2.5 ms of sleep per command — the dominant cost of bulk
                # command streams like the NVCM burn (#233).
                start_time = time.monotonic()
                while True:
                    if self._transport_down_evt.is_set():
                        raise ConnectionError(
                            f"{self.desc}: transport down, packet id "
                            f"0x{id:04X} not deliverable"
                        )
                    remaining = timeout - (time.monotonic() - start_time)
                    if remaining <= 0:
                        raise TimeoutError(
                            f"No response in async mode, packet id 0x{id:04X}"
                        )
                    try:
                        pkt = self.response_queue.get(timeout=min(0.05, remaining))
                    except queue.Empty:
                        continue
                    if pkt.id != id:
                        logger.warning(
                            "%s: discarding stale response id=0x%04X (expected 0x%04X)",
                            self.desc, pkt.id, id,
                        )
                        continue
                    return pkt

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
                    if is_usb_timeout(e):
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
                    # A stalled OUT endpoint (EPIPE) leaves the host-side pipe
                    # unusable until a CLEAR_HALT control transfer resets it.
                    # Clear it so a *later* write on a still-present device has a
                    # working pipe, then always re-raise: this write failed either
                    # way, and _read_loop has usually already latched
                    # _transport_down_evt by the time we get here.
                    #
                    # We deliberately do NOT re-send.  The stall can arrive after
                    # the device already accepted and executed the transfer, and
                    # the sensor firmware has no id dedup — it runs
                    # process_if_command() on every frame that passes framing +
                    # CRC (sensor-fw Core/Src/uart_comms.c:331).  A duplicate
                    # inverts OW_CMD_TOGGLE_LED (if_commands.c:139) and
                    # double-appends an OW_FPGA_BITSTREAM chunk
                    # (if_commands.c:527-529), walking the write pointer past the
                    # buffer.  Retry with duplicate suppression belongs one layer
                    # up, in send_packet.  See #189.
                    #
                    # pyusb puts the libusb code in .backend_error_code and the
                    # POSIX errno in .errno, and _libusb_errno[-9] is 32 on every
                    # platform, so LIBUSB_ERROR_PIPE always arrives as errno 32.
                    if e.errno in (32, -9):  # EPIPE (32); -9 never reaches .errno
                        logger.warning("%s: OUT endpoint stalled, attempting clear_halt", self.desc)
                        try:
                            # clear_halt is a usb.core.Device method; usb.util has
                            # no such attribute — that was the #189 defect.
                            self.dev.clear_halt(self.ep_out.bEndpointAddress)
                            logger.info("%s: OUT endpoint halt cleared", self.desc)
                        except (usb.core.USBError, NotImplementedError) as recovery_err:
                            # Common after teardown: dispose_resources() has
                            # already released the interface, so libusb returns
                            # NOT_FOUND.  The errno tells that apart from a live
                            # device refusing the request.
                            logger.warning(
                                "%s: clear_halt failed (errno=%s): %s",
                                self.desc, getattr(recovery_err, "errno", None), recovery_err,
                            )
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
        # Fresh read thread implies transport is up again; any prior
        # transport-down latch from a previous USB error must clear or
        # subsequent sends would short-circuit on the stale flag.
        self._transport_down_evt.clear()
        if self.async_mode and not (
            self.response_thread and self.response_thread.is_alive()
        ):
            self.response_thread = threading.Thread(
                target=self._process_responses,
                daemon=True,
                name=f"{self.desc}-resp",
            )
            self.response_thread.start()
        self.read_thread = threading.Thread(
            target=self._read_loop, daemon=True, name=f"{self.desc}-read"
        )
        self.read_thread.start()
        logger.info(f"{self.desc}: Read thread started")

    def stop_read_thread(self):
        """Signal both transport threads (read loop + async response parser)
        to exit and join them, except any we are currently running on (that
        caller is already on its way out).

        The response thread parks in ``_buffer_condition.wait(timeout=0.1)``
        between packets, so wake it explicitly rather than waiting out its
        poll interval."""
        self.stop_event.set()
        with self._buffer_condition:
            self._buffer_condition.notify_all()
        current = threading.current_thread()
        for label, t in (
            ("read", self.read_thread),
            ("response", self.response_thread),
        ):
            if t is None or t is current:
                continue
            t.join(timeout=2.0)
            if t.is_alive():
                logger.warning(
                    "%s: %s thread did not exit within 2s of stop",
                    self.desc, label,
                )
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
                # No pacing sleep: dev.read blocks (timeout=100 ms) when the
                # endpoint is idle, so looping straight back cannot busy-spin.
            except usb.core.USBError as e:
                # During an intentional shutdown the read loop will see USB
                # errors as the transport is closed; suppress them silently.
                if self.stop_event.is_set():
                    break
                if is_usb_timeout(e):
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
                # Latch transport-down BEFORE the io-error callback so any
                # send_packet that is currently spinning in its wait loop
                # observes the dead transport on its next tick.
                self._transport_down_evt.set()
                # A dead transport has no further work for the response
                # parser: no bytes will ever reach _read_buffer again, and
                # pending sends abort on _transport_down_evt, not on parsed
                # responses. Stop it here so the thread pair shuts down even
                # on paths where nobody calls stop_read_thread() afterwards;
                # start_read_thread() clears the event and restarts both on
                # reconnect.
                self.stop_event.set()
                with self._buffer_condition:
                    self._buffer_condition.notify_all()
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
                and uart_packet.packetType == OW_DATA
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
