import logging
import struct
import json
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np

from omotion import _log_root
from omotion.MotionUart import MotionUart
from omotion.ConsoleTelemetry import ConsoleTelemetryPoller
from omotion.connection_state import ConnectionState
from omotion.signal_wrapper import SignalWrapper
from omotion.config import (
    FPGA_PROG_CFG_READ_PAGE,
    FPGA_PROG_CFG_RESET,
    FPGA_PROG_CFG_WRITE_PAGE,
    FPGA_PROG_CFG_WRITE_PAGES,
    FPGA_PROG_CLOSE,
    FPGA_PROG_ERASE,
    FPGA_PROG_FEATROW_READ,
    FPGA_PROG_FEATROW_WRITE,
    FPGA_PROG_READ_STATUS,
    FPGA_PROG_REFRESH,
    FPGA_PROG_SET_DONE,
    FPGA_PROG_UFM_READ_PAGE,
    FPGA_PROG_UFM_RESET,
    FPGA_PROG_UFM_WRITE_PAGE,
    FPGA_PROG_UFM_WRITE_PAGES,
    OW_CMD,
    OW_CMD_DFU,
    OW_FPGA_PROG,
    OW_CMD_ECHO,
    OW_CMD_HWID,
    OW_CMD_PING,
    OW_CMD_RESET,
    OW_CMD_TOGGLE_LED,
    OW_CMD_USR_CFG,
    OW_CMD_VERSION,
    OW_CMD_MESSAGES,
    OW_CONTROLLER,
    OW_CTRL_BOARDID,
    OW_CTRL_GET_FAN,
    OW_CTRL_SET_FAN,
    OW_CTRL_GET_FSYNC,
    OW_CTRL_GET_IND,
    OW_CTRL_GET_LSYNC,
    OW_CTRL_GET_TEMPS,
    OW_CTRL_GET_TRIG,
    OW_CTRL_I2C_RD,
    OW_CTRL_I2C_SCAN,
    OW_CTRL_I2C_WR,
    OW_CTRL_PDUMON,
    OW_CTRL_READ_ADC,
    OW_CTRL_READ_GPIO,
    OW_CTRL_SET_IND,
    OW_CTRL_SET_TRIG,
    OW_CTRL_START_TRIG,
    OW_CTRL_STOP_TRIG,
    OW_CTRL_TEC_DAC,
    OW_CTRL_TEC_STATUS,
    OW_CTRL_TECADC,
    FPGA_PROG_OPEN,
    OW_ERROR,
    XO2_FLASH_PAGE_SIZE,
    MuxChannel,
)

from omotion.GitHubReleases import GitHubReleases
from omotion.MotionConfig import MotionConfig, MotionConfigHeader
from omotion.Calibration import (
    Calibration,
    parse_calibration,
    serialize_calibration,
    CALIBRATION_JSON_KEY,
)
from omotion.CommandError import CommandError

logger = logging.getLogger(f"{_log_root}.Console" if _log_root else "Console")

# --------------------------------------------------------------------------- #
# Dataclasses
# --------------------------------------------------------------------------- #


@dataclass
class TelemetrySample:
    timestamp_ms: int
    acq_time_us: int
    t1: float
    t2: float
    t3: float
    tec_adc: tuple[int, int, int, int]
    tec_good: bool

    def __str__(self):
        return (
            f"TelemetrySample(ts={self.timestamp_ms} ms, "
            f"acq={self.acq_time_us} µs, "
            f"t1={self.t1:.2f}°C, "
            f"t2={self.t2:.2f}°C, "
            f"t3={self.t3:.2f}°C, "
            f"tec_adc={self.tec_adc}, "
            f"tec_good={self.tec_good})"
        )


@dataclass
class PDUMon:
    raws: List[int]  # 16 uint16
    volts: List[float]  # 16 float32


def _parse_pdu_mon(payload: bytes) -> PDUMon:
    if len(payload) != 96:
        raise ValueError(f"Expected 96 bytes, got {len(payload)}")
    # <  = little-endian; 16H = 16 uint16; 16f = 16 float32
    raws_and_volts = struct.unpack("<16H16f", payload)
    raws = list(raws_and_volts[0:16])
    volts = list(raws_and_volts[16:32])
    return PDUMon(raws=raws, volts=volts)


class MotionConsole(SignalWrapper):
    """Stable handle for the console device.

    The handle is constructed once by ``MotionInterface`` and lives for its
    entire lifetime — it is **never replaced**. Apps can cache the reference
    and connect Qt slots to ``signal_state_changed`` once.

    The connection lifecycle is owned by ``ConnectionMonitor``, which calls
    ``_handle_event`` from its single worker thread. The on-entry sequence
    for ``CONNECTING`` (open serial → ping → version) runs synchronously on
    that thread with a 5-step retry backoff for the post-enumeration
    "resource busy" window.
    """

    name: str = "console"

    def __init__(
        self,
        vid: int,
        pid: int,
        baudrate: int = 921600,
        timeout: int = 10,
        demo_mode: bool = False,
    ):
        super().__init__()
        self.uart = MotionUart(
            vid=vid,
            pid=pid,
            baudrate=baudrate,
            timeout=timeout,
            desc="console",
            demo_mode=demo_mode,
            on_io_error=self._on_uart_io_error,
        )
        self.telemetry = ConsoleTelemetryPoller(self)

        self._state = ConnectionState.DISCONNECTED
        self._state_cv = threading.Condition()
        self._monitor = None  # set by MotionInterface.start()
        self._version = "v0.0.0"

    # ──────────────────────────────────────────────────────────────────
    # State (read-only from outside)
    # ──────────────────────────────────────────────────────────────────

    @property
    def state(self) -> ConnectionState:
        return self._state

    def is_connected(self) -> bool:
        return self._state == ConnectionState.CONNECTED

    def wait_for(self, target: ConnectionState, timeout: float = 5.0) -> bool:
        """Block until ``state == target`` or ``timeout`` elapses."""
        with self._state_cv:
            return self._state_cv.wait_for(
                lambda: self._state == target, timeout=timeout
            )

    def request_disconnect(self) -> None:
        """Ask the monitor to disconnect this handle. Idempotent."""
        if self._monitor is None:
            return
        from omotion.connection_monitor import UserStop

        self._monitor.submit(UserStop(handle_name=self.name))

    # ──────────────────────────────────────────────────────────────────
    # Wiring
    # ──────────────────────────────────────────────────────────────────

    def _attach_monitor(self, monitor) -> None:
        """Called by MotionInterface when starting the monitor."""
        self._monitor = monitor

    def _on_uart_io_error(self, errno, message: str) -> None:
        """Forwarded from MotionUart on serial errors. Submits an event so
        the state machine can transition out of CONNECTED."""
        if self._monitor is None:
            return
        from omotion.connection_monitor import IoError

        self._monitor.submit(
            IoError(handle_name=self.name, errno=errno, message=message)
        )

    # ──────────────────────────────────────────────────────────────────
    # State machine
    # ──────────────────────────────────────────────────────────────────

    def _set_state(self, new_state: ConnectionState, reason: str = "") -> None:
        """Internal: transition state, notify waiters, emit signal."""
        with self._state_cv:
            if self._state == new_state:
                return
            old = self._state
            self._state = new_state
            self._state_cv.notify_all()
        try:
            self.signal_state_changed.emit(self, old, new_state, reason)
        except Exception as e:
            logger.debug("signal_state_changed emit suppressed: %s", e)
        logger.info(
            "console state %s -> %s (%s)",
            old.name,
            new_state.name,
            reason or "",
        )

    def _handle_event(self, event) -> None:
        """Entry point called by ConnectionMonitor on its thread.
        Dispatches based on (current_state, event_type)."""
        from omotion.connection_monitor import (
            IoError,
            PollArrived,
            PollGone,
            UserStop,
        )

        st = self._state
        if isinstance(event, PollArrived):
            if st == ConnectionState.DISCONNECTED:
                self._drive_connecting(reason="poll_arrived")
        elif isinstance(event, (PollGone, IoError)):
            if st == ConnectionState.CONNECTED:
                ev_name = type(event).__name__.lower()
                reason = (
                    f"usb_io_error:errno={event.errno}"
                    if isinstance(event, IoError)
                    else f"poll_gone"
                )
                self._drive_disconnecting(reason=reason)
            # else: already DISCONNECTING/DISCONNECTED/CONNECTING — no-op.
            #       This is the dedup that kills duplicate "release failed"
            #       warnings: a second IoError after the first one started
            #       us toward DISCONNECTING is silently absorbed.
        elif isinstance(event, UserStop):
            if st == ConnectionState.CONNECTED:
                self._drive_disconnecting(reason="user_stop")
            elif st == ConnectionState.CONNECTING:
                # If we're mid-CONNECTING the call below is racing with
                # _drive_connecting on the same thread, so it can't
                # actually arrive while we're inside _drive_connecting —
                # the queue serializes. So this branch only fires when
                # _drive_connecting has already returned.
                self._drive_disconnecting(reason="user_stop")
            # DISCONNECTING/DISCONNECTED: nothing to do.

    def _drive_connecting(self, reason: str) -> None:
        if self.uart.demo_mode:
            self._set_state(ConnectionState.CONNECTING, reason=reason)
            self._set_state(ConnectionState.CONNECTED, reason="demo_mode")
            try:
                self.telemetry.start()
            except Exception:
                logger.exception("telemetry start (demo) failed")
            return

        self._set_state(ConnectionState.CONNECTING, reason=reason)

        # Five-step backoff: 50, 100, 250, 500, 1000 ms. Covers the
        # empirical Windows post-enumeration window where the device is
        # visible to libusb/pyserial but `Serial(...)` still raises
        # "Resource busy" or "Permission denied" for ~200–500 ms.
        backoff = [0.05, 0.1, 0.25, 0.5, 1.0]
        last_error: Optional[Exception] = None
        for delay in backoff:
            try:
                port = self.uart.find_port()
                if port is None:
                    raise RuntimeError("console COM port not found")
                self.uart.open(port)
                # Ping to confirm the firmware is responsive.
                r = self.uart.send_packet(
                    id=None, packetType=OW_CMD, command=OW_CMD_PING
                )
                if r is None or r.packet_type == OW_ERROR:
                    raise RuntimeError("console ping failed or returned error")
                # `self._version` is no longer cached during CONNECTING:
                # `get_version()` gates on `is_connected()` (which is
                # state == CONNECTED), so calling it here logged a
                # spurious "Console Module not connected" ERROR and
                # returned "v0.0.0" anyway. Callers that want the
                # version invoke `get_version()` from their own
                # CONNECTED handler (e.g. `log_console_info`), which
                # works correctly post-transition.
                self._set_state(ConnectionState.CONNECTED, reason="ping_ok")
                try:
                    self.telemetry.start()
                except Exception:
                    logger.exception("telemetry start failed")
                return
            except Exception as e:
                last_error = e
                logger.warning(
                    "console connect attempt failed (%s); retrying in %.0f ms",
                    e,
                    delay * 1000,
                )
                # Roll back any partial open.
                try:
                    self.uart.close()
                except Exception:
                    pass
                time.sleep(delay)

        # All attempts failed — back to DISCONNECTED. Next poll/hotplug
        # will retry from scratch.
        self._set_state(
            ConnectionState.DISCONNECTED,
            reason=f"connect_retry_exhausted:{last_error}",
        )

    def _log_command_error(self, method_name: str, e: Exception) -> None:
        """Log a command-method failure with severity that depends on
        whether the device is mid-disconnect.

        Transport errors that fire while the handle is already
        DISCONNECTING/DISCONNECTED are expected (the device is going
        away mid-call) — log at DEBUG. Errors that fire while CONNECTED
        or CONNECTING are unexpected — log at ERROR.
        """
        if self._state in (
            ConnectionState.DISCONNECTING,
            ConnectionState.DISCONNECTED,
        ):
            logger.debug(
                "Command %s failed during disconnect: %s", method_name, e
            )
        else:
            logger.error("Unexpected error during %s: %s", method_name, e)

    def _drive_disconnecting(self, reason: str) -> None:
        self._set_state(ConnectionState.DISCONNECTING, reason=reason)
        try:
            self.telemetry.stop()
        except Exception:
            logger.exception("telemetry stop failed")
        try:
            self.uart.close()
        except Exception:
            logger.exception("uart close failed")
        self._set_state(ConnectionState.DISCONNECTED, reason=reason)

    # ──────────────────────────────────────────────────────────────────
    # Backwards-compat helper used by command bodies further below.
    # ──────────────────────────────────────────────────────────────────
    # (no-op; kept to make the diff to existing command methods minimal)

    def ping(self) -> bool:
        """
        Send a ping command to the MOTIONConsole and receive a response.
        Returns the response from the MOTIONConsole.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console Device not connected")

            logger.info("Send Ping to Device.")
            r = self.uart.send_packet(id=None, packetType=OW_CMD, command=OW_CMD_PING)
            self.uart.clear_buffer()
            logger.info("Received Ping from Device.")
            # r.print_packet()

            if r.packet_type == OW_ERROR:
                logger.error("Error sending ping")
                return False
            else:
                return True

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("ping", e)
            raise  # Re-raise the exception for the caller to handle

    def get_version(self) -> str:
        """
        Retrieve the firmware version of the Console Module.

        Returns:
            str: Firmware version in the format 'vX.Y.Z'.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while fetching the version.
        """
        try:
            if self.uart.demo_mode:
                return "v0.1.1"

            if not self.is_connected():
                logger.error("Console Module not connected")
                return "v0.0.0"

            r = self.uart.send_packet(
                id=None, packetType=OW_CMD, command=OW_CMD_VERSION
            )
            self.uart.clear_buffer()
            # r.print_packet()
            # Older firmwares returned 3 bytes: major, minor, patch.
            # Newer firmware returns a C string in the payload (FW_VERSION_STRING).
            if r.data_len == 3:
                ver = f"v{r.data[0]}.{r.data[1]}.{r.data[2]}"
            elif r.data_len and r.data:
                try:
                    # Decode only the valid length, strip trailing NULs and whitespace
                    ver_str = (
                        r.data[: r.data_len]
                        .decode("utf-8", errors="ignore")
                        .rstrip("\x00")
                        .strip()
                    )
                    ver = ver_str if ver_str else "v0.0.0"
                except Exception:
                    ver = "v0.0.0"
            else:
                ver = "v0.0.0"
            logger.info(ver)
            return ver
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_version", e)
            raise  # Re-raise the exception for the caller to handle

    def echo(self, echo_data=None) -> tuple[bytes, int]:
        """
        Send an echo command to the device with data and receive the same data in response.

        Args:
            echo_data (bytes): The data to send (must be a byte array).

        Returns:
            tuple[bytes, int]: The echoed data and its length.

        Raises:
            ValueError: If the UART is not connected.
            TypeError: If the `echo_data` is not a byte array.
            Exception: If an error occurs during the echo process.
        """
        try:
            if self.uart.demo_mode:
                data = b"Hello Motion!!"
                return data, len(data)

            if not self.is_connected():
                logger.error("Console Module not connected")
                return None, None

            # Check if echo_data is a byte array
            if echo_data is not None and not isinstance(echo_data, (bytes, bytearray)):
                raise TypeError("echo_data must be a byte array")

            r = self.uart.send_packet(
                id=None, packetType=OW_CMD, command=OW_CMD_ECHO, data=echo_data
            )
            self.uart.clear_buffer()
            # r.print_packet()
            if r.data_len > 0:
                return r.data, r.data_len
            else:
                return None, None

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except TypeError as t:
            logger.error("TypeError: %s", t)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("echo", e)
            raise  # Re-raise the exception for the caller to handle

    def toggle_led(self) -> bool:
        """
        Toggle the LED on the Console Module.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while toggling the LED.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                logger.error("Console Module not connected")
                return False

            self.uart.send_packet(id=None, packetType=OW_CMD, command=OW_CMD_TOGGLE_LED)
            self.uart.clear_buffer()
            # r.print_packet()
            return True

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("toggle_led", e)
            raise  # Re-raise the exception for the caller to handle

    def get_hardware_id(self) -> str:
        """
        Retrieve the hardware ID of the Console Module.

        Returns:
            str: Hardware ID in hexadecimal format.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving the hardware ID.
        """
        try:
            if self.uart.demo_mode:
                return bytes.fromhex("deadbeefcafebabe1122334455667788")

            if not self.is_connected():
                logger.error("Console Module not connected")
                return None

            r = self.uart.send_packet(id=None, packetType=OW_CMD, command=OW_CMD_HWID)
            self.uart.clear_buffer()
            # r.print_packet()
            if r.data_len == 16:
                return r.data.hex()
            else:
                return None
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_hardware_id", e)
            raise  # Re-raise the exception for the caller to handle

    def enter_dfu(self) -> bool:
        """
        Perform a soft reset to enter DFU mode on Console device.

        Returns:
            bool: True if the reset was successful, False otherwise.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while resetting the device.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console Device not connected")

            r = self.uart.send_packet(id=None, packetType=OW_CMD, command=OW_CMD_DFU)
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error setting DFU mode for device")
                return False
            else:
                return True
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("enter_dfu", e)
            raise  # Re-raise the exception for the caller to handle

    def soft_reset(self) -> bool:
        """
        Perform a soft reset on the Console device.

        Returns:
            bool: True if the reset was successful, False otherwise.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while resetting the device.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console Module not connected")

            r = self.uart.send_packet(id=None, packetType=OW_CMD, command=OW_CMD_RESET)
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error resetting device")
                return False
            else:
                return True
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("soft_reset", e)
            raise  # Re-raise the exception for the caller to handle

    def get_messages(self) -> str:
        """
        Retrieve messages from the Console device.

        Returns:
            str: Messages from the device as a decoded string, or empty string if no messages.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving messages.
        """
        try:
            if self.uart.demo_mode:
                return "Demo mode: messages not available"

            if not self.is_connected():
                raise ValueError("Console Module not connected")

            logger.info("Sending OW_CMD_MESSAGES command to Console.")
            r = self.uart.send_packet(
                id=None, packetType=OW_CMD, command=OW_CMD_MESSAGES
            )
            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error retrieving messages from device")
                return ""

            # Decode the response as a UTF-8 string, stripping trailing nulls and whitespace
            if r.data_len > 0 and r.data:
                try:
                    msg_str = (
                        r.data[: r.data_len]
                        .decode("utf-8", errors="ignore")
                        .rstrip("\x00")
                        .strip()
                    )
                    logger.info(f"Received messages: {msg_str}")
                    return msg_str
                except Exception as e:
                    logger.error(f"Error decoding messages: {e}")
                    return ""
            else:
                logger.info("No messages received from device")
                return ""

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_messages", e)
            raise  # Re-raise the exception for the caller to handle

    def scan_i2c_mux_channel(self, mux_index: int, channel: int) -> list[int]:
        """
        Scan a specific channel on an I2C MUX and return detected I2C addresses.

        Args:
            mux_index (int): Index of the I2C MUX (e.g., 0 for MUX at 0x70, 1 for 0x71).
            channel (int): Channel number on the MUX to activate (0-7).

        Returns:
            list[int]: List of detected I2C device addresses on the specified mux/channel.
                    Returns an empty list if no devices are found.

        Raises:
            ValueError: If the mux index or channel is out of range.
            Exception: For unexpected UART communication issues.
        """
        if channel < 0 or channel > 7:
            raise ValueError(f"Invalid channel {channel}, must be 0-7")
        if mux_index not in [0, 1]:
            raise ValueError(f"Invalid mux index {mux_index}, must be 0 or 1")

        try:
            # Send I2C scan command with mux index and channel as payload
            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_I2C_SCAN,
                data=bytes([mux_index, channel]),
            )
            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error scanning I2C mux %d channel %d", mux_index, channel)
                return []

            # Return list of detected I2C addresses
            return list(r.data) if r.data else []

        except Exception as e:
            logger.error(
                "Exception while scanning I2C mux %d channel %d: %s",
                mux_index,
                channel,
                e,
            )
            raise

    def read_i2c_packet(
        self,
        mux_index: int,
        channel: int,
        device_addr: int,
        reg_addr: int,
        read_len: int,
    ) -> tuple[bytes, int]:
        """
        Read data from I2C device through MUX

        Args:
            mux_index: Which MUX to use (0 or 1)
            channel: Which MUX channel to select (0-7)
            device_addr: I2C device address (7-bit)
            reg_addr: Register address to read from
            read_len: Number of bytes to read

        Returns:
            tuple[bytes, int]: The received data and its length, None if failed
        """
        """Validate MUX index and channel parameters"""
        if mux_index not in (0, 1):
            raise ValueError(f"Invalid mux_index {mux_index}. Must be 0 or 1")
        if channel < 0 or channel > 7:
            raise ValueError(f"Invalid channel {channel}. Must be 0-7")

        try:
            # Build packet: [CMD, MUX_IDX, CHANNEL, DEV_ADDR, REG_ADDR, READ_LEN]
            packet = struct.pack(
                "BBBBB", mux_index, channel, device_addr, reg_addr, read_len
            )

            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_I2C_RD,
                data=packet,
            )

            self.uart.clear_buffer()
            # r.print_packet()

            if r.packet_type == OW_ERROR:
                logger.error("Error Reading I2C Device")
                return None, None

            if r.data_len > 0:
                return r.data, r.data_len
            else:
                return None, None

        except Exception as e:
            # The underlying error is already logged by MotionUart.send_packet()
            # Only log here if we want additional context about the I2C operation
            logger.debug(
                f"I2C read operation failed (underlying error logged by UART layer): {str(e)}"
            )
            return None, None

    def write_i2c_packet(
        self, mux_index: int, channel: int, device_addr: int, reg_addr: int, data: bytes
    ) -> bool:
        """
        Write data to I2C device through MUX

        Args:
            mux_index: Which MUX to use (0 or 1)
            channel: Which MUX channel to select (0-7)
            device_addr: I2C device address (7-bit)
            reg_addr: Register address to write to
            data: Bytes to write

        Returns:
            bool: True if write succeeded, False otherwise
        """
        """Validate MUX index and channel parameters"""
        if mux_index not in (0, 1):
            raise ValueError(f"Invalid mux_index {mux_index}. Must be 0 or 1")
        if channel < 0 or channel > 7:
            raise ValueError(f"Invalid channel {channel}. Must be 0-7")

        try:
            # Build packet: [CMD, MUX_IDX, CHANNEL, DEV_ADDR, REG_ADDR] + data
            header = struct.pack(
                "BBBBB", mux_index, channel, device_addr, reg_addr, len(data)
            )
            packet = header + data

            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_I2C_WR,
                data=packet,
            )

            self.uart.clear_buffer()
            # r.print_packet()

            if r.packet_type == OW_ERROR:
                logger.error("Error Writing I2C Device")
                return False
            else:
                return True

        except Exception as e:
            print(f"I2C Write failed: {str(e)}")
            return False

    def set_fan_speed(self, fan_speed: int = 50) -> int:
        """
        Drive the console fan PWM.

        Sends OW_CTRL_SET_FAN with the requested duty cycle. Use
        :meth:`get_fan_rpm` (with ``fan_index=1..3``) to read per-fan
        tachometer RPM.

        Args:
            fan_speed (int): Desired fan duty cycle, 0..100. Default 50.

        Returns:
            int: The fan_speed that was set, or -1 on OW_ERROR.

        Raises:
            ValueError: If the controller is not connected, or fan_speed
                is not in 0..100.
        """
        if not self.is_connected():
            raise ValueError("Console controller not connected")

        if fan_speed not in range(101):
            raise ValueError("Invalid fan speed. Must be 0 to 100")

        try:
            if self.uart.demo_mode:
                return fan_speed

            data = bytes([fan_speed & 0xFF])

            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_SET_FAN,
                data=data,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error setting Fan Speed")
                return -1

            logger.info("Set fan speed to %d", fan_speed)
            return fan_speed

        except ValueError:
            raise
        except Exception as e:
            self._log_command_error("set_fan_speed", e)
            raise

    def get_fan_rpm(self, fan_index: int) -> Optional[int]:
        """
        Read measured tachometer RPM for a console fan.

        The console fan lines are read-only tach inputs. For each call, the
        firmware samples the GPIO over a measurement window and blocks for
        ~50 ms before responding.

        Args:
            fan_index (int): Fan to read, 1, 2, or 3.

        Returns:
            Optional[int]: Measured RPM as a 16-bit unsigned value, or None
            if the firmware reports an error.

        Raises:
            ValueError: If the controller is not connected, or fan_index is
                not in 1..3.
        """
        if fan_index not in (1, 2, 3):
            raise ValueError("fan_index must be 1, 2, or 3")

        if not self.is_connected():
            raise ValueError("Console controller not connected")

        try:
            if self.uart.demo_mode:
                return 2400

            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_GET_FAN,
                addr=fan_index,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error reading fan %d RPM", fan_index)
                return None

            if r.data_len == 2:
                rpm = r.data[0] | (r.data[1] << 8)
                logger.info("Fan %d RPM: %d", fan_index, rpm)
                return rpm

            logger.error("Unexpected fan RPM payload length: %d", r.data_len)
            return None

        except ValueError:
            raise
        except Exception as e:
            self._log_command_error("get_fan_rpm", e)
            raise

    def set_rgb_led(self, rgb_state: int) -> int:
        """
        Set the RGB LED state.

        Args:
            rgb_state (int): The desired RGB state (0 = OFF, 1 = IND1, 2 = IND2, 3 = IND3).

        Returns:
            int: The current RGB state after setting.

        Raises:
            ValueError: If the controller is not connected or the RGB state is invalid.
        """
        if not self.is_connected():
            raise ValueError("Console controller not connected")

        if rgb_state not in [0, 1, 2, 3]:
            raise ValueError(
                "Invalid RGB state. Must be 0 (OFF), 1 (IND1), 2 (IND2), or 3 (IND3)"
            )

        try:
            if self.uart.demo_mode:
                return rgb_state

            logger.info("Setting RGB LED state.")

            # Send the RGB state as the reserved byte in the packet
            r = self.uart.send_packet(
                id=None,
                reserved=rgb_state & 0xFF,  # Send the RGB state as a single byte
                packetType=OW_CONTROLLER,
                command=OW_CTRL_SET_IND,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error setting RGB LED state")
                return -1

            logger.info(f"Set RGB LED state to {rgb_state}")
            return rgb_state

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("set_rgb_led", e)
            raise  # Re-raise the exception for the caller to handle

    def get_rgb_led(self) -> int:
        """
        Get the current RGB LED state.

        Returns:
            int: The current RGB state (0 = OFF, 1 = IND1, 2 = IND2, 3 = IND3).

        Raises:
            ValueError: If the controller is not connected.
        """
        if not self.is_connected():
            raise ValueError("Console controller not connected")

        try:
            if self.uart.demo_mode:
                return 1  # Default to RED in demo mode

            logger.info("Getting current RGB LED state.")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_GET_IND
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error getting RGB LED state")
                return -1

            rgb_state = r.reserved
            logger.info(f"Current RGB LED state is {rgb_state}")
            return rgb_state

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_rgb_led", e)
            raise  # Re-raise the exception for the caller to handle

    def set_trigger_json(self, data=None) -> dict:
        """
        Set the trigger configuration for console device.

        Args:
            data (dict): A dictionary containing the trigger configuration.

        Returns:
            dict: JSON response from the device.

        Raises:
            ValueError: If `data` is None or the UART is not connected.
            Exception: If an error occurs while setting the trigger.
        """
        try:
            if self.uart.demo_mode:
                return None

            # Ensure data is not None and is a valid dictionary
            if data is None:
                logger.error("Data cannot be None.")
                return None

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            try:
                json_string = json.dumps(data)
            except json.JSONDecodeError as e:
                logger.error(f"Data must be valid JSON: {e}")
                return None

            payload = json_string.encode("utf-8")

            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_SET_TRIG,
                data=payload,
            )
            self.uart.clear_buffer()

            if r.packet_type != OW_ERROR and r.data_len > 0:
                # Parse response as JSON, if possible
                try:
                    response_json = json.loads(r.data.decode("utf-8"))
                    return response_json
                except json.JSONDecodeError as e:
                    logger.error(f"Error decoding JSON: {e}")
                    return None
            else:
                return None
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("set_trigger_json", e)
            raise  # Re-raise the exception for the caller to handle

    def get_trigger_json(self) -> dict:
        """
        Start the trigger on the Console device.

        Returns:
            bool: True if the trigger was started successfully, False otherwise.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while starting the trigger.
        """
        try:
            if self.uart.demo_mode:
                return None

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_GET_TRIG, data=None
            )
            self.uart.clear_buffer()
            data_object = None
            try:
                data_object = json.loads(r.data.decode("utf-8"))
            except json.JSONDecodeError as e:
                logger.error(f"Error decoding JSON: {e}")
            return data_object
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_trigger_json", e)
            raise  # Re-raise the exception for the caller to handle

    def start_trigger(self) -> bool:
        """
        Start the trigger on the Console device.

        Returns:
            bool: True if the trigger was started successfully, False otherwise.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while starting the trigger.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_START_TRIG, data=None
            )
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error starting trigger")
                return False
            else:
                return True
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("start_trigger", e)
            raise  # Re-raise the exception for the caller to handle

    def stop_trigger(self) -> bool:
        """
        Stop the trigger on the Console device.

        Returns:
            bool: True if the trigger was started successfully, False otherwise.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while starting the trigger.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_STOP_TRIG, data=None
            )
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error stopping trigger")
                return False
            else:
                return True
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("stop_trigger", e)
            raise  # Re-raise the exception for the caller to handle

    def get_fsync_pulsecount(self) -> int:
        """
        Get FSYNC pulse count from the Console device.

        Returns:
            int: The number of FSYNC pulses received.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving the pulse count.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_GET_FSYNC, data=None
            )
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error retrieving FSYNC pulse count")
                return 0

            if r.data_len == 4:
                # Assuming the pulse count is returned as a 4-byte integer
                pulse_count = struct.unpack("<I", r.data)[0]
                return pulse_count
            else:
                logger.error("Unexpected data length for FSYNC pulse count")
                return 0

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_fsync_pulsecount", e)
            raise  # Re-raise the exception for the caller to handle

    def get_lsync_pulsecount(self) -> int:
        """
        Get LSYNC pulse count from the Console device.

        Returns:
            int: The number of LSYNC pulses received.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving the pulse count.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_GET_LSYNC, data=None
            )
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error retrieving LSYNC pulse count")
                return 0
            if r.data_len == 4:
                # Assuming the pulse count is returned as a 4-byte integer
                pulse_count = struct.unpack("<I", r.data)[0]
                return pulse_count
            else:
                logger.error("Unexpected data length for LSYNC pulse count")
                return 0

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("get_lsync_pulsecount", e)

    def read_gpio_value(self) -> int:
        """
        Read GPIO value.

        Returns:
            int: The GPIO register value (4-byte unsigned integer).

        Raises:
            ValueError: If the UART is not connected or the firmware returns
                an unexpected response.
            Exception: If an error occurs during communication.
        """
        try:
            if self.uart.demo_mode:
                return 0

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_READ_GPIO, data=None
            )
            self.uart.clear_buffer()
            if r.packet_type == OW_ERROR:
                raise ValueError("Device returned an error for OW_CTRL_READ_GPIO")
            if r.data_len == 4:
                return struct.unpack("<I", r.data)[0]
            raise ValueError(
                f"Unexpected data length for GPIO read: got {r.data_len}, expected 4"
            )

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            self._log_command_error("read_gpio_value", e)
            raise

    def read_adc_value(self) -> float:
        """
        Read ADC value.

        Returns:
            float: The ADC reading as a 32-bit float.

        Raises:
            ValueError: If the UART is not connected or the firmware returns
                an unexpected response.
            Exception: If an error occurs during communication.
        """
        try:
            if self.uart.demo_mode:
                return 0.0

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_READ_ADC, data=None
            )
            self.uart.clear_buffer()
            if r.packet_type == OW_ERROR:
                raise ValueError("Device returned an error for OW_CTRL_READ_ADC")
            if r.data_len == 4:
                return struct.unpack("<f", r.data)[0]
            raise ValueError(
                f"Unexpected data length for ADC read: got {r.data_len}, expected 4"
            )

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            self._log_command_error("read_adc_value", e)
            raise

    def get_temperatures(self, return_all: bool = False) -> tuple[float, float, float]:
        """
        Get the current temperatures from the Console device.

        The firmware response contains N packed TelemetrySample records:
            struct TelemetrySample (28 bytes, little-endian, packed):
                uint32  timestamp_ms
                uint32  acq_time_us
                float   t1   (MAX31875 sensor 1, °C)
                float   t2   (MAX31875 sensor 2, °C)
                float   t3   (MAX31875 sensor 3, °C)
                uint16  tec_adc[4]  (ADS7924 channels 0-3, raw 12-bit)
                bool    tec_status

        All samples are printed; the last sample's (t1, t2, t3) is returned.

        Returns:
            tuple[float, float, float]: (t1, t2, t3) in °C from the most recent sample.

        Raises:
            ValueError: If the UART is not connected or the payload is malformed.
        """
        SAMPLE_FMT = "<IIfff4H?"  # matches TelemetrySample layout
        SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)  # 28 bytes

        try:
            if self.uart.demo_mode:
                # Demo values: stable 3-tuple
                if return_all:
                    return [TelemetrySample(0, 0, 35.0, 45.0, 25.0, (0, 0, 0, 0), True)]
                return (35.0, 45.0, 25.0)

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_GET_TEMPS
            )
            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                raise ValueError("Device returned OW_ERROR for temperatures")

            if r.data_len == 0 or r.data_len % SAMPLE_SIZE != 0:
                raise ValueError(
                    f"Unexpected telemetry payload length: {r.data_len} "
                    f"(must be non-zero multiple of {SAMPLE_SIZE})"
                )

            n_samples = r.data_len // SAMPLE_SIZE
            samples: list[TelemetrySample] = []

            for i in range(n_samples):
                unpacked = struct.unpack_from(
                    SAMPLE_FMT, r.data, offset=i * SAMPLE_SIZE
                )

                (ts_ms, acq_us, t1, t2, t3, adc0, adc1, adc2, adc3, tec_good) = unpacked

                samples.append(
                    TelemetrySample(
                        timestamp_ms=ts_ms,
                        acq_time_us=acq_us,
                        t1=t1,
                        t2=t2,
                        t3=t3,
                        tec_adc=(adc0, adc1, adc2, adc3),
                        tec_good=tec_good,
                    )
                )

            if return_all:
                return samples

            # default (backwards compatible)
            last = samples[-1]
            return (last.t1, last.t2, last.t3)

        except Exception:
            logger.exception("Failed to get temperatures")
            raise

    def tec_voltage(self, voltage: float | None = None) -> float:
        """
        Get/Set TEC Setpoint voltage.

        Returns:

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving the TEC Enable.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Motion Console not connected")

            if voltage is not None and voltage >= -5.0 and voltage <= 5.0:
                # Set TEC Voltage
                logger.info("Setting TEC Voltage to %.2f V", voltage)
                data = struct.pack(
                    "<f", float(voltage)
                )  # Convert to 2-byte unsigned int
                r = self.uart.send_packet(
                    id=None,
                    packetType=OW_CONTROLLER,
                    command=OW_CTRL_TEC_DAC,
                    reserved=1,
                    data=data,
                )
            elif voltage is None:
                # Get TEC Voltage
                logger.info("Getting TEC Voltage")
                r = self.uart.send_packet(
                    id=None,
                    packetType=OW_CONTROLLER,
                    command=OW_CTRL_TEC_DAC,
                    reserved=0,
                    data=None,
                )
            else:
                raise ValueError(
                    "Invalid voltage value. Must be between -5.0 and 5.0 V"
                )

            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error executing tec_voltage command")
                return 0
            elif r.data_len == 4:
                tec_voltage = struct.unpack("<f", r.data)[0]
                logger.info(f"TEC Voltage is {tec_voltage} V")
                return tec_voltage

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("tec_voltage", e)
            raise  # Re-raise the exception for the caller to handle

    def tec_adc(self, channel: int) -> float:
        """
        Get TEC ADC voltages.

        Returns:
            float: The TEC ADC voltage for the specified channel.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving the TEC Enable.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Motion Console not connected")

            if channel not in [0, 1, 2, 3, 4]:
                raise ValueError("Invalid channel. Must be 0, 1, 2, 3 or 4")

            # Get TEC Voltage
            logger.info(f"Getting TEC ADC CH{channel} Voltage")
            r = self.uart.send_packet(
                id=None,
                packetType=OW_CONTROLLER,
                command=OW_CTRL_TECADC,
                reserved=channel,
                data=None,
            )

            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error executing tec_adc command")
                return 0
            elif r.data_len == 4:
                tec_voltage = struct.unpack("<f", r.data)[0]
                logger.info(f"CHANNEL {channel}: {tec_voltage} V")
                return tec_voltage
            elif r.data_len == 16:
                ch0, ch1, ch2, ch3 = struct.unpack("<4f", r.data)
                vals = [ch0, ch1, ch2, ch3]
                logger.info(f"CHANNELS 0-3: {vals} V")
                return vals
            else:
                logger.error("Unexpected data length for TEC ADC voltage")
                raise ValueError("Unexpected data length for TEC ADC voltage")

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("tec_adc", e)
            raise  # Re-raise the exception for the caller to handle

    def tec_status(self) -> Tuple[str, str, str, str, bool]:
        """
        Get TEC status: (voltage, Temperature Setpoint, TEC Current, TEC Voltage, TEC Good)

        Returns:
            tuple: (volt, temp_set, tec_curr, tec_volt, tec_good)

        Raises:
            ValueError: If not connected or response lengths are unexpected.
            Exception:  If the device reports an OW_ERROR.
        """
        try:
            # Demo mode mock
            if getattr(self.uart, "demo_mode", False):
                return (1.0, 0.5, 0.5, 25.0, True)

            if not self.is_connected():
                raise ValueError("Motion Console not connected")

            logger.debug("Getting TEC Status")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_TEC_STATUS, data=None
            )
            if r.packet_type == OW_ERROR:
                logger.error("Device returned OW_ERROR for OW_CTRL_TEC_STATUS")
                raise Exception("Error executing tec_status command")

            self.uart.clear_buffer()

            # TecStats: uint32 timestamp_ms, float vout, float temp_set,
            #           float tec_curr, float tec_volt, bool tec_status  (21 bytes)
            TEC_STATS_FMT = "<I4f?"
            TEC_STATS_SIZE = struct.calcsize(TEC_STATS_FMT)  # 21 bytes

            if r.data_len < TEC_STATS_SIZE:
                raise ValueError(
                    f"TecStats response too short: {r.data_len} bytes, expected {TEC_STATS_SIZE}"
                )
            elif r.data_len > TEC_STATS_SIZE:
                logger.warning(
                    "TecStats response has %d extra bytes (got %d, expected %d); extra bytes will be ignored",
                    r.data_len - TEC_STATS_SIZE,
                    r.data_len,
                    TEC_STATS_SIZE,
                )
            _ts_ms, vout, temp_set, tec_curr, tec_volt, tec_good = struct.unpack(
                TEC_STATS_FMT, r.data[:TEC_STATS_SIZE]
            )

            logger.debug(
                "TEC Status - V: %.6f V, SET: %.6f V, TEC_C: %.6f V, TEC_V: %.6f V, GOOD: %s",
                vout,
                temp_set,
                tec_curr,
                tec_volt,
                tec_good,
            )
            
            return (
                f"{vout:.6f}",
                f"{temp_set:.6f}",
                f"{tec_curr:.6f}",
                f"{tec_volt:.6f}",
                tec_good,
            )
        except Exception as e:
            self._log_command_error("tec_status", e)
            raise  # Re-raise the exception for the caller to handle

    def read_board_id(self) -> int:
        """
        Read Board ID

        Returns:
            int: The read value.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving Board ID.
        """
        try:
            if self.uart.demo_mode:
                return True

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_BOARDID, data=None
            )
            self.uart.clear_buffer()
            # r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error retrieving Board ID")
                return 0
            if r.data_len == 1:
                # Assuming the pulse count is returned as a 4-byte integer
                boardID = r.data[0]
                return boardID
            else:
                logger.error("Unexpected data length for Board ID")
                return 0

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("read_board_id", e)
            raise  # Re-raise the exception for the caller to handle

    def read_pdu_mon(self) -> Optional[PDUMon]:
        """
        Read PDU MON

        Returns:
            int: 16 raw values read from ADC.
            float: 16 voltage values converted.

        Raises:
            ValueError: If the UART is not connected.
            Exception: If an error occurs while retrieving PDU MON data.
        """
        try:
            if self.uart.demo_mode:
                # Return a fake structure in demo mode
                return PDUMon(raws=[0] * 16, volts=[0.0] * 16)

            if not self.is_connected():
                raise ValueError("Console controller not connected")

            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_PDUMON, data=None
            )
            self.uart.clear_buffer()
            r.print_packet()
            if r.packet_type == OW_ERROR:
                logger.error("Error retrieving PDU MON data")
                return None

            if r.data_len != 96 or r.data is None:
                logger.error("Unexpected data length for PDU MON data: %s", r.data_len)
                return None

            # r.data should be a bytes-like object
            pdu = _parse_pdu_mon(bytes(r.data[:96]))
            # logger.info("PDU MON: raws=%s", pdu.raws)
            # logger.info("PDU MON: volts=%s", ["%.3f" % v for v in pdu.volts])
            return pdu

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise  # Re-raise the exception for the caller to handle

        except Exception as e:
            self._log_command_error("read_pdu_mon", e)
            raise  # Re-raise the exception for the caller to handle

    @staticmethod
    def get_latest_version_info():
        """
        Query GitHub for the console firmware releases and return a JSON-serializable
        structure containing the latest official release and a map of all releases.

        Returned structure:
        {
            "latest": {"tag_name": str|None, "published_at": str|None},
            "releases": {
                "tag": {"published_at": str|None, "prerelease": bool},
                ...
            }
        }

        Uses the OpenwaterHealth/openmotion-console-fw repository.
        """
        gh = GitHubReleases("OpenwaterHealth", "openmotion-console-fw")

        # Get latest official release (get_latest_release excludes prereleases by default)
        try:
            latest = gh.get_latest_release()
        except Exception:
            latest = None

        # Get all releases including prereleases so we can label them
        try:
            all_releases = gh.get_all_releases(include_prerelease=True)
        except Exception:
            all_releases = []

        releases_map = {}
        for r in all_releases:
            tag = r.get("tag_name")
            if not tag:
                continue
            published = r.get("published_at")
            # consider prerelease flag or tag names that start with 'pre-'
            prerelease_flag = bool(r.get("prerelease")) or str(tag).lower().startswith(
                "pre-"
            )
            releases_map[tag] = {
                "published_at": published,
                "prerelease": prerelease_flag,
            }

        result = {
            "latest": {
                "tag_name": latest.get("tag_name") if latest else None,
                "published_at": latest.get("published_at") if latest else None,
            },
            "releases": releases_map,
        }

        return result

    def read_config(self) -> Optional[MotionConfig]:
        """
        Read the user configuration from device flash.

        The configuration is stored as JSON with metadata (magic, version, sequence, CRC).

        Returns:
            MotionConfig: Parsed configuration object, or None on error

        Raises:
            ValueError: If the UART is not connected
            Exception: If an error occurs during communication
        """
        try:
            if self.uart.demo_mode:
                logger.info("Demo mode: returning empty config")
                return MotionConfig()

            if not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send read command (reserved=0 for READ)
            logger.info("Reading user config from device...")
            r = self.uart.send_packet(
                id=None,
                packetType=OW_CMD,
                command=OW_CMD_USR_CFG,
                reserved=0,  # 0 = READ
            )
            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error reading config from device")
                return None

            # Parse wire format response
            try:
                config = MotionConfig.from_wire_bytes(r.data)
                logger.info(
                    f"Read config: seq={config.header.seq}, json_len={config.header.json_len}"
                )
                return config
            except Exception as e:
                logger.error(f"Failed to parse config response: {e}")
                return None

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error reading config: %s", e)
            raise

    def write_config(self, config: MotionConfig) -> Optional[MotionConfig]:
        """
        Write user configuration to device flash.

        Can pass either:
        - Full wire format (header + JSON)
        - Raw JSON bytes (device will parse as JSON)

        Args:
            config: MotionConfig object to write

        Returns:
            MotionConfig: Updated configuration from device (with new seq/crc), or None on error

        Raises:
            ValueError: If the UART is not connected
            Exception: If an error occurs during communication
        """
        try:
            if self.uart.demo_mode:
                logger.info("Demo mode: simulating config write")
                return config

            if not self.is_connected():
                raise ValueError("Console Device not connected")

            # Convert config to wire format bytes
            wire_data = config.to_wire_bytes()

            logger.info(f"Writing config to device: {len(wire_data)} bytes")

            # Send write command (reserved=1 for WRITE)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_CMD,
                command=OW_CMD_USR_CFG,
                reserved=1,  # 1 = WRITE
                data=wire_data,
            )
            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("Error writing config to device")
                return None

            # The firmware write ACK returns only the updated 16-byte header
            # (with the new seq/crc stamped by the device) — it does NOT echo
            # back the full JSON payload.  Parse just the header and reattach
            # the json_data we sent so the caller gets a fully-populated object.
            try:
                updated_header = MotionConfigHeader.from_bytes(bytes(r.data[:16]))
                updated_config = MotionConfig(
                    header=updated_header,
                    json_data=config.json_data.copy(),
                )
                logger.info(
                    "Config written successfully: new seq=%d", updated_config.header.seq
                )
                return updated_config
            except Exception as e:
                logger.error("Failed to parse write ACK header: %s", e)
                return None

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error writing config: %s", e)
            raise

    def write_config_json(self, json_str: str) -> Optional[MotionConfig]:
        """
        Write user configuration from a JSON string.

        This is a convenience method that creates a MotionConfig from JSON
        and writes it to the device.

        Args:
            json_str: JSON string to write

        Returns:
            MotionConfig: Updated configuration from device, or None on error

        Raises:
            ValueError: If JSON is invalid or UART is not connected
            Exception: If an error occurs during communication
        """
        try:
            config = MotionConfig()
            config.set_json_str(json_str)
            return self.write_config(config)
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON: {e}")
            raise ValueError(f"Invalid JSON: {e}")

    def read_calibration(self) -> Calibration:
        """Read the BFI/BVI calibration from the console EEPROM JSON.

        Returns a :class:`omotion.Calibration` with ``source="console"`` if
        the JSON contains a valid ``calibration`` block, otherwise a fresh
        :meth:`Calibration.default` (``source="default"``). Never raises on
        bad data — invalid blocks are logged and replaced with defaults.

        Raises:
            ValueError: if the UART is not connected (propagates from
                ``read_config``).
        """
        try:
            cfg = self.read_config()
        except ValueError:
            raise
        except Exception as e:
            logger.warning(
                "read_calibration: read_config raised %s; using SDK defaults.",
                e,
            )
            return Calibration.default()

        if cfg is None:
            logger.info(
                "read_calibration: no config returned from device; "
                "using SDK defaults."
            )
            return Calibration.default()

        parsed = parse_calibration(cfg.json_data or {})
        if parsed is None:
            logger.info(
                "read_calibration: no calibration on device or invalid; "
                "using SDK defaults."
            )
            return Calibration.default()

        logger.info("read_calibration: loaded calibration from console.")
        return parsed

    def write_calibration(
        self, c_min, c_max, i_min, i_max
    ) -> Calibration:
        """Write a new BFI/BVI calibration to the console EEPROM.

        Validates inputs (shape ``(2, 8)``, finite, monotonic) before
        touching the wire. Performs a read-modify-write so other JSON
        keys (``EE_THRESH``, etc.) are preserved.

        Returns the :class:`Calibration` (``source="console"``) that was
        written. The caller is responsible for refreshing any cached copy
        — :meth:`omotion.MotionInterface.write_calibration` chains a
        refresh automatically.

        Raises:
            ValueError: invalid input or UART not connected.
            RuntimeError: existing config could not be read for the
                read-modify-write.
        """
        new_block = serialize_calibration(c_min, c_max, i_min, i_max)

        cfg = self.read_config()
        if cfg is None:
            raise RuntimeError(
                "write_calibration: could not read existing config for "
                "read-modify-write."
            )

        if cfg.json_data is None:
            cfg.json_data = {}
        cfg.json_data[CALIBRATION_JSON_KEY] = new_block[CALIBRATION_JSON_KEY]

        result = self.write_config(cfg)
        if result is None:
            raise RuntimeError("write_calibration: write_config returned None.")

        logger.info("write_calibration: calibration written to console.")
        return Calibration(
            c_min=np.asarray(c_min, dtype=float),
            c_max=np.asarray(c_max, dtype=float),
            i_min=np.asarray(i_min, dtype=float),
            i_max=np.asarray(i_max, dtype=float),
            source="console",
        )

    # ------------------------------------------------------------------ #
    # Page-by-page direct FPGA programming commands (0x30–0x3C)
    # These map 1-to-1 onto the XO2ECAcmd_* functions on the firmware so
    # that the host fully controls the programming sequence and can stream
    # one 16-byte page at a time.
    # ------------------------------------------------------------------ #

    def fpga_prog_open(self, fpga_chan: MuxChannel) -> None:
        """Open the FPGA configuration interface in Offline mode.

        `fpga_chan` must be a `MuxChannel` (from `omotion.config`).
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating FPGA open")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_OPEN,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_OPEN returned ERROR")
                raise CommandError("FPGA_PROG_OPEN failed", response=r)

            logger.info("FPGA_PROG_OPEN OK")

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error opening FPGA prog interface: %s", e)
            raise

    def fpga_prog_erase(self, fpga_chan: MuxChannel, mode: int) -> None:
        """
        Erase FPGA flash sectors.

        Parameters
        ----------
        mode:
            Erase mode bitmap (matches ``XO2ECAcmd_EraseFlash`` argument):

            * bit 3 = UFM sector
            * bit 2 = CFG sector
            * bit 1 = Feature Row
            * bit 0 = SRAM
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating FPGA erase")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_ERASE,
                reserved=int(fpga_chan),
                data=bytes([mode & 0xFF]),
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_ERASE returned ERROR")
                raise CommandError("FPGA_PROG_ERASE failed", response=r)

            logger.info("FPGA_PROG_ERASE OK (mode=0x%02X)", mode)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA erase: %s", e)
            raise

    def fpga_prog_cfg_reset(self, fpga_chan: MuxChannel) -> None:
        """Reset the CFG sector address pointer to page 0."""
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating FPGA reset page 0")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_CFG_RESET,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_CFG_RESET returned ERROR")
                raise CommandError("FPGA_PROG_CFG_RESET failed", response=r)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA reset page: %s", e)
            raise

    def fpga_prog_cfg_write_page(self, fpga_chan: MuxChannel, page: bytes) -> None:
        """
        Write one 16-byte CFG flash page at the current address pointer.

        Parameters
        ----------
        page:
            Exactly :data:`~protocol.constants.XO2_FLASH_PAGE_SIZE` (16) bytes.
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating FPGA write page")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            if len(page) != XO2_FLASH_PAGE_SIZE:
                raise ValueError(
                    f"CFG page must be {XO2_FLASH_PAGE_SIZE} bytes, got {len(page)}"
                )

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_CFG_WRITE_PAGE,
                reserved=int(fpga_chan),
                data=bytes(page),
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_CFG_WRITE_PAGE returned ERROR")
                raise CommandError("FPGA_PROG_CFG_WRITE_PAGE failed", response=r)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA page write: %s", e)
            raise

    def fpga_prog_cfg_write_pages(self, fpga_chan: MuxChannel, pages: bytes) -> None:
        """
        Write multiple consecutive CFG flash pages in a single command.

        Parameters
        ----------
        pages:
            A byte string whose length is a non-zero multiple of
            :data:`~protocol.constants.XO2_FLASH_PAGE_SIZE` (16).  Each
            16-byte chunk is written as one page in sequence starting from
            the current address pointer.
        """

        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating FPGA write pages")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            if len(pages) == 0 or len(pages) % XO2_FLASH_PAGE_SIZE != 0:
                raise ValueError(
                    f"pages length must be a non-zero multiple of "
                    f"{XO2_FLASH_PAGE_SIZE}, got {len(pages)}"
                )

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_CFG_WRITE_PAGES,
                reserved=int(fpga_chan),
                data=bytes(pages),
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_CFG_WRITE_PAGES returned ERROR")
                raise CommandError("FPGA_PROG_CFG_WRITE_PAGES failed", response=r)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA pages write: %s", e)
            raise

    def fpga_prog_cfg_read_page(self, fpga_chan: MuxChannel) -> bytes:
        """
        Read back one 16-byte CFG flash page from the current address pointer.

        Returns
        -------
        bytes
            16 bytes read from flash.
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating FPGA read page")
                # Return a randomized 16-byte page for demo/testing purposes
                return os.urandom(16)

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_CFG_READ_PAGE,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_CFG_READ_PAGE returned ERROR")
                raise CommandError("FPGA_PROG_CFG_READ_PAGE failed", response=r)

            return r.data

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA page read: %s", e)
            raise

    def fpga_prog_ufm_reset(self, fpga_chan: MuxChannel) -> None:
        """Reset the UFM sector address pointer to page 0."""

        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating ufm reset page 0")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_UFM_RESET,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_UFM_RESET returned ERROR")
                raise CommandError("FPGA_PROG_UFM_RESET failed", response=r)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA ufm reset page 0: %s", e)
            raise

    def fpga_prog_ufm_write_page(self, fpga_chan: MuxChannel, page: bytes) -> None:
        """
        Write one 16-byte UFM flash page at the current address pointer.

        Parameters
        ----------
        page:
            Exactly :data:`~protocol.constants.XO2_FLASH_PAGE_SIZE` (16) bytes.
        """

        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating ufm write page")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            if len(page) != XO2_FLASH_PAGE_SIZE:
                raise ValueError(
                    f"UFM page must be {XO2_FLASH_PAGE_SIZE} bytes, got {len(page)}"
                )

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_UFM_WRITE_PAGE,
                reserved=int(fpga_chan),
                data=bytes(page),
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_UFM_WRITE_PAGE returned ERROR")
                raise CommandError("FPGA_PROG_UFM_WRITE_PAGE failed", response=r)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA ufm write page : %s", e)
            raise

    def fpga_prog_ufm_write_pages(self, fpga_chan: MuxChannel, pages: bytes) -> None:
        """
        Write multiple consecutive UFM flash pages in a single command.

        Parameters
        ----------
        pages:
            A byte string whose length is a non-zero multiple of
            :data:`~protocol.constants.XO2_FLASH_PAGE_SIZE` (16).  Each
            16-byte chunk is written as one page in sequence starting from
            the current address pointer.
        """

        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating ufm write pages")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            if len(pages) == 0 or len(pages) % XO2_FLASH_PAGE_SIZE != 0:
                raise ValueError(
                    f"pages length must be a non-zero multiple of "
                    f"{XO2_FLASH_PAGE_SIZE}, got {len(pages)}"
                )

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_UFM_WRITE_PAGES,
                reserved=int(fpga_chan),
                data=bytes(pages),
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_UFM_WRITE_PAGES returned ERROR")
                raise CommandError("FPGA_PROG_UFM_WRITE_PAGES failed", response=r)

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA ufm write pages : %s", e)
            raise

    def fpga_prog_ufm_read_page(self, fpga_chan: MuxChannel) -> bytes:
        """
        Read back one 16-byte UFM flash page from the current address pointer.

        Returns
        -------
        bytes
            16 bytes read from flash.
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating ufm write pages")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_UFM_READ_PAGE,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_UFM_READ_PAGE returned ERROR")
                raise CommandError("FPGA_PROG_UFM_READ_PAGE failed", response=r)

            if len(r.data) != XO2_FLASH_PAGE_SIZE:
                raise ValueError(
                    f"UFM read page returned {len(r.data)} bytes, expected {XO2_FLASH_PAGE_SIZE}",
                    response=r,
                )

            return r.data

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA ufm write pages : %s", e)
            raise

    def fpga_prog_read_status(self, fpga_chan: MuxChannel) -> int:
        """
        Read the FPGA 32-bit Status Register.

        Does not require the configuration interface to be open — can be
        called at any time after the device handle is registered.

        Returns
        -------
        int
            Raw 32-bit status value.  Useful bit fields:

            * Bit 12 ``(sr >> 12) & 1`` – BUSY flag
            * Bit 13 ``(sr >> 13) & 1`` – FAIL flag
            * Bit 14 ``(sr >> 14) & 1`` – ISC_ENABLED flag
            * Bits 6:4 ``(sr >> 4) & 7`` – programming DONE / DEVICE_SECURITY
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating fpga read status")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_READ_STATUS,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_READ_STATUS returned ERROR")
                raise CommandError("FPGA_PROG_READ_STATUS failed", response=r)

            if len(r.data) != 4:
                raise ValueError(
                    f"READ_STATUS returned {len(r.data)} bytes, expected 4",
                    response=r,
                )
            sr = (r.data[0] << 24) | (r.data[1] << 16) | (r.data[2] << 8) | r.data[3]
            logger.debug(
                "FPGA Status Register: 0x%08X  BUSY=%d FAIL=%d ISC_EN=%d",
                sr,
                (sr >> 12) & 1,
                (sr >> 13) & 1,
                (sr >> 14) & 1,
            )
            return sr

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA read status : %s", e)
            raise

    def fpga_prog_featrow_write(
        self, fpga_chan: MuxChannel, feature: bytes, feabits: bytes
    ) -> None:
        """
        Write the Feature Row to the FPGA.

        Parameters
        ----------
        feature:
            8-byte feature row data.
        feabits:
            2-byte FEABITS data.
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating fpga read status")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            if len(feature) != 8:
                raise ValueError(f"feature must be 8 bytes, got {len(feature)}")
            if len(feabits) != 2:
                raise ValueError(f"feabits must be 2 bytes, got {len(feabits)}")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_FEATROW_WRITE,
                reserved=int(fpga_chan),
                data=bytes(feature) + bytes(feabits),
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_FEATROW_WRITE returned ERROR")
                raise CommandError("FPGA_PROG_FEATROW_WRITE failed", response=r)

            logger.info("FPGA_PROG_FEATROW_WRITE OK")
        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA read status : %s", e)
            raise

    def fpga_prog_featrow_read(self, fpga_chan: MuxChannel) -> tuple[bytes, bytes]:
        """
        Read back the Feature Row from the FPGA.

        Returns
        -------
        tuple[bytes, bytes]
            ``(feature_8_bytes, feabits_2_bytes)``
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating fpga featrow read")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_FEATROW_READ,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_FEATROW_READ returned ERROR")
                raise CommandError("FPGA_PROG_FEATROW_READ failed", response=r)

            if len(r.data) < 10:
                raise ValueError(
                    f"Feature row read returned {len(r.data)} bytes, expected 10",
                    response=r,
                )

            return r.data[:8], r.data[8:10]

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA featrow read: %s", e)
            raise

    def fpga_prog_set_done(self, fpga_chan: MuxChannel) -> None:
        """Set the DONE bit in the FPGA configuration flash."""

        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating fpga done")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_SET_DONE,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_SET_DONE returned ERROR")
                raise CommandError("FPGA_PROG_SET_DONE failed", response=r)

            logger.info("FPGA_PROG_SET_DONE OK")

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA done: %s", e)
            raise

    def fpga_prog_refresh(self, fpga_chan: MuxChannel) -> None:
        """
        Refresh the FPGA: loads configuration from flash and boots to user mode.

        Raises
        ------
        CommandError
            If the firmware fails to complete the refresh within its retry limit.
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating fpga refresh")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_REFRESH,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_REFRESH returned ERROR")
                raise CommandError("FPGA_PROG_REFRESH failed", response=r)

            logger.info("FPGA_PROG_REFRESH OK")

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA refresh: %s", e)
            raise

    def fpga_prog_close(self, fpga_chan: MuxChannel) -> None:
        """
        Close the FPGA configuration interface (abort path).

        Safe to call even if :meth:`fpga_prog_open` was never called.
        """
        try:
            if getattr(self.uart, "demo_mode", False):
                logger.info("Demo mode: simulating fpga close")
                return None

            if not self.uart or not self.is_connected():
                raise ValueError("Console Device not connected")

            # Send command (reserved = channel index)
            r = self.uart.send_packet(
                id=None,
                packetType=OW_FPGA_PROG,
                command=FPGA_PROG_CLOSE,
                reserved=int(fpga_chan),
                data=None,
            )

            self.uart.clear_buffer()

            if r.packet_type == OW_ERROR:
                logger.error("FPGA_PROG_CLOSE returned ERROR")
                raise CommandError("FPGA_PROG_CLOSE failed", response=r)

            logger.info("FPGA_PROG_CLOSE OK")

        except ValueError as v:
            logger.error("ValueError: %s", v)
            raise

        except Exception as e:
            logger.error("Unexpected error FPGA close: %s", e)
            raise

    def log_device_info(self) -> None:
        """Log console firmware version and hardware ID to the SDK logger."""
        try:
            fw_version = self.get_version()
            hw_id      = self.get_hardware_id()
            logger.info("Console: firmware=%s  hw_id=%s", fw_version, hw_id)
        except Exception as e:
            logger.warning("Console: failed to read device info: %s", e)

# Note: graceful disconnect is now driven by ConnectionMonitor via
# `request_disconnect()` (which submits an EVT_USER_STOP). The old
# `disconnect()`/`__del__` pair has been removed — the monitor owns the
# transport lifecycle.
