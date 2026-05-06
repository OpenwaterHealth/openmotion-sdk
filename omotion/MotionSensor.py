import logging
import struct
import threading
import time
from typing import Literal, Optional

import usb.core

from omotion.MotionComposite import MotionComposite
from omotion.usb_backend import get_libusb1_backend
from omotion.connection_state import ConnectionState
from omotion.signal_wrapper import SignalWrapper
from omotion.config import (
    OW_BAD_CRC,
    OW_BAD_PARSE,
    OW_CAMERA,
    OW_CAMERA_GET_HISTOGRAM,
    OW_CAMERA_SET_TESTPATTERN,
    OW_CAMERA_SINGLE_HISTOGRAM,
    OW_CAMERA_SET_CONFIG,
    OW_CMD,
    OW_CMD_ECHO,
    OW_CMD_HWID,
    OW_CMD_PING,
    OW_CMD_RESET,
    OW_CMD_TOGGLE_LED,
    OW_CMD_VERSION,
    OW_CTRL_FAN_CTL,
    OW_CMD_DEBUG_FLAGS,
    OW_CONTROLLER,
    OW_ERROR,
    OW_FPGA,
    OW_FPGA_ACTIVATE,
    OW_FPGA_BITSTREAM,
    OW_FPGA_ENTER_SRAM_PROG,
    OW_FPGA_ERASE_SRAM,
    OW_FPGA_EXIT_SRAM_PROG,
    OW_FPGA_ID,
    OW_FPGA_PROG_SRAM,
    OW_FPGA_RESET,
    OW_FPGA_STATUS,
    OW_FPGA_USERCODE,
    OW_IMU,
    OW_IMU_INIT,
    OW_IMU_ON,
    OW_IMU_OFF,
    OW_IMU_GET_ACCEL,
    OW_IMU_GET_GYRO,
    OW_IMU_GET_TEMP,
    OW_CAMERA_FSIN,
    OW_CAMERA_STREAM,
    OW_CAMERA_STATUS,
    OW_CAMERA_FSIN_EXTERNAL,
    OW_UNKNOWN,
    OW_CAMERA_SWITCH,
    OW_I2C_PASSTHRU,
    OW_CAMERA_POWER_OFF,
    OW_CAMERA_POWER_ON,
    OW_CAMERA_POWER_STATUS,
    OW_CAMERA_READ_SECURITY_UID,
    OW_CMD_DFU,
)
from omotion.i2c_packet import I2C_Packet
from omotion.GitHubReleases import GitHubReleases
from omotion.MotionProcessing import bytes_to_integers
from omotion.utils import calculate_file_crc
from omotion import _log_root

logger = logging.getLogger(f"{_log_root}.Sensor" if _log_root else "Sensor")

# Firmware response types that indicate an error condition.
_ERROR_TYPES = frozenset({OW_ERROR, OW_BAD_CRC, OW_BAD_PARSE, OW_UNKNOWN})


class MotionSensor(SignalWrapper):
    """Stable handle for a sensor module (left or right).

    Identified by USB ``port_numbers[-1]`` (2 = left, 3 = right). The handle
    is constructed once by ``MotionInterface`` and lives for its entire
    lifetime — never replaced. Apps cache the reference once and gate any
    use on ``handle.is_connected()``.

    The lifecycle is owned by ``ConnectionMonitor``. The on-entry sequence
    for ``CONNECTING`` is: usb.core.find by VID/PID + port → claim 3
    interfaces → ping → ``refresh_id_cache()`` (HWID + 8 camera UIDs) →
    version. Five-step retry backoff for the post-enumeration "resource
    busy" window. ``self.uart`` is None when DISCONNECTED and a fresh
    ``MotionComposite`` while CONNECTING/CONNECTED.
    """

    def __init__(
        self,
        side: Literal["left", "right"],
        vid: int,
        pid: int,
    ):
        super().__init__()
        self.side: Literal["left", "right"] = side
        self.name: str = side
        self.vid = vid
        self.pid = pid
        self._port_suffix = 2 if side == "left" else 3

        # Transport — None when DISCONNECTED; populated during CONNECTING.
        self.uart: Optional[MotionComposite] = None

        # Cached IDs (populated by refresh_id_cache during CONNECTING)
        self._cached_camera_uids: Optional[dict[int, str]] = None
        self._cached_hwid: Optional[str] = None
        self.hardware_id: Optional[str] = None  # alias kept on the handle for clarity
        self._version: str = "v0.0.0"

        # State machine
        self._state = ConnectionState.DISCONNECTED
        self._state_cv = threading.Condition()
        self._monitor = None  # set by MotionInterface.start()

    # ──────────────────────────────────────────────────────────────────
    # Compatibility: MotionSensor itself does not support demo mode in the
    # new design (it constructs its own MotionComposite from a real libusb
    # dev). Existing command method bodies use `self.demo_mode` to decide
    # whether to short-circuit with a mock value; with this set to False
    # they always proceed to `_send`, which raises cleanly when uart is
    # None. If a demo-mode sensor is ever needed, expose a constructor
    # parameter and override this attribute.
    # ──────────────────────────────────────────────────────────────────

    demo_mode: bool = False

    # ──────────────────────────────────────────────────────────────────
    # State (read-only from outside)
    # ──────────────────────────────────────────────────────────────────

    @property
    def state(self) -> ConnectionState:
        return self._state

    def is_connected(self) -> bool:
        return self._state == ConnectionState.CONNECTED

    def wait_for(self, target: ConnectionState, timeout: float = 5.0) -> bool:
        with self._state_cv:
            return self._state_cv.wait_for(
                lambda: self._state == target, timeout=timeout
            )

    def request_disconnect(self) -> None:
        if self._monitor is None:
            return
        from omotion.connection_monitor import UserStop

        self._monitor.submit(UserStop(handle_name=self.name))

    # ──────────────────────────────────────────────────────────────────
    # Wiring
    # ──────────────────────────────────────────────────────────────────

    def _attach_monitor(self, monitor) -> None:
        self._monitor = monitor

    def _on_uart_io_error(self, errno, message: str) -> None:
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
            "%s state %s -> %s (%s)",
            self.name,
            old.name,
            new_state.name,
            reason or "",
        )

    def _handle_event(self, event) -> None:
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
                reason = (
                    f"usb_io_error:errno={event.errno}"
                    if isinstance(event, IoError)
                    else "poll_gone"
                )
                self._drive_disconnecting(reason=reason)
            # Already DISCONNECTING/DISCONNECTED/CONNECTING → no-op (dedup).
        elif isinstance(event, UserStop):
            if st in (ConnectionState.CONNECTED, ConnectionState.CONNECTING):
                self._drive_disconnecting(reason="user_stop")

    def _find_dev(self):
        """Locate the libusb device matching this sensor's VID/PID + port suffix."""
        backend = get_libusb1_backend()
        for dev in usb.core.find(
            find_all=True, idVendor=self.vid, idProduct=self.pid, backend=backend
        ):
            try:
                ports = getattr(dev, "port_numbers", []) or []
                if ports and ports[-1] == self._port_suffix:
                    return dev
            except Exception:
                continue
        return None

    def _drive_connecting(self, reason: str) -> None:
        self._set_state(ConnectionState.CONNECTING, reason=reason)

        backoff = [0.05, 0.1, 0.25, 0.5, 1.0]
        last_error: Optional[Exception] = None
        for delay in backoff:
            try:
                dev = self._find_dev()
                if dev is None:
                    raise RuntimeError(
                        f"sensor device not found (VID=0x{self.vid:04X} "
                        f"PID=0x{self.pid:04X} port_suffix={self._port_suffix})"
                    )
                composite = MotionComposite(
                    dev,
                    desc=self.side.upper(),
                    async_mode=True,
                    on_io_error=self._on_uart_io_error,
                )
                composite.open()
                self.uart = composite

                # Ping to confirm firmware is responsive. Bound the wait so
                # a non-responsive device falls into our retry/backoff
                # loop instead of hanging on the default 10 s timeout
                # inherited from CommInterface.send_packet (which is sized
                # for normal in-scan command latency, not connect probes).
                # 2 s per attempt × 5 attempts + backoffs ≈ 12 s worst case,
                # which covers typical post-power-on firmware boot.
                r = self.uart.comm.send_packet(
                    id=None, packetType=OW_CMD, command=OW_CMD_PING,
                    timeout=2.0,
                )
                if r is None or r.packet_type in _ERROR_TYPES:
                    raise RuntimeError("sensor ping failed or returned error")

                # Read HWID + 8 camera security UIDs. HWID failure → retry.
                # Per-camera UID failures are tolerated (dead camera marks
                # its slot as "" but the connect still succeeds — same
                # lenient policy as the legacy refresh_id_cache).
                self.refresh_id_cache()
                if not self._cached_hwid:
                    raise RuntimeError("sensor HWID read returned empty")
                self.hardware_id = self._cached_hwid

                # Cache version (best-effort).
                try:
                    self._version = self.get_version()
                except Exception as e:
                    logger.debug("get_version during connect failed: %s", e)

                self._set_state(ConnectionState.CONNECTED, reason="ping_ok")
                return
            except Exception as e:
                last_error = e
                logger.warning(
                    "%s connect attempt failed (%s); retrying in %.0f ms",
                    self.name, e, delay * 1000,
                )
                # Roll back any partial open.
                try:
                    if self.uart is not None:
                        self.uart.close()
                except Exception:
                    pass
                self.uart = None
                self._cached_camera_uids = None
                self._cached_hwid = None
                self.hardware_id = None
                time.sleep(delay)

        self._set_state(
            ConnectionState.DISCONNECTED,
            reason=f"connect_retry_exhausted:{last_error}",
        )

    def _drive_disconnecting(self, reason: str) -> None:
        self._set_state(ConnectionState.DISCONNECTING, reason=reason)
        try:
            if self.uart is not None:
                self.uart.close()
        except Exception:
            logger.exception("uart close failed")
        self.uart = None
        self._cached_camera_uids = None
        self._cached_hwid = None
        self.hardware_id = None
        self._set_state(ConnectionState.DISCONNECTED, reason=reason)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _send(self, **kwargs):
        """Send a command packet and return the firmware response.

        Raises ValueError if the transport is not open. We allow sending
        during CONNECTING (after USB claim succeeds) so that the on-entry
        ping/refresh_id_cache/version sequence works without state
        gymnastics; ``self.uart`` is the gate, not state.
        """
        if self.uart is None:
            raise ValueError("Sensor Module not connected")
        return self.uart.comm.send_packet(id=None, **kwargs)

    def _check_camera_mask(self, camera_position: int) -> None:
        """Raise ValueError if camera_position is not a valid byte bitmask."""
        if not (0x00 <= camera_position <= 0xFF):
            raise ValueError(
                f"camera_position must be a byte (0x00 to 0xFF), got {camera_position:#04x}"
            )

    # ------------------------------------------------------------------
    # Basic commands
    # ------------------------------------------------------------------

    def ping(self) -> bool:
        """Send a ping and return True if the device acknowledges."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CMD, command=OW_CMD_PING)
        return r.packet_type not in _ERROR_TYPES

    def get_version(self) -> str:
        """Return the firmware version string (e.g. 'v1.2.3')."""
        if self.demo_mode:
            return "v0.1.1"
        r = self._send(packetType=OW_CMD, command=OW_CMD_VERSION)
        if r.data_len == 3:
            return f"v{r.data[0]}.{r.data[1]}.{r.data[2]}"
        if r.data_len and r.data:
            ver_str = (
                r.data[: r.data_len]
                .decode("utf-8", errors="ignore")
                .rstrip("\x00")
                .strip()
            )
            return ver_str or "v0.0.0"
        return "v0.0.0"

    def echo(self, echo_data=None) -> tuple[bytes, int]:
        """Send echo_data and return (echoed_bytes, length), or (None, None)."""
        if self.demo_mode:
            data = b"Hello Motion!!"
            return data, len(data)
        if echo_data is not None and not isinstance(echo_data, (bytes, bytearray)):
            raise TypeError("echo_data must be a byte array")
        r = self._send(packetType=OW_CMD, command=OW_CMD_ECHO, data=echo_data)
        return (r.data, r.data_len) if r.data_len > 0 else (None, None)

    def toggle_led(self) -> bool:
        """Toggle the status LED."""
        if self.demo_mode:
            return True
        self._send(packetType=OW_CMD, command=OW_CMD_TOGGLE_LED)
        return True

    def soft_reset(self) -> bool:
        """Perform a soft reset."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CMD, command=OW_CMD_RESET)
        return r.packet_type not in _ERROR_TYPES

    def enter_dfu(self) -> bool:
        """Reset into DFU (firmware update) mode."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CMD, command=OW_CMD_DFU)
        return r.packet_type != OW_ERROR

    def get_hardware_id(self) -> str | None:
        """Return the 16-byte hardware ID as a hex string, or None."""
        if self.demo_mode:
            return bytes.fromhex("deadbeefcafebabe1122334455667788")
        r = self._send(packetType=OW_CMD, command=OW_CMD_HWID)
        return r.data.hex() if r.data_len == 16 else None

    # ------------------------------------------------------------------
    # Fan control
    # ------------------------------------------------------------------

    def set_fan_control(self, fan_on: bool) -> bool:
        """Turn the fan ON (True) or OFF (False)."""
        if self.demo_mode:
            return True
        reserved = 0x01 | (0x02 if fan_on else 0x00)
        r = self._send(
            packetType=OW_CONTROLLER, command=OW_CTRL_FAN_CTL, reserved=reserved
        )
        return r.packet_type not in _ERROR_TYPES

    def get_fan_control_status(self) -> bool:
        """Return True if the fan is currently ON."""
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CONTROLLER, command=OW_CTRL_FAN_CTL, reserved=0x00
        )
        if r.packet_type in _ERROR_TYPES:
            return False
        return r.reserved == 1

    # ------------------------------------------------------------------
    # Debug flags
    # ------------------------------------------------------------------

    def set_debug_flags(self, flags: int) -> bool:
        """Set firmware debug flags (32-bit bitmask).

        Bit 0 (DEBUG_FLAG_USB_PRINTF) enables firmware printf output over USB.
        Bit 4 (DEBUG_FLAG_COMM_VERBOSE) enables cmd id and "." response prints.
        Bit 5 (DEBUG_FLAG_CMD_VERBOSE) enables printf in command handlers.
        """
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CMD,
            command=OW_CMD_DEBUG_FLAGS,
            reserved=1,
            data=struct.pack("<I", flags),
        )
        if r.packet_type in _ERROR_TYPES:
            return False
        if r.data_len == 4:
            logger.debug("Debug flags set to: 0x%08X", struct.unpack("<I", r.data)[0])
        return True

    def get_debug_flags(self) -> int:
        """Return the current firmware debug flags, or 0 on error."""
        if self.demo_mode:
            return 0
        r = self._send(packetType=OW_CMD, command=OW_CMD_DEBUG_FLAGS, reserved=0)
        if r.packet_type in _ERROR_TYPES or r.data_len != 4:
            return 0
        flags = struct.unpack("<I", r.data)[0]
        logger.info("Debug flags: 0x%08X", flags)
        return flags

    # ------------------------------------------------------------------
    # IMU
    # ------------------------------------------------------------------

    def imu_init(self) -> bool:
        """Initialise the IMU hardware.

        Must be called before :meth:`imu_on`.
        """
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_IMU, command=OW_IMU_INIT)
        return r is not None

    def imu_on(self) -> bool:
        """Power on the IMU (accelerometer and gyroscope).

        Includes a 100 ms startup delay so data registers are valid when
        the caller proceeds to read motion data.
        """
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_IMU, command=OW_IMU_ON)
        # Most IMU chips require 50–100 ms after power-on before data registers
        # are valid.
        time.sleep(0.1)
        return r is not None

    def imu_off(self) -> bool:
        """Power down the IMU."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_IMU, command=OW_IMU_OFF)
        return r is not None

    def imu_get_temperature(self) -> float:
        """Return IMU temperature in degrees Celsius."""
        if self.demo_mode:
            return 25.0
        r = self._send(packetType=OW_IMU, command=OW_IMU_GET_TEMP)
        if r.data_len != 4:
            raise ValueError(
                f"Invalid data length for IMU temperature: expected 4, got {r.data_len}"
            )
        return round(struct.unpack("<f", r.data)[0], 2)

    def imu_get_accelerometer(self) -> list[int]:
        """Return raw accelerometer readings as [x, y, z] signed 16-bit integers."""
        if self.demo_mode:
            return [0, 0, 0]
        r = self._send(packetType=OW_IMU, command=OW_IMU_GET_ACCEL)
        if r.data_len != 6:
            raise ValueError(
                f"Invalid data length for accelerometer: expected 6, got {r.data_len}"
            )
        return list(struct.unpack("<hhh", r.data))

    def imu_get_gyroscope(self) -> list[int]:
        """Return raw gyroscope readings as [x, y, z] signed 16-bit integers."""
        if self.demo_mode:
            return [0, 0, 0]
        r = self._send(packetType=OW_IMU, command=OW_IMU_GET_GYRO)
        if r.data_len != 6:
            raise ValueError(
                f"Invalid data length for gyroscope: expected 6, got {r.data_len}"
            )
        return list(struct.unpack("<hhh", r.data))

    # ------------------------------------------------------------------
    # FPGA management
    # ------------------------------------------------------------------

    def reset_camera_sensor(self, camera_position: int) -> bool:
        """Reset the camera sensor(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_FPGA, command=OW_FPGA_RESET, addr=camera_position)
        return r.packet_type not in _ERROR_TYPES

    def activate_camera_fpga(self, camera_position: int) -> bool:
        """Activate the FPGA for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA, command=OW_FPGA_ACTIVATE, addr=camera_position
        )
        return r.packet_type not in _ERROR_TYPES

    def check_camera_fpga(self, camera_position: int) -> bool:
        """Return True if the FPGA ID check passes for the given bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_FPGA, command=OW_FPGA_ID, addr=camera_position)
        return r.packet_type not in _ERROR_TYPES

    def enter_sram_prog_fpga(self, camera_position: int) -> bool:
        """Enter SRAM programming mode for the FPGA(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA,
            command=OW_FPGA_ENTER_SRAM_PROG,
            addr=camera_position,
        )
        return r.packet_type not in _ERROR_TYPES

    def exit_sram_prog_fpga(self, camera_position: int) -> bool:
        """Exit SRAM programming mode for the FPGA(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA,
            command=OW_FPGA_EXIT_SRAM_PROG,
            addr=camera_position,
        )
        return r.packet_type not in _ERROR_TYPES

    def erase_sram_fpga(self, camera_position: int) -> bool:
        """Erase SRAM for the FPGA(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA,
            command=OW_FPGA_ERASE_SRAM,
            addr=camera_position,
            timeout=30,
        )
        return r.packet_type not in _ERROR_TYPES

    def get_status_fpga(self, camera_position: int) -> bool:
        """Return the FPGA status for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA, command=OW_FPGA_STATUS, addr=camera_position
        )
        return r.packet_type not in _ERROR_TYPES

    def get_usercode_fpga(self, camera_position: int) -> bool:
        """Return the FPGA usercode for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA, command=OW_FPGA_USERCODE, addr=camera_position
        )
        return r.packet_type not in _ERROR_TYPES

    def send_bitstream_fpga(self, filename=None) -> bool:
        """Send a bitstream file to the FPGA in 1 kB blocks.

        Args:
            filename: Full path to the bitstream file.

        Returns:
            True on success, False if the file is missing or a block is rejected.
        """
        if filename is None:
            raise ValueError("Filename cannot be None")

        max_bytes_per_block = 1024
        block_count = 0
        total_bytes_sent = 0

        try:
            file_crc = calculate_file_crc(filename)
            logger.info("CRC16 of file: %s", hex(file_crc))

            with open(filename, "rb") as f:
                while True:
                    data = f.read(max_bytes_per_block)

                    if not data:
                        # EOF — send final block carrying the file CRC
                        r = self._send(
                            packetType=OW_FPGA,
                            command=OW_FPGA_BITSTREAM,
                            addr=block_count,
                            reserved=1,
                            data=file_crc.to_bytes(2, byteorder="big"),
                        )
                        if r.packet_type in _ERROR_TYPES:
                            logger.error("Error sending final CRC block")
                            return False
                        break

                    r = self._send(
                        packetType=OW_FPGA,
                        command=OW_FPGA_BITSTREAM,
                        addr=block_count,
                        reserved=0,
                        data=data,
                    )
                    if r.packet_type in _ERROR_TYPES:
                        logger.error("Error sending block %d", block_count)
                        return False

                    total_bytes_sent += len(data)
                    block_count += 1

            logger.info(
                "Bitstream upload complete. Blocks sent: %d, Total bytes: %d",
                block_count,
                total_bytes_sent,
            )
            return True

        except FileNotFoundError:
            logger.error("File %s not found.", filename)
            return False

    def program_fpga(self, camera_position: int, manual_process: bool) -> bool:
        """Program the FPGA SRAM for the camera(s) indicated by the bitmask.

        This command triggers the firmware to load the bitstream; it can take
        up to 60 seconds for a full load.
        """
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA,
            command=OW_FPGA_PROG_SRAM,
            addr=camera_position,
            reserved=1,
            timeout=60,
        )
        return r.packet_type not in _ERROR_TYPES

    # ------------------------------------------------------------------
    # Camera configuration
    # ------------------------------------------------------------------

    def camera_configure_registers(self, camera_position: int) -> bool:
        """Write the default register set to the camera sensor(s)."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_SET_CONFIG,
            addr=camera_position,
            timeout=60,
        )
        return r.packet_type not in _ERROR_TYPES

    def camera_configure_test_pattern(
        self, camera_position: int, test_pattern: int = 0
    ) -> bool:
        """Load a test pattern into the camera sensor register(s).

        Args:
            camera_position: Bitmask of target camera(s).
            test_pattern: Pattern index 0–4 (default 0 = colour bars).
        """
        self._check_camera_mask(camera_position)
        if not (0x00 <= test_pattern <= 0x04):
            raise ValueError(
                f"test_pattern must be 0x00 to 0x04, got {test_pattern:#04x}"
            )
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_SET_TESTPATTERN,
            addr=camera_position,
            data=bytearray([test_pattern]),
            timeout=60,
        )
        return r.packet_type not in _ERROR_TYPES

    def camera_capture_histogram(self, camera_position: int) -> bool:
        """Trigger a single-frame histogram capture for the given camera(s)."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_SINGLE_HISTOGRAM,
            addr=camera_position,
            reserved=0,
            timeout=15,
        )
        return r.packet_type not in _ERROR_TYPES

    def camera_get_histogram(self, camera_position: int) -> bytearray | None:
        """Retrieve the last captured histogram as raw bytes.

        Returns 4100 bytes: 4096 bytes of uint32-LE histogram bins followed by
        a 4-byte float32 temperature.  Returns None on firmware error.
        """
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return None
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_GET_HISTOGRAM,
            addr=camera_position,
            timeout=15,
        )
        if r.packet_type in _ERROR_TYPES:
            return None
        logger.debug("HIST Data Len: %d", len(r.data))
        return r.data

    def get_camera_histogram(
        self,
        camera_id: int,
        test_pattern_id: int = 4,
        auto_upload: bool = True,
    ) -> tuple[list[int], list[int]] | None:
        """High-level convenience method: program, configure, capture, and return a histogram."""
        if not (0 <= camera_id <= 7):
            logger.error("Camera ID must be 0-7.")
            return None

        camera_mask = 1 << camera_id

        status_map = self.get_camera_status(camera_mask)
        if not status_map or camera_id not in status_map:
            logger.error("Failed to get camera status.")
            return None

        status = status_map[camera_id]
        logger.debug(
            "Camera %d status: 0x%02X -> %s",
            camera_id,
            status,
            self.decode_camera_status(status),
        )

        if not status & (1 << 0):
            logger.debug("Camera peripheral not READY.")
            return None

        if not (status & (1 << 1) and status & (1 << 2)):
            logger.debug("FPGA Configuration Started")
            start_time = time.time()
            if auto_upload:
                if not self.program_fpga(
                    camera_position=camera_mask, manual_process=False
                ):
                    logger.error("Failed to program FPGA.")
                    return None
            logger.debug(
                "FPGAs programmed | Time: %.2f ms",
                (time.time() - start_time) * 1000,
            )

        if not (status & (1 << 1) and status & (1 << 2)):
            logger.debug("Programming camera sensor registers.")
            if not self.camera_configure_registers(camera_mask):
                logger.error("Failed to configure registers.")
                return None

        logger.debug("Setting test pattern...")
        if not self.camera_configure_test_pattern(camera_mask, test_pattern_id):
            logger.error("Failed to set test pattern.")
            return None

        status_map = self.get_camera_status(camera_mask)
        if not status_map or camera_id not in status_map:
            logger.error("Failed to get camera status.")
            return None

        status = status_map[camera_id]
        logger.debug(
            "Camera %d status: 0x%02X -> %s",
            camera_id,
            status,
            self.decode_camera_status(status),
        )
        if not (status & (1 << 0) and status & (1 << 1) and status & (1 << 2)):
            logger.error("Not configured for histogram.")
            return None

        logger.debug("Capturing histogram...")
        if not self.camera_capture_histogram(camera_mask):
            logger.error("Capture failed.")
            return None

        logger.debug("Retrieving histogram...")
        histogram = self.camera_get_histogram(camera_mask)
        if histogram is None:
            logger.error("Histogram retrieval failed.")
            return None

        logger.debug("Histogram frame received successfully.")
        return bytes_to_integers(histogram[:4096])

    def get_camera_status(self, camera_position: int) -> dict[int, int] | None:
        """Return a mapping of camera ID → status byte for each queried camera.

        Status byte bits:
            0 — Peripheral READY (SPI/USART)
            1 — Firmware programmed
            2 — Configured
            7 — Streaming enabled
        """
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return {i: 0x07 for i in range(8) if (camera_position >> i) & 1}
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_STATUS,
            addr=camera_position,
        )
        if r.packet_type == OW_ERROR or len(r.data) != 8:
            logger.error("Error getting camera status")
            return None
        return {i: r.data[i] for i in range(8) if (camera_position >> i) & 1}

    # ------------------------------------------------------------------
    # Camera power
    # ------------------------------------------------------------------

    def enable_camera_power(self, camera_mask: int) -> bool:
        """Power on the camera(s) indicated by the bitmask (0x01–0xFF)."""
        if not (0x01 <= camera_mask <= 0xFF):
            raise ValueError(
                f"camera_mask must be between 0x01 and 0xFF, got {camera_mask:#04x}"
            )
        # Firmware may delay 200 ms + I2C scan per camera; use extended timeout.
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_POWER_ON,
            addr=camera_mask,
            timeout=8,
        )
        if r.packet_type in _ERROR_TYPES:
            logger.error(
                "enable_camera_power(0x%02x) rejected by firmware: packet_type=%s",
                camera_mask, r.packet_type,
            )
            return False
        return True

    def disable_camera_power(self, camera_mask: int) -> bool:
        """Power off the camera(s) indicated by the bitmask (0x01–0xFF)."""
        if not (0x01 <= camera_mask <= 0xFF):
            raise ValueError(
                f"camera_mask must be between 0x01 and 0xFF, got {camera_mask:#04x}"
            )
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_POWER_OFF,
            addr=camera_mask,
            timeout=8,
        )
        if r.packet_type in _ERROR_TYPES:
            logger.error(
                "disable_camera_power(0x%02x) rejected by firmware: packet_type=%s",
                camera_mask, r.packet_type,
            )
            return False
        return True

    def get_camera_power_status(self) -> list:
        """Return a list of 8 booleans indicating per-camera power state (index 0–7)."""
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_POWER_STATUS,
            addr=0xFF,
            timeout=0.12,
        )
        if r.packet_type in _ERROR_TYPES:
            return [False] * 8
        power_status = [False] * 8
        if r.data and len(r.data) >= 1:
            power_mask = r.data[0]
            for i in range(8):
                power_status[i] = bool(power_mask & (1 << i))
        return power_status

    def read_camera_security_uid(self, camera_id: int) -> bytes:
        """Return the 6-byte security UID for camera_id (0–7).

        Returns 6 zero bytes if the camera is absent or returns invalid data.
        """
        if not (0 <= camera_id <= 7):
            raise ValueError(f"camera_id must be 0–7, got {camera_id}")
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_READ_SECURITY_UID,
            addr=camera_id,
        )
        if r.packet_type in _ERROR_TYPES:
            return bytes(6)
        if r.data and len(r.data) >= 6:
            return bytes(r.data[:6])
        logger.warning(
            "Invalid UID data length for camera %d: %d",
            camera_id,
            len(r.data) if r.data else 0,
        )
        return bytes(6)

    # ------------------------------------------------------------------
    # Frame synchronisation / streaming
    # ------------------------------------------------------------------

    def enable_aggregator_fsin(self) -> bool:
        """Enable the internal frame-sync signal generator."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CAMERA, command=OW_CAMERA_FSIN, reserved=1)
        return r.packet_type not in _ERROR_TYPES

    def disable_aggregator_fsin(self) -> bool:
        """Disable the internal frame-sync signal generator."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CAMERA, command=OW_CAMERA_FSIN, reserved=0)
        return r.packet_type not in _ERROR_TYPES

    def enable_camera(self, camera_position) -> bool:
        """Enable streaming for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        # 1.5 s accommodates stream-armed IF0 contention: when streaming on
        # IF1 has just been armed, the MCU can take ~1 s to service the
        # enable request. A tighter timeout causes the SDK to discard the
        # eventual (stale) response and poison the next packet ID.
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_STREAM,
            reserved=1,
            addr=camera_position,
            timeout=1.5,
        )
        return r.packet_type not in _ERROR_TYPES

    def disable_camera(self, camera_position) -> bool:
        """Disable streaming for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_STREAM,
            reserved=0,
            addr=camera_position,
            timeout=0.3,
        )
        return r.packet_type not in _ERROR_TYPES

    def enable_camera_fsin_ext(self) -> bool:
        """Enable external frame-sync input."""
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_FSIN_EXTERNAL,
            reserved=1,
            timeout=0.6,
        )
        return r.packet_type not in _ERROR_TYPES

    def disable_camera_fsin_ext(self) -> bool:
        """Disable external frame-sync input."""
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA, command=OW_CAMERA_FSIN_EXTERNAL, reserved=0
        )
        return r.packet_type not in _ERROR_TYPES

    def switch_camera(self, camera_id):
        """Switch the active camera mux to camera_id."""
        return self._send(
            packetType=OW_CAMERA,
            command=OW_CAMERA_SWITCH,
            data=camera_id.to_bytes(1, "big"),
        )

    # ------------------------------------------------------------------
    # I2C passthrough / direct sensor control
    # ------------------------------------------------------------------

    def camera_i2c_write(self, packet, packet_id=None):
        """Write a single register via the I2C passthrough interface."""
        if self.demo_mode:
            return True
        data = packet.register_address.to_bytes(2, "big") + packet.data.to_bytes(
            1, "big"
        )
        r = self._send(
            packetType=OW_I2C_PASSTHRU, command=packet.device_address, data=data
        )
        return r.packet_type not in _ERROR_TYPES

    def camera_set_gain(self, gain, packet_id=None):
        """Set the analogue gain register on the image sensor."""
        gain = gain & 0xFF
        ret = self.camera_i2c_write(
            I2C_Packet(device_address=0x36, register_address=0x3508, data=gain)
        )
        time.sleep(0.05)
        ret |= self.camera_i2c_write(
            I2C_Packet(device_address=0x36, register_address=0x3509, data=0x00)
        )
        time.sleep(0.05)
        logger.info("Gain set to %d", gain)
        return ret

    def camera_set_exposure(self, exposure_selection, us=None):
        """Set the exposure time via the I2C passthrough interface."""
        exposures = [0x1F, 0x20, 0x2C, 0x2D, 0x7A]
        exposure_byte = exposures[exposure_selection]
        if us is not None:
            exposure_byte = int((us / 9)) & 0xFF
        ret = self.camera_i2c_write(
            I2C_Packet(device_address=0x36, register_address=0x3501, data=0x00)
        )
        time.sleep(0.05)
        ret |= self.camera_i2c_write(
            I2C_Packet(device_address=0x36, register_address=0x3502, data=exposure_byte)
        )
        time.sleep(0.05)
        logger.info("Exposure set to %d (%d us)", exposure_byte, exposure_byte * 9)
        return ret

    # ------------------------------------------------------------------
    # ID cache
    # ------------------------------------------------------------------

    def refresh_id_cache(self) -> None:
        """Read and cache all camera security UIDs (0–7) and the sensor hardware ID.

        Used both as part of the CONNECTING on-entry sequence (called from
        ``_drive_connecting`` while ``state`` is CONNECTING — gating on
        ``is_connected()`` would early-return there) and as a manually-
        invoked refresh once connected. The transport (``self.uart``) is
        the gate: if it's None, the inner command sends raise cleanly.

        Also updates :data:`omotion.MotionProcessing.PEDESTAL_HEIGHT` based
        on the sensor firmware version (64 for ≤ 1.5.2, 128 for ≥ 1.5.3).
        """
        self._cached_camera_uids = None
        self._cached_hwid = None
        try:
            if self.uart is None:
                return
            uids = {}
            for camera_id in range(8):
                try:
                    uid_bytes = self.read_camera_security_uid(camera_id)
                    uid_hex = "".join(f"{b:02X}" for b in uid_bytes)
                    uids[camera_id] = f"0x{uid_hex}" if uid_hex else ""
                except Exception as e:
                    logger.debug("Could not read camera %s UID: %s", camera_id, e)
                    uids[camera_id] = ""
            self._cached_camera_uids = uids
            try:
                hw_id = self.get_hardware_id()
                self._cached_hwid = (
                    hw_id.hex() if isinstance(hw_id, bytes) else (hw_id or "")
                ) or ""
            except Exception as e:
                logger.debug("Could not read HWID: %s", e)
                self._cached_hwid = ""
            self._refresh_pedestal_height()
        except Exception as e:
            logger.warning("Failed to refresh sensor ID cache: %s", e)
            self._cached_camera_uids = None
            self._cached_hwid = None

    def _refresh_pedestal_height(self) -> None:
        """Set :data:`omotion.MotionProcessing.PEDESTAL_HEIGHT` from the firmware version.

        Sensor firmware 1.5.2 and earlier use a pedestal of 64; firmware 1.5.3
        and later use 128.  If the version cannot be parsed the existing value
        is left unchanged and a warning is logged.
        """
        import omotion.MotionProcessing as _mp

        try:
            version_str = self.get_version().lstrip("v")
            parts = tuple(int(x) for x in version_str.split("."))
        except Exception as e:
            logger.warning(
                "Could not parse firmware version for pedestal selection: %s", e
            )
            return

        pedestal = 64.0 if parts <= (1, 5, 2) else 128.0
        _mp.PEDESTAL_HEIGHT = pedestal
        logger.info(
            "Pedestal height set to %g based on sensor firmware v%s",
            pedestal,
            version_str,
        )

    def clear_id_cache(self) -> None:
        """Clear cached camera UIDs and hardware ID (e.g. on disconnect)."""
        self._cached_camera_uids = None
        self._cached_hwid = None

    def get_cached_camera_security_uid(self, camera_id: int) -> str:
        """Return the cached security UID hex string for the given camera (0–7).

        Returns "" if not connected, cache not populated, or invalid camera_id.
        """
        if not self.is_connected() or self._cached_camera_uids is None:
            return ""
        cid = int(camera_id)
        out = self._cached_camera_uids.get(cid, "")
        if not out and 1 <= cid <= 8:
            out = self._cached_camera_uids.get(cid - 1, "")
        return out or ""

    def get_cached_hardware_id(self) -> str:
        """Return the cached sensor hardware ID as a hex string.

        Returns "" if not connected or cache not populated.
        """
        if not self.is_connected() or self._cached_hwid is None:
            return ""
        return self._cached_hwid or ""

    # ------------------------------------------------------------------
    # Firmware version / release info
    # ------------------------------------------------------------------

    @staticmethod
    def get_latest_version_info():
        """Query GitHub for the sensor firmware releases.

        Returns a dict with keys ``"latest"`` (tag + date of the newest
        non-prerelease) and ``"releases"`` (all tags with date and prerelease
        flag).
        """
        gh = GitHubReleases("OpenwaterHealth", "openmotion-sensor-fw")

        try:
            latest = gh.get_latest_release()
        except Exception:
            latest = None

        try:
            all_releases = gh.get_all_releases(include_prerelease=True)
        except Exception:
            all_releases = []

        releases_map = {}
        for r in all_releases:
            tag = r.get("tag_name")
            if not tag:
                continue
            prerelease_flag = bool(r.get("prerelease")) or str(tag).lower().startswith(
                "pre-"
            )
            releases_map[tag] = {
                "published_at": r.get("published_at"),
                "prerelease": prerelease_flag,
            }

        return {
            "latest": {
                "tag_name": latest.get("tag_name") if latest else None,
                "published_at": latest.get("published_at") if latest else None,
            },
            "releases": releases_map,
        }

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def log_device_info(self) -> None:
        """Log sensor firmware version, hardware ID, and cached camera UIDs to the SDK logger."""
        try:
            fw_version = self.get_version()
            hw_id      = self.get_cached_hardware_id() or self.get_hardware_id()
            if self._cached_camera_uids:
                uid_summary = ", ".join(
                    f"cam{k}={v}" for k, v in sorted(self._cached_camera_uids.items()) if v
                )
            else:
                uid_summary = "none cached"
            logger.info(
                "Sensor: firmware=%s  hw_id=%s  camera_uids=[%s]",
                fw_version, hw_id, uid_summary,
            )
        except Exception as e:
            logger.warning("Sensor: failed to read device info: %s", e)

# Note: graceful disconnect is now driven by ConnectionMonitor via
# `request_disconnect()` (which submits an EVT_USER_STOP). The old
# `disconnect()`/`__del__` pair has been removed — the monitor owns the
# transport lifecycle.

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def decode_camera_status(status: int) -> str:
        """Decode a camera status byte into a human-readable string."""
        flags = []
        if status & (1 << 0):
            flags.append("READY")
        if status & (1 << 1):
            flags.append("PROGRAMMED")
        if status & (1 << 2):
            flags.append("CONFIGURED")
        if status & (1 << 7):
            flags.append("STREAMING")
        return " | ".join(flags) if flags else "NONE"
