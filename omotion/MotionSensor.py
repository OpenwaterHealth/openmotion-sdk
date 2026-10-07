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
    DEBUG_FLAG_CAMERA_RAW,
    OW_CAMERA_SET_CONFIG,
    OW_CMD,
    OW_CMD_DIAG_STATS,
    OW_CMD_ECHO,
    OW_CMD_HWID,
    OW_CMD_I2C_REG_READ,
    OW_CMD_I2C_STATUS,
    OW_CMD_BOOT_INFO,
    OW_CMD_PING,
    OW_CMD_RESET,
    OW_CMD_TOGGLE_LED,
    OW_CMD_VERSION,
    OW_CTRL_FAN_CTL,
    OW_CMD_DEBUG_FLAGS,
    OW_CONTROLLER,
    OW_ERROR,
    OW_FACTORY_CRESET,
    OW_FACTORY_I2C_SCAN,
    OW_FACTORY_I2C_RD,
    OW_FACTORY_I2C_WR,
    OW_FACTORY_I2C_WRRD,
    OW_FACTORY_NVCM_CHECK,
    OW_FPGA,
    OW_FPGA_ACTIVATE,
    OW_FPGA_BITSTREAM,
    OW_FPGA_ENTER_SRAM_PROG,
    OW_FPGA_ERASE_SRAM,
    OW_FPGA_EXIT_SRAM_PROG,
    OW_FPGA_ID,
    OW_FPGA_PROG,
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
    OW_CAMERA_GET_TELEMETRY,
    OW_CMD_DFU,
    OW_CMD_SERIAL,
    is_valid_serial,
)
from omotion.i2c_packet import I2C_Packet
from omotion.boot_mode import BootMode, parse_boot_info
from omotion.GitHubReleases import GitHubReleases
from omotion.MotionProcessing import bytes_to_integers
from omotion.utils import calculate_file_crc, log_i2c_health
from omotion import _log_root

logger = logging.getLogger(f"{_log_root}.Sensor" if _log_root else "Sensor")

# Firmware response types that indicate an error condition.
_ERROR_TYPES = frozenset({OW_ERROR, OW_BAD_CRC, OW_BAD_PARSE, OW_UNKNOWN})


from omotion.firmware_update import parse_version as _parse_firmware_version


# --- Camera telemetry (sensor-fw#94, OB/black-level block sensor-fw#103) ---
# Wire format: cam_telemetry_response_t in sensor-fw Core/Inc/camera_telemetry.h
# — 12-byte header {version, valid_mask, struct_size, reserved,
# fsin_pulse_count, uptime_ms} followed by 8 packed 110-byte per-camera
# records (little-endian). The firmware sends raw register values; this parser
# owns every engineering-unit conversion.
CAM_TELEMETRY_VERSION = 2
_CAM_TELEM_HDR_FMT = "<BBBBII"  # version, valid_mask, struct_size, rsvd, fsin_pulse_count, uptime_ms
_CAM_TELEM_CAM_FMT = "<II22H22B11H14B"
_CAM_TELEM_CAM_SIZE = struct.calcsize(_CAM_TELEM_CAM_FMT)  # 110
_CAM_TELEM_HDR_SIZE = struct.calcsize(_CAM_TELEM_HDR_FMT)  # 12
_CAM_TELEM_SIZE = _CAM_TELEM_HDR_SIZE + 8 * _CAM_TELEM_CAM_SIZE  # 892


def _vm_volts(raw: int) -> float:
    """On-die voltage-monitor code -> volts (OX02C1B DS table A-39)."""
    return (raw & 0x0FFF) * 6.0 / 4096.0


def _tpm_celsius(raw: int) -> float:
    """8.8 fixed-point die temperature; >0xC000 encodes negative (DS 10.5.23)."""
    if raw > 0xC000:
        return -((raw - 0xC000) / 256.0)
    return raw / 256.0


def parse_camera_telemetry(data: bytes) -> dict | None:
    """Parse a cam_telemetry_response_t blob; None if malformed or version-mismatched.

    Returned dict: ``{"version", "valid_mask", "fsin_pulse_count", "uptime_ms",
    "cameras": [dict * 8]}`` where each camera dict carries converted values
    (``avdd_v``/``dovdd_v``/``dvdd_v`` volts, ``tpm_avg_c``/``tpm0_c``/``tpm1_c``
    degC, ``again_x``/``dgain_x`` gain factors) alongside the raw
    fault/state/counter bytes.

    The optical-black block (sensor-fw#103) adds ``z_avg`` — the four
    zero-line (dark row) averages, Bayer positions 00/01/10/11 — plus the
    derived ``z_avg_mean``/``z_avg_spread`` and the window, target, trigger
    and fault context that produced them (``zl_start``/``zl_end``,
    ``blk_lvl_target``, ``blc_trig_ctrl``, ``blc_fault_latch``, ...).

    Bench-established (2026-07-20, left sensor): ``z_avg`` and ``blc_offsets``
    update **only while the sensor is scanning rows** — they read 0 at idle
    and populate within a sweep of stream-on, so judge them against
    ``sc_state`` (0x9 = streaming), not ``updated_ms``. They are *not* gated
    by ``blc_en``: raw mode (sensor-fw#89, ``blc_ctrl`` 0x00) yields the same
    values, because the statistics engine runs whether or not the correction
    is applied. ``z_avg`` carries 6 fractional bits — ``z_avg / 64`` is DN
    (measured 121.7 DN against the sensor's own raw-mode ``yavg`` of 121).

    The sensor is mono, so all four ``z_avg`` values average the same physical
    dark rows, but they do **not** agree: Bayer positions 10/11 sit ~60 LSB
    (~0.8 %) above 00/01 on every camera, a reproducible row-parity split.
    ``z_avg_spread`` therefore has a nonzero floor — track it against that
    baseline rather than against 0.

    ``fsin_pulse_count`` is the firmware's frame
    trigger counter — external (console-driven) FSIN edges only; the sensor's
    own frame counter has no readable SCCB value register, and internal-FSIN
    frames don't increment this. ``valid`` is False for a camera the firmware
    has never completed
    a sweep on (fields all zero); ``updated_ms`` (firmware HAL_GetTick, compare
    against ``uptime_ms``) reveals staleness, ``sweep_count`` liveness.
    """
    if data is None or len(data) < _CAM_TELEM_SIZE:
        return None
    (version, valid_mask, struct_size, _,
     fsin_pulse_count, uptime_ms) = struct.unpack_from(_CAM_TELEM_HDR_FMT, data, 0)
    if version != CAM_TELEMETRY_VERSION or struct_size != _CAM_TELEM_CAM_SIZE:
        logger.warning(
            "camera telemetry format mismatch (version %d size %d, expected %d/%d)",
            version, struct_size, CAM_TELEMETRY_VERSION, _CAM_TELEM_CAM_SIZE)
        return None

    cameras = []
    for i in range(8):
        f = struct.unpack_from(
            _CAM_TELEM_CAM_FMT, data, _CAM_TELEM_HDR_SIZE + i * _CAM_TELEM_CAM_SIZE)
        updated_ms, dgain_raw = f[0:2]
        (avdd, dovdd, dvdd, tpm_avg, tpm0, tpm1, tc_row, expo_cmd, expo_applied,
         again_raw, isp_real_gain, isp_dig_gain, isp_blc, isp_expo) = f[2:16]
        blc_offsets = [v & 0x7FFF for v in f[16:24]]
        (tpm_status, vm_live, vm_cp, vm_latched, vm_cp_latched,
         wd_a, wd_b, wd_sticky, _wd_tpm_hi, _wd_tpm_lo, wd_state,
         sc_state, otp_crc0, otp_crc1, trig_error, yavg, aec_mode,
         dcg_state, blc_ctrl, isp_ctrl, i2c_err_count, sweep_count) = f[24:46]
        # OB / black-level block (sensor-fw#103, DS table A-25). z_avg and the
        # offsets are 15-bit in a 16-bit register; bit 15 is reserved.
        z_avg = [v & 0x7FFF for v in f[46:50]]
        blc_offsets_z = [v & 0x7FFF for v in f[50:54]]
        blc_thres, blk_lvl_target, zero_ln_num = f[54:57]
        (blc_trig_ctrl, bl_start, bl_end, blk_ln_num, blc_ln_mode,
         zl_start, zl_end, zavg_ctrl, zl_start2, zl_end2,
         blc_fault_latch, blc_fault_state, dig_test_fail, dtr_fault) = f[57:71]

        # Analog gain: 0x3508[4:0] = code[8:4], 0x3509[7:4] = code[3:0]; x = code/16.
        again_code = (((again_raw >> 8) & 0x1F) << 4) | ((again_raw >> 4) & 0x0F)
        # Digital gain: 0x350A[3:0]=code[13:10], 0x350B=code[9:2], 0x350C[7:6]=code[1:0].
        dgain_code = ((((dgain_raw >> 16) & 0x0F) << 10)
                      | (((dgain_raw >> 8) & 0xFF) << 2)
                      | ((dgain_raw & 0xFF) >> 6))

        cameras.append({
            "valid": bool(valid_mask & (1 << i)),
            "updated_ms": updated_ms,
            "sweep_count": sweep_count,
            "i2c_err_count": i2c_err_count,
            "avdd_v": _vm_volts(avdd),
            "dovdd_v": _vm_volts(dovdd),
            "dvdd_v": _vm_volts(dvdd),
            "tpm_avg_c": _tpm_celsius(tpm_avg),
            "tpm0_c": _tpm_celsius(tpm0),
            "tpm1_c": _tpm_celsius(tpm1),
            "tpm_status": tpm_status,
            "vm_live": vm_live,
            "vm_cp": vm_cp,
            "vm_latched": vm_latched,
            "vm_cp_latched": vm_cp_latched,
            "wd_fault_a": wd_a,
            "wd_fault_b": wd_b,
            "wd_sticky": wd_sticky & 0x01,
            "wd_state": wd_state,
            "sc_state": sc_state & 0x0F,
            "otp_crc": (otp_crc0, otp_crc1),
            "trig_error": trig_error,
            "yavg": yavg,
            "tc_row": tc_row,
            "expo_cmd": expo_cmd,
            "expo_applied": expo_applied,
            "again_cmd": again_raw,
            "again_x": again_code / 16.0,
            "dgain_x": dgain_code / 1024.0,
            "aec_mode": aec_mode,
            "dcg_state": dcg_state,
            "blc_ctrl": blc_ctrl,
            "isp_ctrl": isp_ctrl,
            "isp_real_gain": isp_real_gain,
            "isp_dig_gain": isp_dig_gain,
            "isp_blc": isp_blc,
            "isp_expo": isp_expo,
            "blc_offsets": blc_offsets,
            # --- OB / black-level block (sensor-fw#103) ---
            # z_avg_00/01/10/11 are the zero-line (dark row) averages per
            # Bayer position. The OX02C1B is mono here, so all four sample the
            # same physical dark rows and should agree — z_avg_spread is the
            # per-camera sanity metric, z_avg_mean the dark pedestal estimate.
            "z_avg": z_avg,
            "z_avg_mean": sum(z_avg) / 4.0,
            "z_avg_spread": max(z_avg) - min(z_avg),
            "blc_offsets_z": blc_offsets_z,
            "blc_thres": blc_thres & 0x07FF,
            "blk_lvl_target": blk_lvl_target & 0x07FF,
            "zero_ln_num": zero_ln_num & 0x03FF,
            "blc_trig_ctrl": blc_trig_ctrl,
            "bl_start": bl_start & 0x3F,
            "bl_end": bl_end & 0x3F,
            "blk_ln_num": blk_ln_num,
            "blc_ln_mode": blc_ln_mode,
            "zl_start": zl_start,
            "zl_end": zl_end,
            "zavg_ctrl": zavg_ctrl,
            "z_avg_sel": zavg_ctrl & 0x03,
            "zl_start2": zl_start2,
            "zl_end2": zl_end2,
            "blc_fault_latch": blc_fault_latch,
            "blc_fault_state": blc_fault_state & 0x01,
            "dig_test_fail": dig_test_fail,
            "dtr_fault": dtr_fault,
        })
    return {"version": version, "valid_mask": valid_mask,
            "fsin_pulse_count": fsin_pulse_count, "uptime_ms": uptime_ms,
            "cameras": cameras}


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

        # Boot-time I2C health snapshot, populated at connection (None until
        # then, or if the device firmware predates the I2C-status command).
        self._i2c_health: Optional[dict] = None

        # State machine
        self._state = ConnectionState.DISCONNECTED
        self._state_reason = ""
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

    @property
    def state_reason(self) -> str:
        """Reason given for the last state transition ("" before any)."""
        return self._state_reason

    def _set_state(self, new_state: ConnectionState, reason: str = "") -> None:
        with self._state_cv:
            if self._state == new_state:
                return
            old = self._state
            self._state = new_state
            self._state_reason = reason
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
            composite: Optional[MotionComposite] = None
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
                if r is None or r.packetType in _ERROR_TYPES:
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

                # Assess device health from the firmware's boot-time I2C scan.
                # Best-effort: never blocks or fails the connection. Done
                # before the CONNECTED transition so handle.i2c_health is
                # ready the instant a waiter observes is_connected().
                self._check_i2c_health()
                self._set_state(ConnectionState.CONNECTED, reason="ping_ok")
                return
            except Exception as e:
                last_error = e
                logger.warning(
                    "%s connect attempt failed (%s); retrying in %.0f ms",
                    self.name, e, delay * 1000,
                )
                # Roll back any partial open. Close the *local* composite,
                # not just self.uart: when open() itself raised, self.uart
                # was never assigned, and skipping close orphaned the
                # composite — leaking its claimed USB interfaces and (before
                # CommInterface deferred it to start_read_thread) a
                # _process_responses thread per failed attempt, ~52 of which
                # showed up in a packaged-app fault dump (2026-08-17) after
                # a day of connect/disconnect cycles. On attempts where open
                # succeeded, composite is self.uart — same object, same
                # close.
                try:
                    if composite is not None:
                        composite.close()
                except Exception:
                    pass
                self.uart = None
                self._cached_camera_uids = None
                self._cached_hwid = None
                self.hardware_id = None
                self._i2c_health = None
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
        self._i2c_health = None
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
        return r.packetType not in _ERROR_TYPES

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

    def get_boot_mode(self) -> BootMode:
        """Whether this sensor runs a bare-metal or bootloader-slot image.

        Queries OW_CMD_BOOT_INFO over the normal command interface — no DFU
        cycle — and classifies the reported ``SCB->VTOR``. Firmware without the
        command replies OW_UNKNOWN, which yields :data:`BootMode.UNKNOWN`; so
        does any garbled/short reply. Never raises: callers treat UNKNOWN as
        "couldn't determine" and must not make flashing decisions on it (the DFU
        alt-setting check remains the authoritative gate before any write).
        """
        if self.demo_mode:
            return BootMode.BARE_METAL
        try:
            r = self._send(packetType=OW_CMD, command=OW_CMD_BOOT_INFO)
        except Exception:
            return BootMode.UNKNOWN
        if r is None or r.packetType in _ERROR_TYPES:
            return BootMode.UNKNOWN
        return parse_boot_info(bytes(r.data[: r.data_len]) if r.data else b"")

    def read_serial_number(self) -> str | None:
        """Read the sensor module hardware serial number (None if unprogrammed/error)."""
        try:
            if self.demo_mode:
                return "QWW04Q10003"
            if not self.is_connected():
                logger.error("Sensor Module not connected")
                return None
            r = self._send(packetType=OW_CMD, command=OW_CMD_SERIAL, reserved=0)
            if r is None or r.packetType in _ERROR_TYPES:
                logger.error("Error reading sensor serial number")
                return None
            if r.data_len == 0:
                return None  # unprogrammed
            return bytes(r.data[: r.data_len]).decode("ascii", errors="replace")
        except Exception as e:
            logger.error("read_serial_number failed: %s", e)
            return None

    def write_serial_number(self, serial: str, force: bool = False) -> bool:
        """Write the sensor module hardware serial number.

        Args:
            serial: 1-24 uppercase-alphanumeric characters.
            force: if False, refuses to overwrite an already-programmed serial.
        Returns:
            bool: True on ACK, False on NAK/error/invalid input.
        """
        if not is_valid_serial(serial):
            logger.error("Invalid sensor serial %r (need 1-24 of [A-Z0-9])", serial)
            return False
        try:
            if self.demo_mode:
                return True
            if not self.is_connected():
                logger.error("Sensor Module not connected")
                return False
            r = self._send(
                packetType=OW_CMD,
                command=OW_CMD_SERIAL,
                reserved=(2 if force else 1),
                data=serial.encode("ascii"),
            )
            if r is None or r.packetType in _ERROR_TYPES:
                logger.error("Sensor rejected serial write (already programmed? use force)")
                return False

            # Read-back verify. Guards against firmware that ACKs the command but
            # doesn't persist it, so a write only reports success once the value
            # is actually readable back.
            readback = self.read_serial_number()
            if readback != serial:
                logger.error(
                    "Sensor serial write not persisted (read back %r, expected %r); "
                    "firmware may not support OW_CMD_SERIAL",
                    readback, serial,
                )
                return False
            return True
        except Exception as e:
            logger.error("write_serial_number failed: %s", e)
            return False

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
        return r.packetType not in _ERROR_TYPES

    def enter_dfu(self) -> bool:
        """Reset into DFU (firmware update) mode."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CMD, command=OW_CMD_DFU)
        return r.packetType != OW_ERROR

    def get_hardware_id(self) -> str | None:
        """Return the 16-byte hardware ID as a hex string, or None."""
        if self.demo_mode:
            return bytes.fromhex("deadbeefcafebabe1122334455667788")
        r = self._send(packetType=OW_CMD, command=OW_CMD_HWID)
        return r.data.hex() if r.data_len == 16 else None

    # ------------------------------------------------------------------
    # I2C health
    # ------------------------------------------------------------------

    def get_i2c_health(self, rescan: bool = False) -> dict | None:
        """Return the boot-time I2C health snapshot, or None on error.

        The firmware verifies, at startup, that every expected I2C device is
        present: the TCA9548A mux, the ICM-20948 IMU, and all 8 cameras
        (OX02C1B) + 8 FPGAs (CrossLink) behind the mux. The USB PHY is not on
        I2C (ULPI) and is excluded.

        Args:
            rescan: if True, ask the firmware to re-run the scan live (powers
                each camera one at a time, ~2 s) before returning. If False,
                returns the cached boot snapshot immediately.

        Returns a dict::

            {
                "version": int,
                "mux": bool,             # TCA9548A 0x70
                "imu": bool,             # ICM-20948 0x68
                "cameras": [bool] * 8,   # OX02C1B 0x36 per mux channel
                "fpgas":   [bool] * 8,   # CrossLink 0x40 per mux channel
                "cameras_expected": int, # bitmask, 0xFF = all 8
                "all_present": bool,
            }
        """
        if self.demo_mode:
            return {
                "version": 1,
                "mux": True,
                "imu": True,
                "cameras": [True] * 8,
                "fpgas": [True] * 8,
                "cameras_expected": 0xFF,
                "all_present": True,
            }
        r = self._send(
            packetType=OW_CMD,
            command=OW_CMD_I2C_STATUS,
            reserved=(1 if rescan else 0),
        )
        if r is None or r.packetType in _ERROR_TYPES or r.data_len < 8:
            return None
        d = r.data
        cam_mask = d[3]
        fpga_mask = d[4]
        return {
            "version": d[0],
            "mux": bool(d[1]),
            "imu": bool(d[2]),
            "cameras": [bool(cam_mask & (1 << i)) for i in range(8)],
            "fpgas": [bool(fpga_mask & (1 << i)) for i in range(8)],
            "cameras_expected": d[5],
            "all_present": bool(d[6]),
        }

    # ------------------------------------------------------------------
    # Diagnostics (sensor-fw#70)
    # ------------------------------------------------------------------

    _DIAG_STATS_FMT = "<B3x8IIIII"  # version, pad[3], cam_overrun_count[8], cmp_fail/timeout/fallback_count, cmp_max_time_us
    _DIAG_STATS_SIZE = struct.calcsize(_DIAG_STATS_FMT)  # 52 bytes

    def get_diag_stats(self) -> dict | None:
        """Return the live firmware diagnostics snapshot, or None on error.

        Printf-independent (works regardless of DEBUG_FLAG_USB_PRINTF) —
        queries cam_diag_stats_t directly via OW_CMD_DIAG_STATS. Counters are
        for the CURRENT scan (reset at scan start/end by the firmware); query
        mid-scan to see live values, see sensor-fw camera_manager.c.

        Returns a dict::

            {
                "version": int,
                "cam_overrun_count": [int] * 8,  # per-camera SPI/USART RX overruns
                "cmp_fail_count": int,           # rle_compress dst_max overflow
                "cmp_timeout_count": int,         # rle_compress hit its time budget (#70)
                "cmp_fallback_count": int,        # frames sent uncompressed (either above)
                "cmp_max_time_us": int,           # worst-case compression time this scan
            }
        """
        if self.demo_mode:
            return {
                "version": 1,
                "cam_overrun_count": [0] * 8,
                "cmp_fail_count": 0,
                "cmp_timeout_count": 0,
                "cmp_fallback_count": 0,
                "cmp_max_time_us": 0,
            }
        r = self._send(packetType=OW_CMD, command=OW_CMD_DIAG_STATS)
        if r is None or r.packetType in _ERROR_TYPES or r.data_len < self._DIAG_STATS_SIZE:
            return None
        (version, *counts) = struct.unpack(
            self._DIAG_STATS_FMT, r.data[: self._DIAG_STATS_SIZE]
        )
        return {
            "version": version,
            "cam_overrun_count": list(counts[0:8]),
            "cmp_fail_count": counts[8],
            "cmp_timeout_count": counts[9],
            "cmp_fallback_count": counts[10],
            "cmp_max_time_us": counts[11],
        }

    def get_camera_telemetry(self) -> dict | None:
        """Return the firmware's cached per-camera condition telemetry, or None.

        sensor-fw#94: firmware continuously sweeps every powered OX02C1B in the
        background (rails from the on-die voltage monitor, dual die temps,
        VM/watchdog fault latches, sensor state machine, OTP CRC status, MIPI
        frame counter, FSIN trigger errors, on-chip frame mean, commanded vs
        applied exposure/gain, applied BLC offsets, and the optical-black
        block: dark-row averages plus their window/target/fault context) and
        this command returns
        the cached snapshot — no camera I2C happens at query time, so it is
        safe to poll during scans. Fleet refresh is ~1 s; per-camera
        ``updated_ms``/``sweep_count`` reveal staleness. See
        ``parse_camera_telemetry`` for the returned structure.
        """
        if self.demo_mode:
            cam = {
                "valid": True, "updated_ms": 1000, "sweep_count": 1,
                "i2c_err_count": 0,
                "avdd_v": 2.8, "dovdd_v": 1.8, "dvdd_v": 1.2,
                "tpm_avg_c": 45.0, "tpm0_c": 45.0, "tpm1_c": 45.0,
                "tpm_status": 0, "vm_live": 0, "vm_cp": 0,
                "vm_latched": 0, "vm_cp_latched": 0,
                "wd_fault_a": 0, "wd_fault_b": 0, "wd_sticky": 0,
                "wd_state": 0, "sc_state": 0x9, "otp_crc": (0, 0),
                "trig_error": 0, "yavg": 128,
                "tc_row": 0, "expo_cmd": 0x48, "expo_applied": 0x48,
                "again_cmd": 0x0100, "again_x": 1.0, "dgain_x": 1.0,
                "aec_mode": 0xA8, "dcg_state": 0x40,
                "blc_ctrl": 0x23, "isp_ctrl": 0x34,
                "isp_real_gain": 0x10, "isp_dig_gain": 0x400,
                "isp_blc": 0x80, "isp_expo": 0x48,
                "blc_offsets": [0] * 8,
                "z_avg": [128] * 4, "z_avg_mean": 128.0, "z_avg_spread": 0,
                "blc_offsets_z": [0] * 4,
                "blc_thres": 0, "blk_lvl_target": 128, "zero_ln_num": 2,
                "blc_trig_ctrl": 0xF9, "bl_start": 4, "bl_end": 0x1B,
                "blk_ln_num": 4, "blc_ln_mode": 0x50,
                "zl_start": 2, "zl_end": 0x0D,
                "zavg_ctrl": 0, "z_avg_sel": 0, "zl_start2": 8, "zl_end2": 0x0D,
                "blc_fault_latch": 0, "blc_fault_state": 0,
                "dig_test_fail": 0, "dtr_fault": 0,
            }
            return {"version": CAM_TELEMETRY_VERSION, "valid_mask": 0xFF,
                    "fsin_pulse_count": 0, "uptime_ms": 1000,
                    "cameras": [dict(cam) for _ in range(8)]}
        r = self._send(packetType=OW_CAMERA, command=OW_CAMERA_GET_TELEMETRY)
        if r is None or r.packetType in _ERROR_TYPES or r.data_len < _CAM_TELEM_SIZE:
            return None
        return parse_camera_telemetry(bytes(r.data[:_CAM_TELEM_SIZE]))

    def _check_i2c_health(self) -> None:
        """Read and cache the boot-time I2C health snapshot (connection step).

        Best-effort: reads the cached firmware snapshot (no disruptive rescan),
        never raises, and never affects the connection result. Stores the
        snapshot on the handle and logs the outcome.
        """
        try:
            self._i2c_health = self.get_i2c_health()
        except Exception as e:
            logger.debug("%s: I2C health check failed: %s", self.name, e)
            self._i2c_health = None
        log_i2c_health(self.name, self._i2c_health, logger)

    @property
    def i2c_health(self) -> Optional[dict]:
        """Cached boot-time I2C health snapshot, or None if unavailable.

        Populated at connection. See :meth:`get_i2c_health` for the shape.
        """
        return self._i2c_health

    def is_i2c_healthy(self) -> bool:
        """True iff a health snapshot is present and every expected device responded."""
        return bool(self._i2c_health and self._i2c_health.get("all_present"))

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
        return r.packetType not in _ERROR_TYPES

    def get_fan_control_status(self) -> bool:
        """Return True if the fan is currently ON."""
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CONTROLLER, command=OW_CTRL_FAN_CTL, reserved=0x00
        )
        if r.packetType in _ERROR_TYPES:
            return False
        return r.reserved == 1

    # ------------------------------------------------------------------
    # Factory Commands
    # ------------------------------------------------------------------
    def i2c_scan(self) -> list[int] | Literal[False]:
        """Scan the I2C bus and return a list of found device addresses.

        Returns:
            List of 7-bit I2C addresses (integers) that responded, or
            ``False`` if the device returned an error response (check
            with ``is False``).
        """
        r = self._send(packetType=OW_FPGA_PROG, command=OW_FACTORY_I2C_SCAN)
        if r.packetType in _ERROR_TYPES:
            return False
        addresses = list(r.data[:r.data_len]) if r.data and r.data_len else []
        logger.info("LP i2c_scan: found %d device(s): %s",
                    len(addresses),
                    [f"0x{a:02X}" for a in addresses])
        return addresses
    
    def creset(self, state: bool | None = None) -> int | Literal[False]:
        """Control or read the FPGA CRESET pin.

        Args:
            state: True  → drive CRESET high (release reset).
                   False → drive CRESET low  (assert reset).
                   None  → read current state without changing it.

        Returns:
            Current CRESET pin state: 1 = high, 0 = low, or ``False``
            if the device returned an error response. 0 is a valid pin
            state — distinguish errors with ``is False``.
        """
        if state is None:
            data = None          # 0-byte payload → firmware reads pin
        else:
            data = bytearray([0x01 if state else 0x00])
        r = self._send(packetType=OW_FPGA_PROG, command=OW_FACTORY_CRESET, data=data)
        if r.packetType in _ERROR_TYPES:
            return False
                
        pin = r.data[0] if r.data and r.data_len >= 1 else 0
        logger.debug("LP creset: pin=%d", pin)
        return pin

    def i2c_write(
        self, dev_addr: int, data: bytes | bytearray
    ) -> Literal[False] | None:
        """Write bytes to an I2C device.

        Payload: [dev_addr, write_len_hi, write_len_lo, data...]

        Args:
            dev_addr: 7-bit I2C device address.
            data: Bytes to write.

        Returns:
            None on success, or ``False`` if the device returned an
            error response (check with ``is False``).

        Raises:
            ValueError: If data is empty.
        """
        if not data:
            raise ValueError("i2c_write requires at least 1 data byte")
        write_len = len(data)
        payload = bytearray([(write_len >> 8) & 0xFF,
                              write_len       & 0xFF])
        payload += bytearray(data)
        
        r = self._send(packetType=OW_FPGA_PROG, command=OW_FACTORY_I2C_WR, data=payload)
        if r.packetType in _ERROR_TYPES:
            return False
        
        logger.debug("LP i2c_write: addr=0x%02X len=%d data=%s",
                     dev_addr, write_len, [f"0x{b:02X}" for b in data])
    
    def i2c_read(self, dev_addr: int, read_len: int) -> bytes | Literal[False]:
        """Read bytes from an I2C device.

        Payload: [dev_addr, read_len_hi, read_len_lo]

        Args:
            dev_addr: 7-bit I2C device address.
            read_len: Number of bytes to read.

        Returns:
            Bytes read from the device, or ``False`` if the device
            returned an error response (check with ``is False``).

        Raises:
            ValueError: If read_len < 1.
        """
        if read_len < 1:
            raise ValueError("i2c_read requires read_len >= 1")
        payload = bytearray([(read_len >> 8) & 0xFF,
                              read_len       & 0xFF])
        
        r = self._send(packetType=OW_FPGA_PROG, command=OW_FACTORY_I2C_RD, data=payload)
        if r.packetType in _ERROR_TYPES:
            return False
        
        result = bytes(r.data[:r.data_len]) if r.data and r.data_len else b""
        logger.debug("LP i2c_read: addr=0x%02X len=%d data=%s",
                     dev_addr, len(result), [f"0x{b:02X}" for b in result])
        return result

    def i2c_write_read(self, dev_addr: int, data: bytes | bytearray,
                       read_len: int) -> bytes | Literal[False]:
        """Write bytes then read bytes from an I2C device (combined transfer).

        Payload: [dev_addr, write_len_hi, write_len_lo,
                  read_len_hi, read_len_lo, write_data...]

        Args:
            dev_addr: 7-bit I2C device address.
            data: Bytes to write.
            read_len: Number of bytes to read back.

        Returns:
            Bytes read from the device, or ``False`` if the device
            returned an error response (check with ``is False``).

        Raises:
            ValueError: If data is empty or read_len < 1.
        """
        if not data:
            raise ValueError("i2c_write_read requires at least 1 write byte")
        if read_len < 1:
            raise ValueError("i2c_write_read requires read_len >= 1")
        write_len = len(data)
        payload = bytearray([(write_len >> 8) & 0xFF,
                              write_len       & 0xFF,
                             (read_len  >> 8) & 0xFF,
                              read_len        & 0xFF])
        payload += bytearray(data)
        
        r = self._send(packetType=OW_FPGA_PROG, command=OW_FACTORY_I2C_WRRD, data=payload)
        if r.packetType in _ERROR_TYPES:
            return False
        
        result = bytes(r.data[:r.data_len]) if r.data and r.data_len else b""
        logger.debug("LP i2c_write_read: addr=0x%02X wrote=%d read=%d data=%s",
                     dev_addr, write_len, len(result),
                     [f"0x{b:02X}" for b in result])
        return result

    def i2c_read_register(self, dev_addr: int, reg_addr: int, read_len: int = 1,
                          reg_addr_size: int = 1,
                          mux_channel: Optional[int] = None) -> bytes | Literal[False]:
        """Read register bytes from an arbitrary I2C device on the sensor bus.

        Payload (7 bytes, big-endian):
            [dev_addr, reg_addr_size, mux_channel,
             reg_addr_hi, reg_addr_lo, read_len_hi, read_len_lo]
        where ``mux_channel == 0xFF`` means "do not touch the TCA9548A mux".

        Args:
            dev_addr: 7-bit I2C device address (0x00-0x7F).
            reg_addr: Register / memory address to read from.
            read_len: Number of bytes to read (1-256).
            reg_addr_size: Register address width in bytes: 1 (8-bit) or 2 (16-bit).
            mux_channel: TCA9548A (0x70) channel 0-7 to select before reading,
                or None to read a device directly on the bus.

        Returns:
            Bytes read from the device, or ``False`` on an error
            response (check with ``is False``).

        Raises:
            ValueError: On out-of-range arguments.
        """
        if not (0x00 <= dev_addr <= 0x7F):
            raise ValueError(f"dev_addr must be 0x00-0x7F, got {dev_addr:#04x}")
        if reg_addr_size not in (1, 2):
            raise ValueError(f"reg_addr_size must be 1 or 2, got {reg_addr_size}")
        max_reg = 0xFF if reg_addr_size == 1 else 0xFFFF
        if not (0 <= reg_addr <= max_reg):
            raise ValueError(
                f"reg_addr 0x{reg_addr:X} does not fit in "
                f"{reg_addr_size * 8}-bit address")
        if not (1 <= read_len <= 256):
            raise ValueError(f"read_len must be 1-256, got {read_len}")
        if mux_channel is not None and not (0 <= mux_channel <= 7):
            raise ValueError(f"mux_channel must be 0-7 or None, got {mux_channel}")

        mux_byte = 0xFF if mux_channel is None else mux_channel
        payload = bytearray([
            dev_addr      & 0xFF,
            reg_addr_size & 0xFF,
            mux_byte      & 0xFF,
            (reg_addr >> 8) & 0xFF,
            reg_addr        & 0xFF,
            (read_len >> 8) & 0xFF,
            read_len        & 0xFF,
        ])

        r = self._send(packetType=OW_CMD, command=OW_CMD_I2C_REG_READ, data=payload)
        if r.packetType in _ERROR_TYPES:
            return False

        result = bytes(r.data[:r.data_len]) if r.data and r.data_len else b""
        logger.debug("i2c_read_register: addr=0x%02X reg=0x%X size=%d len=%d data=%s",
                     dev_addr, reg_addr, reg_addr_size, len(result),
                     [f"0x{b:02X}" for b in result])
        return result

    def nvcm_check(self, isc_operand: int = 0x08, num_rows: int = 1,
                   boot_test: bool = True) -> bytes:
        """Probe the active camera's CrossLink NVCM state.

        Dumps the ISC register discriminators over I2C for diagnostics and —
        on firmware with sensor-fw#92 — appends the pin-drive boot verdict
        byte, the ONLY field that answers "is it programmed": 1 = the NVCM
        design booted and drove the camera bus, 0 = no boot, 0xFF = probe
        refused (camera unpowered). The register reads cannot answer it:
        STATUS bit 19 ("SDM Enable") merely mirrors the NVCM Done fuse — a
        part can have the fuse burned yet never boot (openmotion-test-app#44)
        — the content reads float 0xFF (the NVCM array is not read-enabled
        in this flow), and the SRAM Done bit reads 0 on every part. Older
        firmware returns the blob without the trailing byte; for a verdict
        there, use the behavioral fallback (reset_camera_sensor + timed
        non-forced program_fpga; see scripts/nvcm_probe.py).

        Select the camera first with switch_camera() — checking its response
        — and make sure it is powered.

        Args:
            isc_operand: ISC_ENABLE operand1 — 0x08 = NVCM access (default),
                         0x00 = SRAM access.
            num_rows:    Number of 16-byte NVCM array rows to read back (0-8).
            boot_test:   Also release CRESETB without the activation key and
                         probe 0x40.  Informational only — NOT a programmed/
                         blank discriminator: the config port needs the
                         activation key to respond, so 0x40 never ACKs here
                         regardless of NVCM state (openmotion-test-app#44).

        Returns:
            Raw fixed-layout response blob (see scripts/nvcm_probe.py for the
            field layout incl. the trailing verdict byte), or b"" on error.
        """
        payload = bytearray([isc_operand & 0xFF, num_rows & 0xFF,
                             1 if boot_test else 0])
        r = self._send(packetType=OW_FPGA_PROG,
                       command=OW_FACTORY_NVCM_CHECK,
                       data=payload,
                       timeout=8)
        if r.packetType in _ERROR_TYPES:
            logger.error("nvcm_check: firmware returned error type 0x%02X",
                         r.packetType)
            return b""
        return bytes(r.data[:r.data_len]) if r.data and r.data_len else b""

    # ------------------------------------------------------------------
    # Debug flags
    # ------------------------------------------------------------------

    def set_debug_flags(self, flags: int) -> bool:
        """Set firmware debug flags (32-bit bitmask).

        Bit 0 (DEBUG_FLAG_USB_PRINTF) enables firmware printf output over USB.
        Bit 4 (DEBUG_FLAG_COMM_VERBOSE) enables cmd id and "." response prints.
        Bit 5 (DEBUG_FLAG_CMD_VERBOSE) enables printf in command handlers.
        Bit 7 (DEBUG_FLAG_SEND_DEFER) defers the per-frame histogram send out
        of the FSIN ISR into the main loop (sensor-fw#68).
        Bit 8 (DEBUG_FLAG_HISTO_STALL) stops histogram sends after ~45 s of
        streaming while USB stays alive — deterministic camera-stall repro
        (sensor-fw#75).
        Bit 9 (DEBUG_FLAG_CAMERA_CROP) crops camera output to 1720x1280 at
        camera (re)configuration (sensor-fw#86).
        Bit 10 (DEBUG_FLAG_CAMERA_RAW) disables all on-sensor pixel
        corrections at camera (re)configuration (sensor-fw#89) — prefer
        :meth:`set_camera_raw_mode`.
        """
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CMD,
            command=OW_CMD_DEBUG_FLAGS,
            reserved=1,
            data=struct.pack("<I", flags),
        )
        if r.packetType in _ERROR_TYPES:
            return False
        if r.data_len == 4:
            logger.debug("Debug flags set to: 0x%08X", struct.unpack("<I", r.data)[0])
        return True

    def get_debug_flags(self) -> int:
        """Return the current firmware debug flags, or 0 on error."""
        if self.demo_mode:
            return 0
        r = self._send(packetType=OW_CMD, command=OW_CMD_DEBUG_FLAGS, reserved=0)
        if r.packetType in _ERROR_TYPES or r.data_len != 4:
            return 0
        flags = struct.unpack("<I", r.data)[0]
        logger.info("Debug flags: 0x%08X", flags)
        return flags

    def set_camera_raw_mode(self, enable: bool) -> bool:
        """Enable/disable the raw "scientific sensor" camera mode (sensor-fw#89).

        Sets or clears DEBUG_FLAG_CAMERA_RAW (bit 10), preserving all other
        debug flags. While the flag is set, camera (re)configuration disables
        every on-sensor pixel correction — BLC, DC-BLC, BLC dither and OTP
        defect-pixel correction — so pixels are bare ADC codes.

        The flag is read at camera-configure time only: power-cycle the
        cameras (or the sensor) and re-run the configure workflow for it to
        take effect — OW_CAMERA_SET_CONFIG skips cameras it considers
        already configured. In raw mode the dark level sits at the raw
        per-channel pedestal (roughly 255 DN at 1x analog gain, 495 DN at
        16x) instead of the servoed target, so PEDESTAL_HEIGHT-based dark
        handling is invalid — engineering/scientific captures only, not
        production scans.
        """
        flags = self.get_debug_flags()
        if enable:
            flags |= DEBUG_FLAG_CAMERA_RAW
        else:
            flags &= ~DEBUG_FLAG_CAMERA_RAW
        return self.set_debug_flags(flags)

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
        return r.packetType not in _ERROR_TYPES

    def activate_camera_fpga(self, camera_position: int) -> bool:
        """Activate the FPGA for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA, command=OW_FPGA_ACTIVATE, addr=camera_position
        )
        return r.packetType not in _ERROR_TYPES

    def check_camera_fpga(self, camera_position: int) -> bool:
        """Return True if the FPGA ID check passes for the given bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_FPGA, command=OW_FPGA_ID, addr=camera_position)
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

    def get_status_fpga(self, camera_position: int) -> bool:
        """Return the FPGA status for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA, command=OW_FPGA_STATUS, addr=camera_position
        )
        return r.packetType not in _ERROR_TYPES

    def get_usercode_fpga(self, camera_position: int) -> bool:
        """Return the FPGA usercode for the camera(s) indicated by the bitmask."""
        self._check_camera_mask(camera_position)
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_FPGA, command=OW_FPGA_USERCODE, addr=camera_position
        )
        return r.packetType not in _ERROR_TYPES

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
                        if r.packetType in _ERROR_TYPES:
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
                    if r.packetType in _ERROR_TYPES:
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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        if r.packetType in _ERROR_TYPES:
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
        if r.packetType == OW_ERROR or len(r.data) != 8:
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
        if r.packetType in _ERROR_TYPES:
            logger.error(
                "enable_camera_power(0x%02x) rejected by firmware: packetType=%s",
                camera_mask, r.packetType,
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
        if r.packetType in _ERROR_TYPES:
            logger.error(
                "disable_camera_power(0x%02x) rejected by firmware: packetType=%s",
                camera_mask, r.packetType,
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
        if r.packetType in _ERROR_TYPES:
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
        if r.packetType in _ERROR_TYPES:
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
        return r.packetType not in _ERROR_TYPES

    def disable_aggregator_fsin(self) -> bool:
        """Disable the internal frame-sync signal generator."""
        if self.demo_mode:
            return True
        r = self._send(packetType=OW_CAMERA, command=OW_CAMERA_FSIN, reserved=0)
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

    def disable_camera_fsin_ext(self) -> bool:
        """Disable external frame-sync input."""
        if self.demo_mode:
            return True
        r = self._send(
            packetType=OW_CAMERA, command=OW_CAMERA_FSIN_EXTERNAL, reserved=0
        )
        return r.packetType not in _ERROR_TYPES

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
        return r.packetType not in _ERROR_TYPES

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

        version_str = self.get_version()
        try:
            parts = _parse_firmware_version(version_str)
        except (ValueError, TypeError) as e:
            logger.warning(
                "Could not parse firmware version for pedestal selection: %s", e
            )
            return

        pedestal = 64.0 if parts <= (1, 5, 2) else 128.0
        _mp.PEDESTAL_HEIGHT = pedestal
        logger.info(
            "Pedestal height set to %g based on sensor firmware %s",
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

    def log_device_info(self, label: str | None = None) -> None:
        """Log a ``====``-guarded, side-labeled block with everything we need
        to identify this sensor: firmware version, hardware ID, serial number,
        and all 8 camera UIDs.

        ``label`` is the side ("left"/"right") supplied by
        :meth:`omotion.MotionInterface.log_sensor_info`; it tags the block so
        the two sensors are distinguishable in the log. The whole block is
        emitted as a single log record so it stays intact even when both
        sensors log concurrently (camera UIDs that failed to read are shown
        as ``<none>`` rather than dropped, so a dead camera stays visible).
        """
        title = (label or "sensor").upper()
        try:
            fw_version = self.get_version()
            hw_id      = self.get_cached_hardware_id() or self.get_hardware_id()
            serial     = self.read_serial_number() or "unprogrammed"
            uids       = self._cached_camera_uids or {}
            rule = "=" * 60
            lines = [
                rule,
                f"{title} SENSOR",
                f"  firmware = {fw_version}",
                f"  hw_id    = {hw_id}",
                f"  serial   = {serial}",
                "  camera UIDs:",
            ]
            for cam in range(8):
                lines.append(f"    cam{cam} = {uids.get(cam) or '<none>'}")
            lines.append(rule)
            logger.info("\n".join(lines))
        except Exception as e:
            logger.warning("%s sensor: failed to read device info: %s", title, e)

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
