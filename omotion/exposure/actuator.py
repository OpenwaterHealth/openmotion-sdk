"""Device writes for Optimize Laser: camera exposure and gain, the TA laser
driver's pulse width and drive current, and the Safety EE/OPT limits that
have to move with them (bloodflow-app docs/auto_exposure/IMPLEMENTATION_PLAN.md
section 5).

Every write happens with the trigger off, between scans. The order keeps the
safety interlock from latching (E-202, console power-cycle to clear):
:meth:`LaserCameraActuator.loosen_limits` raises ``PULSE_WIDTH_UL`` and
``DRIVE_CL`` and lowers ``RATE_LL`` before :meth:`LaserCameraActuator.apply`
widens the pulse, raises the current or a scan runs the 2400 us dark skip.
:meth:`LaserCameraActuator.restore` puts everything back through
``apply_laser_power`` (laser_params baseline plus the console's flash
overrides, safety limits read back) and the sensor firmware's camera defaults.

Only for console firmware without the 1.8.2-rc.3 safety envelope:
:meth:`LaserCameraActuator.preflight` refuses anything else before the first
write (:mod:`.firmware`). Nothing here writes flash.
"""

from __future__ import annotations

import contextlib
import logging
import math
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping

from .firmware import ExtendedLimitsRefused, require_extended_limits
from .settings import (
    ABSOLUTE_MAX_CURRENT_MA,
    ABSOLUTE_MAX_PULSE_WIDTH_US,
    HardwareLimits,
    OptimizeLaserSettings,
)

logger = logging.getLogger("openmotion.sdk.exposure")

Camera = tuple[int, int]    # (side: 0 left / 1 right, position 0..7)

# Loosened safety limits while optimized settings are active.
SAFETY_PULSE_WIDTH_UL_US = ABSOLUTE_MAX_PULSE_WIDTH_US   # above the longest pulse
# The 2400 us dark skip leaves 25000 - 2400 = 22600 us from the displaced dark
# pulse to the next one at 40 Hz; real consoles hold 23125 (stock 22500).
SAFETY_RATE_LL_US = 22400.0                              # raw 70000
# What the WI-15 laser calibration swept with; above any 5000 mA peak.
SAFETY_DRIVE_CL = 9999.0                                 # raw 5376

# The TA driver compare underflows below 55 ticks (17.6 us) and leaves the
# drive latched on (driver_control.v); never write a width that short.
TA_PULSE_WIDTH_FLOOR_US = 20.0

CAMERA_I2C_ADDR = 0x36
CAMERA_EXPOSURE_ROW_US = 9.0
CAMERA_EXPOSURE_ROWS = (11, 244)                         # 99 .. 2196 us
GAIN_CODE_RANGE = (16, 256)                              # 1x .. 16x
# What the sensor firmware sets at power-on (X02C1B_configure_sensor).
FW_DEFAULT_EXPOSURE_ROWS = 72                            # 648 us
FW_DEFAULT_GAIN_CODES = (256, 64, 32, 16, 16, 32, 64, 256)

_SAFETY_REGISTERS = (
    "EE_PULSE_WIDTH_LL", "OPT_PULSE_WIDTH_LL",
    "EE_PULSE_WIDTH_UL", "OPT_PULSE_WIDTH_UL",
    "EE_RATE_LL", "OPT_RATE_LL",
    "EE_DRIVE_CL", "OPT_DRIVE_CL",
)
# Safety FPGA status: rate / pulse-width / peak-current fail bits, latched.
_STATUS_REGISTERS = ("EE_STATUS", "OPT_STATUS")


@dataclass(frozen=True)
class LaserBaseline:
    """What the console held when the run started (all read-only)."""

    firmware: str
    ta_pulse_width_us: float
    ta_current_ma: float
    ta_current_limit_ma: float
    registers: Mapping[str, float]      # the safety limits and status, by name


def camera_register_writes(exposure_rows: int, gain_code: int) -> tuple[tuple[int, int], ...]:
    """(register, value) pairs for one camera's exposure and analog gain.

    Exposure is in rows (0x3501 MSB, 0x3502 LSB). The gain code is gain x 16:
    0x3508[4:0] holds its integer part, 0x3509[7:4] the sixteenths.
    """
    lo, hi = CAMERA_EXPOSURE_ROWS
    if not lo <= exposure_rows <= hi:
        raise ValueError(f"exposure {exposure_rows} rows outside {lo}..{hi}")
    lo, hi = GAIN_CODE_RANGE
    if not lo <= gain_code <= hi:
        raise ValueError(f"gain code {gain_code} outside {lo}..{hi}")
    return (
        (0x3501, (exposure_rows >> 8) & 0xFF),
        (0x3502, exposure_rows & 0xFF),
        (0x3508, (gain_code >> 4) & 0x1F),
        (0x3509, (gain_code & 0x0F) << 4),
    )


def exposure_rows(exposure_us: float) -> int:
    return int(round(float(exposure_us) / CAMERA_EXPOSURE_ROW_US))


class LaserCameraActuator:
    """Applies Optimize Laser settings to a console and its sensor modules.

    ``sensors`` maps side (0 left, 1 right) to its ``MotionSensor``.
    ``registers`` defaults to ``FpgaRegisterIO(console)``; ``restore_laser``
    to :func:`omotion.laser.apply_laser_power`. ``lock`` (anything with
    ``lock()`` / ``unlock()``, e.g. the app's console mutex) is held around
    each write sequence.
    """

    def __init__(self, console, sensors: Mapping[int, object], *, registers=None,
                 restore_laser: Callable | None = None, lock=None):
        if registers is None:
            from omotion.calibration.motion_bench import FpgaRegisterIO
            registers = FpgaRegisterIO(console)
        if restore_laser is None:
            from omotion.laser import apply_laser_power as restore_laser
        from omotion.laser import FpgaMap
        self._map = FpgaMap()
        self._console = console
        self._sensors = dict(sensors)
        self._registers = registers
        self._restore_laser = restore_laser
        self._lock = lock

    # ---- run start --------------------------------------------------------

    def preflight(self, settings: OptimizeLaserSettings,
                  limits: HardwareLimits) -> LaserBaseline:
        """Check the console can take the run, and read its starting state.

        Reads only. Raises :class:`~.firmware.ExtendedLimitsRefused` for
        envelope firmware and ``RuntimeError`` for anything else that rules
        the run out: a latched safety fault, unverified laser limits, a
        pulse-width lower limit the shortest pulse would trip, or a TA
        current limit below the current upper bound.
        """
        console = self._console
        if not console.is_connected():
            raise RuntimeError("console not connected")
        firmware = str(console.get_version())
        require_extended_limits(firmware)
        owns = getattr(console, "firmware_owns_safety_state", None)
        if callable(owns) and owns():
            raise ExtendedLimitsRefused(
                f"console firmware {firmware} reports that it owns the laser-safety limits, "
                "so it refuses the extended limits Optimize Laser needs")
        limits_error = getattr(console, "laser_limits_error", None)
        if limits_error:
            raise RuntimeError(f"laser power not verified: {limits_error}")

        with self._locked():
            regs = {name: self._registers.read(name)
                    for name in _SAFETY_REGISTERS + _STATUS_REGISTERS}
            ta_pw = self._registers.read("TA_PULSE_WIDTH")
            ta_current = self._registers.read("TA_CURRENT_DRV")
            ta_limit = self._registers.read("TA_CURRENT_LIMIT")

        latched = [f"{n} = 0x{int(regs[n]):02X}" for n in _STATUS_REGISTERS if int(regs[n])]
        if latched:
            raise RuntimeError("safety fault latched (" + ", ".join(latched)
                               + "); power-cycle the console")
        shortest = settings.pulse_width_bounds.low - limits.pulse_truncation_us
        for name in ("EE_PULSE_WIDTH_LL", "OPT_PULSE_WIDTH_LL"):
            if regs[name] > 0 and regs[name] >= shortest:
                raise RuntimeError(
                    f"{name} is {regs[name]:.1f} us: the {settings.pulse_width_bounds.low:g} us "
                    "pulse-width lower bound would trip it")
        if ta_limit < settings.current_bounds.high:
            raise RuntimeError(
                f"TA_CURRENT_LIMIT is {ta_limit:.0f} mA, below the "
                f"{settings.current_bounds.high:g} mA current upper bound: the driver would "
                "clamp the current")

        baseline = LaserBaseline(firmware, ta_pw, ta_current, ta_limit, regs)
        logger.info("Optimize Laser preflight: firmware %s, TA %.1f us / %.1f mA (limit %.0f), "
                    "%s", firmware, ta_pw, ta_current, ta_limit,
                    ", ".join(f"{n} {v:g}" for n, v in regs.items()))
        return baseline

    def loosen_limits(self, baseline: LaserBaseline, current_high_ma: float) -> list[str]:
        """Raise ``PULSE_WIDTH_UL``, lower ``RATE_LL`` and, when the current
        upper bound is above the console's current, raise ``DRIVE_CL``, on
        both safety FPGAs. Never tightens a limit. Each write is read back;
        a failure raises ``RuntimeError`` (the caller restores). Returns the
        registers written."""
        plan = []
        for side in ("EE", "OPT"):
            ul = f"{side}_PULSE_WIDTH_UL"
            if baseline.registers[ul] < SAFETY_PULSE_WIDTH_UL_US:
                plan.append((ul, SAFETY_PULSE_WIDTH_UL_US))
            rate = f"{side}_RATE_LL"
            if baseline.registers[rate] > SAFETY_RATE_LL_US:
                plan.append((rate, SAFETY_RATE_LL_US))
            cl = f"{side}_DRIVE_CL"
            if (current_high_ma > baseline.ta_current_ma + 0.5
                    and baseline.registers[cl] < SAFETY_DRIVE_CL):
                plan.append((cl, SAFETY_DRIVE_CL))
        with self._locked():
            for name, value in plan:
                self._write(name, value)
        if plan:
            logger.warning("Optimize Laser: safety limits loosened (volatile): %s",
                           ", ".join(f"{n} -> {v:g}" for n, v in plan))
        return [name for name, _ in plan]

    # ---- each iteration ---------------------------------------------------

    def apply(self, gain_codes: Mapping[Camera, int], pulse_width_us: float,
              current_ma: float, exposure_us: float) -> None:
        """Write the TA pulse width and drive current (read back), then each
        camera's exposure and gain. Call only after :meth:`loosen_limits`."""
        pw, current = float(pulse_width_us), float(current_ma)
        if not (math.isfinite(pw) and TA_PULSE_WIDTH_FLOOR_US <= pw <= ABSOLUTE_MAX_PULSE_WIDTH_US):
            raise ValueError(f"pulse width {pulse_width_us!r} us outside "
                             f"{TA_PULSE_WIDTH_FLOOR_US:g}..{ABSOLUTE_MAX_PULSE_WIDTH_US:g}")
        if not (math.isfinite(current) and 0 < current <= ABSOLUTE_MAX_CURRENT_MA):
            raise ValueError(
                f"drive current {current_ma!r} mA outside 0..{ABSOLUTE_MAX_CURRENT_MA:g}")
        rows = exposure_rows(exposure_us)
        writes = {cam: camera_register_writes(rows, int(code)) for cam, code in gain_codes.items()}
        with self._locked():
            self._write("TA_PULSE_WIDTH", pw)
            self._write("TA_CURRENT_DRV", current)
            self._write_cameras(writes)

    # ---- restore ----------------------------------------------------------

    def restore(self, cameras: Iterable[Camera]) -> list[str]:
        """Put the laser driver, safety limits and the cameras back to their
        defaults. Returns what could not be restored (empty = all of it).
        Never raises."""
        problems = []
        try:
            if not self._restore_laser(self._console, lock=self._lock):
                problems.append("laser power restore failed: "
                                + str(getattr(self._console, "laser_limits_error", "") or "?"))
        except Exception as e:
            problems.append(f"laser power restore raised: {e}")
        rows = FW_DEFAULT_EXPOSURE_ROWS
        writes = {cam: camera_register_writes(rows, FW_DEFAULT_GAIN_CODES[cam[1]])
                  for cam in cameras}
        try:
            with self._locked():
                self._write_cameras(writes)
        except Exception as e:
            problems.append(str(e))
        if problems:
            logger.error("Optimize Laser restore incomplete: %s", "; ".join(problems))
        else:
            logger.info("Optimize Laser: laser, safety limits and %d camera(s) restored",
                        len(writes))
        return problems

    # ---- helpers ----------------------------------------------------------

    def _write(self, name: str, value: float) -> None:
        readback = self._registers.write(name, value)
        if readback is None:
            raise RuntimeError(f"{name} write to {value:g} was not accepted")
        # One register LSB of rounding is expected; anything more is a refusal.
        scale = abs(float(self._map.get_entry_by_friendly_name(name).get("scale") or 1.0))
        if abs(readback.actual - value) > scale:
            raise RuntimeError(f"{name} reads back {readback.actual:g} after writing {value:g}")

    def _write_cameras(self, writes: Mapping[Camera, tuple[tuple[int, int], ...]]) -> None:
        from omotion.i2c_packet import I2C_Packet

        failures = []
        for (side, pos), regs in sorted(writes.items()):
            sensor = self._sensors.get(side)
            label = f"{('left', 'right')[side]} camera {pos + 1}"
            if sensor is None:
                failures.append(f"{label}: no sensor")
                continue
            try:
                # Mux reads clear the selection, so switch for every camera.
                sensor.switch_camera(pos)
                for reg, value in regs:
                    ok = sensor.camera_i2c_write(I2C_Packet(
                        device_address=CAMERA_I2C_ADDR, register_address=reg, data=value))
                    if ok is False:
                        failures.append(f"{label} reg 0x{reg:04X}")
            except Exception as e:
                failures.append(f"{label}: {e}")
        if failures:
            raise RuntimeError("camera writes failed: " + ", ".join(failures))

    @contextlib.contextmanager
    def _locked(self):
        if self._lock is None:
            yield
            return
        self._lock.lock()
        try:
            yield
        finally:
            self._lock.unlock()
