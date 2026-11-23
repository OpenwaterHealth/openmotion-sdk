"""Dark-frame skip for long camera exposures (bloodflow-app#747).

A scheduled dark frame moves the laser pulse later instead of dropping it:
it starts at ``LaserPulseDelayUsec + LaserPulseSkipDelayUsec`` (100 + 1800 =
1900 us by default). An exposure that is still open at that point catches the
displaced pulse and contaminates the dark reference. On the bench the onset
was between 1700 and 1800 us of exposure (2026-08-10), and at 2196 us the
darks read 207-1017 DN instead of the laser-off frame's ~128 DN (2026-10-21).

A run whose camera exposure, or pulse delay + pulse width, exceeds
:data:`CLEAN_EXPOSURE_MAX_US` therefore runs a :data:`LONG_DARK_SKIP_US`
skip. That shortens the gap after a dark to 25000 - 2400 = 22600 us at 40 Hz,
under the ``RATE_LL`` floor real consoles hold (stock 22500, 23125 seen in
flash), and the safety FPGAs would latch ``RATE_LOWER_LIMIT_FAIL`` (E-202,
power-cycle to clear) on the first dark. So both safety FPGAs' ``RATE_LL`` is
lowered to :data:`SAFETY_RATE_LL_US` first, each write read back, and put back
to what it held once a run no longer needs it.

Fail-safe direction: if ``RATE_LL`` can't be lowered (a console whose
firmware refuses the write, a readback mismatch), the run keeps the default
skip. Its darks catch the laser, but nothing latches.

The host app only tells the SDK what it set the cameras and the laser pulse
to (:meth:`omotion.ScanWorkflow.ScanWorkflow.set_device_timing`). The skip,
the safety floor and the restore all live here, applied by
``ScanWorkflow.start_scan`` for every scan path (app scans, contact-quality
checks, calibration sub-scans, scripts). Optimize Laser keeps its own
settings and limits: while its scan preparer sets the skip, this stays out.
"""

from __future__ import annotations

import logging
import threading

logger = logging.getLogger("openmotion.sdk.dark_skip")

# What every other scan runs (omotion.config's default trigger config).
DEFAULT_DARK_SKIP_US = 1800
# The displaced pulse starts at 100 + 2400 = 2500 us, after the longest
# (2196 us) exposure ends.
LONG_DARK_SKIP_US = 2400
# Longest exposure whose dark frames stay clean at the default skip (bench).
CLEAN_EXPOSURE_MAX_US = 1700
# Below the 22600 us post-dark gap of a 2400 us skip at 40 Hz. The same value
# Optimize Laser lowers it to (omotion.exposure.actuator), raw 70000.
SAFETY_RATE_LL_US = 22400.0
RATE_LL_REGISTERS = ("EE_RATE_LL", "OPT_RATE_LL")
STATUS_REGISTERS = ("EE_STATUS", "OPT_STATUS")
# LaserPulseDelayUsec when a trigger config doesn't say.
_DEFAULT_PULSE_DELAY_US = 100


def needs_long_skip(exposure_us=None, pulse_width_us=None,
                    pulse_delay_us=_DEFAULT_PULSE_DELAY_US) -> bool:
    """True when the exposure, or the pulse delay + pulse width, reaches past
    :data:`CLEAN_EXPOSURE_MAX_US`. ``None`` means the firmware default
    (648 us exposure, ~500 us pulse), which never does."""
    reach = []
    if exposure_us is not None:
        reach.append(float(exposure_us))
    if pulse_width_us is not None:
        reach.append(float(pulse_delay_us or 0) + float(pulse_width_us))
    return any(r > CLEAN_EXPOSURE_MAX_US for r in reach)


class LongExposureDarkSkip:
    """Owns the long-exposure skip and the ``RATE_LL`` floor it needs.

    ``registers`` is anything with ``read(name)`` and ``write(name, value)``
    returning a readback with ``.actual`` (or None on a refused write), like
    :class:`omotion.calibration.motion_bench.FpgaRegisterIO`. A callable is
    called for it on first use, so a console that connects later is used.
    """

    def __init__(self, registers):
        self._registers_src = registers
        self._lock = threading.Lock()
        self._exposure_us = None
        self._pulse_width_us = None
        # RATE_LL values from before this lowered them, by name, while lowered.
        self._lowered_from: dict[str, float] = {}

    # ---- what the host set ------------------------------------------------

    def set_device_timing(self, *, camera_exposure_us=None,
                          laser_pulse_width_us=None) -> None:
        """Record the camera exposure and TA laser pulse width the host has
        written, ``None`` for the firmware defaults. Applies to every scan
        that starts after it, until called again."""
        with self._lock:
            self._exposure_us = (None if camera_exposure_us is None
                                 else float(camera_exposure_us))
            self._pulse_width_us = (None if laser_pulse_width_us is None
                                    else float(laser_pulse_width_us))

    @property
    def lowered(self) -> bool:
        """RATE_LL is currently lowered by this (and not yet put back)."""
        return bool(self._lowered_from)

    # ---- per scan (trigger off) -------------------------------------------

    def prepare(self, trigger_config: dict | None, *, disable_laser: bool = False,
                pulse_delay_us=None) -> dict | None:
        """Called by ``start_scan`` before the trigger starts. Returns the
        trigger override this scan needs, or None.

        ``trigger_config`` is the request's override so far: when it already
        sets the skip (Optimize Laser, a caller), this changes nothing and
        leaves any lowered floor for a later scan to put back.
        """
        if trigger_config and "LaserPulseSkipDelayUsec" in trigger_config:
            return None
        with self._lock:
            need = (not disable_laser and needs_long_skip(
                self._exposure_us, self._pulse_width_us,
                _DEFAULT_PULSE_DELAY_US if pulse_delay_us is None else pulse_delay_us))
            if not need:
                if self._lowered_from:
                    self._put_back()
                return None
            if not self._lower():
                return None
        logger.warning("LONG DARK SKIP ACTIVE: %s us (exposure %s us, pulse width %s us)",
                       LONG_DARK_SKIP_US, self._exposure_us, self._pulse_width_us)
        return {"LaserPulseSkipDelayUsec": LONG_DARK_SKIP_US}

    # ---- register writes ----------------------------------------------------

    def _io(self):
        src = self._registers_src
        return src() if callable(src) and not hasattr(src, "write") else src

    def _lower(self) -> bool:
        """Lower RATE_LL on both safety FPGAs. On any failure, put back what
        was written and return False (the scan keeps the default skip)."""
        io = self._io()
        try:
            for name in RATE_LL_REGISTERS:
                before = float(io.read(name))
                if before <= SAFETY_RATE_LL_US:
                    continue        # still lowered, or the console's own floor is low enough
                # Above the floor: never lowered, or the console reloaded its
                # limits since (reconnect, apply_laser_power). Either way this
                # is the value to put back.
                self._write(io, name, SAFETY_RATE_LL_US)
                self._lowered_from[name] = before
            latched = self._latched(io)
        except Exception as e:
            logger.error("RATE_LL could not be lowered for the long dark skip (%s); "
                         "this scan keeps the %s us skip, so its darks may catch the laser",
                         e, DEFAULT_DARK_SKIP_US)
            if self._lowered_from:
                self._put_back()
            return False
        if latched:
            logger.error("Safety status reads %s after lowering RATE_LL", ", ".join(latched))
        logger.warning("SAFETY RATE_LL lowered to %g us (volatile) for the long dark skip",
                       SAFETY_RATE_LL_US)
        return True

    def _put_back(self) -> None:
        """Put RATE_LL back to what it held. A failure keeps the remaining
        names, so the next scan tries again."""
        io = self._io()
        for name, value in list(self._lowered_from.items()):
            try:
                current = float(io.read(name))
                if abs(current - SAFETY_RATE_LL_US) > 0.32:
                    # Not what this wrote: the console reloaded its limits
                    # (reconnect, apply_laser_power), so nothing to put back.
                    del self._lowered_from[name]
                    continue
                self._write(io, name, value)
            except Exception as e:
                logger.error("%s could not be put back to %g us (%s); retrying at the next scan",
                             name, value, e)
                continue
            del self._lowered_from[name]
            logger.warning("SAFETY %s put back to %g us", name, value)

    @staticmethod
    def _write(io, name: str, value: float) -> None:
        readback = io.write(name, value)
        if readback is None:
            raise RuntimeError(f"{name} write to {value:g} was not accepted")
        # 0.32 us per register LSB; more than one LSB off is a refusal.
        if abs(float(readback.actual) - float(value)) > 0.32:
            raise RuntimeError(f"{name} reads back {readback.actual:g} after writing {value:g}")

    @staticmethod
    def _latched(io) -> list[str]:
        out = []
        for name in STATUS_REGISTERS:
            try:
                value = int(io.read(name))
            except Exception:
                continue
            if value:
                out.append(f"{name} = 0x{value:02X}")
        return out
