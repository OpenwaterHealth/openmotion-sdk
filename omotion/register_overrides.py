"""Runtime register overrides: camera exposure / analog gain, the TA driver's
pulse width, and the safety FPGAs' pulse-width and rate limits.

Each writer changes hardware state that outlives the scan that wrote it:

* camera registers hold until the camera loses power (``OW_CAMERA_SET_CONFIG``
  skips already-configured cameras, so nothing else rewrites them);
* the TA and safety-FPGA registers hold until the next
  :func:`omotion.laser.apply_laser_power` (console connect) or a console
  power-cycle.

So every caller pairs a write with a restore: :data:`FW_DEFAULT_EXPOSURE_ROWS`
/ :data:`FW_DEFAULT_CAMERA_GAINS` for the cameras, ``apply_laser_power`` for
the console (it rewrites the stock values plus the console's stored config
and verifies the safety limits by read-back).

Every console write here is read back and raises :class:`RegisterWriteError`
on a failed write or a mismatch: these registers decide what the laser emits
and when the interlock trips, so a silent partial write is never acceptable.

Used by the laser optimizer (sdk#335). The bloodflow app's engineering
"alternative camera / laser settings" carry their own copies of the camera
and TA writers (app ``motion_config.py``).
"""

from __future__ import annotations

import logging
from typing import Any, Sequence

from omotion.config import CAMERA_GAIN_MAP
from omotion.laser import FpgaMap

logger = logging.getLogger("openmotion.sdk.register_overrides")


class RegisterWriteError(RuntimeError):
    """A console register write failed or read back a different value."""


# --- Camera exposure + analog gain (OX02C1B over the sensor I2C passthrough) --
# Exposure is set in whole rows: Tline = HTS / pixel clock = 432 px / 48 MHz
# = 9.0 µs. 0x3501/0x3502 hold the coarse exposure (rows, MSB/LSB). Analog
# gain is {0x3508[4:0], 0x3509[7:4]} / 16, so 0x3508 = N with 0x3509 = 0 is
# exactly N.0x; digital gain (0x350A-C) stays at the firmware's 1.0x.
CAMERA_I2C_ADDR = 0x36
CAMERA_EXPOSURE_ROW_US = 9.0
CAMERA_EXPOSURE_MIN_ROWS = 11    # 99 µs
CAMERA_EXPOSURE_MAX_ROWS = 244   # 2196 µs
CAMERA_GAIN_CHOICES = (1, 2, 4, 8, 16)

# What the sensor firmware itself programs at camera configure.
FW_DEFAULT_EXPOSURE_ROWS = 72    # 648 µs
FW_DEFAULT_CAMERA_GAINS = tuple(int(g) for g in CAMERA_GAIN_MAP)  # 16/4/2/1/1/2/4/16


def exposure_rows_for_us(exposure_us: float) -> int:
    """Nearest whole exposure row for ``exposure_us``; ``ValueError`` outside
    the camera's valid window."""
    rows = int(round(float(exposure_us) / CAMERA_EXPOSURE_ROW_US))
    if not CAMERA_EXPOSURE_MIN_ROWS <= rows <= CAMERA_EXPOSURE_MAX_ROWS:
        raise ValueError(
            f"exposure {exposure_us} µs is outside the camera's "
            f"{CAMERA_EXPOSURE_MIN_ROWS * CAMERA_EXPOSURE_ROW_US:g}-"
            f"{CAMERA_EXPOSURE_MAX_ROWS * CAMERA_EXPOSURE_ROW_US:g} µs window"
        )
    return rows


def write_camera_settings(
    sensor: Any,
    camera_mask: int,
    *,
    exposure_rows: int,
    gains: Sequence[int],
) -> list[str]:
    """Write exposure + analog gain to every camera in ``camera_mask``.

    ``gains`` is 8 values indexed by camera position. Write with the trigger
    off: the pipeline's warmup-frame discard absorbs the sensor's register
    latch delay when streaming resumes. Returns one string per failed write
    (empty = every camera written); never raises, so one bad camera does not
    stop the rest.
    """
    from omotion.i2c_packet import I2C_Packet

    if not CAMERA_EXPOSURE_MIN_ROWS <= int(exposure_rows) <= CAMERA_EXPOSURE_MAX_ROWS:
        return [f"exposure {exposure_rows} rows out of range"]
    if len(gains) != 8 or any(int(g) not in CAMERA_GAIN_CHOICES for g in gains):
        return [f"gains {list(gains)} are not 8 values from {CAMERA_GAIN_CHOICES}"]

    failures = []
    exp_hi = (int(exposure_rows) >> 8) & 0xFF
    exp_lo = int(exposure_rows) & 0xFF
    for cam_id in range(8):
        if not (camera_mask & (1 << cam_id)):
            continue
        regs = (
            (0x3501, exp_hi),               # coarse exposure MSB (rows)
            (0x3502, exp_lo),               # coarse exposure LSB
            (0x3508, int(gains[cam_id])),   # real gain = code/16, MSB
            (0x3509, 0x00),                 # real gain LSB (fractional)
        )
        try:
            # Mux reads clear the selection, so always re-switch per camera.
            sensor.switch_camera(cam_id)
            for reg, value in regs:
                ok = sensor.camera_i2c_write(
                    I2C_Packet(device_address=CAMERA_I2C_ADDR,
                               register_address=reg, data=value)
                )
                if ok is False:
                    failures.append(f"camera {cam_id + 1} reg 0x{reg:04X}")
        except Exception as e:
            failures.append(f"camera {cam_id + 1}: {e}")
    return failures


# --- Console FPGA registers (I2C 0x41 behind the console's mux) ---------------
# TA driver pulse_width: 24-bit LE, 0.32 µs/tick. The TA edge-detects the
# console's laser trigger and times the drive itself, so this register (not
# the trigger config's LaserPulseWidthUsec) shapes the optical pulse.
# driver_control.v drops the drive at `pulse_count > pulse_width - 55`, so
# the emitted pulse is ~54 ticks (~17.3 µs) shorter than the register, and
# below 55 ticks that 24-bit compare wraps and the drive stays latched ON.
# Hence the floor, re-applied at the write.
TA_PULSE_TICK_US = 0.32
TA_PULSE_TRUNCATION_TICKS = 54
TA_PULSE_MIN_TICKS = 55
TA_PULSE_WIDTH_MIN_US = 20       # 62 ticks: 7 ticks above the wrap
TA_PULSE_WIDTH_MAX_US = 2200     # console trigger limit (TRIGGER_LASER_WIDTH_MAX_US)

# Safety EE / OPT limit registers, both 32-bit LE at 0.32 µs/tick.
SAFETY_TICK_US = 0.32
SAFETY_FPGA_PREFIXES = ("EE", "OPT")


def ta_pulse_ticks(width_us: float) -> int:
    """TA pulse_width register value for a commanded width (µs)."""
    width = float(width_us)
    if not TA_PULSE_WIDTH_MIN_US <= width <= TA_PULSE_WIDTH_MAX_US:
        raise ValueError(
            f"TA pulse width {width_us} µs outside "
            f"{TA_PULSE_WIDTH_MIN_US}-{TA_PULSE_WIDTH_MAX_US} µs"
        )
    return max(TA_PULSE_MIN_TICKS, int(round(width / TA_PULSE_TICK_US)))


def emitted_pulse_us(width_us: float) -> float:
    """Optical pulse length the TA emits for a commanded width (µs): the
    register minus the driver's ~54-tick truncation."""
    return (ta_pulse_ticks(width_us) - TA_PULSE_TRUNCATION_TICKS) * TA_PULSE_TICK_US


def _write_verified(console: Any, fpga_map: FpgaMap, name: str, raw: int) -> None:
    """Write ``raw`` to the register ``name`` and read it back."""
    entry = fpga_map.get_entry_by_friendly_name(name)
    if entry is None:
        raise RegisterWriteError(f"{name}: not in the FPGA register map")
    num_bytes = int(str(entry["data_size"]).rstrip("B")) // 8
    order = "big" if entry.get("isMsbFirst", False) else "little"
    if not 0 <= raw < (1 << (8 * num_bytes)):
        raise RegisterWriteError(f"{name}: value {raw} does not fit {num_bytes} bytes")
    data = raw.to_bytes(num_bytes, order)
    location = dict(
        mux_index=entry["mux_idx"],
        channel=entry["channel"],
        device_addr=entry["i2c_addr"],
        reg_addr=entry["start_address"],
    )
    if not console.write_i2c_packet(data=bytearray(data), **location):
        raise RegisterWriteError(f"{name}: write of {raw} failed")
    read, _ = console.read_i2c_packet(read_len=num_bytes, **location)
    if read is None or bytes(read) != data:
        got = "nothing" if read is None else int.from_bytes(bytes(read), order)
        raise RegisterWriteError(f"{name}: wrote {raw}, read back {got}")


def write_ta_pulse_width(console: Any, width_us: float,
                         *, fpga_map: FpgaMap | None = None) -> int:
    """Program the TA driver's pulse width and verify it; returns the ticks.

    Raise the safety FPGAs' ``PULSE_WIDTH_UL`` above the new width first
    (:func:`write_safety_limits`), or the first pulse latches the interlock.
    """
    ticks = ta_pulse_ticks(width_us)
    _write_verified(console, fpga_map or FpgaMap(), "TA_PULSE_WIDTH", ticks)
    # WARNING on purpose: this changes what the laser emits.
    logger.warning("TA_PULSE_WIDTH WRITTEN: %s µs = %d ticks", width_us, ticks)
    return ticks


def read_safety_faults(console: Any) -> list[str] | None:
    """Latched interlock faults on the Safety EE / OPT FPGAs ([] = clear),
    or None when either STATUS register could not be read.

    A latched fault inhibits the TA until a console power-cycle, so a laser
    scan run on top of one sees no light at all and would read as "not
    enough signal".
    """
    from omotion.ConsoleTelemetry import _SAFETY_FAULT_MASK, _decode_safety_faults

    fpga_map = FpgaMap()
    faults: list[str] = []
    for prefix in SAFETY_FPGA_PREFIXES:
        entry = fpga_map.get_entry_by_friendly_name(f"{prefix}_STATUS")
        if entry is None:
            return None
        data, _ = console.read_i2c_packet(
            mux_index=entry["mux_idx"], channel=entry["channel"],
            device_addr=entry["i2c_addr"], reg_addr=entry["start_address"],
            read_len=1,
        )
        if not data:
            return None
        faults.extend(f"{prefix} {label}" for label in
                      _decode_safety_faults(data[0] & _SAFETY_FAULT_MASK))
    return faults


def write_safety_limits(
    console: Any,
    *,
    pulse_width_ul_us: float,
    rate_ll_us: float,
    fpga_map: FpgaMap | None = None,
) -> dict[str, int]:
    """Set ``PULSE_WIDTH_UL`` and ``RATE_LL`` on both safety FPGAs (EE, OPT)
    and verify each by read-back. Returns {friendlyName: raw ticks}.

    The safety FPGAs trip and LATCH ``TA_shutdown`` (console power-cycle to
    clear) on a measured pulse longer than ``PULSE_WIDTH_UL`` or a pulse-to-
    pulse interval shorter than ``RATE_LL``. Call with the trigger stopped;
    restore with ``apply_laser_power``, which rewrites the stock values and
    the console's stored config.
    """
    fpga_map = fpga_map or FpgaMap()
    written: dict[str, int] = {}
    for prefix in SAFETY_FPGA_PREFIXES:
        for suffix, value_us in (("PULSE_WIDTH_UL", pulse_width_ul_us),
                                 ("RATE_LL", rate_ll_us)):
            name = f"{prefix}_{suffix}"
            raw = int(round(float(value_us) / SAFETY_TICK_US))
            _write_verified(console, fpga_map, name, raw)
            written[name] = raw
    logger.warning(
        "SAFETY LIMITS WRITTEN: PULSE_WIDTH_UL %s µs, RATE_LL %s µs on EE and OPT",
        pulse_width_ul_us, rate_ll_us,
    )
    return written
