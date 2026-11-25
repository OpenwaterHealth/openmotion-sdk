"""SEEDLESS register control: seed off, long unseeded TA pulse, long exposure.

Engineering test only. The TA is driven with the seed laser OFF (the test
*intends* to make the TA self-lase), with a pulse longer than the normal
500 us and a camera exposure of 2295 us that has room for it. Requested by
bahartl; laser-engineer approved 2026-07-16 (relayed by Ethan). Design and
bench findings: SDK issue #146 and the ``claude/github-issue-link-3ca73e``
branch (spec ``docs/superpowers/specs/2026-07-15-seedless-frames-design.md``),
where this controller was bench-validated on 2026-07-17 (N=25, mask 0x66).

That branch wires the controller into ``ScanWorkflow`` as a "first N frames
of a scan" mode. This module carries only the register lifecycle, so it can
be driven around whole scans (the seedless C_min procedure,
``omotion.calibration.seedless_cmin``) as well. Two changes from the branch:
the TA pulse width is a parameter (``set_ta_pulse_width_us`` between scans,
never while the trigger runs) instead of a fixed 2 ms, and every console
write is verified by read-back.

Everything is host-side register writes:
- console FPGAs over UART I2C (seed gains, TA pulse width, safety limits)
- camera exposure over USB I2C passthrough

The safety configuration it relaxes for the seedless window: the EE/OPT
pulse-width UPPER limits (to clear the long pulse) and the EE/OPT rate LOWER
limits. With the seed off the safety monitor sees only faint sub-threshold
pulses and trips rate_lower_limit_fail -> TA_shutdown (bench 2026-07-16: the
scan died at ~12 frames). The sdk#146 branch set the rate limit to 0
(disabled); here it is lowered to ``RATE_LL_SEEDLESS_US`` (19 ms) instead,
which still clears the seedless dark-frame spacing but keeps a guard on the
pulse rate (Brad's review, 2026-11-24). Not yet run on hardware at 19 ms:
if the faint pulses still trip it, the first seedless burst gets too few
frames and the run stops and restores. Both relaxations are
Ethan-authorized for this engineering test and restored on every exit
path.

**Console firmware that owns the safety state** (boot-info v2, PTR-2026-1-5)
refuses the widened limits on its Release/Debug images (WI-00015 envelope:
pulse-width upper limit at most 660 us); :func:`seedless_unsupported_reason`
says so before anything is written. 1.8.2-factory refused them too (bench
2026-11-24); a Factory build whose pass-through skips the envelope
(console-fw feature/factory-passthrough-unbounded) accepts them. On a
Factory image that still refuses, :meth:`SeedlessController.apply` fails on
the read-back and rolls back before the seed is touched.

SAFETY PROPERTIES this module must preserve:
1. The widened pulse-width upper limits AND relaxed rate lower limits exist
   only between apply() and restore(). restore() is idempotent; callers run
   it in a ``finally``.
2. Restore ORDER: TA_PULSE_WIDTH is narrowed FIRST, then (after a settle
   delay covering one in-flight pulse) the pulse-width ULs are re-tightened,
   then the seed comes back on, then exposure. Re-tightening the ULs while
   the TA still fires long pulses latches TA_shutdown in the safety FPGA
   (pulse_upper_limit_fail). If the TA-narrow write itself FAILS, restore
   re-tightens the ULs anyway: closing the widened-safety window is the
   higher priority, and the worst case is a protective TA_shutdown (laser
   off = failsafe), not an unbounded pulse.
3. apply() snapshots live register values first (I2C read-back) so restore
   writes back what was actually there; the bundled baseline is only a
   fallback when a read fails.
4. The rate LLs are re-armed only by the FINAL restore (trigger stopped). The
   seedless trigger override (LaserPulseSkipDelayUsec=2500) shortens every
   dark frame's return interval to 22.5 ms, below the 23.125 ms baseline
   limit, so re-arming while pulses flow latches rate_lower_limit_fail
   (bench 2026-07-17).
5. The TA pulse width can never be set above ``MAX_TA_PULSE_WIDTH_US``,
   which sits inside both the widened safety UL (2.5 ms) and the 2295 us
   exposure window.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

logger = logging.getLogger("openmotion.sdk.seedless")

# Console FPGA I2C locations (omotion/data/fpga_model.py): all mux 1,
# device 0x41, little-endian.
_MUX = 1
_DEV = 0x41
_TA_CH, _TA_PW_REG, _TA_PW_LEN = 4, 0x00, 3
_TA_CUR_REG, _TA_CUR_LEN = 0x06, 2   # TA CURRENT DRV, 0.16 mA per tick
_SEED_CH, _SEED_DDS_REG, _SEED_CW_REG, _SEED_GAIN_LEN = 5, 0x02, 0x04, 2
_EE_CH, _OPT_CH, _UL_REG, _UL_LEN = 6, 7, 0x04, 4
_RATE_LL_REG, _RATE_LL_LEN = 0x08, 4  # EE/OPT RATE lower limit
_DYN_CTRL_REG = 0x22                  # EE/OPT dynamic control; bit0 = clear_fail
_STATUS_REG = 0x24                    # EE/OPT status; bits 2..0 = rate/pulse/power fail

#: TA_PULSE_WIDTH register scale (fpga_model: 0.32 us per tick).
TA_TICK_US = 0.32
#: TA_CURRENT_DRV register scale (fpga_model: 0.16 mA per tick).
TA_CURRENT_TICK_MA = 0.16
#: Ceiling for the TA drive current in seedless mode: the 5000 mA EE drive
#: rating as the safety FPGA holds it (31248 x 0.16 = 4999.68 mA, the
#: laser_params baseline). Brad / Ethan 2026-11-24: "up to 5A".
MAX_TA_CURRENT_MA = 31248 * TA_CURRENT_TICK_MA
#: Floor: a current request below this is a mistake, not a measurement.
MIN_TA_CURRENT_MA = 1000.0
TA_CURRENT_BASELINE = bytes([0x10, 0x7A])                 # 31248 -> 4999.68 mA

# Values. Baselines mirror omotion/data/laser_params.py (locked data); they
# are only restore fallbacks for a failed snapshot read.
TA_PULSE_WIDTH_BASELINE = bytes([0x1B, 0x06, 0x00])   # 1563 * 0.32us = 500 us
SEED_DDS_GAIN_BASELINE  = bytes([0x00, 0x00])
SEED_CW_GAIN_BASELINE   = bytes([0x0E, 0x08])         # 2062 -> ~142 mV
SEED_GAIN_OFF           = bytes([0x00, 0x00])
PULSE_WIDTH_UL_BASELINE = bytes([0x35, 0x0C, 0x00, 0x00])  # 3125 -> 1.000 ms
PULSE_WIDTH_UL_SEEDLESS = bytes([0x85, 0x1E, 0x00, 0x00])  # 7813 -> 2.500 ms
# RATE lower limit (EE/OPT, reg 0x08, 0.32 us ticks). The safety FPGA trips
# rate_lower_limit_fail when a detected pulse arrives with count <
# rate_lower_limit (safety-fpga/src/logic_check.v:179): pulses closer
# together than the limit. NOTE the direction: 0 disables it; a MAX value
# would trip on every real pulse instead.
#
# Seedless scans move the dark-frame pulse to 2500 us into its frame, so the
# shortest gap is dark -> next light: 25000 - 2500 = 22500 us start to start
# (20350 us end to start with a 2150 us pulse). That is under the stock
# 23125 us limit, so the limit is LOWERED for the run, not disabled (Brad's
# review 2026-11-24): 19000 us keeps a guard against a faster trigger (40 Hz
# doubled is 12500 us) with margin to both gaps.
RATE_LL_SEEDLESS_US = 19000.0
RATE_LL_SEEDLESS = round(RATE_LL_SEEDLESS_US / 0.32).to_bytes(4, "little")
RATE_LL_BASELINE = bytes([0xA9, 0x12, 0x01, 0x00])    # 70313 (laser_params EE/OPT_RATE_LL)

# OV2312 exposure via passthrough: byte = us/9. Restore value mirrors the
# sensor-fw config-table default (X02C1B_Sensor_Config.h: 0x3502=0x48,
# 72 rows = 648 us). If sensor-fw changes its default, update this.
EXPOSURE_SEEDLESS_BYTE = 0xFF   # 255 rows = 2295 us (the largest one-byte value)
EXPOSURE_RESTORE_BYTE  = 0x48   # 72 rows = 648 us
EXPOSURE_SEEDLESS_US = EXPOSURE_SEEDLESS_BYTE * 9

#: The bench-validated seedless pulse (2026-07-17).
DEFAULT_TA_PULSE_WIDTH_US = 2000.0
#: Hard ceiling for the TA pulse in seedless mode. The pulse starts
#: LaserPulseDelayUsec (100 us) into the frame and must end inside the
#: 2295 us exposure (100 + 2150 = 2250 us, 45 us margin); it also stays
#: well inside the widened 2.5 ms safety UL.
MAX_TA_PULSE_WIDTH_US = 2150.0
#: Floor: below this the TA one-shot is not meaningfully exercised.
MIN_TA_PULSE_WIDTH_US = 50.0

#: Trigger-config override every seedless scan must use. Dark frames are
#: laser pulses delayed past the exposure window; with the 2295 us exposure
#: the normal 1800 us delay would land inside it, so the pulse moves to
#: 100 + 2500 = 2600 us. A normal pre-scan set_trigger_json merge.
SEEDLESS_TRIGGER_OVERRIDES = {"LaserPulseSkipDelayUsec": 2500}


def ta_ticks_from_us(us: float) -> int:
    """TA_PULSE_WIDTH register ticks for a width in microseconds (rounded)."""
    return int(round(float(us) / TA_TICK_US))


def ta_us_from_ticks(ticks: int) -> float:
    return int(ticks) * TA_TICK_US


def ta_current_ticks_from_ma(ma: float) -> int:
    return int(round(float(ma) / TA_CURRENT_TICK_MA))


def ta_current_ma_from_ticks(ticks: int) -> float:
    return ticks * TA_CURRENT_TICK_MA


def read_ta_current_ma(console: Any) -> Optional[float]:
    """The TA drive current the console's TA FPGA holds now (mA), or None."""
    try:
        data, dlen = console.read_i2c_packet(
            mux_index=_MUX, channel=_TA_CH, device_addr=_DEV,
            reg_addr=_TA_CUR_REG, read_len=_TA_CUR_LEN)
    except Exception:
        logger.exception("seedless: TA_CURRENT_DRV read raised")
        return None
    if data is None or dlen != _TA_CUR_LEN:
        return None
    return ta_current_ma_from_ticks(int.from_bytes(bytes(data[:2]), "little"))


def ta_current_problem(console: Any) -> Optional[str]:
    """Why the nominal (seeded) target would not be at the unit's calibrated
    drive current, or None when it will be. Call after the laser bring-up.

    Bench 2026-11-24: flashing console 1.8.1 empties the user config, and
    the bring-up then left TA_CURRENT_DRV at the 4999.68 mA baseline instead
    of the unit's 3900 mA, so three runs measured a 5 A target with no room
    to step the current."""
    try:
        cfg = console.read_config()
        data = dict(getattr(cfg, "json_data", None) or {})
    except Exception as exc:
        return f"the console config could not be read ({exc})"
    want = data.get("TA_CURRENT_DRV")
    if want is None:
        return ("the console config has no TA_CURRENT_DRV, so the laser runs "
                "at the bundled baseline instead of this unit's calibrated "
                "current; write the unit's config first")
    have = read_ta_current_ma(console)
    if have is None:
        return "the TA drive current could not be read back"
    if abs(have - float(want)) > TA_CURRENT_TICK_MA * 1.5:
        return (f"the TA drive current is {have:.0f} mA but the console config "
                f"says {float(want):.0f} mA")
    return None


def _ta_payload(us: float) -> bytes:
    return ta_ticks_from_us(us).to_bytes(_TA_PW_LEN, "little")


def seedless_unsupported_reason(console: Any) -> Optional[str]:
    """Why this console cannot run seedless mode, or ``None`` if it can.

    Console firmware that owns the laser-safety state (boot-info v2,
    PTR-2026-1-5) range-checks every host write to the EE/OPT limits against
    its WI-00015 safety envelope, on EVERY image: a Factory image lifts the
    firmware's ownership of the registers but not the envelope
    (console-fw ``fpga_register_map.c`` ``passthrough_ownership_problem``
    against ``motion_cfg_effective_max``). On 1.8.2 the envelope caps the
    pulse-width upper limit at 660 us and holds the rate lower limit at
    >= 23125 us (``safety_envelope.h``), so the 2.5 ms limit and the disabled
    rate check seedless mode needs are refused (bench 2026-11-24, 1.8.2 and
    1.8.2-factory). A Factory build whose pass-through skips the envelope
    (console-fw feature/factory-passthrough-unbounded) can run it, so a
    Factory image is let through; an old Factory build refuses the first
    write and the controller rolls back. Release/Debug images are refused
    here. Never writes anything.
    """
    get_boot_info = getattr(console, "get_boot_info", None)
    info = get_boot_info() if callable(get_boot_info) else None
    if info is None or not getattr(info, "firmware_owns_safety_state", False):
        return None
    if getattr(info, "factory_image", None):
        # A Factory build with unbounded pass-through (console-fw
        # feature/factory-passthrough-unbounded) accepts the seedless
        # limits; 1.8.2-factory still refuses the first write, which the
        # controller reads back and rolls back before the seed is touched.
        return None
    return ("this console firmware enforces the WI-00015 laser-safety "
            "envelope on its Release/Debug image: it refuses the 2.5 ms "
            "pulse-width limit seedless mode needs (it caps the pulse-width "
            "limit at 660 us), so the unseeded TA cannot be pulsed long "
            "enough. Run it on the console's Factory image")


class SeedlessController:
    """Owns the SEEDLESS register lifecycle.

    Args:
        console: connected MotionConsole (read_i2c_packet/write_i2c_packet).
        sensors: list of (side_name, MotionSensor, camera_mask) for the
            active sides. Cameras are addressed by mask bit index (0-7).
        ta_pulse_width_us: TA pulse width applied by apply(); bounded to
            [MIN_TA_PULSE_WIDTH_US, MAX_TA_PULSE_WIDTH_US].
        settle_s: delay between narrowing TA_PULSE_WIDTH and re-tightening
            the safety ULs on restore, and between camera register writes
            (default 0.05 s = 2 frame periods). Tests pass 0.
    """

    def __init__(self, *, console: Any, sensors: list,
                 ta_pulse_width_us: float = DEFAULT_TA_PULSE_WIDTH_US,
                 settle_s: float = 0.05):
        self._console = console
        self._sensors = [(s, sen, m) for (s, sen, m) in sensors
                         if sen is not None and m]
        self._check_width(ta_pulse_width_us)
        self._ta_us = float(ta_pulse_width_us)
        self._settle_s = float(settle_s)
        self._snapshot: dict[str, bytes] = {}
        self._current: Optional[bytes] = None
        self._applied = False
        self._restored = threading.Event()
        self._lock = threading.Lock()

    # -- properties --

    @property
    def applied(self) -> bool:
        return self._applied and not self._restored.is_set()

    @property
    def ta_pulse_width_us(self) -> float:
        """The TA width as written to the register (tick-rounded)."""
        return ta_us_from_ticks(ta_ticks_from_us(self._ta_us))

    @property
    def ta_current_ma(self) -> Optional[float]:
        """The TA drive current now in the register (as last written, or the
        snapshot before any change); ``None`` before apply()."""
        raw = self._current or self._snapshot.get("TA_CURRENT_DRV")
        if raw is None:
            return None
        return ta_current_ma_from_ticks(int.from_bytes(raw, "little"))

    @property
    def snapshot(self) -> dict[str, bytes]:
        """Register values captured by apply() (what restore writes back)."""
        return dict(self._snapshot)

    @staticmethod
    def _check_width(us: float) -> None:
        us = float(us)
        if not (MIN_TA_PULSE_WIDTH_US <= us <= MAX_TA_PULSE_WIDTH_US):
            raise ValueError(
                f"seedless TA pulse width {us:g} us outside "
                f"{MIN_TA_PULSE_WIDTH_US:g}..{MAX_TA_PULSE_WIDTH_US:g} us")

    # -- I2C helpers --

    def _write(self, name: str, ch: int, reg: int, data: bytes) -> bool:
        """Write, retrying once (spec error-handling table)."""
        for attempt in (1, 2):
            try:
                ok = self._console.write_i2c_packet(
                    mux_index=_MUX, channel=ch, device_addr=_DEV,
                    reg_addr=reg, data=bytearray(data))
            except Exception:
                logger.exception("seedless: write %s raised", name)
                ok = False
            if ok:
                return True
            logger.warning("seedless: write %s failed (attempt %d)", name, attempt)
        return False

    def _read(self, name: str, ch: int, reg: int, length: int) -> Optional[bytes]:
        try:
            data, dlen = self._console.read_i2c_packet(
                mux_index=_MUX, channel=ch, device_addr=_DEV,
                reg_addr=reg, read_len=length)
        except Exception:
            logger.exception("seedless: read %s raised", name)
            return None
        if data is None or dlen != length:
            return None
        return bytes(data[:length])

    def _write_verified(self, name: str, ch: int, reg: int, data: bytes) -> bool:
        """Write and confirm by read-back. A console that silently clamps or
        drops a write (firmware that bounds the operating point) must not
        leave the procedure measuring a setting it did not get."""
        if not self._write(name, ch, reg, data):
            return False
        back = self._read(name, ch, reg, len(data))
        if back != bytes(data):
            logger.error("seedless: %s read back %s after writing %s", name,
                         back.hex() if back is not None else "nothing",
                         bytes(data).hex())
            return False
        return True

    def _restore_reg(self, name: str, ch: int, reg: int, value: bytes) -> bool:
        """Write ``value`` back unless the register already holds it. A
        register whose seedless write was refused (or never reached) is
        unchanged, and firmware that refuses the write would otherwise turn
        a clean rollback into a reported failure."""
        if self._read(name, ch, reg, len(value)) == bytes(value):
            return True
        return self._write(name, ch, reg, value)

    def _set_exposure(self, byte: int) -> bool:
        from omotion.i2c_packet import I2C_Packet
        ok = True
        for side, sensor, mask in self._sensors:
            for cam in range(8):
                if not (mask >> cam) & 1:
                    continue
                try:
                    sensor.switch_camera(cam)
                    if not sensor.camera_i2c_write(I2C_Packet(
                            device_address=0x36, register_address=0x3501,
                            data=0x00)):
                        ok = False
                    if self._settle_s:
                        time.sleep(self._settle_s)
                    if not sensor.camera_i2c_write(I2C_Packet(
                            device_address=0x36, register_address=0x3502,
                            data=byte)):
                        ok = False
                    if self._settle_s:
                        time.sleep(self._settle_s)
                except Exception:
                    logger.exception(
                        "seedless: exposure write failed (%s cam %d)", side, cam)
                    ok = False
        return ok

    def read_safety_status(self) -> dict[str, Optional[int]]:
        """EE/OPT STATUS bytes (bits 2..0 = rate/pulse/power fail latched);
        ``None`` for a failed read."""
        out: dict[str, Optional[int]] = {}
        for name, ch in (("EE", _EE_CH), ("OPT", _OPT_CH)):
            st = self._read(f"{name}_STATUS", ch, _STATUS_REG, 1)
            out[name] = None if st is None else st[0]
        return out

    def _clear_safety_faults(self) -> bool:
        """Clear any LATCHED EE/OPT safety fault; verify by STATUS read-back.

        A latched fault (e.g. rate_lower_limit_fail from a prior seedless
        scan's terminal dark frame) keeps TA_shutdown asserted, which starves
        the next scan to 0 frames (bench 2026-07-16/17). Clear = pulse
        dynamic_control[0]. The EE FPGA needs a LONG assert: a 50 ms pulse
        cleared it only intermittently on the bench; 300 ms was reliable.
        Retries up to 3x.
        """
        assert_s = max(self._settle_s * 6, 0.3) if self._settle_s else 0.0
        for attempt in range(1, 4):
            try:
                for ch in (_EE_CH, _OPT_CH):
                    self._console.write_i2c_packet(
                        mux_index=_MUX, channel=ch, device_addr=_DEV,
                        reg_addr=_DYN_CTRL_REG, data=bytearray([0x01, 0x00]))
                if assert_s:
                    time.sleep(assert_s)
                for ch in (_EE_CH, _OPT_CH):
                    self._console.write_i2c_packet(
                        mux_index=_MUX, channel=ch, device_addr=_DEV,
                        reg_addr=_DYN_CTRL_REG, data=bytearray([0x00, 0x00]))
                if assert_s:
                    time.sleep(assert_s / 3)
            except Exception:
                logger.exception("seedless: clear_safety_faults raised")
                return False
            status = self.read_safety_status()
            still = [f"{n}=0x{v:02X}" for n, v in status.items()
                     if v is not None and (v & 0x07)]
            if not still:
                if attempt > 1:
                    logger.info("seedless: safety faults cleared on attempt %d",
                                attempt)
                return True
            logger.warning("seedless: safety fault still latched after clear "
                           "attempt %d: %s", attempt, ", ".join(still))
        logger.error("seedless: could not clear latched safety fault -- a scan "
                     "would get 0 frames (TA_shutdown held)")
        return False

    # -- lifecycle --

    def _apply_sequence(self) -> list[tuple[str, int, int, int, bytes, bytes]]:
        """(name, channel, reg, len, seedless value, fallback baseline) in
        APPLY order: safety limits widen first (rate LL before the seed goes
        off, else the dim seed-off output trips rate_lower_limit_fail), THEN
        the seed is turned off, TA width raised last."""
        return [
            ("EE_PULSE_WIDTH_UL",  _EE_CH,   _UL_REG,       _UL_LEN,        PULSE_WIDTH_UL_SEEDLESS, PULSE_WIDTH_UL_BASELINE),
            ("OPT_PULSE_WIDTH_UL", _OPT_CH,  _UL_REG,       _UL_LEN,        PULSE_WIDTH_UL_SEEDLESS, PULSE_WIDTH_UL_BASELINE),
            ("EE_RATE_LL",         _EE_CH,   _RATE_LL_REG,  _RATE_LL_LEN,   RATE_LL_SEEDLESS,        RATE_LL_BASELINE),
            ("OPT_RATE_LL",        _OPT_CH,  _RATE_LL_REG,  _RATE_LL_LEN,   RATE_LL_SEEDLESS,        RATE_LL_BASELINE),
            ("SEED_DDS_GAIN",      _SEED_CH, _SEED_DDS_REG, _SEED_GAIN_LEN, SEED_GAIN_OFF,           SEED_DDS_GAIN_BASELINE),
            ("SEED_CW_GAIN",       _SEED_CH, _SEED_CW_REG,  _SEED_GAIN_LEN, SEED_GAIN_OFF,           SEED_CW_GAIN_BASELINE),
            ("TA_PULSE_WIDTH",     _TA_CH,   _TA_PW_REG,    _TA_PW_LEN,     _ta_payload(self._ta_us), TA_PULSE_WIDTH_BASELINE),
        ]

    def apply(self) -> bool:
        """Snapshot live values, then write and verify the seedless
        configuration. Call with the trigger STOPPED.

        Returns False on any failure -- the caller must not start the
        trigger, and must call restore() (any partial apply is rolled back
        by it).
        """
        with self._lock:
            if self._restored.is_set():
                raise RuntimeError("SeedlessController is single-use: "
                                   "already restored")
            # A fault that will not clear means every scan gets 0 frames.
            if not self._clear_safety_faults():
                return False
            seq = self._apply_sequence()
            for name, ch, reg, length, _, baseline in seq:
                live = self._read(name, ch, reg, length)
                if live is None:
                    logger.warning(
                        "seedless: snapshot read of %s failed; restore will "
                        "use the bundled baseline", name)
                    live = baseline
                self._snapshot[name] = live
            # The drive current is not changed by apply(); snapshot it so a
            # later set_ta_current_ma() is restored too.
            cur = self._read("TA_CURRENT_DRV", _TA_CH, _TA_CUR_REG, _TA_CUR_LEN)
            if cur is None:
                logger.warning("seedless: snapshot read of TA_CURRENT_DRV "
                               "failed; restore will use the bundled baseline")
                cur = TA_CURRENT_BASELINE
            self._snapshot["TA_CURRENT_DRV"] = cur
            self._applied = True   # before writes: any partial apply must restore

            logger.info("seedless: applying: seed OFF, TA %.2f us, safety ULs "
                        "widened, rate LLs %.0f us, exposure %d us",
                        self.ta_pulse_width_us, RATE_LL_SEEDLESS_US,
                        EXPOSURE_SEEDLESS_US)
            for name, ch, reg, _, seedless_val, _ in seq:
                if not self._write_verified(name, ch, reg, seedless_val):
                    logger.error("seedless: apply failed at %s -- aborting", name)
                    return False
            if not self._set_exposure(EXPOSURE_SEEDLESS_BYTE):
                logger.error("seedless: exposure apply failed -- aborting")
                return False
            return True

    def set_ta_pulse_width_us(self, us: float) -> bool:
        """Change the TA pulse width while seedless is applied.

        Call ONLY with the trigger stopped (between scans). Bounded by
        ``MAX_TA_PULSE_WIDTH_US``; verified by read-back. Returns False on a
        failed write -- the caller must stop and restore().
        """
        self._check_width(us)
        with self._lock:
            if not self.applied:
                raise RuntimeError("set_ta_pulse_width_us before apply() "
                                   "or after restore()")
            self._ta_us = float(us)
            ok = self._write_verified("TA_PULSE_WIDTH", _TA_CH, _TA_PW_REG,
                                      _ta_payload(self._ta_us))
            if ok:
                logger.info("seedless: TA pulse width -> %.2f us",
                            self.ta_pulse_width_us)
            return ok

    def set_ta_current_ma(self, ma: float) -> bool:
        """Change the TA drive current while seedless is applied.

        Call ONLY with the trigger stopped (between scans). Bounded to
        ``MIN_TA_CURRENT_MA``..``MAX_TA_CURRENT_MA``; verified by read-back.
        Returns False on a failed write -- the caller must stop and
        restore().
        """
        ma = float(ma)
        if not (MIN_TA_CURRENT_MA <= ma <= MAX_TA_CURRENT_MA + 1e-6):
            raise ValueError(
                f"seedless TA drive current {ma:g} mA outside "
                f"{MIN_TA_CURRENT_MA:g}..{MAX_TA_CURRENT_MA:g} mA")
        with self._lock:
            if not self.applied:
                raise RuntimeError("set_ta_current_ma before apply() "
                                   "or after restore()")
            payload = ta_current_ticks_from_ma(ma).to_bytes(_TA_CUR_LEN, "little")
            ok = self._write_verified("TA_CURRENT_DRV", _TA_CH, _TA_CUR_REG,
                                      payload)
            if ok:
                self._current = payload
                logger.info("seedless: TA drive current -> %.2f mA",
                            self.ta_current_ma)
            return ok

    def restore(self) -> bool:
        """Write everything back. Idempotent; safe from any thread. Call
        with the trigger STOPPED (it re-arms the rate lower limits).

        ORDER MATTERS -- see the module docstring.
        """
        with self._lock:
            if not self._applied or self._restored.is_set():
                return True
            ok = True
            snap = self._snapshot
            ok &= self._restore_reg("TA_PULSE_WIDTH", _TA_CH, _TA_PW_REG,
                                    snap.get("TA_PULSE_WIDTH", TA_PULSE_WIDTH_BASELINE))
            # The drive current back down with the pulse, before anything else.
            if "TA_CURRENT_DRV" in snap:
                ok &= self._restore_reg("TA_CURRENT_DRV", _TA_CH, _TA_CUR_REG,
                                        snap["TA_CURRENT_DRV"])
            if self._settle_s:
                time.sleep(self._settle_s)   # let any in-flight long pulse clear
            for name, ch, reg, fallback in (
                ("EE_PULSE_WIDTH_UL", _EE_CH, _UL_REG, PULSE_WIDTH_UL_BASELINE),
                ("OPT_PULSE_WIDTH_UL", _OPT_CH, _UL_REG, PULSE_WIDTH_UL_BASELINE),
                ("SEED_DDS_GAIN", _SEED_CH, _SEED_DDS_REG, SEED_DDS_GAIN_BASELINE),
                ("SEED_CW_GAIN", _SEED_CH, _SEED_CW_REG, SEED_CW_GAIN_BASELINE),
                ("EE_RATE_LL", _EE_CH, _RATE_LL_REG, RATE_LL_BASELINE),
                ("OPT_RATE_LL", _OPT_CH, _RATE_LL_REG, RATE_LL_BASELINE),
            ):
                ok &= self._restore_reg(name, ch, reg, snap.get(name, fallback))
            ok &= self._set_exposure(EXPOSURE_RESTORE_BYTE)
            if ok:
                self._restored.set()
                logger.info("seedless: restore complete")
            else:
                logger.error(
                    "seedless: restore INCOMPLETE -- laser/safety registers "
                    "may still hold test values; they self-heal on the next "
                    "apply_laser_power() or console power-cycle")
            return ok

    def verify_restored(self) -> dict[str, tuple[Optional[str], str]]:
        """Read every snapshot register back after restore(). Returns the
        mismatches as ``{name: (read hex or None, expected hex)}``; empty
        means the console holds exactly what apply() found."""
        mismatches: dict[str, tuple[Optional[str], str]] = {}
        regs = [(n, ch, reg, ln) for n, ch, reg, ln, _, _ in self._apply_sequence()]
        regs.append(("TA_CURRENT_DRV", _TA_CH, _TA_CUR_REG, _TA_CUR_LEN))
        for name, ch, reg, length in regs:
            expected = self._snapshot.get(name)
            if expected is None:
                continue
            back = self._read(name, ch, reg, length)
            if back != expected:
                mismatches[name] = (back.hex() if back is not None else None,
                                    expected.hex())
        return mismatches
