"""Laser-power configuration for the Open-Motion console.

Sets the seed/TA/safety laser-driver registers over I2C so the laser actually
emits when the trigger fires. This is a *cold-start prerequisite*: after a
power-cycle the driver registers are cleared, so the laser pulses produce no
light until these are written. The bloodflow app does this on its scan path
(its "Issue #108" guard); this module is the SDK-owned equivalent so any SDK
consumer (scripts, headless tools) can do it without the app.

The register values are Python modules under ``omotion.data`` (they were
JSON data files until openmotion-sdk#278 compiled them into the package so a
shipped application carries no editable data file):

* :data:`omotion.data.laser_params.LASER_PARAMS` — list of
  ``{"friendlyName", "dataToSend"}`` driver register payloads (the locked
  baseline).
* :data:`omotion.data.laser_params_fault.LASER_PARAMS_FAULT` — the same set
  with register(s) deliberately faulted, for testing the safety interlock.
* :data:`omotion.data.fpga_model.FPGA_MODEL` — maps each ``friendlyName`` to
  its I2C location (mux/channel/device addr/register offset/size).

This is laser-sensitive: editing the bundled values risks wrong pulse widths
or tripping the safety interlock. Treat them as locked baseline data.
"""

from __future__ import annotations

import copy
import logging
from typing import Any, Optional

from omotion.data.fpga_model import FPGA_MODEL
from omotion.data.laser_params import LASER_PARAMS
from omotion.data.laser_params_fault import LASER_PARAMS_FAULT

logger = logging.getLogger("openmotion.sdk.laser")


class FpgaMap:
    """Maps a laser-driver ``friendlyName`` to its I2C location.

    Backed by the bundled :data:`omotion.data.fpga_model.FPGA_MODEL`. This is
    the minimal lookup the laser-power write needs — it deliberately omits the
    bloodflow app's QML scale-override machinery and the legacy
    ``FpgaModel.js`` fallback.
    """

    def __init__(self, model: Optional[list] = None) -> None:
        # Deep copy so no caller can mutate the compiled-in baseline through
        # the map (each instance used to parse its own fresh copy from disk).
        self._model = model if model is not None else copy.deepcopy(FPGA_MODEL)

    def get_entry_by_friendly_name(self, friendly_name: str) -> Optional[dict]:
        """Return the I2C location + format for ``friendly_name`` or None.

        Keys: label, mux_idx, channel, i2c_addr, isMsbFirst, start_address,
        data_size, scale (scale may be None).
        """
        for fpga in self._model:
            for fn in fpga.get("functions", []):
                if fn.get("friendlyName") == friendly_name or fn.get("name") == friendly_name:
                    return {
                        "label": fpga.get("label"),
                        "mux_idx": fpga.get("mux_idx"),
                        "channel": fpga.get("channel"),
                        "i2c_addr": fpga.get("i2c_addr"),
                        "isMsbFirst": fpga.get("isMsbFirst", False),
                        "start_address": fn.get("start_address"),
                        "data_size": fn.get("data_size"),
                        "scale": fn.get("scale"),
                    }
        return None


def load_laser_params(force_fault: bool = False) -> list:
    """Load the bundled laser-driver register payloads.

    Returns a fresh list of ``{"friendlyName", "dataToSend"}`` dicts — a deep
    copy, so callers may edit their copy (the app's alt-laser overrides do)
    without touching the compiled-in baseline. ``force_fault`` selects
    :data:`~omotion.data.laser_params_fault.LASER_PARAMS_FAULT` — a set
    engineered to trip the laser-safety interlock for testing the safety path.
    """
    if force_fault:
        params, source = LASER_PARAMS_FAULT, "omotion.data.laser_params_fault"
    else:
        params, source = LASER_PARAMS, "omotion.data.laser_params"
    logger.info("Loaded %d laser parameter sets from %s", len(params), source)
    return copy.deepcopy(params)


def _fault_diff_names() -> set:
    """friendlyNames whose value the fault set deliberately changes.

    Computed by diffing the two bundled sets so a future fault vector
    (a different register, or several) is picked up automatically.
    """
    normal = {e["friendlyName"]: e["dataToSend"] for e in load_laser_params()}
    fault = {
        e["friendlyName"]: e["dataToSend"]
        for e in load_laser_params(force_fault=True)
    }
    return {k for k, v in fault.items() if normal.get(k) != v}


def apply_laser_power(
    console: Any,
    *,
    laser_params: Optional[list] = None,
    fpga_map: Optional[FpgaMap] = None,
    force_fault: bool = False,
    lock: Optional[Any] = None,
    trigger_freq_hz: Optional[float] = None,
) -> bool:
    """Write the laser-driver configuration to ``console`` over I2C.

    Reads user overrides from ``console.read_config()`` and applies the
    ``laser_params`` list, honoring per-key overrides and the safety DRIVE CL
    values. Returns True on success, False if any I2C write fails.

    ``force_fault`` additionally exempts the deliberately-faulted registers
    (the fault file's diff vs the baseline) from every user-config override
    path. Without that, a console whose config carries the same key — e.g.
    ``EE_PULSE_WIDTH_UL`` written during safety-param calibration — silently
    restores the safe value right after the fault is staged and the interlock
    test never trips (sdk#252).

    Args:
        console: a connected ``MotionConsole`` (has ``read_config`` and
            ``write_i2c_packet``).
        laser_params: register payloads; defaults to the bundled set
            (``load_laser_params(force_fault)``).
        fpga_map: friendlyName→I2C map; defaults to the bundled ``FpgaMap``.
        force_fault: when ``laser_params`` is None, load the fault set instead;
            always shields the faulted registers from user-config overrides.
        lock: optional mutex (anything with ``lock()``/``unlock()``) held for
            the duration of the I2C writes so the whole sequence is atomic
            w.r.t. other console access. Pass the app's console mutex when
            delegating from a multithreaded context; ``None`` = no external
            lock (the console serializes individual packets itself).
        trigger_freq_hz: the trigger frequency the system will run at. The
            bundled ``EE_RATE_LL``/``OPT_RATE_LL`` payloads encode the
            minimum inter-pulse period for 40 Hz (22.5 ms = 0.9 x period);
            at other rates the floor is rescaled by ``40 / trigger_freq_hz``
            so the safety margin stays proportional (sdk#129 — 60 Hz mode).
            ``None`` or 40 leaves the baseline values untouched. An explicit
            per-key user-config override still wins.
    """
    if laser_params is None:
        laser_params = load_laser_params(force_fault=force_fault)
    if fpga_map is None:
        fpga_map = FpgaMap()
    if not laser_params:
        logger.error("apply_laser_power: no laser parameters to apply")
        return False

    logger.info("Setting laser power from config...")

    faulted_names: set = set()
    faulted_coords: set = set()
    if force_fault:
        faulted_names = _fault_diff_names()
        for name in faulted_names:
            entry = fpga_map.get_entry_by_friendly_name(name)
            if entry is not None:
                faulted_coords.add((entry["channel"], entry["start_address"]))
        logger.info(
            "force_fault: exempting deliberately-faulted register(s) from "
            "user-config overrides: %s", sorted(faulted_names),
        )

    user_cfg: dict = {}
    try:
        cfg_obj = console.read_config()
        if cfg_obj is not None:
            user_cfg = cfg_obj.json_data or {}
    except Exception as e:
        logger.warning("Could not read user config before laser init: %s", e)

    ee_thresh = user_cfg.get("EE_THRESH")
    ee_gain = user_cfg.get("EE_GAIN")
    opt_thresh = user_cfg.get("OPT_THRESH")
    opt_gain = user_cfg.get("OPT_GAIN")

    # (channel, offset) entries to skip in the JSON pass when a user override
    # supersedes them.
    _EE_DRIVE_CL = (6, 0x10)   # Safety EE  DRIVE CL
    _OPT_DRIVE_CL = (7, 0x10)  # Safety OPT DRIVE CL
    skip_entries: set = set()
    if ee_thresh is not None or ee_gain is not None:
        skip_entries.add(_EE_DRIVE_CL)
    if opt_thresh is not None or opt_gain is not None:
        skip_entries.add(_OPT_DRIVE_CL)
    # A faulted DRIVE CL must be written from the fault file, not skipped
    # here and rewritten from user config below.
    skip_entries -= faulted_coords

    # Laser-safety limit scaling for non-baseline rates (sdk#129).
    # RATE_LL (min inter-pulse period) and PULSE_WIDTH_UL (max gate width)
    # both scale by baseline/rate: the period shrinks with the rate, and
    # the pulse width shrinks with it to hold the IEC 60825 duty cycle at
    # the 40 Hz-validated 2.0% (per the "Ultrasound & Laser Safety Limits
    # Calculator" AEL sheet: λ=795 nm, 500 µs @ 40 Hz, T=300 s, 3 mm beam
    # — the average-power AEL rows scale as 1/rate, so constant duty
    # preserves them exactly while the per-pulse t^0.75 AEL margin only
    # improves). Scaling the UL means the interlock ENFORCES the shorter
    # 60 Hz pulse rather than merely permitting it.
    # Fail-loud bookkeeping: if scaling is needed, every expected entry
    # must actually be found and rescaled — a silently-unscaled floor at
    # 60 Hz means the interlock trips on every pulse (dark laser, no
    # error); a silently-unscaled width ceiling means 60825 headroom
    # assumed by the app isn't enforced.
    from omotion.config import DEFAULT_TRIGGER_CONFIG
    _RATE_SCALED_PARAMS = frozenset({
        "EE_RATE_LL", "OPT_RATE_LL",
        "EE_PULSE_WIDTH_UL", "OPT_PULSE_WIDTH_UL",
    })
    _baseline_freq_hz = float(DEFAULT_TRIGGER_CONFIG["TriggerFrequencyHz"])
    _rate_scale_needed = (
        trigger_freq_hz is not None
        and float(trigger_freq_hz) != _baseline_freq_hz
    )
    rate_scaled_names: set = set()

    if lock is not None:
        lock.lock()
    try:
        for idx, laser_param in enumerate(laser_params, start=1):
            friendly_name = laser_param["friendlyName"]
            fpga_entry = fpga_map.get_entry_by_friendly_name(friendly_name)
            if fpga_entry is None:
                logger.error("Laser parameter entry not found: %s", friendly_name)
                continue

            mux_idx = fpga_entry["mux_idx"]
            channel = fpga_entry["channel"]
            i2c_addr = fpga_entry["i2c_addr"]
            data_size = fpga_entry["data_size"]
            offset = fpga_entry["start_address"]

            data_to_send = bytearray(laser_param["dataToSend"])

            if (
                _rate_scale_needed
                and friendly_name in _RATE_SCALED_PARAMS
            ):
                # Rescale the baseline min-period floor to the requested
                # rate, preserving the proportional margin (sdk#129).
                baseline_raw = int.from_bytes(data_to_send, "little")
                scaled_raw = int(round(
                    baseline_raw * _baseline_freq_hz / trigger_freq_hz
                ))
                data_to_send = bytearray(
                    scaled_raw.to_bytes(len(data_to_send), "little")
                )
                rate_scaled_names.add(friendly_name)
                logger.info(
                    "Rescaled %s for %.4g Hz trigger: raw %d -> %d (%.0f us)",
                    friendly_name, trigger_freq_hz, baseline_raw, scaled_raw,
                    scaled_raw * 0.32,
                )

            if (channel, offset) in skip_entries:
                logger.info(
                    "Skipping JSON entry ch=%d off=0x%02X (overridden by user config)",
                    channel, offset,
                )
                continue

            if friendly_name in user_cfg and friendly_name in faulted_names:
                logger.info(
                    "force_fault: keeping fault value for %s "
                    "(user-config override suppressed)", friendly_name,
                )
            elif friendly_name in user_cfg:
                override_val = user_cfg[friendly_name]
                num_bytes = int(data_size.rstrip("B")) // 8
                scale = fpga_entry.get("scale")
                try:
                    raw_int = float(override_val)
                    if scale:
                        raw_int = raw_int / scale
                    if _rate_scale_needed and friendly_name in _RATE_SCALED_PARAMS:
                        # A stored per-key RATE_LL override is calibrated
                        # for the 40 Hz baseline; written verbatim at 60 Hz
                        # it would EXCEED the pulse period and trip the
                        # interlock on every pulse. Rescale it exactly like
                        # the bundled baseline (sdk#129).
                        raw_int = raw_int * _baseline_freq_hz / float(trigger_freq_hz)
                        rate_scaled_names.add(friendly_name)
                        logger.info(
                            "Rescaled user-config %s for %.4g Hz trigger",
                            friendly_name, trigger_freq_hz,
                        )
                    max_val = (1 << (num_bytes * 8)) - 1
                    raw_int = max(0, min(max_val, int(round(raw_int))))
                    byteorder = "big" if fpga_entry.get("isMsbFirst", False) else "little"
                    data_to_send = bytearray(raw_int.to_bytes(num_bytes, byteorder=byteorder))
                    logger.info("Override %s raw=%d", friendly_name, raw_int)
                except Exception as e:
                    logger.warning(
                        "Could not convert override for %s: %s, using default",
                        friendly_name, e,
                    )

            logger.info(
                "(%d/%d) Writing I2C: muxIdx=%d, channel=%d, i2cAddr=0x%02X, "
                "offset=0x%02X, data=%s",
                idx, len(laser_params), mux_idx, channel, i2c_addr, offset,
                [f"0x{b:02X}" for b in data_to_send],
            )
            if not console.write_i2c_packet(
                mux_index=mux_idx,
                channel=channel,
                device_addr=i2c_addr,
                reg_addr=offset,
                data=data_to_send,
            ):
                logger.error(
                    "Failed to set laser power (muxIdx=%d, channel=%d)", mux_idx, channel
                )
                return False

        if _rate_scale_needed:
            missing = _RATE_SCALED_PARAMS - rate_scaled_names
            if missing:
                logger.error(
                    "apply_laser_power: %.4g Hz trigger requested but "
                    "RATE_LL entries %s were not found in laser_params — "
                    "safety floor NOT scaled; refusing to continue",
                    trigger_freq_hz, sorted(missing),
                )
                return False

        # User-config safety DRIVE CL overrides, written after the JSON pass.
        # 16-bit LSB-first uint16 raw register value (isMsbFirst=false).
        def _write_drive_cl(ch: int, thresh, gain, label: str) -> bool:
            if thresh is None:
                return True
            set_value = thresh
            gain_f = float(gain) if gain is not None else 0.0
            if gain_f != 0.0:
                set_value = thresh / gain_f
            raw = max(0, min(0xFFFF, int(round(set_value))))
            data = bytearray([raw & 0xFF, (raw >> 8) & 0xFF])
            logger.info("Writing user-config %s DRIVE CL: raw=%d, gain=%s", label, raw, gain_f)
            return console.write_i2c_packet(
                mux_index=1, channel=ch, device_addr=0x41, reg_addr=0x10, data=data
            )

        if _EE_DRIVE_CL in faulted_coords:
            logger.info("force_fault: Safety EE DRIVE CL kept at fault value")
        elif not _write_drive_cl(6, ee_thresh, ee_gain, "Safety EE"):
            logger.error("Failed to write user-config Safety EE DRIVE CL")
            return False
        if _OPT_DRIVE_CL in faulted_coords:
            logger.info("force_fault: Safety OPT DRIVE CL kept at fault value")
        elif not _write_drive_cl(7, opt_thresh, opt_gain, "Safety OPT"):
            logger.error("Failed to write user-config Safety OPT DRIVE CL")
            return False

        logger.info("Laser power set successfully.")
        return True
    finally:
        if lock is not None:
            lock.unlock()
