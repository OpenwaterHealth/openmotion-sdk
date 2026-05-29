"""Laser-power configuration for the Open-Motion console.

Sets the seed/TA/safety laser-driver registers over I2C so the laser actually
emits when the trigger fires. This is a *cold-start prerequisite*: after a
power-cycle the driver registers are cleared, so the laser pulses produce no
light until these are written. The bloodflow app does this on its scan path
(its "Issue #108" guard); this module is the SDK-owned equivalent so any SDK
consumer (scripts, headless tools) can do it without the app.

The register values live in two bundled data files under ``omotion/data``:

* ``laser_params.json``  — list of ``{"friendlyName", "dataToSend"}`` driver
  register payloads (the locked baseline; mirrors the bloodflow app's
  ``config/laser_params.json``).
* ``fpga_model.json``    — maps each ``friendlyName`` to its I2C location
  (mux/channel/device addr/register offset/size).

This is laser-sensitive: editing the bundled values risks wrong pulse widths
or tripping the safety interlock. Treat them as locked baseline data.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger("openmotion.sdk.laser")

_DATA_DIR = Path(__file__).resolve().parent / "data"
_FPGA_MODEL_PATH = _DATA_DIR / "fpga_model.json"


class FpgaMap:
    """Maps a laser-driver ``friendlyName`` to its I2C location.

    Backed by the bundled ``fpga_model.json``. This is the minimal lookup the
    laser-power write needs — it deliberately omits the bloodflow app's QML
    scale-override machinery and the legacy ``FpgaModel.js`` fallback.
    """

    def __init__(self, model: Optional[list] = None) -> None:
        if model is not None:
            self._model = model
            return
        try:
            with open(_FPGA_MODEL_PATH, "r", encoding="utf-8") as f:
                self._model = json.load(f)
        except Exception as e:  # pragma: no cover - data file ships with package
            logger.error("Failed to load bundled fpga_model.json: %s", e)
            self._model = []

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

    Returns a list of ``{"friendlyName", "dataToSend"}`` dicts, or an empty
    list on error. ``force_fault`` loads ``laser_params_fault.json`` — a set
    engineered to trip the laser-safety interlock for testing the safety path.
    """
    name = "laser_params_fault.json" if force_fault else "laser_params.json"
    path = _DATA_DIR / name
    try:
        with open(path, "r", encoding="utf-8") as f:
            params = json.load(f)
        logger.info("Loaded %d laser parameter sets from %s", len(params), path)
        return params
    except Exception as e:
        logger.error("Failed to load laser params from %s: %s", path, e)
        return []


def apply_laser_power(
    console: Any,
    *,
    laser_params: Optional[list] = None,
    fpga_map: Optional[FpgaMap] = None,
    force_fault: bool = False,
    lock: Optional[Any] = None,
) -> bool:
    """Write the laser-driver configuration to ``console`` over I2C.

    Reads user overrides from ``console.read_config()`` and applies the
    ``laser_params`` list, honoring per-key overrides and the safety DRIVE CL
    values. Returns True on success, False if any I2C write fails.

    Args:
        console: a connected ``MotionConsole`` (has ``read_config`` and
            ``write_i2c_packet``).
        laser_params: register payloads; defaults to the bundled set
            (``load_laser_params(force_fault)``).
        fpga_map: friendlyName→I2C map; defaults to the bundled ``FpgaMap``.
        force_fault: when ``laser_params`` is None, load the fault set instead.
        lock: optional mutex (anything with ``lock()``/``unlock()``) held for
            the duration of the I2C writes so the whole sequence is atomic
            w.r.t. other console access. Pass the app's console mutex when
            delegating from a multithreaded context; ``None`` = no external
            lock (the console serializes individual packets itself).
    """
    if laser_params is None:
        laser_params = load_laser_params(force_fault=force_fault)
    if fpga_map is None:
        fpga_map = FpgaMap()
    if not laser_params:
        logger.error("apply_laser_power: no laser parameters to apply")
        return False

    logger.info("Setting laser power from config...")

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

            if (channel, offset) in skip_entries:
                logger.info(
                    "Skipping JSON entry ch=%d off=0x%02X (overridden by user config)",
                    channel, offset,
                )
                continue

            if friendly_name in user_cfg:
                override_val = user_cfg[friendly_name]
                num_bytes = int(data_size.rstrip("B")) // 8
                scale = fpga_entry.get("scale")
                try:
                    raw_int = float(override_val)
                    if scale:
                        raw_int = raw_int / scale
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

        if not _write_drive_cl(6, ee_thresh, ee_gain, "Safety EE"):
            logger.error("Failed to write user-config Safety EE DRIVE CL")
            return False
        if not _write_drive_cl(7, opt_thresh, opt_gain, "Safety OPT"):
            logger.error("Failed to write user-config Safety OPT DRIVE CL")
            return False

        logger.info("Laser power set successfully.")
        return True
    finally:
        if lock is not None:
            lock.unlock()
