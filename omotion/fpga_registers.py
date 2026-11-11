"""Scaled, friendly-name access to the console FPGA registers.

Shared by the WI-00015 calibration adapters (``omotion.calibration``) and by
Optimize Laser (``omotion.exposure``), so neither has to import the other.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from omotion.laser import FpgaMap


@dataclass(frozen=True)
class SettingReadback:
    name: str
    requested: float
    actual: float


class FpgaRegisterIO:
    """Scaled friendly-name register access through MotionConsole I2C."""

    def __init__(self, console, *, fpga_map=None):
        self._console = console
        self._map = fpga_map if fpga_map is not None else FpgaMap()

    def _entry(self, name: str) -> tuple[dict, int, str, float]:
        entry = self._map.get_entry_by_friendly_name(name)
        if entry is None:
            raise KeyError(f"Unknown FPGA register: {name}")
        data_size = entry.get("data_size")
        if not isinstance(data_size, str) or not data_size.endswith("B"):
            raise ValueError(f"Invalid data size for {name}: {data_size!r}")
        bits = int(data_size[:-1])
        if bits <= 0 or bits % 8:
            raise ValueError(f"Invalid data size for {name}: {data_size!r}")
        byteorder = "big" if entry.get("isMsbFirst", False) else "little"
        scale = entry.get("scale")
        scale = 1.0 if scale is None else float(scale)
        if not math.isfinite(scale) or scale == 0:
            raise ValueError(f"Invalid scale for {name}: {scale!r}")
        return entry, bits // 8, byteorder, scale

    @staticmethod
    def _i2c_args(entry: dict) -> dict:
        return {
            "mux_index": entry["mux_idx"],
            "channel": entry["channel"],
            "device_addr": entry["i2c_addr"],
            "reg_addr": entry["start_address"],
        }

    def read(self, name: str) -> float:
        entry, width, byteorder, scale = self._entry(name)
        data, length = self._console.read_i2c_packet(
            **self._i2c_args(entry), read_len=width
        )
        if (
            not isinstance(data, (bytes, bytearray))
            or length != width
            or len(data) != width
        ):
            raise RuntimeError(
                f"No I2C readback for {name} (mux {entry['mux_idx']} "
                f"ch {entry['channel']} addr 0x{entry['i2c_addr']:02X} "
                f"reg 0x{entry['start_address']:02X})"
            )
        raw = int.from_bytes(data, byteorder=byteorder, signed=False)
        return raw * scale

    def write(self, name: str, value: float) -> SettingReadback | None:
        entry, width, byteorder, scale = self._entry(name)
        requested = float(value)
        if not math.isfinite(requested):
            raise ValueError(f"FPGA value for {name} must be finite")
        raw = int(round(requested / scale))
        if not 0 <= raw < (1 << (width * 8)):
            raise ValueError(f"FPGA value for {name} is out of range")
        data = raw.to_bytes(width, byteorder=byteorder, signed=False)
        if not self._console.write_i2c_packet(**self._i2c_args(entry), data=data):
            return None
        return SettingReadback(name, requested, self.read(name))
