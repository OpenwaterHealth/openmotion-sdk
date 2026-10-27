"""Shared Motion console bench plumbing for the WI-00015 hardware adapters.

This module is deliberately meter-agnostic: the safety bench must stay free of
any Ophir dependency, so everything Ophir-specific lives in ``laser_hardware``.
"""

from __future__ import annotations

import math
from typing import Mapping

from omotion.MotionConfig import MotionConfig
from omotion.MotionInterface import MotionInterface
from .laser import (
    DeviceIdentity,
    FpgaFirmwareRevision,
    SettingReadback,
    TopologySnapshot,
)
from omotion.laser import FpgaMap


def default_interface_factory() -> MotionInterface:
    return MotionInterface()


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


_CONSOLE_FPGA_VERSION_PREFIXES = (
    ("TA", "TA"),
    ("SEED", "SEED"),
    ("SAFETY_EE", "EE"),
    ("SAFETY_OPT", "OPT"),
)


def read_console_fpga_firmware_revisions(
    registers: FpgaRegisterIO,
) -> tuple[FpgaFirmwareRevision, ...]:
    """Read the four complete console-board FPGA semantic revisions."""
    revisions = []
    for controller, prefix in _CONSOLE_FPGA_VERSION_PREFIXES:
        components = []
        for component in ("MAJOR", "MINOR", "REVISION"):
            register_name = f"{prefix}_{component}"
            try:
                value = registers.read(register_name)
            except Exception as error:
                # str(error) is what reaches the operator (the workflow drops
                # the __cause__ chain), so carry the cause text here.
                raise RuntimeError(
                    f"Could not read {controller} FPGA firmware revision "
                    f"register {register_name}: {error}"
                ) from error
            if (
                isinstance(value, bool)
                or not isinstance(value, int | float)
                or not math.isfinite(float(value))
                or not float(value).is_integer()
                or not 0 <= int(value) <= 255
            ):
                raise RuntimeError(
                    f"Invalid {controller} FPGA firmware revision register {register_name}: {value!r}."
                )
            components.append(int(value))
        revisions.append(
            FpgaFirmwareRevision(controller, ".".join(map(str, components)))
        )
    return tuple(revisions)


class MotionConsoleBenchBase:
    """Console identity, configuration, and trigger plumbing both benches share.

    Subclasses set ``_interface``, ``_console``, and ``_registers`` in their
    ``__init__`` before any of these methods run.
    """

    _interface: MotionInterface

    @staticmethod
    def _safe_call(device, method_name: str):
        try:
            return getattr(device, method_name)()
        except Exception:
            return None

    def _identity(self, role: str, device) -> DeviceIdentity:
        # Each field is independent: a failed firmware query must not replace a
        # successfully-read serial (especially not with exception text).
        return DeviceIdentity(
            role=role,
            serial=self._safe_call(device, "read_serial_number"),
            firmware=self._safe_call(device, "get_version"),
            hardware_id=self._safe_call(device, "get_hardware_id"),
        )

    def _console_identity(self) -> DeviceIdentity:
        identity = self._identity("console", self._console)
        return DeviceIdentity(
            role=identity.role,
            serial=identity.serial,
            firmware=identity.firmware,
            hardware_id=identity.hardware_id,
            fpga_firmware=identity.fpga_firmware,
            fpga_firmware_revisions=read_console_fpga_firmware_revisions(
                self._registers
            ),
        )

    def _console_responsive(self) -> bool:
        try:
            echoed, length = self._console.echo(b"WI15")
            return echoed == b"WI15" and length == 4
        except Exception:
            return False

    def _not_ready_reason(self, timeout_s: float) -> str:
        """Why a readiness wait expired, with the interface's own evidence.

        ``wait_for_ready`` only returns False. Without this, preflight went on
        to the first console command whose failure is not swallowed and
        reported that instead (#263: "could not read TA_MAJOR" for a console
        that never connected).
        """
        describe = getattr(self._interface, "describe_connections", None)
        try:
            details = describe() if callable(describe) else ""
        except Exception as error:
            details = f"unavailable ({error})"
        if not details:
            topology = self._topology_snapshot()
            details = ", ".join(
                f"{label} {'connected' if connected else 'not connected'}"
                for label, connected in (
                    ("console", topology.console_connected),
                    ("left sensor", topology.left_connected),
                    ("right sensor", topology.right_connected),
                )
            )
        return f"Motion devices not ready within {timeout_s:g} s: {details}"

    def _console_preflight(self) -> tuple[bool, DeviceIdentity]:
        """Echo first; read FPGA revisions only from a console that answers,
        so an unresponsive console is reported as that."""
        if not self._console_responsive():
            return False, self._identity("console", self._console)
        return True, self._console_identity()

    def _topology_snapshot(self) -> TopologySnapshot:
        return TopologySnapshot(
            console_connected=bool(self._console.is_connected()),
            left_connected=bool(self._interface.left.is_connected()),
            right_connected=bool(self._interface.right.is_connected()),
        )

    def read_user_configuration(self) -> Mapping[str, object]:
        config = self._console.read_config()
        if not isinstance(config, MotionConfig) or not isinstance(
            config.json_data, dict
        ):
            raise RuntimeError("Complete MotionConfig readback was not returned")
        return dict(config.json_data)

    def write_user_configuration(
        self, configuration: Mapping[str, object]
    ) -> Mapping[str, object] | None:
        write_result = self._console.write_config(
            MotionConfig(json_data=dict(configuration))
        )
        if not isinstance(write_result, MotionConfig):
            return None
        try:
            return self.read_user_configuration()
        except Exception:
            return None

    def bring_up_laser_configuration(self) -> None:
        if not self._interface.apply_laser_power():
            raise RuntimeError("Laser configuration bring-up failed")

    def read_register(self, name: str) -> float:
        return self._registers.read(name)

    def read_trigger_rate_hz(self) -> float:
        response = self._console.get_trigger_json()
        if not isinstance(response, dict) or "TriggerFrequencyHz" not in response:
            raise RuntimeError("Trigger-frequency readback was unavailable")
        rate = float(response["TriggerFrequencyHz"])
        if not math.isfinite(rate):
            raise RuntimeError("Trigger-frequency readback was not finite")
        return rate

    def write_trigger_rate_hz(self, rate_hz: float) -> SettingReadback | None:
        requested = float(rate_hz)
        try:
            current = self._console.get_trigger_json()
            if not isinstance(current, dict):
                return None
            updated = dict(current)
            updated["TriggerFrequencyHz"] = requested
            if not self._console.set_trigger_json(updated):
                return None
            response = self._console.get_trigger_json()
            if not isinstance(response, dict) or "TriggerFrequencyHz" not in response:
                return None
            actual = float(response["TriggerFrequencyHz"])
        except Exception:
            return None
        return SettingReadback("trigger_rate_hz_write", requested, actual)
