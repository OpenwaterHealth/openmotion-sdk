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
from omotion.fpga_registers import FpgaRegisterIO  # noqa: F401  (moved; re-exported)


def default_interface_factory() -> MotionInterface:
    return MotionInterface()


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

    def _console_safety_keys_writable(self) -> bool | None:
        """Whether the console accepts the WI-00015 safety config keys: False
        on a Release/Debug image of firmware that owns the safety state, True
        on its Factory image, None on older firmware (no boot-info v2)."""
        get_boot_info = getattr(self._console, "get_boot_info", None)
        info = get_boot_info() if callable(get_boot_info) else None
        if info is None or not info.firmware_owns_safety_state:
            return None
        return info.factory_image

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
