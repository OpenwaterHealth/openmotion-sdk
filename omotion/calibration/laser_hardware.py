"""Production hardware boundaries for WI-00015 laser calibration.

The adapters in this module keep hardware lifetime and wire-format details out
of the procedure workflow.  Tests inject fakes; importing this module never
constructs an Ophir COM object or touches attached hardware.
"""

from __future__ import annotations

import math
import re
import statistics
import time
from typing import Callable

from omotion.MotionInterface import MotionInterface
from .laser import (
    MIN_PULSE_COUNT_EXCLUSIVE,
    EnergyMeasurement,
    OphirIdentity,
    SettingReadback,
    TopologySnapshot,
    validate_exact_dual_topology,
    validate_exact_single_topology,
)
from .dual_sensor_laser import DualPreflightSnapshot
from .motion_bench import (
    FpgaRegisterIO,
    MotionConsoleBenchBase,
    default_interface_factory as _default_interface_factory,
    read_console_fpga_firmware_revisions,
)
from .single_sensor_laser import (
    OphirEvidenceApplicability,
    OphirSettingEvidence,
    PreflightSnapshot,
)

__all__ = [
    "FpgaRegisterIO",
    "MotionLaserCalibrationBench",
    "OphirEnergyMeter",
    "read_console_fpga_firmware_revisions",
]


def _default_ophir_com_factory():
    # pywin32 is declared in pyproject.toml with a sys_platform == 'win32'
    # marker.  Keep the import here so SDK import and all fake-driven tests
    # work on platforms where it is absent.
    import win32com.client

    return win32com.client.Dispatch("OphirLMMeasurement.CoLMMeasurement")


class OphirEnergyMeter:
    """Channel-0 Ophir COM adapter with exact WI setting evidence."""

    _CHANNEL = 0
    _MINIMUM_VALID_SAMPLES = MIN_PULSE_COUNT_EXCLUSIVE + 1
    _TIMESTAMP_HOST_TOLERANCE_MS = 50.0

    def __init__(
        self,
        *,
        com_factory=_default_ophir_com_factory,
        duration_s: float = 2.0,
        poll_interval_s: float = 0.05,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        duration_s = float(duration_s)
        poll_interval_s = float(poll_interval_s)
        if (
            not math.isfinite(duration_s)
            or duration_s <= 0
            or not math.isfinite(poll_interval_s)
            or poll_interval_s <= 0
        ):
            raise ValueError("duration and poll interval must be finite and positive")
        self._com_factory = com_factory
        self._duration_s = duration_s
        self._poll_interval_s = poll_interval_s
        self._clock = clock
        self._sleep = sleep
        self._com = None
        self._handle = None
        self._preflight_passed = False
        self._stream_active = False

    def _ensure_com(self):
        if self._com is not None:
            return self._com
        try:
            self._com = self._com_factory()
        except Exception as exc:
            raise RuntimeError(f"Ophir COM construction failed: {exc}") from exc
        if self._com is None:
            raise RuntimeError("Ophir COM construction failed: factory returned None")
        return self._com

    @staticmethod
    def _required_text(value) -> str:
        if value is None:
            raise ValueError
        text = str(value).strip()
        if not text:
            raise ValueError
        return text

    @staticmethod
    def _option_number(
        option,
        expected_unit: str | None = None,
        *,
        allow_unitless: bool = False,
    ) -> float | None:
        text = str(option).strip().lower().replace(" ", "")
        match = re.fullmatch(r"([+-]?(?:\d+(?:\.\d*)?|\.\d+))([a-z%]*)", text)
        if match is None:
            return None
        unit = match.group(2)
        if expected_unit is not None and unit != expected_unit:
            if not (allow_unitless and unit == ""):
                return None
        elif expected_unit is None and unit:
            return None
        return float(match.group(1))

    def _close_handle_best_effort(self) -> None:
        handle = self._handle
        self._handle = None
        if self._com is None or handle is None:
            return
        try:
            self._com.Close(handle)
        except Exception:
            pass

    @staticmethod
    def _options_response(response, setting_name: str) -> tuple[int, list]:
        if not isinstance(response, (tuple, list)) or len(response) != 2:
            raise RuntimeError(f"Ophir {setting_name} getter returned malformed data")
        index, options = response
        try:
            options = list(options)
            index = int(index)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"Ophir {setting_name} getter returned malformed data"
            ) from exc
        if not options or not 0 <= index < len(options):
            raise RuntimeError(
                f"Ophir {setting_name} getter returned invalid selection"
            )
        return index, options

    def _configure_setting(
        self,
        *,
        name: str,
        requested,
        getter_name: str,
        setter_name: str,
        matches,
        actual_value,
    ) -> OphirSettingEvidence:
        com = self._com
        _, options = self._options_response(
            getattr(com, getter_name)(self._handle, self._CHANNEL), name
        )
        selected = next(
            (i for i, option in enumerate(options) if matches(option)), None
        )
        if selected is None:
            raise RuntimeError(f"Required Ophir {name} option is unavailable")
        getattr(com, setter_name)(self._handle, self._CHANNEL, selected)
        readback_index, readback_options = self._options_response(
            getattr(com, getter_name)(self._handle, self._CHANNEL), name
        )
        readback_option = readback_options[readback_index]
        if not matches(readback_option):
            raise RuntimeError(f"Ophir {name} readback mismatch")
        return OphirSettingEvidence(
            name,
            requested,
            actual_value(readback_option),
            OphirEvidenceApplicability.APPLICABLE,
            True,
        )

    def preflight(self) -> tuple[OphirIdentity, tuple[OphirSettingEvidence, ...]]:
        self._preflight_passed = False
        self._close_handle_best_effort()
        com = self._ensure_com()
        serials = com.ScanUSB()
        if not serials:
            raise RuntimeError("No Ophir USB meters were found")
        try:
            handle = com.OpenUSBDevice(serials[0])
        except Exception as exc:
            raise RuntimeError(f"Ophir meter open failed: {exc}") from exc
        if (
            isinstance(handle, bool)
            or not isinstance(handle, (int, float))
            or handle <= 0
        ):
            raise RuntimeError("Ophir meter did not return a valid handle")
        self._handle = handle
        try:
            if not com.IsSensorExists(handle, self._CHANNEL):
                raise RuntimeError("Ophir channel 0 energy sensor is missing")

            try:
                device_info = com.GetDeviceInfo(handle)
                sensor_info = com.GetSensorInfo(handle, self._CHANNEL)
                if len(device_info) < 3 or len(sensor_info) < 3:
                    raise ValueError
                meter_model = self._required_text(device_info[0])
                meter_serial = self._required_text(device_info[2])
                sensor_serial = self._required_text(sensor_info[0])
                sensor_model = self._required_text(sensor_info[2])
                device_due = self._required_text(
                    com.GetDeviceCalibrationDueDate(handle)
                )
                sensor_due = self._required_text(
                    com.GetSensorCalibrationDueDate(handle, self._CHANNEL)
                )
            except Exception as exc:
                raise RuntimeError(
                    "Ophir identity and calibration due information is incomplete"
                ) from exc

            evidence = (
                self._configure_setting(
                    name="measurement_mode",
                    requested="Energy",
                    getter_name="GetMeasurementMode",
                    setter_name="SetMeasurementMode",
                    matches=lambda option: str(option).strip().lower() == "energy",
                    actual_value=lambda option: "Energy",
                ),
                self._configure_setting(
                    name="range_mj",
                    requested=2.0,
                    getter_name="GetRanges",
                    setter_name="SetRange",
                    matches=lambda option: self._option_number(option, "mj") == 2.0,
                    actual_value=lambda option: 2.0,
                ),
                self._configure_setting(
                    name="wavelength_nm",
                    requested=795,
                    getter_name="GetWavelengths",
                    setter_name="SetWavelength",
                    matches=lambda option: (
                        self._option_number(option, "nm", allow_unitless=True) == 795.0
                    ),
                    actual_value=lambda option: 795,
                ),
                self._configure_setting(
                    name="pulse_length_ms",
                    requested=1.0,
                    getter_name="GetPulseLengths",
                    setter_name="SetPulseLength",
                    matches=lambda option: self._option_number(option, "ms") == 1.0,
                    actual_value=lambda option: 1.0,
                ),
                self._configure_setting(
                    name="threshold",
                    requested="minimum_available",
                    getter_name="GetThreshold",
                    setter_name="SetThreshold",
                    matches=lambda option: (
                        str(option).strip().lower() in ("min", "minimum")
                    ),
                    actual_value=lambda option: "minimum_available",
                ),
                OphirSettingEvidence(
                    "display_averaging_s",
                    3,
                    None,
                    OphirEvidenceApplicability.NOT_APPLICABLE,
                    True,
                ),
                OphirSettingEvidence(
                    "graph_mode",
                    "Statistics",
                    None,
                    OphirEvidenceApplicability.NOT_APPLICABLE,
                    True,
                ),
            )
            identity = OphirIdentity(
                meter_model,
                meter_serial,
                sensor_model,
                sensor_serial,
                f"meter: {device_due}; sensor: {sensor_due}",
            )
        except Exception:
            self._close_handle_best_effort()
            raise
        self._preflight_passed = True
        return identity, evidence

    def measure(self) -> EnergyMeasurement:
        if not self._preflight_passed or self._com is None or self._handle is None:
            raise RuntimeError("Ophir preflight must pass before measurement")
        values_uj: list[float] = []
        timestamps_ms: list[float] = []
        discarded = 0
        priming_batch_drained = False
        started_at = self._clock()
        try:
            self._com.StartStream(self._handle, self._CHANNEL)
            self._stream_active = True
            while True:
                elapsed = self._clock() - started_at
                if elapsed >= self._duration_s:
                    break
                self._sleep(min(self._poll_interval_s, self._duration_s - elapsed))
                values, timestamps, statuses = self._com.GetData(
                    self._handle, self._CHANNEL
                )
                if not (len(values) == len(timestamps) == len(statuses)):
                    raise RuntimeError("Ophir stream returned misaligned sample arrays")
                if not priming_batch_drained:
                    if len(values) > 0:
                        priming_batch_drained = True
                    continue
                for value, timestamp, status in zip(values, timestamps, statuses):
                    if status != 0:
                        discarded += 1
                        continue
                    timestamp_ms = float(timestamp)
                    if timestamps_ms and math.isfinite(timestamp_ms):
                        timestamp_gap_ms = timestamp_ms - timestamps_ms[-1]
                        if timestamp_gap_ms <= 0:
                            discarded += 1
                            continue
                        host_elapsed_ms = (self._clock() - started_at) * 1000.0
                        if timestamp_gap_ms > (
                            host_elapsed_ms + self._TIMESTAMP_HOST_TOLERANCE_MS
                        ):
                            # The gap is longer than this stream has existed, so the
                            # accepted prefix is buffered data from an earlier epoch.
                            discarded += len(values_uj)
                            values_uj.clear()
                            timestamps_ms.clear()
                    values_uj.append(float(value) * 1e6)
                    timestamps_ms.append(timestamp_ms)
                if len(values_uj) >= self._MINIMUM_VALID_SAMPLES:
                    break
        finally:
            if self._stream_active:
                self._com.StopStream(self._handle, self._CHANNEL)
                self._stream_active = False
        duration = self._clock() - started_at
        n = len(values_uj)
        mean = statistics.fmean(values_uj) if values_uj else math.nan
        stdev = statistics.stdev(values_uj) if n >= 2 else math.nan
        minimum = min(values_uj) if values_uj else math.nan
        maximum = max(values_uj) if values_uj else math.nan
        timestamp_delta_s = (
            (timestamps_ms[-1] - timestamps_ms[0]) / 1000.0 if n >= 2 else 0.0
        )
        rate = (n - 1) / timestamp_delta_s if timestamp_delta_s > 0 else math.nan
        return EnergyMeasurement(
            n=n,
            discarded=discarded,
            mean_uj=mean,
            stdev_uj=stdev,
            rate_hz=rate,
            min_uj=minimum,
            max_uj=maximum,
            duration_s=duration,
        )

    def close(self) -> None:
        self._preflight_passed = False
        handle = self._handle
        self._handle = None
        if self._com is None:
            return
        first_error = None
        cleanups = []
        if handle is not None:
            def stop_active_stream() -> None:
                self._com.StopStream(handle, self._CHANNEL)
                self._stream_active = False

            if self._stream_active:
                cleanups.append(stop_active_stream)
            cleanups.extend((self._com.StopAllStreams, lambda: self._com.Close(handle)))
        else:
            cleanups.append(self._com.StopAllStreams)
        cleanups.append(self._com.CloseAll)
        for cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        self._stream_active = False
        if first_error is not None:
            raise first_error


class MotionLaserCalibrationBench(MotionConsoleBenchBase):
    """One long-lived MotionInterface plus an Ophir energy-meter adapter."""

    def __init__(
        self,
        energy_meter,
        *,
        interface_factory: Callable[[], MotionInterface] = _default_interface_factory,
        fpga_map=None,
        wait_timeout: float = 10.0,
        topology_quiet_period_s: float = 0.3,
        topology_poll_interval_s: float = 0.05,
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        if not all(
            math.isfinite(value)
            for value in (
                wait_timeout,
                topology_quiet_period_s,
                topology_poll_interval_s,
            )
        ):
            raise ValueError("topology timing values must be finite")
        if wait_timeout <= 0:
            raise ValueError("wait_timeout must be positive")
        if topology_quiet_period_s < 0 or topology_poll_interval_s <= 0:
            raise ValueError(
                "topology quiet period must be nonnegative and poll interval positive"
            )
        self._meter = energy_meter
        self._interface = interface_factory()
        self._console = self._interface.console
        self._registers = FpgaRegisterIO(self._console, fpga_map=fpga_map)
        self._wait_timeout = float(wait_timeout)
        self._topology_quiet_period_s = float(topology_quiet_period_s)
        self._topology_poll_interval_s = float(topology_poll_interval_s)
        self._clock = clock
        self._sleep = sleep
        self._started = False
        self._declared_side: str | None = None
        self._declared_dual = False
        self._ready_sensor_count = 0

    def _ensure_started(self, required_sensor_count: int = 1) -> None:
        if not self._started:
            self._interface.start(wait=False)
            self._started = True
        if self._ready_sensor_count >= required_sensor_count:
            return
        ready = self._interface.wait_for_ready(
            console=True,
            sensors=required_sensor_count,
            timeout=self._wait_timeout,
        )
        if not ready:
            raise RuntimeError(self._not_ready_reason(self._wait_timeout))
        self._ready_sensor_count = required_sensor_count

    def _wait_for_stable_topology(self) -> TopologySnapshot:
        snapshot = self._topology_snapshot()
        stable_since = self._clock()
        deadline = stable_since + self._wait_timeout
        while self._clock() - stable_since < self._topology_quiet_period_s:
            now = self._clock()
            if now >= deadline:
                raise RuntimeError("Motion topology did not become stable before timeout")
            remaining_quiet = self._topology_quiet_period_s - (now - stable_since)
            self._sleep(
                min(
                    self._topology_poll_interval_s,
                    remaining_quiet,
                    deadline - now,
                )
            )
            current = self._topology_snapshot()
            if current != snapshot:
                snapshot = current
                stable_since = self._clock()
        return snapshot

    def preflight(self, side: str) -> PreflightSnapshot:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._ensure_started()
        ophir_identity = None
        ophir_evidence = ()
        ophir_ready = False
        ophir_failure_reason = None
        try:
            ophir_identity, ophir_evidence = self._meter.preflight()
            ophir_evidence = tuple(ophir_evidence)
            ophir_ready = True
        except Exception as exc:
            ophir_failure_reason = str(exc) or exc.__class__.__name__
        topology = self._wait_for_stable_topology()
        self._declared_side = side
        self._declared_dual = False
        selected = self._interface.left if side == "left" else self._interface.right
        console_responsive, console_identity = self._console_preflight()
        return PreflightSnapshot(
            topology=topology,
            console_identity=console_identity,
            selected_sensor_identity=self._identity("sensor", selected),
            console_responsive=console_responsive,
            ophir_identity=ophir_identity,
            ophir_ready=ophir_ready,
            ophir_setting_evidence=ophir_evidence,
            ophir_failure_reason=ophir_failure_reason,
        )

    def preflight_dual(self) -> DualPreflightSnapshot:
        """Preflight the console, both shipping sensors, and common Ophir meter."""
        self._ensure_started(required_sensor_count=2)
        ophir_identity = None
        ophir_evidence = ()
        ophir_ready = False
        ophir_failure_reason = None
        try:
            ophir_identity, ophir_evidence = self._meter.preflight()
            ophir_evidence = tuple(ophir_evidence)
            ophir_ready = True
        except Exception as exc:
            ophir_failure_reason = str(exc) or exc.__class__.__name__
        topology = self._wait_for_stable_topology()
        self._declared_side = None
        self._declared_dual = True
        console_responsive, console_identity = self._console_preflight()
        return DualPreflightSnapshot(
            topology=topology,
            console_identity=console_identity,
            left_sensor_identity=self._identity("left sensor", self._interface.left),
            right_sensor_identity=self._identity(
                "right sensor", self._interface.right
            ),
            console_responsive=console_responsive,
            ophir_identity=ophir_identity,
            ophir_ready=ophir_ready,
            ophir_setting_evidence=ophir_evidence,
            ophir_failure_reason=ophir_failure_reason,
        )

    def revalidate_topology(self, side: str) -> TopologySnapshot:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self._ensure_started()
        return self._wait_for_stable_topology()

    def revalidate_dual_topology(self) -> TopologySnapshot:
        self._ensure_started(required_sensor_count=2)
        return self._wait_for_stable_topology()

    def write_register(self, name: str, value: float) -> SettingReadback | None:
        return self._registers.write(name, value)

    def measure_energy(self) -> EnergyMeasurement:
        rate_hz = self.read_trigger_rate_hz()
        if rate_hz != 40.0:
            raise RuntimeError("Trigger frequency must read back exactly 40 Hz")
        ta_pulse = self.read_register("TA_PULSE_WIDTH")
        ee_limit = self.read_register("EE_PULSE_WIDTH_UL")
        opt_limit = self.read_register("OPT_PULSE_WIDTH_UL")
        if not (
            all(math.isfinite(value) for value in (ta_pulse, ee_limit, opt_limit))
            and ta_pulse < ee_limit
            and ta_pulse < opt_limit
        ):
            raise RuntimeError(
                "TA pulse width must be finite and below both active safety limits"
            )
        topology = self._wait_for_stable_topology()
        if self._declared_dual:
            if not validate_exact_dual_topology(topology).passed:
                raise RuntimeError(
                    "Motion topology must match the exact declared dual topology before firing"
                )
        elif self._declared_side is None or not validate_exact_single_topology(
            topology, self._declared_side
        ).passed:
            raise RuntimeError(
                "Motion topology must match the exact declared topology before firing"
            )
        try:
            if not self._console.start_trigger():
                raise RuntimeError("Console failed to start trigger")
            return self._meter.measure()
        finally:
            self.stop_trigger()

    def stop_trigger(self) -> None:
        if not self._console.stop_trigger():
            raise RuntimeError("Console failed to stop trigger")

    def close(self) -> None:
        first_error = None
        for cleanup in (
            self.stop_trigger,
            self._meter.close,
            self._interface.stop,
        ):
            try:
                cleanup()
            except Exception as exc:
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error
