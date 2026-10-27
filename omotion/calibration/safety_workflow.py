"""UI-neutral workflow for WI-00015 Safety Calibration."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import math
from pathlib import Path
import time
from typing import Callable, Mapping, Protocol

try:
    from omotion import __version__ as _RUNTIME_SDK_VERSION
except (ImportError, AttributeError):
    _RUNTIME_SDK_VERSION = "unavailable"

from .laser import (
    CriterionResult,
    DeviceIdentity,
    FailureKind,
    FinalSettingCheck,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
    percent_difference,
    validate_console_fpga_revisions,
    validate_serial,
    within_percent,
)
from .safety import (
    MINIMUM_ADC_SAMPLES,
    MINIMUM_POWER_OFF_S,
    SAFETY_EE_MULTIPLIER,
    SAFETY_OPT_MULTIPLIER,
    AdcReadEvidence,
    NormalScanEvidence,
    PowerCycleEvidence,
    PulseLimitCalculation,
    SafetyController,
    SafetyLimitCalculation,
    SafetyWarningEvidence,
    ShippingTopology,
    calculate_pulse_limits,
    calculate_safety_limit,
    validate_current_configuration,
    validate_shipping_topology,
)
from ._procedure import (
    ProcedureEvent,
    ProcedureFailure as _ProcedureFailure,
    ReportArtifactEvidence,
    deeply_immutable as _deeply_immutable,
    finite_number as _finite_number,
)


@dataclass(frozen=True)
class AdcSamplingPolicy:
    minimum_valid_samples: int = MINIMUM_ADC_SAMPLES
    maximum_attempts_per_controller: int = 30
    interval_s: float = 0.05

    def __post_init__(self) -> None:
        if (
            isinstance(self.minimum_valid_samples, bool)
            or self.minimum_valid_samples < MINIMUM_ADC_SAMPLES
        ):
            raise ValueError("minimum_valid_samples must be at least 10")
        if (
            isinstance(self.maximum_attempts_per_controller, bool)
            or self.maximum_attempts_per_controller < self.minimum_valid_samples
        ):
            raise ValueError(
                "maximum_attempts_per_controller must cover the minimum samples"
            )
        if (
            isinstance(self.interval_s, bool)
            or not _finite_number(self.interval_s)
            or self.interval_s < 0
        ):
            raise ValueError("interval_s must be finite and nonnegative")


@dataclass(frozen=True)
class SafetyCalibrationRequest:
    shipping_topology: ShippingTopology
    operator: str
    build_id: str
    fixture_id: str
    procedure_id: str
    output_root: Path | str
    run_id: str
    sdk_version: str = _RUNTIME_SDK_VERSION
    started_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))


@dataclass(frozen=True)
class ConsolePreflightSnapshot:
    topology: TopologySnapshot
    console_identity: DeviceIdentity
    console_responsive: bool


@dataclass(frozen=True)
class SafetyCalibrationResult:
    status: ProcedureStatus
    shipping_topology: ShippingTopology
    sdk_version: str = _RUNTIME_SDK_VERSION
    started_at: datetime | None = None
    ended_at: datetime | None = None
    failure_kind: FailureKind | None = None
    failure_reason: str | None = None
    initial_topology: TopologySnapshot | None = None
    console_identity: DeviceIdentity | None = None
    current_configuration: Mapping[str, object] | None = None
    configuration_criteria: tuple[CriterionResult, ...] = ()
    active_setting_checks: tuple[FinalSettingCheck, ...] = ()
    trigger_readbacks: tuple[SettingReadback, ...] = ()
    adc_reads: tuple[AdcReadEvidence, ...] = ()
    safety_observations: tuple[SafetyWarningEvidence, ...] = ()
    opt_calculation: SafetyLimitCalculation | None = None
    ee_calculation: SafetyLimitCalculation | None = None
    pulse_calculation: PulseLimitCalculation | None = None
    intended_configuration: Mapping[str, object] | None = None
    immediate_configuration_readback: Mapping[str, object] | None = None
    power_cycle: PowerCycleEvidence | None = None
    post_restart_configuration: Mapping[str, object] | None = None
    persistence_criteria: tuple[CriterionResult, ...] = ()
    normal_scan: NormalScanEvidence | None = None
    trigger_cleanup_failure: str | None = None
    resource_cleanup_failure: str | None = None
    events: tuple[ProcedureEvent, ...] = ()
    report_paths: tuple[Path | str, ...] = ()
    report_artifact: ReportArtifactEvidence | None = None

    def __post_init__(self) -> None:
        for name in (
            "current_configuration",
            "intended_configuration",
            "immediate_configuration_readback",
            "post_restart_configuration",
        ):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _deeply_immutable(value))
        for name in (
            "configuration_criteria",
            "active_setting_checks",
            "trigger_readbacks",
            "adc_reads",
            "safety_observations",
            "persistence_criteria",
            "events",
            "report_paths",
        ):
            object.__setattr__(self, name, tuple(getattr(self, name)))


class SafetyCalibrationBench(Protocol):
    def preflight_console(self) -> ConsolePreflightSnapshot: ...

    def read_user_configuration(self) -> Mapping[str, object]: ...

    def bring_up_laser_configuration(self) -> None: ...

    def read_register(self, name: str) -> float: ...

    def read_trigger_rate_hz(self) -> float: ...

    def write_trigger_rate_hz(self, rate_hz: float) -> SettingReadback | None: ...

    def start_trigger(self) -> None: ...

    def read_adc_ma(self, controller: SafetyController) -> float | None: ...

    def read_safety_warning(self) -> SafetyWarningEvidence: ...

    def stop_trigger(self) -> None: ...

    def write_user_configuration(
        self, configuration: Mapping[str, object]
    ) -> Mapping[str, object] | None: ...

    def power_cycle(
        self, *, minimum_off_s: float, expected_console_serial: str
    ) -> PowerCycleEvidence: ...

    def run_normal_scan(
        self, declared_topology: ShippingTopology, *, duration_s: float
    ) -> NormalScanEvidence: ...


class RunRecorder(Protocol):
    def record(self, event: ProcedureEvent) -> None: ...

    def checkpoint(self, result: SafetyCalibrationResult) -> None: ...


class SafetyCalibrationWorkflow:
    """Execute the complete fail-closed Safety Calibration procedure."""

    def __init__(
        self,
        bench: SafetyCalibrationBench,
        recorder: RunRecorder,
        *,
        sampling_policy: AdcSamplingPolicy | None = None,
        sleep_func: Callable[[float], None] = time.sleep,
        now_func: Callable[[], datetime] | None = None,
    ):
        self._bench = bench
        self._recorder = recorder
        self._sampling_policy = sampling_policy or AdcSamplingPolicy()
        self._sleep = sleep_func
        self._now = now_func or (lambda: datetime.now(timezone.utc))

    def run(self, request: SafetyCalibrationRequest) -> SafetyCalibrationResult:
        events: list[ProcedureEvent] = []
        preflight: ConsolePreflightSnapshot | None = None
        current_configuration: Mapping[str, object] | None = None
        configuration_criteria: list[CriterionResult] = []
        active_setting_checks: list[FinalSettingCheck] = []
        trigger_readbacks: list[SettingReadback] = []
        adc_reads: list[AdcReadEvidence] = []
        safety_observations: list[SafetyWarningEvidence] = []
        opt_calculation: SafetyLimitCalculation | None = None
        ee_calculation: SafetyLimitCalculation | None = None
        pulse_calculation: PulseLimitCalculation | None = None
        intended_configuration: Mapping[str, object] | None = None
        immediate_configuration_readback: Mapping[str, object] | None = None
        power_cycle: PowerCycleEvidence | None = None
        post_restart_configuration: Mapping[str, object] | None = None
        persistence_criteria: list[CriterionResult] = []
        normal_scan: NormalScanEvidence | None = None
        trigger_cleanup_failure: str | None = None
        failure: _ProcedureFailure | None = None

        def snapshot(
            status: ProcedureStatus = ProcedureStatus.IN_PROGRESS,
            *,
            failure_kind: FailureKind | None = None,
            failure_reason: str | None = None,
            ended_at: datetime | None = None,
        ) -> SafetyCalibrationResult:
            return SafetyCalibrationResult(
                status=status,
                shipping_topology=request.shipping_topology,
                sdk_version=request.sdk_version,
                started_at=request.started_at,
                ended_at=ended_at,
                failure_kind=failure_kind,
                failure_reason=failure_reason,
                initial_topology=preflight.topology if preflight else None,
                console_identity=preflight.console_identity if preflight else None,
                current_configuration=current_configuration,
                configuration_criteria=tuple(configuration_criteria),
                active_setting_checks=tuple(active_setting_checks),
                trigger_readbacks=tuple(trigger_readbacks),
                adc_reads=tuple(adc_reads),
                safety_observations=tuple(safety_observations),
                opt_calculation=opt_calculation,
                ee_calculation=ee_calculation,
                pulse_calculation=pulse_calculation,
                intended_configuration=intended_configuration,
                immediate_configuration_readback=immediate_configuration_readback,
                power_cycle=power_cycle,
                post_restart_configuration=post_restart_configuration,
                persistence_criteria=tuple(persistence_criteria),
                normal_scan=normal_scan,
                trigger_cleanup_failure=trigger_cleanup_failure,
                events=tuple(events),
            )

        def checkpoint() -> None:
            self._recorder.checkpoint(snapshot())

        def stage(label: str, message: str) -> None:
            event = ProcedureEvent(self._now(), label, message)
            events.append(event)
            self._recorder.record(event)
            checkpoint()

        try:
            if not isinstance(request.shipping_topology, ShippingTopology):
                raise _ProcedureFailure(
                    FailureKind.SETUP,
                    "Declared shipping topology must be console-only, "
                    "single-left, single-right, or dual.",
                )
            stage(
                "1. Console-only safety calibration preflight",
                "Confirm the console is connected, responsive, and uniquely identified. "
                "Sensor modules are not required for ADC acquisition.",
            )
            try:
                preflight = self._bench.preflight_console()
            except Exception as exc:
                raise _ProcedureFailure(
                    FailureKind.SETUP, f"Console preflight failed: {exc}"
                ) from exc
            checkpoint()
            self._validate_preflight(preflight)

            stage(
                "2. Persisted operating configuration verification",
                "Read the complete persisted User Configuration and verify its active "
                "TA operating point before firing.",
            )
            try:
                current_configuration = dict(self._bench.read_user_configuration())
            except Exception as exc:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Current User Configuration could not be read: {exc}",
                ) from exc
            checkpoint()
            configuration_criteria.extend(
                validate_current_configuration(current_configuration)
            )
            checkpoint()
            failed_criteria = [item for item in configuration_criteria if not item.passed]
            if failed_criteria:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    "Current User Configuration is invalid: "
                    + "; ".join(item.detail for item in failed_criteria),
                )

            try:
                self._bench.bring_up_laser_configuration()
            except Exception as exc:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Persisted laser configuration could not be activated: {exc}",
                ) from exc

            for name in ("TA_CURRENT_DRV", "TA_PULSE_WIDTH"):
                requested = current_configuration[name]
                try:
                    actual = self._bench.read_register(name)
                except Exception as exc:
                    raise _ProcedureFailure(
                        FailureKind.CONFIGURATION,
                        f"Active {name} readback failed: {exc}",
                    ) from exc
                check = self._setting_check(name, requested, actual)
                active_setting_checks.append(check)
                checkpoint()
                if not check.passed:
                    raise _ProcedureFailure(
                        FailureKind.CONFIGURATION,
                        f"Active {name} is outside the inclusive +/-2% tolerance.",
                    )
            self._verify_trigger(trigger_readbacks, checkpoint)

            stage(
                "3. Safety-controller ADC acquisition",
                "Fire once and collect bounded, interleaved scaled-mA readings from "
                "SAFETY_OPT and SAFETY_EE with safety-state evidence.",
            )
            (
                opt_samples,
                ee_samples,
                trigger_cleanup_failure,
                acquisition_failure,
            ) = self._acquire_adc(adc_reads, safety_observations, checkpoint)
            if acquisition_failure is not None:
                if trigger_cleanup_failure is not None:
                    raise _ProcedureFailure(
                        acquisition_failure.kind,
                        f"{acquisition_failure.reason}; trigger cleanup also failed: "
                        f"{trigger_cleanup_failure}",
                    )
                raise acquisition_failure
            if trigger_cleanup_failure is not None:
                raise _ProcedureFailure(
                    FailureKind.MEASUREMENT,
                    f"Laser trigger stop failed after ADC acquisition: "
                    f"{trigger_cleanup_failure}",
                )
            faults = self._known_faults(safety_observations)
            if faults:
                raise _ProcedureFailure(
                    FailureKind.MEASUREMENT,
                    "Laser safety fault observed during ADC acquisition: "
                    + ", ".join(faults),
                )
            if not any(item.safety_known for item in safety_observations):
                raise _ProcedureFailure(
                    FailureKind.MEASUREMENT,
                    "ADC firing has no known laser safety telemetry observation.",
                )
            if len(opt_samples) < self._sampling_policy.minimum_valid_samples:
                raise _ProcedureFailure(
                    FailureKind.MEASUREMENT,
                    f"SAFETY_OPT produced fewer than "
                    f"{self._sampling_policy.minimum_valid_samples} valid scaled-mA samples "
                    f"({len(opt_samples)} accepted).",
                )
            if len(ee_samples) < self._sampling_policy.minimum_valid_samples:
                raise _ProcedureFailure(
                    FailureKind.MEASUREMENT,
                    f"SAFETY_EE produced fewer than "
                    f"{self._sampling_policy.minimum_valid_samples} valid scaled-mA samples "
                    f"({len(ee_samples)} accepted).",
                )
            stage(
                "4. Derived safety-limit calculation",
                "Calculate and round both current limits and the paired pulse-width "
                "limits using the approved explicit tie rule.",
            )
            opt_calculation = calculate_safety_limit(
                "SAFETY_OPT", opt_samples, SAFETY_OPT_MULTIPLIER
            )
            checkpoint()
            ee_calculation = calculate_safety_limit(
                "SAFETY_EE", ee_samples, SAFETY_EE_MULTIPLIER
            )
            checkpoint()
            pulse_calculation = calculate_pulse_limits(
                current_configuration["TA_PULSE_WIDTH"],
                current_ee_limit_us=current_configuration["EE_PULSE_WIDTH_UL"],
                current_opt_limit_us=current_configuration["OPT_PULSE_WIDTH_UL"],
            )
            checkpoint()

            intended_configuration = dict(current_configuration)
            intended_configuration.update(
                {
                    "OPT_DRIVE_CL": opt_calculation.rounded_limit_ma,
                    "EE_DRIVE_CL": ee_calculation.rounded_limit_ma,
                    "OPT_PULSE_WIDTH_UL": pulse_calculation.opt_pulse_width_ul_us,
                    "EE_PULSE_WIDTH_UL": pulse_calculation.ee_pulse_width_ul_us,
                }
            )
            checkpoint()

            stage(
                "5. Complete configuration write and immediate verification",
                "Write the complete intended User Configuration and compare every "
                "immediate readback key and value.",
            )
            try:
                write_result = self._bench.write_user_configuration(
                    intended_configuration
                )
                if not isinstance(write_result, Mapping):
                    raise ValueError("the checked write did not return a mapping")
                immediate_configuration_readback = dict(write_result)
            except Exception as exc:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Complete User Configuration write failed: {exc}",
                ) from exc
            checkpoint()
            if dict(immediate_configuration_readback) != dict(intended_configuration):
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    "Immediate complete User Configuration readback did not exactly "
                    "match the intended configuration.",
                )

            stage(
                "6. Measured power-cycle persistence verification",
                "Observe console disconnection, keep power off for at least "
                f"{MINIMUM_POWER_OFF_S:g} measured second(s), prove restart, and "
                "verify the complete persisted configuration.",
            )
            try:
                power_cycle = self._bench.power_cycle(
                    minimum_off_s=MINIMUM_POWER_OFF_S,
                    expected_console_serial=preflight.console_identity.serial.strip(),
                )
            except Exception as exc:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION, f"Power-cycle operation failed: {exc}"
                ) from exc
            checkpoint()
            self._validate_power_cycle(power_cycle, preflight.console_identity.serial)
            try:
                post_restart_configuration = dict(
                    self._bench.read_user_configuration()
                )
            except Exception as exc:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Post-restart User Configuration could not be read: {exc}",
                ) from exc
            checkpoint()
            persistence_matches = (
                dict(post_restart_configuration) == dict(intended_configuration)
            )
            persistence_criteria.append(
                CriterionResult(
                    "complete_post_restart_configuration",
                    persistence_matches,
                    "Post-restart complete User Configuration must exactly match the "
                    "intended configuration.",
                )
            )
            checkpoint()
            if not persistence_matches:
                raise _ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    "Post-restart complete User Configuration did not exactly match "
                    "the intended configuration.",
                )

            if request.shipping_topology is ShippingTopology.CONSOLE_ONLY:
                stage(
                    "7. Normal 30-second scan not applicable",
                    "The console-only declaration has no sensor modules, so the "
                    "production sensor-data path cannot and need not run.",
                )
                checkpoint()
            else:
                stage(
                    "7. Normal 30-second scan with persisted values",
                    "Run the ordinary production sensor-data path in the declared "
                    "shipping topology, with persisted values active and no overrides.",
                )
                try:
                    normal_scan = self._bench.run_normal_scan(
                        request.shipping_topology, duration_s=30.0
                    )
                except Exception as exc:
                    raise _ProcedureFailure(
                        FailureKind.MEASUREMENT,
                        f"Normal 30-second scan failed: {exc}",
                    ) from exc
                checkpoint()
                self._validate_normal_scan(normal_scan, request.shipping_topology)

            stage(
                "8. Procedure completion",
                "All Safety Calibration gates passed with persisted values active.",
            )
        except _ProcedureFailure as exc:
            failure = exc
        except Exception as exc:
            failure = _ProcedureFailure(
                FailureKind.MEASUREMENT,
                f"Unexpected Safety Calibration failure: {exc}",
            )

        if failure is not None:
            result = snapshot(
                ProcedureStatus.FAILED,
                failure_kind=failure.kind,
                failure_reason=failure.reason,
                ended_at=self._now(),
            )
        else:
            result = snapshot(ProcedureStatus.PASSED, ended_at=self._now())
        self._recorder.checkpoint(result)
        return result

    @staticmethod
    def _validate_preflight(preflight: ConsolePreflightSnapshot) -> None:
        if not preflight.topology.console_connected:
            raise _ProcedureFailure(FailureKind.SETUP, "Console is not connected.")
        if not preflight.console_responsive:
            raise _ProcedureFailure(
                FailureKind.SETUP, "Console did not pass the responsiveness check."
            )
        if not validate_serial(preflight.console_identity.serial).passed:
            raise _ProcedureFailure(
                FailureKind.SETUP, "Console serial number is missing or blank."
            )
        fpga_revisions = validate_console_fpga_revisions(
            preflight.console_identity
        )
        if not fpga_revisions.passed:
            raise _ProcedureFailure(FailureKind.SETUP, fpga_revisions.detail)

    @staticmethod
    def _setting_check(name: str, requested: object, actual: object) -> FinalSettingCheck:
        requested_number = float(requested) if _finite_number(requested) else math.nan
        actual_number = float(actual) if _finite_number(actual) else math.nan
        difference = (
            abs(actual_number - requested_number)
            if math.isfinite(requested_number) and math.isfinite(actual_number)
            else math.nan
        )
        return FinalSettingCheck(
            name=name,
            requested=requested_number,
            actual=actual_number,
            absolute_difference=difference,
            percent_difference=percent_difference(requested_number, actual_number),
            tolerance_percent=2.0,
            passed=within_percent(requested_number, actual_number, 2.0),
        )

    def _verify_trigger(
        self,
        readbacks: list[SettingReadback],
        checkpoint: Callable[[], None],
    ) -> None:
        try:
            initial = self._bench.read_trigger_rate_hz()
        except Exception as exc:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                f"Active trigger rate could not be read: {exc}",
            ) from exc
        readbacks.append(SettingReadback("trigger_rate_hz_initial", 40.0, initial))
        checkpoint()
        if _finite_number(initial) and float(initial) == 40.0:
            return
        try:
            write_result = self._bench.write_trigger_rate_hz(40.0)
        except Exception as exc:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                f"Trigger rate correction failed: {exc}",
            ) from exc
        if not self._valid_trigger_write(write_result):
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Trigger rate correction returned malformed immediate write evidence.",
            )
        readbacks.append(write_result)
        checkpoint()
        try:
            final_rate = self._bench.read_trigger_rate_hz()
        except Exception as exc:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                f"Corrected trigger rate could not be read: {exc}",
            ) from exc
        readbacks.append(SettingReadback("trigger_rate_hz_final", 40.0, final_rate))
        checkpoint()
        if not self._valid_trigger_rate(final_rate):
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Active trigger rate is outside 39 to 41 Hz after correction.",
            )

    @staticmethod
    def _valid_trigger_rate(value: object) -> bool:
        return _finite_number(value) and 39.0 <= float(value) <= 41.0

    @classmethod
    def _valid_trigger_write(cls, value: object) -> bool:
        return (
            isinstance(value, SettingReadback)
            and value.name == "trigger_rate_hz_write"
            and value.requested == 40.0
            and cls._valid_trigger_rate(value.actual)
        )

    def _acquire_adc(
        self,
        reads: list[AdcReadEvidence],
        warnings: list[SafetyWarningEvidence],
        checkpoint: Callable[[], None],
    ) -> tuple[
        list[float],
        list[float],
        str | None,
        _ProcedureFailure | None,
    ]:
        samples = {"SAFETY_OPT": [], "SAFETY_EE": []}
        trigger_attempted = False
        cleanup_failure: str | None = None
        acquisition_failure: _ProcedureFailure | None = None
        try:
            trigger_attempted = True
            self._bench.start_trigger()
            for attempt in range(1, self._sampling_policy.maximum_attempts_per_controller + 1):
                for controller in ("SAFETY_OPT", "SAFETY_EE"):
                    if len(samples[controller]) >= self._sampling_policy.minimum_valid_samples:
                        continue
                    read = self._read_adc_once(controller, attempt)
                    reads.append(read)
                    if read.accepted:
                        samples[controller].append(read.value_ma)
                    checkpoint()
                try:
                    warning = self._bench.read_safety_warning()
                    if not isinstance(warning, SafetyWarningEvidence):
                        raise TypeError("warning source returned invalid evidence")
                    warnings.append(warning)
                    checkpoint()
                except Exception as exc:
                    acquisition_failure = _ProcedureFailure(
                        FailureKind.MEASUREMENT,
                        f"Safety warning state could not be read during ADC acquisition: {exc}",
                    )
                    break
                if warning.safety_known and not warning.safety_ok:
                    break
                if all(
                    len(values) >= self._sampling_policy.minimum_valid_samples
                    for values in samples.values()
                ):
                    break
                if self._sampling_policy.interval_s:
                    self._sleep(self._sampling_policy.interval_s)
        except Exception as exc:
            acquisition_failure = _ProcedureFailure(
                FailureKind.MEASUREMENT,
                f"ADC firing or acquisition failed: {exc}",
            )
        finally:
            if trigger_attempted:
                try:
                    self._bench.stop_trigger()
                except Exception as exc:
                    cleanup_failure = str(exc) or type(exc).__name__
            checkpoint()
        return (
            samples["SAFETY_OPT"],
            samples["SAFETY_EE"],
            cleanup_failure,
            acquisition_failure,
        )

    def _read_adc_once(
        self, controller: SafetyController, attempt: int
    ) -> AdcReadEvidence:
        timestamp = self._now()
        try:
            value = self._bench.read_adc_ma(controller)
        except Exception as exc:
            return AdcReadEvidence(
                controller,
                attempt,
                timestamp,
                False,
                None,
                f"{type(exc).__name__}: {exc}",
            )
        if not _finite_number(value):
            return AdcReadEvidence(
                controller,
                attempt,
                timestamp,
                False,
                None,
                "Read must be a finite scaled-mA number and not a boolean.",
            )
        if float(value) < 0:
            return AdcReadEvidence(
                controller,
                attempt,
                timestamp,
                False,
                None,
                "Scaled-mA read must be nonnegative.",
            )
        return AdcReadEvidence(
            controller, attempt, timestamp, True, float(value), None
        )

    @staticmethod
    def _known_faults(warnings: list[SafetyWarningEvidence]) -> tuple[str, ...]:
        faults: list[str] = []
        for warning in warnings:
            if warning.safety_known and (not warning.safety_ok or warning.faults):
                faults.extend(warning.faults or ("unspecified laser safety fault",))
        return tuple(dict.fromkeys(faults))

    @staticmethod
    def _validate_power_cycle(
        evidence: PowerCycleEvidence, expected_serial: str | None
    ) -> None:
        if not isinstance(evidence, PowerCycleEvidence):
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION, "Power-cycle evidence is missing."
            )
        if not evidence.disconnect_observed:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Power cycle did not prove console disconnection.",
            )
        if (
            not _finite_number(evidence.off_duration_s)
            or float(evidence.off_duration_s) < MINIMUM_POWER_OFF_S
        ):
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Power-off dwell was not at least "
                f"{MINIMUM_POWER_OFF_S:g} measured second(s) - the console was "
                "cycled too quickly for the dwell to be provable.",
            )
        if not evidence.reconnect_observed:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Power cycle did not prove console reconnection.",
            )
        if not evidence.restart_proven or not evidence.restart_proof:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Power cycle did not include acceptable restart proof.",
            )
        timestamps = (
            evidence.off_requested_at,
            evidence.disconnect_observed_at,
            evidence.on_allowed_at,
            evidence.on_requested_at,
            evidence.reconnect_observed_at,
        )
        try:
            timestamps_valid = all(
                isinstance(timestamp, datetime) for timestamp in timestamps
            ) and all(
                earlier <= later
                for earlier, later in zip(timestamps, timestamps[1:])
            )
            dwell_timestamps_valid = (
                timestamps_valid
                and (evidence.on_allowed_at - evidence.disconnect_observed_at)
                .total_seconds()
                >= MINIMUM_POWER_OFF_S
            )
        except (TypeError, ValueError):
            dwell_timestamps_valid = False
        if not dwell_timestamps_valid:
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Power-cycle timestamp evidence is missing, out of order, or does "
                f"not show the complete {MINIMUM_POWER_OFF_S:g}-second off dwell.",
            )
        expected = expected_serial.strip() if isinstance(expected_serial, str) else None
        if (
            not expected
            or evidence.console_serial_before != expected
            or evidence.console_serial_after != expected
        ):
            raise _ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Console serial changed or was missing across the power cycle.",
            )

    @classmethod
    def _validate_normal_scan(
        cls,
        scan: NormalScanEvidence,
        declared_topology: ShippingTopology,
    ) -> None:
        if not isinstance(scan, NormalScanEvidence):
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT, "Normal scan evidence is missing."
            )
        if not scan.started:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT, "Normal scan did not start."
            )
        if scan.error:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT, f"Normal scan error: {scan.error}"
            )
        if scan.canceled:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT, "Normal scan was canceled."
            )
        if not scan.completed:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT, "Normal scan did not complete normally."
            )
        if (
            not _finite_number(scan.actual_duration_s)
            or scan.actual_duration_s < 30.0
        ):
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Normal scan did not run for at least 30 measured seconds.",
            )
        if (
            not _finite_number(scan.requested_duration_s)
            or float(scan.requested_duration_s) != 30.0
        ):
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Normal scan evidence did not preserve the requested 30-second duration.",
            )
        if scan.declared_topology is not declared_topology:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Normal scan topology declaration changed during execution.",
            )
        identities = {identity.role.lower(): identity for identity in scan.identities}
        left = next(
            (identity for role, identity in identities.items() if role.startswith("left")),
            DeviceIdentity("left sensor", None, None, None),
        )
        right = next(
            (identity for role, identity in identities.items() if role.startswith("right")),
            DeviceIdentity("right sensor", None, None, None),
        )
        topology_result = validate_shipping_topology(
            declared_topology,
            scan.topology,
            left_identity=left,
            right_identity=right,
        )
        if not topology_result.passed:
            raise _ProcedureFailure(FailureKind.MEASUREMENT, topology_result.detail)
        if dict(scan.overrides):
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Normal scan included a prohibited configuration override.",
            )
        known = [item for item in scan.safety_observations if item.safety_known]
        if not known:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Normal scan has no known laser safety telemetry observation.",
            )
        faults = cls._known_faults(list(scan.safety_observations))
        if faults:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Laser safety fault observed during the normal scan: "
                + ", ".join(faults),
            )
        if scan.warnings:
            raise _ProcedureFailure(
                FailureKind.MEASUREMENT,
                "Laser safety warning observed during the normal scan: "
                + ", ".join(scan.warnings),
            )
