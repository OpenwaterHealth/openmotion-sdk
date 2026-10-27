"""UI-neutral WI-00015 dual-sensor laser calibration workflow."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Callable, Mapping, Protocol

try:
    from omotion import __version__ as _RUNTIME_SDK_VERSION
except (ImportError, AttributeError):
    _RUNTIME_SDK_VERSION = "unavailable"

from .laser import (
    CURRENT_FLOOR_MA,
    CURRENT_STEP_MA,
    MAX_ACCEPTABLE_ENERGY_UJ,
    MAX_PULSE_WIDTH_US,
    MIN_ACCEPTABLE_ENERGY_UJ,
    PULSE_WIDTH_STEP_US,
    TARGET_ENERGY_UJ,
    TEMPORARY_PULSE_WIDTH_LIMIT_US,
    CriterionResult,
    DeviceIdentity,
    EnergyMeasurement,
    FailureKind,
    FinalSettingCheck,
    OphirIdentity,
    PairMetrics,
    ProcedureStatus,
    SensorSide,
    SettingReadback,
    TopologySnapshot,
    both_energies_accepted,
    calculate_pair_metrics,
    select_closest_valid_setting_to_target,
    validate_energy_measurement,
    validate_exact_dual_topology,
)
from ._procedure import (
    LaserBench,
    LaserWorkflowBase,
    OphirSettingEvidence,
    ProcedureEvent,
    ProcedureFailure,
    ReportArtifactEvidence,
    RunRecorder,
    deeply_immutable,
)
from .override import (
    OverrideConsentFn,
    OverrideDecision,
    OverrideRequest,
    OverrideSettings,
    factory_band_description,
    within_factory_band,
)


@dataclass(frozen=True)
class DualSensorLaserCalibrationRequest:
    operator: str
    build_id: str
    fixture_id: str
    procedure_id: str
    output_root: Path | str
    run_id: str
    fixture_calibration_status: str | None = None
    sdk_version: str = _RUNTIME_SDK_VERSION
    started_at: datetime = field(default_factory=lambda: datetime.now(timezone.utc))


@dataclass(frozen=True)
class DualPreflightSnapshot:
    topology: TopologySnapshot
    console_identity: DeviceIdentity
    left_sensor_identity: DeviceIdentity
    right_sensor_identity: DeviceIdentity
    console_responsive: bool
    ophir_identity: OphirIdentity | None
    ophir_ready: bool
    ophir_setting_evidence: tuple[OphirSettingEvidence, ...]
    ophir_failure_reason: str | None = None


@dataclass(frozen=True)
class PlacementChangeRequest:
    from_side: SensorSide | None
    to_side: SensorSide
    sensor_serial: str
    phase: str
    label: str


@dataclass(frozen=True)
class PlacementAcknowledgement:
    request: PlacementChangeRequest
    acknowledged: bool
    timestamp: datetime


@dataclass(frozen=True)
class SensorEnergyObservation:
    side: SensorSide
    sensor_serial: str
    label: str
    measurement: EnergyMeasurement
    criteria: tuple[CriterionResult, ...]


@dataclass(frozen=True)
class PairObservation:
    label: str
    left: SensorEnergyObservation
    right: SensorEnergyObservation
    metrics: PairMetrics


@dataclass(frozen=True)
class TuningStep:
    number: int
    label: str
    side: SensorSide
    register_name: str
    requested_value: float
    readback: SettingReadback
    observation: SensorEnergyObservation


@dataclass(frozen=True)
class TuningSelection:
    direction: str
    selected_side: SensorSide | None
    target_uj: float
    requested_current_ma: float
    requested_pulse_width_us: float
    selected_mean_uj: float
    rationale: str


@dataclass(frozen=True)
class TuningRound:
    number: int
    label: str
    input_pair: PairObservation
    direction: str
    selected_side: SensorSide | None
    reason: str
    target_uj: float
    steps: tuple[TuningStep, ...]
    selection: TuningSelection | None


@dataclass(frozen=True)
class CrossCheck:
    number: int
    label: str
    pair: PairObservation
    accepted: bool


@dataclass(frozen=True)
class DualSensorLaserCalibrationResult:
    status: ProcedureStatus
    sdk_version: str = _RUNTIME_SDK_VERSION
    started_at: datetime | None = None
    ended_at: datetime | None = None
    failure_kind: FailureKind | None = None
    failure_reason: str | None = None
    topology: TopologySnapshot | None = None
    topology_revalidation: TopologySnapshot | None = None
    identities: tuple[DeviceIdentity, ...] = ()
    ophir_identity: OphirIdentity | None = None
    ophir_setting_evidence: tuple[OphirSettingEvidence, ...] = ()
    pre_existing_config: Mapping[str, float] | None = None
    requested_default_config: Mapping[str, float] | None = None
    default_config_readback: Mapping[str, float] | None = None
    configurations: tuple[SettingReadback, ...] = ()
    placements: tuple[PlacementAcknowledgement, ...] = ()
    observations: tuple[SensorEnergyObservation, ...] = ()
    initial_pair: PairObservation | None = None
    tuning_rounds: tuple[TuningRound, ...] = ()
    crosschecks: tuple[CrossCheck, ...] = ()
    adjustments: tuple[SettingReadback, ...] = ()
    requested_final_config: Mapping[str, float] | None = None
    final_config_readback: Mapping[str, float] | None = None
    final_setting_checks: tuple[FinalSettingCheck, ...] = ()
    active_default_restore: tuple[SettingReadback, ...] = ()
    active_default_restore_failure: str | None = None
    trigger_cleanup_failure: str | None = None
    resource_cleanup_failure: str | None = None
    events: tuple[ProcedureEvent, ...] = ()
    target_energy_uj: float = TARGET_ENERGY_UJ
    minimum_accepted_energy_uj: float = MIN_ACCEPTABLE_ENERGY_UJ
    maximum_accepted_energy_uj: float = MAX_ACCEPTABLE_ENERGY_UJ
    # Override mode (omotion.calibration.override): the settings the run was
    # started with and, when the operator was asked, the decision. Both stay
    # None outside override mode.
    override: OverrideSettings | None = None
    override_decision: OverrideDecision | None = None
    report_paths: tuple[Path | str, ...] = ()
    report_artifact: ReportArtifactEvidence | None = None

    def __post_init__(self) -> None:
        for name in (
            "pre_existing_config",
            "requested_default_config",
            "default_config_readback",
            "requested_final_config",
            "final_config_readback",
        ):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, deeply_immutable(value))


class DualLaserCalibrationBench(LaserBench, Protocol):
    def preflight_dual(self) -> DualPreflightSnapshot: ...

    def revalidate_dual_topology(self) -> TopologySnapshot: ...


DualRunRecorder = RunRecorder

PlacementCallback = Callable[[PlacementChangeRequest], bool]


@dataclass
class _RunState:
    request: DualSensorLaserCalibrationRequest
    target_energy_uj: float = TARGET_ENERGY_UJ
    minimum_accepted_energy_uj: float = MIN_ACCEPTABLE_ENERGY_UJ
    maximum_accepted_energy_uj: float = MAX_ACCEPTABLE_ENERGY_UJ
    override: OverrideSettings | None = None
    override_decision: OverrideDecision | None = None
    preflight: DualPreflightSnapshot | None = None
    topology_revalidation: TopologySnapshot | None = None
    pre_existing_config: Mapping[str, float] | None = None
    requested_default_config: Mapping[str, float] | None = None
    default_config_readback: Mapping[str, float] | None = None
    configurations: list[SettingReadback] = field(default_factory=list)
    placements: list[PlacementAcknowledgement] = field(default_factory=list)
    observations: list[SensorEnergyObservation] = field(default_factory=list)
    initial_pair: PairObservation | None = None
    tuning_rounds: list[TuningRound] = field(default_factory=list)
    crosschecks: list[CrossCheck] = field(default_factory=list)
    adjustments: list[SettingReadback] = field(default_factory=list)
    requested_final_config: Mapping[str, float] | None = None
    final_config_readback: Mapping[str, float] | None = None
    final_setting_checks: list[FinalSettingCheck] = field(default_factory=list)
    active_default_restore: list[SettingReadback] = field(default_factory=list)
    active_default_restore_failure: str | None = None
    trigger_cleanup_failure: str | None = None
    events: list[ProcedureEvent] = field(default_factory=list)
    current_requested_ma: float = 5000.0
    pulse_requested_us: float = 500.0
    used_upward_tuning: bool = False
    active_defaults_established: bool = False
    measurement_started: bool = False

    def result(
        self,
        status: ProcedureStatus,
        *,
        failure: ProcedureFailure | None = None,
        ended_at: datetime | None = None,
    ) -> DualSensorLaserCalibrationResult:
        preflight = self.preflight
        identities = (
            (
                preflight.console_identity,
                preflight.left_sensor_identity,
                preflight.right_sensor_identity,
            )
            if preflight
            else ()
        )
        return DualSensorLaserCalibrationResult(
            status=status,
            target_energy_uj=self.target_energy_uj,
            minimum_accepted_energy_uj=self.minimum_accepted_energy_uj,
            maximum_accepted_energy_uj=self.maximum_accepted_energy_uj,
            override=self.override,
            override_decision=self.override_decision,
            sdk_version=self.request.sdk_version,
            started_at=self.request.started_at,
            ended_at=ended_at,
            failure_kind=failure.kind if failure else None,
            failure_reason=failure.reason if failure else None,
            topology=preflight.topology if preflight else None,
            topology_revalidation=self.topology_revalidation,
            identities=identities,
            ophir_identity=preflight.ophir_identity if preflight else None,
            ophir_setting_evidence=(
                preflight.ophir_setting_evidence if preflight else ()
            ),
            pre_existing_config=self.pre_existing_config,
            requested_default_config=self.requested_default_config,
            default_config_readback=self.default_config_readback,
            configurations=tuple(self.configurations),
            placements=tuple(self.placements),
            observations=tuple(self.observations),
            initial_pair=self.initial_pair,
            tuning_rounds=tuple(self.tuning_rounds),
            crosschecks=tuple(self.crosschecks),
            adjustments=tuple(self.adjustments),
            requested_final_config=self.requested_final_config,
            final_config_readback=self.final_config_readback,
            final_setting_checks=tuple(self.final_setting_checks),
            active_default_restore=tuple(self.active_default_restore),
            active_default_restore_failure=self.active_default_restore_failure,
            trigger_cleanup_failure=self.trigger_cleanup_failure,
            events=tuple(self.events),
        )


class DualSensorLaserCalibrationWorkflow(LaserWorkflowBase):
    """Run the approved two-sensor midpoint calibration without owning UI."""

    _MAX_CROSSCHECKS = 3

    def __init__(
        self,
        bench: DualLaserCalibrationBench,
        recorder: DualRunRecorder,
        placement_callback: PlacementCallback,
        *,
        target_energy_uj: float = TARGET_ENERGY_UJ,
        minimum_accepted_energy_uj: float = MIN_ACCEPTABLE_ENERGY_UJ,
        maximum_accepted_energy_uj: float = MAX_ACCEPTABLE_ENERGY_UJ,
        override: OverrideSettings | None = None,
        override_consent: OverrideConsentFn | None = None,
    ):
        self._configure_override(override, override_consent)
        if override is not None:
            # Override mode: the operator's band and target replace the
            # injected/default window for this run.
            target_energy_uj = override.target_energy_uj
            minimum_accepted_energy_uj = override.minimum_energy_uj
            maximum_accepted_energy_uj = override.maximum_energy_uj
        target_energy_uj = float(target_energy_uj)
        minimum_accepted_energy_uj = float(minimum_accepted_energy_uj)
        maximum_accepted_energy_uj = float(maximum_accepted_energy_uj)
        if not (
            all(
                math.isfinite(value)
                for value in (
                    target_energy_uj,
                    minimum_accepted_energy_uj,
                    maximum_accepted_energy_uj,
                )
            )
            and minimum_accepted_energy_uj
            <= target_energy_uj
            <= maximum_accepted_energy_uj
        ):
            raise ValueError(
                "target and acceptance bounds must be finite with "
                "minimum <= target <= maximum"
            )
        self._bench = bench
        self._recorder = recorder
        self._placement_callback = placement_callback
        self._target_energy_uj = target_energy_uj
        self._minimum_accepted_energy_uj = minimum_accepted_energy_uj
        self._maximum_accepted_energy_uj = maximum_accepted_energy_uj
        self._seated_side: SensorSide | None = None

    def run(
        self, request: DualSensorLaserCalibrationRequest
    ) -> DualSensorLaserCalibrationResult:
        self._seated_side = None
        state = _RunState(
            request=request,
            target_energy_uj=self._target_energy_uj,
            minimum_accepted_energy_uj=self._minimum_accepted_energy_uj,
            maximum_accepted_energy_uj=self._maximum_accepted_energy_uj,
            override=self._override,
        )
        failure: ProcedureFailure | None = None
        stage = "setup"
        try:
            self._preflight(state)
            stage = "configuration"
            self._establish_defaults(
                state,
                revalidate=self._bench.revalidate_dual_topology,
                validate_topology=validate_exact_dual_topology,
                revalidation_failure=(
                    "Dual topology revalidation failed before configuration mutation."
                ),
                topology_changed_failure=(
                    "Exact dual-sensor topology changed before configuration mutation."
                ),
            )
            stage = "measurement"
            state.measurement_started = True
            state.initial_pair = self._measure_pair(
                state,
                phase="Initial paired measurement",
                pair_label="Initial paired measurement — paired baseline",
            )
            self._checkpoint(state)
            if state.initial_pair.metrics.difference_uj > 100.0:
                self._record_event(
                    state,
                    "initial_differential",
                    "Initial differential gate — NCR because the paired differential exceeded 100 uJ.",
                )
                raise ProcedureFailure(
                    FailureKind.NCR,
                    "Initial left/right energy differential exceeded 100 uJ.",
                )
            self._record_event(
                state,
                "initial_differential",
                "Initial differential gate — accepted because the paired differential was 100 uJ or below.",
            )
            latest_pair = state.initial_pair
            for crosscheck_number in range(1, self._MAX_CROSSCHECKS + 1):
                tuning_round = self._tune_from_pair(
                    state, latest_pair, crosscheck_number
                )
                self._record_event(state, "tuning", tuning_round.label)
                self._checkpoint(state)

                pair = self._measure_pair(
                    state,
                    phase=f"Cross-check {crosscheck_number}",
                    pair_label=f"Cross-check {crosscheck_number} — paired verification result",
                )
                accepted = both_energies_accepted(
                    pair.metrics.left_mean_uj,
                    pair.metrics.right_mean_uj,
                    self._minimum_accepted_energy_uj,
                    self._maximum_accepted_energy_uj,
                )
                crosscheck = CrossCheck(
                    number=crosscheck_number,
                    label=self._crosscheck_label(crosscheck_number, accepted),
                    pair=pair,
                    accepted=accepted,
                )
                state.crosschecks.append(crosscheck)
                self._record_event(state, "crosscheck", crosscheck.label)
                self._checkpoint(state)
                if accepted:
                    self._accept_pair(state, pair)
                    self._write_final_configuration(state)
                    break
                latest_pair = pair
            else:
                reason = (
                    "Both sensors were not within "
                    f"{self._minimum_accepted_energy_uj:g} to "
                    f"{self._maximum_accepted_energy_uj:g} uJ after three "
                    "complete cross-checks."
                )
                if self._override is None:
                    raise ProcedureFailure(FailureKind.NCR, reason)
                # Override mode: the third cross-check is the best this unit
                # reached; the operator decides against those numbers.
                decision = self._request_override(
                    state, self._pair_override_request(state, latest_pair, reason)
                )
                if not decision.accepted:
                    raise ProcedureFailure(FailureKind.NCR, reason)
                self._write_final_configuration(state)
        except ProcedureFailure as caught:
            failure = caught
        except Exception as error:
            if stage == "measurement":
                failure = ProcedureFailure(
                    FailureKind.MEASUREMENT,
                    str(error) or "Dual-sensor energy measurement failed.",
                )
            elif stage == "configuration":
                failure = ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    str(error) or "Dual-sensor configuration failed.",
                )
            else:
                failure = ProcedureFailure(
                    FailureKind.SETUP,
                    str(error) or "Dual-sensor bench preflight failed.",
                )
        finally:
            try:
                self._bench.stop_trigger()
            except Exception:
                state.trigger_cleanup_failure = "Trigger stop failed."

        if failure is None and state.trigger_cleanup_failure:
            failure = ProcedureFailure(
                FailureKind.MEASUREMENT, state.trigger_cleanup_failure
            )
        if failure is None:
            result = state.result(
                self._terminal_success_status(state),
                ended_at=datetime.now(timezone.utc),
            )
            self._recorder.checkpoint(result)
            return result

        assert failure is not None
        if state.active_defaults_established and state.measurement_started:
            self._restore_active_defaults(
                state,
                restored_message=(
                    "Active default laser registers were restored after the failed run."
                ),
            )
        if state.trigger_cleanup_failure:
            self._record_event(
                state, "trigger_cleanup", state.trigger_cleanup_failure
            )
        self._record_event(state, "failure", failure.reason)
        status = (
            ProcedureStatus.FAILED_NCR
            if failure.kind is FailureKind.NCR
            else ProcedureStatus.CANCELED
            if failure.kind is FailureKind.CANCELED
            else ProcedureStatus.FAILED
        )
        result = state.result(
            status, failure=failure, ended_at=datetime.now(timezone.utc)
        )
        self._recorder.checkpoint(result)
        return result

    def _preflight(self, state: _RunState) -> None:
        preflight = self._bench.preflight_dual()
        state.preflight = preflight
        self._record_event(state, "preflight", "Dual-sensor bench preflight completed.")
        self._checkpoint(state)
        topology = validate_exact_dual_topology(preflight.topology)
        if not topology.passed:
            raise ProcedureFailure(FailureKind.SETUP, topology.detail)
        self._validate_shared_preflight(
            preflight,
            (
                (preflight.console_identity, "Console"),
                (preflight.left_sensor_identity, "Left-sensor"),
                (preflight.right_sensor_identity, "Right-sensor"),
            ),
        )

    def _measure_pair(
        self, state: _RunState, *, phase: str, pair_label: str
    ) -> PairObservation:
        left = self._measure_side(state, "left", phase)
        right = self._measure_side(state, "right", phase)
        pair = PairObservation(
            label=pair_label,
            left=left,
            right=right,
            metrics=calculate_pair_metrics(
                left.measurement.mean_uj,
                right.measurement.mean_uj,
                self._target_energy_uj,
            ),
        )
        return pair

    def _measure_side(
        self, state: _RunState, side: SensorSide, phase: str
    ) -> SensorEnergyObservation:
        self._acknowledge_placement_if_needed(state, side, phase)
        label = f"{phase} — {side} sensor"
        try:
            measurement = self._bench.measure_energy()
        except Exception as error:
            raise ProcedureFailure(
                FailureKind.MEASUREMENT,
                f"{label} acquisition failed.",
            ) from error
        finally:
            try:
                self._bench.stop_trigger()
            except Exception as error:
                state.trigger_cleanup_failure = "Trigger stop failed."
                raise ProcedureFailure(
                    FailureKind.MEASUREMENT, state.trigger_cleanup_failure
                ) from error
        serial = self._sensor_serial(state, side)
        raw = SensorEnergyObservation(side, serial, label, measurement, ())
        state.observations.append(raw)
        self._checkpoint(state)
        criteria = validate_energy_measurement(measurement)
        observation = SensorEnergyObservation(
            side, serial, label, measurement, criteria
        )
        state.observations[-1] = observation
        self._checkpoint(state)
        if not all(item.passed for item in criteria):
            raise ProcedureFailure(
                FailureKind.MEASUREMENT,
                f"{label} failed the WI-00015 measurement-quality criteria.",
            )
        self._record_event(
            state,
            "measurement",
            f"{label} completed with valid Ophir evidence.",
        )
        return observation

    def _acknowledge_placement_if_needed(
        self, state: _RunState, side: SensorSide, phase: str
    ) -> None:
        if self._seated_side == side:
            return
        request = PlacementChangeRequest(
            from_side=self._seated_side,
            to_side=side,
            sensor_serial=self._sensor_serial(state, side),
            phase=phase,
            label=f"{phase} — place the {side} sensor in the Ophir 0 cm fixture",
        )
        try:
            response = self._placement_callback(request)
        except (EOFError, KeyboardInterrupt) as error:
            raise ProcedureFailure(
                FailureKind.CANCELED,
                f"Operator canceled the requested switch to the {side} sensor.",
            ) from error
        except Exception as error:
            raise ProcedureFailure(
                FailureKind.CANCELED,
                f"Placement acknowledgement failed for the {side} sensor.",
            ) from error
        acknowledgement = PlacementAcknowledgement(
            request=request,
            acknowledged=response is True,
            timestamp=datetime.now(timezone.utc),
        )
        state.placements.append(acknowledgement)
        self._checkpoint(state)
        if response is not True:
            raise ProcedureFailure(
                FailureKind.CANCELED,
                f"Operator did not confirm the requested switch to the {side} sensor.",
            )
        self._seated_side = side
        self._record_event(
            state,
            "placement",
            f"Operator confirmed {phase.lower()} placement for the {side} sensor "
            f"(serial {request.sensor_serial}).",
        )

    def _tune_from_pair(
        self, state: _RunState, pair: PairObservation, round_number: int
    ) -> TuningRound:
        midpoint = pair.metrics.midpoint_uj
        if midpoint == self._target_energy_uj:
            reason = (
                "the paired midpoint was already exactly "
                f"{self._target_energy_uj:g} uJ"
            )
            selection = TuningSelection(
                direction="none",
                selected_side=None,
                target_uj=self._target_energy_uj,
                requested_current_ma=state.current_requested_ma,
                requested_pulse_width_us=state.pulse_requested_us,
                selected_mean_uj=midpoint,
                rationale=(
                    "No setting changed because the paired midpoint was already exactly "
                    f"{self._target_energy_uj:g} uJ."
                ),
            )
            tuning_round = TuningRound(
                number=round_number,
                label=f"Midpoint adjustment round {round_number} — no setting changed because {reason}.",
                input_pair=pair,
                direction="none",
                selected_side=None,
                reason=reason,
                target_uj=self._target_energy_uj,
                steps=(),
                selection=selection,
            )
            state.tuning_rounds.append(tuning_round)
            self._checkpoint(state)
            return tuning_round
        if midpoint > self._target_energy_uj:
            return self._tune_downward(state, pair, round_number)
        return self._tune_upward(state, pair, round_number)

    def _tune_downward(
        self, state: _RunState, pair: PairObservation, round_number: int
    ) -> TuningRound:
        side: SensorSide = (
            "left"
            if pair.left.measurement.mean_uj >= pair.right.measurement.mean_uj
            else "right"
        )
        source = pair.left if side == "left" else pair.right
        target = self._target_energy_uj + pair.metrics.difference_uj / 2.0
        reason = f"the {side} sensor had the higher energy reading"
        tuning_round = TuningRound(
            number=round_number,
            label=(
                f"Midpoint adjustment round {round_number} — selected {side} sensor "
                f"because it had the higher energy reading; target {target:g} uJ."
            ),
            input_pair=pair,
            direction="downward_current",
            selected_side=side,
            reason=reason,
            target_uj=target,
            steps=(),
            selection=None,
        )
        state.tuning_rounds.append(tuning_round)
        self._checkpoint(state)
        candidates = [(state.current_requested_ma, source.measurement)]
        steps: list[TuningStep] = []
        active_setting = state.current_requested_ma
        while active_setting > CURRENT_FLOOR_MA:
            requested = max(CURRENT_FLOOR_MA, active_setting - CURRENT_STEP_MA)
            readback = self._checked_register_write(
                state, "TA_CURRENT_DRV", requested
            )
            observation = self._measure_side(
                state,
                side,
                f"Midpoint adjustment round {round_number}, step {len(steps) + 1}",
            )
            step = TuningStep(
                number=len(steps) + 1,
                label=(
                    f"Adjustment step {len(steps) + 1} — reduced TA current "
                    f"from {active_setting:g} mA to {requested:g} mA for the {side} sensor"
                ),
                side=side,
                register_name="TA_CURRENT_DRV",
                requested_value=requested,
                readback=readback,
                observation=observation,
            )
            steps.append(step)
            tuning_round = replace(tuning_round, steps=tuple(steps))
            state.tuning_rounds[-1] = tuning_round
            candidates.append((requested, observation.measurement))
            self._checkpoint(state)
            active_setting = requested
            if observation.measurement.mean_uj <= target:
                break
        selected = select_closest_valid_setting_to_target(candidates, target)
        if selected is None:
            raise ProcedureFailure(
                FailureKind.MEASUREMENT,
                "No valid downward-current candidate was available for selection.",
            )
        selected_setting, selected_measurement = selected
        if (
            active_setting == CURRENT_FLOOR_MA
            and not both_energies_accepted(
                selected_measurement.mean_uj,
                selected_measurement.mean_uj,
                self._minimum_accepted_energy_uj,
                self._maximum_accepted_energy_uj,
            )
        ):
            self._handle_out_of_band(
                state,
                "The current floor was reached without an acceptable "
                "selected-sensor setting.",
            )
        if selected_setting != active_setting:
            self._checked_register_write(
                state, "TA_CURRENT_DRV", selected_setting
            )
        state.current_requested_ma = selected_setting
        selection = TuningSelection(
            direction="downward_current",
            selected_side=side,
            target_uj=target,
            requested_current_ma=state.current_requested_ma,
            requested_pulse_width_us=state.pulse_requested_us,
            selected_mean_uj=selected_measurement.mean_uj,
            rationale=(
                f"Selected {selected_setting:g} mA because its valid {side}-sensor "
                f"reading was closest to the calculated {target:g} uJ target."
            ),
        )
        tuning_round = replace(tuning_round, selection=selection)
        state.tuning_rounds[-1] = tuning_round
        self._checkpoint(state)
        return tuning_round

    def _tune_upward(
        self, state: _RunState, pair: PairObservation, round_number: int
    ) -> TuningRound:
        side: SensorSide = (
            "left"
            if pair.left.measurement.mean_uj <= pair.right.measurement.mean_uj
            else "right"
        )
        source = pair.left if side == "left" else pair.right
        target = self._target_energy_uj - pair.metrics.difference_uj / 2.0
        reason = f"the {side} sensor had the lower energy reading"
        tuning_round = TuningRound(
            number=round_number,
            label=(
                f"Midpoint adjustment round {round_number} — selected {side} sensor "
                f"because it had the lower energy reading; target {target:g} uJ."
            ),
            input_pair=pair,
            direction="upward_pulse",
            selected_side=side,
            reason=reason,
            target_uj=target,
            steps=(),
            selection=None,
        )
        state.tuning_rounds.append(tuning_round)
        self._checkpoint(state)
        if (
            state.pulse_requested_us >= MAX_PULSE_WIDTH_US
            and source.measurement.mean_uj < self._minimum_accepted_energy_uj
        ):
            ceiling_reason = (
                f"Energy remained below {self._minimum_accepted_energy_uj:g} uJ "
                "at the 600 us pulse-width ceiling."
            )
            if self._override is None:
                raise ProcedureFailure(FailureKind.NCR, ceiling_reason)
            # Override mode: nothing left to adjust on this side - keep the
            # ceiling setting and let the cross-check / operator decide.
            self._record_event(
                state,
                "override",
                f"{ceiling_reason} Override mode keeps the ceiling setting; "
                "the operator is asked before anything is written.",
            )
            selection = TuningSelection(
                direction="upward_pulse",
                selected_side=side,
                target_uj=target,
                requested_current_ma=state.current_requested_ma,
                requested_pulse_width_us=state.pulse_requested_us,
                selected_mean_uj=source.measurement.mean_uj,
                rationale=(
                    "The 600 us pulse-width ceiling was already active; "
                    "override mode keeps it."
                ),
            )
            tuning_round = replace(tuning_round, selection=selection)
            state.tuning_rounds[-1] = tuning_round
            self._checkpoint(state)
            return tuning_round
        if not state.used_upward_tuning:
            self._checked_register_write(
                state, "EE_PULSE_WIDTH_UL", TEMPORARY_PULSE_WIDTH_LIMIT_US
            )
            self._checked_register_write(
                state, "OPT_PULSE_WIDTH_UL", TEMPORARY_PULSE_WIDTH_LIMIT_US
            )
            state.used_upward_tuning = True
        candidates = [(state.pulse_requested_us, source.measurement)]
        steps: list[TuningStep] = []
        active_setting = state.pulse_requested_us
        while active_setting < MAX_PULSE_WIDTH_US:
            requested = min(
                MAX_PULSE_WIDTH_US, active_setting + PULSE_WIDTH_STEP_US
            )
            readback = self._checked_register_write(
                state, "TA_PULSE_WIDTH", requested
            )
            observation = self._measure_side(
                state,
                side,
                f"Midpoint adjustment round {round_number}, step {len(steps) + 1}",
            )
            step = TuningStep(
                number=len(steps) + 1,
                label=(
                    f"Adjustment step {len(steps) + 1} — increased TA pulse width "
                    f"from {active_setting:g} us to {requested:g} us for the {side} sensor"
                ),
                side=side,
                register_name="TA_PULSE_WIDTH",
                requested_value=requested,
                readback=readback,
                observation=observation,
            )
            steps.append(step)
            tuning_round = replace(tuning_round, steps=tuple(steps))
            state.tuning_rounds[-1] = tuning_round
            candidates.append((requested, observation.measurement))
            self._checkpoint(state)
            active_setting = requested
            if (
                active_setting == MAX_PULSE_WIDTH_US
                and observation.measurement.mean_uj
                < self._minimum_accepted_energy_uj
            ):
                self._handle_out_of_band(
                    state,
                    f"Energy remained below {self._minimum_accepted_energy_uj:g} uJ "
                    "at the 600 us pulse-width ceiling.",
                )
                break
            if observation.measurement.mean_uj >= target:
                break
        selected = select_closest_valid_setting_to_target(candidates, target)
        if selected is None:
            raise ProcedureFailure(
                FailureKind.MEASUREMENT,
                "No valid upward-pulse candidate was available for selection.",
            )
        selected_setting, selected_measurement = selected
        if selected_setting != active_setting:
            self._checked_register_write(state, "TA_PULSE_WIDTH", selected_setting)
        state.pulse_requested_us = selected_setting
        selection = TuningSelection(
            direction="upward_pulse",
            selected_side=side,
            target_uj=target,
            requested_current_ma=state.current_requested_ma,
            requested_pulse_width_us=state.pulse_requested_us,
            selected_mean_uj=selected_measurement.mean_uj,
            rationale=(
                f"Selected {selected_setting:g} us because its valid {side}-sensor "
                f"reading was closest to the calculated {target:g} uJ target."
            ),
        )
        tuning_round = replace(tuning_round, selection=selection)
        state.tuning_rounds[-1] = tuning_round
        self._checkpoint(state)
        return tuning_round

    def _accept_pair(self, state: _RunState, pair: PairObservation) -> None:
        """A cross-check inside the acceptance band, about to be written.

        Outside override mode the band *is* the factory window, so there is
        nothing more to check. In override mode a pair inside the operator's
        band but outside the factory window still asks the operator first; a
        decline is the NCR the run would otherwise have ended in.
        """
        if self._override is None:
            return
        left = pair.metrics.left_mean_uj
        right = pair.metrics.right_mean_uj
        if within_factory_band(left) and within_factory_band(right):
            return
        reason = (
            "Both sensors are inside the override band "
            f"({self._override.band_description()}) but at least one is "
            f"outside the factory {factory_band_description()} band: "
            f"left {left:.1f} uJ, right {right:.1f} uJ."
        )
        decision = self._request_override(
            state, self._pair_override_request(state, pair, reason)
        )
        if not decision.accepted:
            raise ProcedureFailure(FailureKind.NCR, reason)

    def _pair_override_request(
        self, state: _RunState, pair: PairObservation, reason: str
    ) -> OverrideRequest:
        assert self._override is not None
        return OverrideRequest(
            criterion="pair_energy_band",
            reason=reason,
            measured={
                "left_mean_uj": pair.metrics.left_mean_uj,
                "right_mean_uj": pair.metrics.right_mean_uj,
                "midpoint_uj": pair.metrics.midpoint_uj,
                "target_uj": self._target_energy_uj,
            },
            accepted_band=self._override.band_description(),
            factory_band=factory_band_description(),
            proposed_configuration={
                "TA_CURRENT_DRV": state.current_requested_ma,
                "TA_PULSE_WIDTH": state.pulse_requested_us,
            },
        )

    def _write_final_configuration(self, state: _RunState) -> None:
        self._write_passing_configuration(
            state,
            written_message=(
                "Final configuration verification — tuned User Configuration "
                "was written under operator override and read back exactly."
                if self._written_under_override(state)
                else "Final configuration verification — passing tuned "
                "User Configuration was written and read back exactly."
            ),
        )

    def _crosscheck_label(self, number: int, accepted: bool) -> str:
        accepted_range = (
            f"{self._minimum_accepted_energy_uj:g}-"
            f"{self._maximum_accepted_energy_uj:g} uJ"
        )
        if accepted:
            return (
                f"Cross-check {number} — paired result: both sensors were within "
                f"the configured {accepted_range} range."
            )
        return (
            f"Cross-check {number} — paired result: at least one sensor was outside "
            f"the configured {accepted_range} range."
        )

    @staticmethod
    def _sensor_serial(state: _RunState, side: SensorSide) -> str:
        assert state.preflight is not None
        serial = (
            state.preflight.left_sensor_identity.serial
            if side == "left"
            else state.preflight.right_sensor_identity.serial
        )
        assert isinstance(serial, str) and serial.strip()
        return serial.strip()
