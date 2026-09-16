"""Shared WI-00015 procedure plumbing: evidence records and fail-closed helpers.

Public names defined here are re-exported by ``single_sensor_laser`` (their
historical home), which operator scripts, hardware adapters, and tests import
from. New code may import from either module.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
import math
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping, Protocol

from .laser import (
    DeviceIdentity,
    EnergyMeasurement,
    FailureKind,
    FinalSettingCheck,
    OphirIdentity,
    ProcedureStatus,
    SettingReadback,
    TEMPORARY_PULSE_WIDTH_LIMIT_US,
    TopologySnapshot,
    default_user_configuration,
    percent_difference,
    validate_console_fpga_revisions,
    validate_serial,
    within_percent,
)
from .override import (
    OverrideConsentFn,
    OverrideDecision,
    OverrideRequest,
    OverrideSettings,
)


def deeply_immutable(value):
    if isinstance(value, Mapping):
        return MappingProxyType(
            {key: deeply_immutable(item) for key, item in value.items()}
        )
    if isinstance(value, tuple | list):
        return tuple(deeply_immutable(item) for item in value)
    if isinstance(value, set | frozenset):
        return frozenset(deeply_immutable(item) for item in value)
    return value


def finite_number(value: object) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, int | float)
        and math.isfinite(float(value))
    )


def valid_trigger_rate(rate_hz: object) -> bool:
    try:
        return 39.0 <= float(rate_hz) <= 41.0
    except (TypeError, ValueError):
        return False


@dataclass(frozen=True)
class ProcedureFailure(Exception):
    kind: FailureKind
    reason: str


@dataclass(frozen=True)
class ProcedureEvent:
    timestamp: datetime
    stage: str
    message: str
    data: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "data", deeply_immutable(self.data))


class OphirEvidenceApplicability(str, Enum):
    APPLICABLE = "applicable"
    NOT_APPLICABLE = "not_applicable"


class ReportArtifactStatus(str, Enum):
    INCOMPLETE = "incomplete"
    FINALIZED = "finalized"
    FAILED = "failed"


@dataclass(frozen=True)
class ReportArtifactEvidence:
    path: Path | str
    status: ReportArtifactStatus
    failure: str | None = None


@dataclass(frozen=True)
class OphirSettingEvidence:
    name: str
    requested: str | float | int
    actual: str | float | int | None
    applicability: OphirEvidenceApplicability
    passed: bool


def has_complete_ophir_identity(identity: OphirIdentity) -> bool:
    return all(
        isinstance(value, str) and bool(value.strip())
        for value in (
            identity.meter_model,
            identity.meter_serial,
            identity.sensor_model,
            identity.sensor_serial,
            identity.calibration_due,
        )
    )


def has_valid_ophir_setting_evidence(
    evidence: tuple[OphirSettingEvidence, ...],
) -> bool:
    expected = {
        "measurement_mode": ("Energy", OphirEvidenceApplicability.APPLICABLE),
        "range_mj": (2.0, OphirEvidenceApplicability.APPLICABLE),
        "wavelength_nm": (795, OphirEvidenceApplicability.APPLICABLE),
        "pulse_length_ms": (1.0, OphirEvidenceApplicability.APPLICABLE),
        "threshold": (
            "minimum_available",
            OphirEvidenceApplicability.APPLICABLE,
        ),
        "display_averaging_s": (3, None),
        "graph_mode": ("Statistics", None),
    }
    if len(evidence) != len(expected):
        return False
    by_name = {item.name: item for item in evidence}
    if len(by_name) != len(evidence) or set(by_name) != set(expected):
        return False
    for name, (requested, required_applicability) in expected.items():
        item = by_name[name]
        if item.requested != requested or not item.passed:
            return False
        if required_applicability is not None:
            if (
                item.applicability is not required_applicability
                or item.actual != requested
            ):
                return False
        elif (
            item.applicability is not OphirEvidenceApplicability.NOT_APPLICABLE
            or item.actual is not None
        ):
            return False
    return True


class LaserBench(Protocol):
    """Console + Ophir operations shared by both laser calibration benches."""

    def read_user_configuration(self) -> Mapping[str, float]: ...

    def write_user_configuration(
        self, configuration: Mapping[str, float]
    ) -> Mapping[str, float] | None: ...

    def bring_up_laser_configuration(self) -> None: ...

    def read_register(self, name: str) -> float: ...

    def write_register(self, name: str, value: float) -> SettingReadback | None: ...

    def read_trigger_rate_hz(self) -> float: ...

    def write_trigger_rate_hz(self, rate_hz: float) -> SettingReadback | None: ...

    def measure_energy(self) -> EnergyMeasurement: ...

    def stop_trigger(self) -> None: ...


class RunRecorder(Protocol):
    def record(self, event: ProcedureEvent) -> None: ...

    def checkpoint(self, result) -> None: ...


class LaserWorkflowBase:
    """Fail-closed primitives shared by the single- and dual-sensor workflows.

    Subclasses set ``_bench`` and ``_recorder`` and carry a mutable run-state
    object exposing the evidence lists these primitives append to, plus a
    ``result(status, *, failure=None, ended_at=None)`` constructor used for
    durable checkpoints.
    """

    _bench: LaserBench
    _recorder: RunRecorder
    # Override mode (omotion.calibration.override). Both stay None outside
    # override mode, which is the default for every constructor.
    _override: OverrideSettings | None = None
    _override_consent: OverrideConsentFn | None = None

    def _checkpoint(self, state) -> None:
        self._recorder.checkpoint(state.result(ProcedureStatus.IN_PROGRESS))

    def _record_event(self, state, stage: str, message: str) -> None:
        event = ProcedureEvent(datetime.now(timezone.utc), stage, message)
        state.events.append(event)
        self._recorder.record(event)

    # ------------------------------------------------------- override mode
    def _configure_override(
        self,
        override: OverrideSettings | None,
        override_consent: OverrideConsentFn | None,
    ) -> None:
        if override is None:
            self._override = None
            self._override_consent = None
            return
        if not isinstance(override, OverrideSettings):
            raise TypeError("override must be an OverrideSettings instance")
        if override_consent is None:
            raise ValueError("override mode requires an override_consent callback")
        self._override = override
        self._override_consent = override_consent

    @property
    def override_mode(self) -> bool:
        return self._override is not None

    @staticmethod
    def _written_under_override(state) -> bool:
        decision = getattr(state, "override_decision", None)
        return decision is not None and decision.accepted

    def _terminal_success_status(self, state) -> ProcedureStatus:
        """PASSED, or OVERRIDDEN when the operator consented to the write."""
        if self._written_under_override(state):
            return ProcedureStatus.OVERRIDDEN
        return ProcedureStatus.PASSED

    def _handle_out_of_band(self, state, reason: str) -> None:
        """Outside override mode this is the NCR it always was.

        In override mode the tuning loop records the miss and carries on with
        the closest candidate: nothing is written yet, and the operator is
        asked against the final measured numbers before any write.
        """
        if self._override is None:
            raise ProcedureFailure(FailureKind.NCR, reason)
        self._record_event(
            state,
            "override",
            f"{reason} Override mode continues with the closest candidate; "
            "the operator is asked before anything is written.",
        )
        self._checkpoint(state)

    def _request_override(self, state, request: OverrideRequest) -> OverrideDecision:
        """Ask the operator to accept a write outside the acceptance criteria.

        Fail-closed: any problem with the consent callback, a missing
        decision, or an acceptance without a justification is a decline.
        The request and the decision are both checkpointed, so an
        interrupted run shows the question that was pending.
        """
        assert self._override is not None and self._override_consent is not None
        operator = getattr(state.request, "operator", "")
        self._record_event(
            state, "override", f"Operator override requested: {request.reason}"
        )
        self._checkpoint(state)
        try:
            decision = self._override_consent(request)
        except Exception as error:
            decision = OverrideDecision(
                request,
                False,
                operator,
                f"consent callback failed: {type(error).__name__}: {error}",
            )
        if not isinstance(decision, OverrideDecision):
            decision = OverrideDecision(
                request, False, operator, "consent callback returned no decision"
            )
        if decision.accepted and not (
            isinstance(decision.justification, str)
            and decision.justification.strip()
        ):
            decision = OverrideDecision(
                request,
                False,
                decision.operator,
                "override accepted without a justification; treated as declined",
            )
        state.override_decision = decision
        if decision.accepted:
            self._record_event(
                state,
                "override",
                f"Operator override accepted by {decision.operator}: "
                f"{decision.justification}",
            )
        else:
            detail = f" ({decision.justification})" if decision.justification else ""
            self._record_event(
                state,
                "override",
                "Operator override declined; the run fails exactly as it would "
                f"without override mode.{detail}",
            )
        self._checkpoint(state)
        return decision

    def _validate_shared_preflight(
        self,
        preflight,
        serial_identities: tuple[tuple[DeviceIdentity, str], ...],
    ) -> None:
        if not preflight.console_responsive:
            raise ProcedureFailure(
                FailureKind.SETUP, "Console must be responsive before continuing."
            )
        for identity, label in serial_identities:
            if not validate_serial(identity.serial).passed:
                raise ProcedureFailure(
                    FailureKind.SETUP, f"{label} serial must be nonblank text."
                )
        fpga_revisions = validate_console_fpga_revisions(preflight.console_identity)
        if not fpga_revisions.passed:
            raise ProcedureFailure(FailureKind.SETUP, fpga_revisions.detail)
        if not preflight.ophir_ready:
            raise ProcedureFailure(
                FailureKind.SETUP,
                preflight.ophir_failure_reason or "Ophir preflight failed.",
            )
        if preflight.ophir_identity is None:
            raise ProcedureFailure(
                FailureKind.SETUP, "Ophir identity must be present."
            )
        if not has_complete_ophir_identity(preflight.ophir_identity):
            raise ProcedureFailure(
                FailureKind.SETUP, "Ophir identity fields must be nonblank text."
            )
        if not has_valid_ophir_setting_evidence(preflight.ophir_setting_evidence):
            raise ProcedureFailure(
                FailureKind.SETUP,
                "Ophir setting evidence is incomplete or invalid.",
            )

    def _establish_defaults(
        self,
        state,
        *,
        revalidate: Callable[[], TopologySnapshot],
        validate_topology,
        revalidation_failure: str,
        topology_changed_failure: str,
    ) -> None:
        approved = default_user_configuration()
        state.pre_existing_config = dict(self._bench.read_user_configuration())
        self._checkpoint(state)
        state.requested_default_config = dict(approved)
        self._checkpoint(state)
        try:
            topology = revalidate()
        except Exception as error:
            raise ProcedureFailure(
                FailureKind.SETUP, revalidation_failure
            ) from error
        state.topology_revalidation = topology
        if not validate_topology(topology).passed:
            raise ProcedureFailure(FailureKind.SETUP, topology_changed_failure)
        immediate = self._bench.write_user_configuration(dict(approved))
        if not isinstance(immediate, Mapping):
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Default User Configuration write did not return a complete readback mapping.",
            )
        state.default_config_readback = dict(immediate)
        self._checkpoint(state)
        if state.default_config_readback != state.requested_default_config:
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Default User Configuration readback must exactly match the request.",
            )
        self._record_event(
            state,
            "default_configuration",
            "Exact default User Configuration was written and read back.",
        )
        self._checkpoint(state)
        self._bench.bring_up_laser_configuration()
        self._record_event(
            state,
            "laser_configuration_bringup",
            "Laser configuration bring-up completed.",
        )
        self._checkpoint(state)
        self._verify_active_defaults(state, approved)
        self._verify_trigger_rate(state)
        state.current_requested_ma = approved["TA_CURRENT_DRV"]
        state.pulse_requested_us = approved["TA_PULSE_WIDTH"]
        state.active_defaults_established = True

    def _verify_active_defaults(
        self, state, approved: Mapping[str, float]
    ) -> None:
        for name in (
            "TA_CURRENT_DRV",
            "TA_PULSE_WIDTH",
            "SEED_CW_GAIN",
            "EE_PULSE_WIDTH_UL",
            "OPT_PULSE_WIDTH_UL",
        ):
            requested = approved[name]
            actual = self._bench.read_register(name)
            state.configurations.append(SettingReadback(name, requested, actual))
            self._checkpoint(state)
            if not within_percent(requested, actual, 2.0):
                raise ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Active {name} is outside the allowed 2 percent tolerance.",
                )

    def _verify_trigger_rate(self, state) -> None:
        rate_hz = self._bench.read_trigger_rate_hz()
        state.configurations.append(
            SettingReadback("trigger_rate_hz_initial", 40.0, rate_hz)
        )
        self._checkpoint(state)
        if rate_hz != 40.0:
            write_result = self._bench.write_trigger_rate_hz(40.0)
            if not isinstance(write_result, SettingReadback):
                raise ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    "Trigger-rate correction returned malformed readback evidence.",
                )
            state.configurations.append(write_result)
            self._checkpoint(state)
            if (
                write_result.name != "trigger_rate_hz_write"
                or write_result.requested != 40.0
                or not math.isfinite(write_result.actual)
                or write_result.actual != 40.0
            ):
                raise ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    "Trigger-rate correction immediate readback must exactly match 40 Hz.",
                )
            rate_hz = self._bench.read_trigger_rate_hz()
        state.configurations.append(SettingReadback("trigger_rate_hz", 40.0, rate_hz))
        self._checkpoint(state)
        if not valid_trigger_rate(rate_hz):
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Active trigger rate must be between 39 and 41 Hz inclusive.",
            )

    def _checked_register_write(
        self, state, name: str, requested: float
    ) -> SettingReadback:
        try:
            result = self._bench.write_register(name, requested)
        except Exception as error:
            raise ProcedureFailure(
                FailureKind.CONFIGURATION, f"Active {name} write failed."
            ) from error
        if not isinstance(result, SettingReadback):
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                f"Active {name} write returned malformed readback evidence.",
            )
        state.adjustments.append(result)
        self._checkpoint(state)
        if result.name != name or result.requested != requested:
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                f"Active {name} write returned mismatched readback identity.",
            )
        if not within_percent(requested, result.actual, 2.0):
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                f"Active {name} is outside the allowed 2 percent tolerance.",
            )
        return result

    def _restore_active_defaults(self, state, *, restored_message: str) -> None:
        failures: list[str] = []
        approved = default_user_configuration()
        for name in (
            "TA_CURRENT_DRV",
            "TA_PULSE_WIDTH",
            "SEED_CW_GAIN",
            "EE_PULSE_WIDTH_UL",
            "OPT_PULSE_WIDTH_UL",
        ):
            requested = approved[name]
            try:
                result = self._bench.write_register(name, requested)
                if not isinstance(result, SettingReadback):
                    failures.append(f"{name} restore returned malformed evidence")
                else:
                    state.active_default_restore.append(result)
                    if result.name != name or result.requested != requested:
                        failures.append(f"{name} restore identity did not match")
                    elif not within_percent(requested, result.actual, 2.0):
                        failures.append(f"{name} restore was outside 2 percent")
            except Exception:
                failures.append(f"{name} restore raised an exception")
            state.active_default_restore_failure = "; ".join(failures) or None
            self._checkpoint(state)
        self._record_event(
            state,
            "active_default_restore",
            state.active_default_restore_failure or restored_message,
        )

    def _write_passing_configuration(
        self, state, *, written_message: str
    ) -> None:
        for name, requested in (
            ("TA_CURRENT_DRV", state.current_requested_ma),
            ("TA_PULSE_WIDTH", state.pulse_requested_us),
        ):
            try:
                actual = self._bench.read_register(name)
            except Exception as error:
                raise ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Final active {name} readback failed.",
                ) from error
            state.configurations.append(SettingReadback(name, requested, actual))
            passed = within_percent(requested, actual, 2.0)
            state.final_setting_checks.append(
                FinalSettingCheck(
                    name=name,
                    requested=requested,
                    actual=actual,
                    absolute_difference=abs(actual - requested),
                    percent_difference=percent_difference(requested, actual),
                    tolerance_percent=2.0,
                    passed=passed,
                )
            )
            self._checkpoint(state)
            if not passed:
                raise ProcedureFailure(
                    FailureKind.CONFIGURATION,
                    f"Final active {name} is outside the allowed 2 percent tolerance.",
                )
        requested_config = default_user_configuration()
        requested_config["TA_CURRENT_DRV"] = state.current_requested_ma
        requested_config["TA_PULSE_WIDTH"] = state.pulse_requested_us
        if state.used_upward_tuning:
            requested_config["EE_PULSE_WIDTH_UL"] = TEMPORARY_PULSE_WIDTH_LIMIT_US
            requested_config["OPT_PULSE_WIDTH_UL"] = TEMPORARY_PULSE_WIDTH_LIMIT_US
        state.requested_final_config = requested_config
        self._checkpoint(state)
        try:
            immediate = self._bench.write_user_configuration(dict(requested_config))
        except Exception as error:
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Passing User Configuration write or readback failed.",
            ) from error
        if not isinstance(immediate, Mapping):
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Passing User Configuration write did not return a complete readback mapping.",
            )
        state.final_config_readback = dict(immediate)
        self._checkpoint(state)
        if state.final_config_readback != state.requested_final_config:
            raise ProcedureFailure(
                FailureKind.CONFIGURATION,
                "Passing User Configuration readback must exactly match the request.",
            )
        self._record_event(state, "final_configuration", written_message)
