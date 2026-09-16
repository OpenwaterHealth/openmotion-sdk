"""Operator override mode (omotion.calibration.override), epic bloodflow-app#482.

Off by default, password-gated before any hardware is touched, asked against
the measured numbers at the write point, and recorded in the evidence as
OVERRIDDEN / ``Final result: OVERRIDE`` / exit code 3 - never as a PASS.
"""

import argparse
from datetime import datetime, timezone
import json
import os
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from omotion.calibration import override as override_module
from omotion.calibration.dual_sensor_laser import (
    DualSensorLaserCalibrationResult,
    DualSensorLaserCalibrationWorkflow,
)
from omotion.calibration.dual_sensor_laser_report import DualSensorHtmlRunReport
from omotion.calibration.laser import FailureKind, ProcedureStatus
from omotion.calibration.override import (
    EXIT_OVERRIDE,
    OverrideDecision,
    OverrideRequest,
    OverrideSettings,
    hash_override_password,
    verify_override_password,
    within_factory_band,
)
from omotion.calibration.reporting import HtmlRunReport
from omotion.calibration.script_support import (
    OverrideNotAuthorized,
    add_override_arguments,
    apply_cleanup_failure,
    authorize_override,
    finalize_run_artifacts,
    make_override_consent,
    override_settings_from_args,
    resolve_override_mode,
)
from omotion.calibration.single_sensor_laser import (
    SingleSensorLaserCalibrationResult,
    SingleSensorLaserCalibrationWorkflow,
)
from omotion.CalibrationWorkflow import CalibrationOutcome, _resolve_outcome
from test_calibration_workflow import (  # noqa: F401 - fixtures by name
    _LEFT,
    _RIGHT,
    _gate_failing_thresholds,
    _make_fake_scan_workflow,
    interface,
    request_obj,
    thresholds,
)
from test_wi15_dual_sensor_laser_calibration import (
    FakeDualBench,
    PlacementResponses,
    valid_request as dual_request,
)
from test_wi15_single_sensor_laser_calibration import (
    FakeLaserBench,
    FakeRecorder as SingleRecorder,
    _preflight,
    _request as single_request,
)
from wi15_builders import valid_measurement, valid_measurement_for_mean
from wi15_fakes import FakeRecorder
from wi15_script_harness import (
    FakeBench,
    FakeMeter,
    FakeRecorder as ScriptRecorder,
    FakeReport,
    complete_args,
    load_wi15_script,
    wi15_script_path,
)


TEST_PASSWORD = "unit-test-override-password"


@pytest.fixture
def test_password(monkeypatch):
    """Point the password check at a test digest - the real one never appears here."""
    monkeypatch.setattr(
        override_module,
        "OVERRIDE_PASSWORD_SHA256",
        hash_override_password(TEST_PASSWORD),
    )
    return TEST_PASSWORD


def _accepting(justification="dim engineering sample, accepted for bench use"):
    calls = []

    def consent(request):
        calls.append(request)
        return OverrideDecision(request, True, "operator", justification)

    consent.calls = calls
    return consent


def _declining():
    calls = []

    def consent(request):
        calls.append(request)
        return OverrideDecision(request, False, "operator", None)

    consent.calls = calls
    return consent


def _answers(*values):
    values = iter(values)
    return lambda _prompt: next(values)


# ------------------------------------------------------------------- model


def test_settings_default_to_the_factory_band_and_validate_custom_ones():
    factory = OverrideSettings()
    assert (factory.minimum_energy_uj, factory.maximum_energy_uj,
            factory.target_energy_uj) == (300.0, 400.0, 350.0)
    assert factory.uses_factory_band

    custom = OverrideSettings(
        minimum_energy_uj=100, maximum_energy_uj=200, target_energy_uj=150,
        authorized_by="eng",
    )
    assert not custom.uses_factory_band
    assert custom.describe() == "100 to 200 uJ, target 150 uJ"

    edges = OverrideSettings(
        minimum_energy_uj=10, maximum_energy_uj=1000, target_energy_uj=500
    )
    assert (edges.minimum_energy_uj, edges.maximum_energy_uj) == (10.0, 1000.0)

    for bad in (
        dict(minimum_energy_uj=360),              # minimum above target
        dict(maximum_energy_uj=340),              # maximum below target
        dict(minimum_energy_uj=0),                # below the 10 uJ floor
        dict(minimum_energy_uj=9.9),              # below the 10 uJ floor
        dict(maximum_energy_uj=1001),             # above the 1000 uJ ceiling
        dict(target_energy_uj=float("nan")),      # not finite
        dict(minimum_energy_uj=True),             # a bool is not a number
    ):
        with pytest.raises(ValueError):
            OverrideSettings(**bad)


def test_password_verification_is_a_salted_digest_compare(test_password):
    assert verify_override_password(test_password)
    assert not verify_override_password(test_password + " ")
    assert not verify_override_password("")
    assert not verify_override_password(None)


def test_shipped_password_is_stored_only_as_a_digest():
    digest = override_module.OVERRIDE_PASSWORD_SHA256
    assert len(digest) == 64 and int(digest, 16) >= 0
    assert hash_override_password("not the password") != digest


def test_within_factory_band():
    assert within_factory_band(300) and within_factory_band(400)
    assert not within_factory_band(299.9) and not within_factory_band("x")
    assert not within_factory_band(float("nan"))


@pytest.mark.parametrize(
    ("ok", "passed", "overridden", "expected"),
    [
        (True, False, True, CalibrationOutcome.OVERRIDDEN),
        (True, True, True, CalibrationOutcome.OVERRIDDEN),
        (True, False, False, CalibrationOutcome.FAILED),
        (False, False, True, CalibrationOutcome.ERROR),
    ],
)
def test_resolve_outcome_overridden(ok, passed, overridden, expected):
    assert _resolve_outcome(
        ok=ok, passed=passed, canceled=False, timed_out=False,
        overridden=overridden,
    ) is expected


# -------------------------------------------------- single-sensor workflow


def _single_workflow(bench, consent, **settings):
    recorder = SingleRecorder()
    workflow = SingleSensorLaserCalibrationWorkflow(
        bench, recorder,
        override=OverrideSettings(authorized_by="operator", **settings),
        override_consent=consent,
    )
    return workflow.run(single_request()), recorder


def test_single_override_asks_against_the_final_numbers_and_writes_the_closest_candidate():
    """Every downward candidate is above 400 uJ: the floor is the best this
    unit can do, so override mode keeps it and asks before the write."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[valid_measurement(mean_uj=450.0)]
        + [valid_measurement(mean_uj=410.0) for _ in range(61)],
    )
    consent = _accepting()

    result, recorder = _single_workflow(bench, consent)

    assert result.status is ProcedureStatus.OVERRIDDEN
    assert result.failure_reason is None
    assert len(consent.calls) == 1
    request = consent.calls[0]
    assert request.criterion == "final_energy_band"
    assert request.measured["final_mean_uj"] == 410.0
    assert request.accepted_band == "300 to 400 uJ"
    assert request.proposed_configuration["TA_CURRENT_DRV"] == 2000
    assert result.override_decision.accepted is True
    assert result.override_decision.justification.startswith("dim engineering")
    assert result.override.uses_factory_band
    # Written through the same exact-readback path as a pass; no restore.
    assert len(bench.written_user_configurations) == 2
    assert bench.written_user_configurations[-1]["TA_CURRENT_DRV"] == 2000
    assert result.requested_final_config["TA_CURRENT_DRV"] == 2000
    assert result.active_default_restore == ()
    assert result.selection.accepted is False
    messages = [event.message for event in result.events]
    assert any("Override mode continues with the closest candidate" in m for m in messages)
    assert any("Operator override accepted by operator" in m for m in messages)
    assert any("written under operator override" in m for m in messages)
    assert recorder.checkpoints[-1].status is ProcedureStatus.OVERRIDDEN


def test_single_override_declined_is_exactly_the_usual_ncr():
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[valid_measurement(mean_uj=450.0)]
        + [valid_measurement(mean_uj=410.0) for _ in range(61)],
    )
    consent = _declining()

    result, _ = _single_workflow(bench, consent)

    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    assert result.failure_reason == (
        "Final energy must be between 300 and 400 uJ inclusive."
    )
    assert len(consent.calls) == 1
    assert result.override_decision.accepted is False
    assert len(bench.written_user_configurations) == 1
    assert len(result.active_default_restore) == 5


def test_single_override_custom_band_steers_tuning_and_still_asks_outside_factory():
    """A 100-200 uJ band on a unit that lands at 145 uJ: tuning accepts the
    candidate, but 145 is outside 300-400 so the operator is still asked."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            valid_measurement_for_mean(450.0),
            valid_measurement_for_mean(400.0),
            valid_measurement_for_mean(300.0),
            valid_measurement_for_mean(200.0),
            valid_measurement_for_mean(140.0),
            valid_measurement_for_mean(145.0),
        ],
    )
    consent = _accepting()

    result, _ = _single_workflow(
        bench, consent,
        minimum_energy_uj=100, maximum_energy_uj=200, target_energy_uj=150,
    )

    assert result.status is ProcedureStatus.OVERRIDDEN
    assert result.target_energy_uj == 150.0
    assert (result.minimum_accepted_energy_uj, result.maximum_accepted_energy_uj) == (100.0, 200.0)
    assert result.selection.accepted is True
    assert result.selection.selected_requested_current_ma == 4800
    assert len(consent.calls) == 1
    assert "inside the override band (100 to 200 uJ)" in consent.calls[0].reason
    assert "outside the factory 300 to 400 uJ band" in consent.calls[0].reason
    assert bench.written_user_configurations[-1]["TA_CURRENT_DRV"] == 4800


def test_single_override_inside_the_factory_band_passes_without_asking():
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            valid_measurement_for_mean(360.0),
            valid_measurement_for_mean(345.0),
            valid_measurement_for_mean(348.0),
        ],
    )
    consent = _accepting()

    result, _ = _single_workflow(bench, consent)

    assert result.status is ProcedureStatus.PASSED
    assert consent.calls == []
    assert result.override is not None      # the mode is still in the evidence
    assert result.override_decision is None
    assert bench.written_user_configurations[-1]["TA_CURRENT_DRV"] == 4950


def test_single_override_at_the_pulse_ceiling_keeps_the_closest_candidate():
    """Below 300 uJ at 600 us is an immediate NCR normally; override mode
    falls through to closest-candidate selection and asks at the end."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[valid_measurement_for_mean(200.0)]
        + [valid_measurement_for_mean(250.0) for _ in range(11)],
    )
    consent = _accepting()

    result, _ = _single_workflow(bench, consent)

    assert result.status is ProcedureStatus.OVERRIDDEN
    assert result.selection.decision_kind == "closest_candidate"
    assert result.selection.selected_requested_pulse_width_us == 510
    assert bench.written_user_configurations[-1]["TA_PULSE_WIDTH"] == 510
    assert bench.written_user_configurations[-1]["EE_PULSE_WIDTH_UL"] == 660
    assert consent.calls[0].proposed_configuration["TA_PULSE_WIDTH"] == 510
    messages = [event.message for event in result.events]
    assert any("600 us pulse-width ceiling" in m and "Override mode" in m
               for m in messages)


def test_single_override_requires_a_consent_callback_and_a_justification():
    bench = FakeLaserBench([_preflight()])
    with pytest.raises(ValueError):
        SingleSensorLaserCalibrationWorkflow(
            bench, SingleRecorder(), override=OverrideSettings()
        )

    def accepts_without_reason(request):
        return OverrideDecision(request, True, "operator", "   ")

    bench = FakeLaserBench(
        [_preflight()],
        measurements=[valid_measurement(mean_uj=450.0)]
        + [valid_measurement(mean_uj=410.0) for _ in range(61)],
    )
    result, _ = _single_workflow(bench, accepts_without_reason)
    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.override_decision.accepted is False
    assert "without a justification" in result.override_decision.justification
    assert len(bench.written_user_configurations) == 1


def test_single_without_override_is_byte_for_byte_the_old_behaviour():
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[valid_measurement(mean_uj=450.0)]
        + [valid_measurement(mean_uj=410.0) for _ in range(60)],
    )
    result = SingleSensorLaserCalibrationWorkflow(bench, SingleRecorder()).run(
        single_request()
    )
    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.override is None and result.override_decision is None
    assert bench.calls.count("measure_energy") == 61


# ---------------------------------------------------- dual-sensor workflow


def _dual_workflow(means, consent, **settings):
    bench = FakeDualBench([valid_measurement_for_mean(mean) for mean in means])
    recorder = FakeRecorder()
    workflow = DualSensorLaserCalibrationWorkflow(
        bench, recorder, PlacementResponses(),
        override=OverrideSettings(authorized_by="operator", **settings),
        override_consent=consent,
    )
    return workflow.run(dual_request()), bench, recorder


_THREE_MISSED_CROSSCHECKS = [260, 340, 290, 310, 299, 370, 315, 299, 380, 310, 299, 390]


def test_dual_override_offers_the_third_crosscheck_and_writes_on_consent():
    consent = _accepting()

    result, bench, _ = _dual_workflow(_THREE_MISSED_CROSSCHECKS, consent)

    assert result.status is ProcedureStatus.OVERRIDDEN
    assert [item.number for item in result.crosschecks] == [1, 2, 3]
    assert len(consent.calls) == 1
    request = consent.calls[0]
    assert request.criterion == "pair_energy_band"
    assert "after three complete cross-checks" in request.reason
    assert request.measured["left_mean_uj"] == 299.0
    assert request.measured["right_mean_uj"] == 390.0
    assert len(bench.user_configuration_writes) == 2
    assert result.override_decision.accepted is True
    assert bench.calls.count("measure_energy") == len(_THREE_MISSED_CROSSCHECKS)


def test_dual_override_declined_after_three_crosschecks_is_the_usual_ncr():
    consent = _declining()

    result, bench, _ = _dual_workflow(_THREE_MISSED_CROSSCHECKS, consent)

    assert result.status is ProcedureStatus.FAILED_NCR
    assert "after three complete cross-checks" in result.failure_reason
    assert len(bench.user_configuration_writes) == 1
    assert result.override_decision.accepted is False
    assert len(result.active_default_restore) == 5


def test_dual_override_custom_band_asks_when_a_side_is_outside_the_factory_window():
    """280-400 uJ accepts the first cross-check (299/370), but 299 is below
    the factory 300, so the operator is asked before the write."""
    consent = _accepting()

    result, bench, _ = _dual_workflow(
        [260, 340, 290, 310, 299, 370], consent,
        minimum_energy_uj=280, maximum_energy_uj=400, target_energy_uj=350,
    )

    assert result.status is ProcedureStatus.OVERRIDDEN
    assert result.crosschecks[-1].accepted is True
    assert len(consent.calls) == 1
    assert "inside the override band (280 to 400 uJ)" in consent.calls[0].reason
    assert len(bench.user_configuration_writes) == 2


def test_dual_without_override_is_unchanged():
    bench = FakeDualBench(
        [valid_measurement_for_mean(m) for m in _THREE_MISSED_CROSSCHECKS]
    )
    result = DualSensorLaserCalibrationWorkflow(
        bench, FakeRecorder(), PlacementResponses()
    ).run(dual_request())
    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.override is None
    with pytest.raises(ValueError):
        DualSensorLaserCalibrationWorkflow(
            bench, FakeRecorder(), PlacementResponses(),
            override=OverrideSettings(),
        )


# ---------------------------------------------------------- script helpers


def _parser(energy_band=True):
    parser = argparse.ArgumentParser()
    add_override_arguments(parser, energy_band=energy_band)
    return parser


def test_energy_flags_require_allow_override_and_validate():
    parser = _parser()
    assert override_settings_from_args(parser.parse_args([]), operator="op") is None
    with pytest.raises(OverrideNotAuthorized, match="require --allow-override"):
        override_settings_from_args(
            parser.parse_args(["--min-energy-uj", "100"]), operator="op"
        )
    settings = override_settings_from_args(
        parser.parse_args(
            ["--allow-override", "--min-energy-uj", "100",
             "--max-energy-uj", "200", "--target-energy-uj", "150"]
        ),
        operator="op",
    )
    assert settings == OverrideSettings(100, 200, 150, "op", settings.authorized_at)
    with pytest.raises(OverrideNotAuthorized, match="invalid override energies"):
        override_settings_from_args(
            parser.parse_args(["--allow-override", "--target-energy-uj", "10"]),
            operator="op",
        )
    # The measurement script has the flag but no band.
    gate_only = _parser(energy_band=False).parse_args(["--allow-override"])
    assert override_settings_from_args(gate_only, operator="op").uses_factory_band


def test_password_gate_allows_three_attempts_and_fails_closed(test_password):
    messages = []
    assert authorize_override(
        _answers("wrong", "wrong", test_password), messages.append
    )
    assert messages == [
        "Incorrect override password. 2 attempt(s) left.",
        "Incorrect override password. 1 attempt(s) left.",
    ]
    messages.clear()
    assert not authorize_override(
        _answers("a", "b", "c", test_password), messages.append
    )
    assert messages[-1] == "Incorrect override password."

    def eof(_prompt):
        raise EOFError

    assert not authorize_override(eof, messages.append)


def test_resolve_override_mode_announces_the_band_after_the_password(test_password):
    args = _parser().parse_args(
        ["--allow-override", "--min-energy-uj", "100",
         "--max-energy-uj", "200", "--target-energy-uj", "150"]
    )
    prompts, messages = [], []

    def ask(prompt):
        prompts.append(prompt)
        return test_password

    settings = resolve_override_mode(
        args, operator="op", input_func=ask, output_func=messages.append
    )

    assert prompts == ["Override password: "]
    assert settings.describe() == "100 to 200 uJ, target 150 uJ"
    assert messages[0] == "*** OVERRIDE MODE is ON for this run. ***"
    assert "factory: 300 to 400 uJ, target 350 uJ" in messages[1]
    assert resolve_override_mode(
        _parser().parse_args([]), operator="op", input_func=ask,
        output_func=messages.append,
    ) is None
    with pytest.raises(OverrideNotAuthorized, match="password was not accepted"):
        resolve_override_mode(
            args, operator="op", input_func=_answers("x", "y", "z"),
            output_func=messages.append,
        )


def test_resolve_override_mode_asks_for_the_band_when_not_supplied(test_password):
    """No flags (the Procedures pane never passes any): after the password
    the operator types the band and target, Enter keeping the factory value."""
    prompts, messages = [], []
    replies = iter([test_password, "", "", ""])

    def ask(prompt):
        prompts.append(prompt)
        return next(replies)

    settings = resolve_override_mode(
        _parser().parse_args(["--allow-override"]), operator="op",
        input_func=ask, output_func=messages.append,
    )

    assert prompts == [
        "Override password: ",
        "Minimum accepted energy in uJ (10-1000) [300]: ",
        "Maximum accepted energy in uJ (10-1000) [400]: ",
        "Target energy in uJ (10-1000) [350]: ",
    ]
    assert settings.uses_factory_band
    assert any(m.startswith("Type the acceptance band and target") for m in messages)

    # Out-of-range and non-numeric entries are asked again; an inconsistent
    # trio is asked again as a whole.
    prompts.clear()
    messages.clear()
    replies = iter(
        [test_password, "5", "abc", "300", "200", "250", "100", "200", "150"]
    )
    settings = resolve_override_mode(
        _parser().parse_args(["--allow-override"]), operator="op",
        input_func=ask, output_func=messages.append,
    )

    assert settings.describe() == "100 to 200 uJ, target 150 uJ"
    assert "Please type a number between 10 and 1000." in messages
    assert "Please type a number." in messages
    assert any(
        m.startswith(
            "The energies must satisfy minimum <= target <= maximum "
            "(got 300 <= 250 <= 200)"
        )
        for m in messages
    )
    assert prompts.count("Minimum accepted energy in uJ (10-1000) [300]: ") == 4

    # A flagged value is not asked for; the rest still are.
    prompts.clear()
    replies = iter([test_password, "", ""])
    settings = resolve_override_mode(
        _parser().parse_args(["--allow-override", "--min-energy-uj", "280"]),
        operator="op", input_func=ask, output_func=messages.append,
    )
    assert settings.describe() == "280 to 400 uJ, target 350 uJ"
    assert not any(p.startswith("Minimum") for p in prompts)

    # The measurement script has no band to ask for.
    prompts.clear()
    replies = iter([test_password])
    settings = resolve_override_mode(
        _parser(energy_band=False).parse_args(["--allow-override"]),
        operator="op", input_func=ask, output_func=messages.append,
    )
    assert prompts == ["Override password: "]
    assert settings.uses_factory_band


def test_consent_question_shows_the_numbers_and_requires_a_reason():
    request = OverrideRequest(
        criterion="final_energy_band",
        reason="Final energy must be between 300 and 400 uJ inclusive.",
        measured={"final_mean_uj": 287.4},
        accepted_band="300 to 400 uJ",
        factory_band="300 to 400 uJ",
        proposed_configuration={"TA_CURRENT_DRV": 2000, "TA_PULSE_WIDTH": 500},
    )
    prompts, messages = [], []
    replies = iter(["yes", "", "NCR-42 accepted by engineering"])

    def ask(prompt):
        prompts.append(prompt)
        return next(replies)

    decision = make_override_consent(ask, messages.append, operator="op")(request)

    assert decision.accepted is True
    assert decision.justification == "NCR-42 accepted by engineering"
    assert decision.operator == "op"
    assert prompts == [
        "Write this calibration to the console anyway? (yes/no): ",
        "Reason for the override: ",
        "Reason for the override: ",
    ]
    assert "  final_mean_uj: 287.4" in messages
    assert "  would write: TA_CURRENT_DRV=2000, TA_PULSE_WIDTH=500" in messages

    declined = make_override_consent(_answers("no"), messages.append, operator="op")(request)
    assert declined.accepted is False and declined.justification is None

    def eof(_prompt):
        raise EOFError

    ended = make_override_consent(eof, messages.append, operator="op")(request)
    assert ended.accepted is False
    assert "operator input ended" in ended.justification


def _overridden_single_result():
    request = OverrideRequest("final_energy_band", "Final energy must be between 300 and 400 uJ inclusive.",
                              {"final_mean_uj": 410.0}, "300 to 400 uJ", "300 to 400 uJ",
                              {"TA_CURRENT_DRV": 2000})
    now = datetime.now(timezone.utc)
    return SingleSensorLaserCalibrationResult(
        status=ProcedureStatus.OVERRIDDEN,
        side="left",
        sdk_version="test-sdk",
        started_at=now,
        ended_at=now,
        requested_default_config={"TA_CURRENT_DRV": 5000},
        requested_final_config={"TA_CURRENT_DRV": 2000},
        override=OverrideSettings(authorized_by="op"),
        override_decision=OverrideDecision(request, True, "op", "accepted for bench"),
    )


def test_finalize_prints_the_override_verdict_and_exits_3(tmp_path):
    recorder = ScriptRecorder(tmp_path)
    messages = []

    code = finalize_run_artifacts(
        request=SimpleNamespace(run_id="run-1"),
        result=_overridden_single_result(),
        recorder=recorder,
        report_factory=FakeReport,
        procedure_revision="rev",
        procedure_slug="single-laser-cal",
        output_func=messages.append,
    )

    assert code == EXIT_OVERRIDE == 3
    assert "Final result: OVERRIDE" in messages
    assert any(
        m.startswith("The calibration was written under operator override by op: accepted for bench")
        for m in messages
    )
    assert "# procedure status: overridden" in messages
    assert recorder.checkpoints[-1].status is ProcedureStatus.OVERRIDDEN


def test_cleanup_and_report_failures_downgrade_an_override_like_a_pass(tmp_path):
    recorder = ScriptRecorder(tmp_path)
    downgraded = apply_cleanup_failure(
        _overridden_single_result(), "Motion shutdown transport failed", recorder
    )
    assert downgraded.status is ProcedureStatus.FAILED
    assert downgraded.failure_reason == "Hardware resource cleanup failed."

    class BrokenReport(FakeReport):
        def write(self, request, result, json_path):
            raise RuntimeError("disk full")

    messages = []
    code = finalize_run_artifacts(
        request=SimpleNamespace(run_id="run-1"),
        result=_overridden_single_result(),
        recorder=recorder,
        report_factory=BrokenReport,
        procedure_revision="rev",
        procedure_slug="single-laser-cal",
        output_func=messages.append,
    )
    assert code == 1
    assert "Final result: FAIL" in messages
    assert recorder.checkpoints[-1].failure_kind is FailureKind.REPORT


# ------------------------------------------------------------ laser scripts


def _single_script(monkeypatch, tmp_path, result=None):
    script = load_wi15_script(
        wi15_script_path("wi15_single_sensor_laser_calibration.py"),
        "wi15_single_override_under_test",
    )
    recorder = ScriptRecorder(tmp_path)
    meter = FakeMeter()
    bench = FakeBench(meter)
    captured = {"constructed": 0}

    class FakeWorkflow:
        def __init__(self, workflow_bench, workflow_recorder, **kwargs):
            captured["constructed"] += 1
            captured["bench"] = workflow_bench
            captured["kwargs"] = kwargs

        def run(self, request):
            captured["request"] = request
            return result or _passing_single_result()

    monkeypatch.setattr(script, "recorder_factory", lambda *args: recorder)
    monkeypatch.setattr(script, "meter_factory", lambda: meter)
    monkeypatch.setattr(script, "bench_factory", lambda value: bench)
    monkeypatch.setattr(script, "workflow_factory", FakeWorkflow)
    monkeypatch.setattr(script, "report_factory", FakeReport)
    return script, captured


def _passing_single_result():
    now = datetime.now(timezone.utc)
    return SingleSensorLaserCalibrationResult(
        status=ProcedureStatus.PASSED, side="left", sdk_version="test-sdk",
        started_at=now, ended_at=now,
    )


def test_single_script_asks_for_the_password_first_and_fails_closed(
    monkeypatch, tmp_path, test_password
):
    script, captured = _single_script(monkeypatch, tmp_path)
    prompts, messages = [], []
    replies = iter(["nope", "still nope", "no"])

    def ask(prompt):
        prompts.append(prompt)
        return next(replies)

    code = script.main(
        [*complete_args(tmp_path), "--allow-override"],
        input_func=ask, output_func=messages.append,
    )

    assert code == 1
    assert prompts == ["Override password: "] * 3
    assert captured["constructed"] == 0
    assert any(m.startswith("Override mode not enabled: the override password was not accepted")
               for m in messages)
    assert messages[-1] == "Final result: FAIL"


def test_single_script_forwards_band_target_and_consent_to_the_workflow(
    monkeypatch, tmp_path, test_password
):
    script, captured = _single_script(monkeypatch, tmp_path)
    messages = []

    code = script.main(
        [*complete_args(tmp_path), "--allow-override", "--min-energy-uj", "100",
         "--max-energy-uj", "200", "--target-energy-uj", "150"],
        input_func=_answers(test_password, "left", "yes", "yes", "yes"),
        output_func=messages.append,
    )

    assert code == 0
    settings = captured["kwargs"]["override"]
    assert (settings.minimum_energy_uj, settings.maximum_energy_uj,
            settings.target_energy_uj) == (100.0, 200.0, 150.0)
    assert settings.authorized_by == "operator"
    assert callable(captured["kwargs"]["override_consent"])
    # The measured-energy heartbeat names the operator's target.
    assert captured["bench"]._target_energy_uj == 150.0
    assert "*** OVERRIDE MODE is ON for this run. ***" in messages


def test_single_script_factory_call_is_unchanged_without_override(
    monkeypatch, tmp_path
):
    script, captured = _single_script(monkeypatch, tmp_path)

    code = script.main(
        complete_args(tmp_path), input_func=_answers("left", "yes", "yes", "yes")
    )

    assert code == 0
    assert captured["kwargs"] == {}
    assert captured["bench"]._target_energy_uj == 350


def test_single_script_refuses_band_flags_without_allow_override(monkeypatch, tmp_path):
    script, captured = _single_script(monkeypatch, tmp_path)
    messages = []

    code = script.main(
        [*complete_args(tmp_path), "--target-energy-uj", "150"],
        input_func=_answers(), output_func=messages.append,
    )

    assert code == 1
    assert captured["constructed"] == 0
    assert any("require --allow-override" in m for m in messages)


def test_single_script_reports_an_overridden_run_with_exit_3(
    monkeypatch, tmp_path, test_password
):
    script, captured = _single_script(
        monkeypatch, tmp_path, result=_overridden_single_result()
    )
    messages = []

    # No band flags, as launched from the Procedures pane: the three band
    # prompts follow the password; Enter keeps the factory values.
    code = script.main(
        [*complete_args(tmp_path), "--allow-override"],
        input_func=_answers(test_password, "", "", "", "left", "yes", "yes", "yes"),
        output_func=messages.append,
    )

    assert code == 3
    assert "Final result: OVERRIDE" in messages
    assert captured["kwargs"]["override"].uses_factory_band


def test_dual_script_forwards_override_settings_and_consent(
    monkeypatch, tmp_path, test_password
):
    script = load_wi15_script(
        wi15_script_path("wi15_dual_sensor_laser_calibration.py"),
        "wi15_dual_override_under_test",
    )
    recorder = ScriptRecorder(tmp_path)
    meter = FakeMeter()
    bench = FakeBench(meter)
    captured = {}
    now = datetime.now(timezone.utc)

    class FakeWorkflow:
        def __init__(self, workflow_bench, workflow_recorder, placement, **kwargs):
            captured["kwargs"] = kwargs

        def run(self, request):
            return DualSensorLaserCalibrationResult(
                status=ProcedureStatus.PASSED, sdk_version="test-sdk",
                started_at=now, ended_at=now,
            )

    monkeypatch.setattr(script, "recorder_factory", lambda *args: recorder)
    monkeypatch.setattr(script, "meter_factory", lambda: meter)
    monkeypatch.setattr(script, "bench_factory", lambda value: bench)
    monkeypatch.setattr(script, "workflow_factory", FakeWorkflow)
    monkeypatch.setattr(script, "report_factory", FakeReport)

    # The flagged minimum is not asked for; maximum and target are (Enter
    # keeps the factory values).
    code = script.main(
        [*complete_args(tmp_path), "--allow-override", "--min-energy-uj", "280"],
        input_func=_answers(test_password, "", ""),
    )
    assert code == 0
    assert captured["kwargs"]["override"].describe() == "280 to 400 uJ, target 350 uJ"
    assert callable(captured["kwargs"]["override_consent"])

    code = script.main(complete_args(tmp_path), input_func=_answers())
    assert code == 0
    assert captured["kwargs"] == {}


# ------------------------------------------------------ measurement script


class _MeasurementInterface:
    """The engine seam the measurement script drives, with the gate hook."""

    def __init__(self, *, gate_fail=False):
        self.console = SimpleNamespace(
            read_config=lambda: SimpleNamespace(
                json_data={"TA_PULSE_WIDTH": 590.0, "TA_CURRENT_DRV": 5000.0}
            ),
            read_serial_number=lambda: "CONSN01",
        )
        self.left = self.right = SimpleNamespace(
            enable_camera_power=lambda mask: True,
            read_serial_number=lambda: "SN",
        )
        self.gate_fail = gate_fail
        self.start_kwargs = None
        self.hook_rows = None

    def start(self):
        pass

    def stop(self):
        pass

    def wait_for_ready(self, **_kwargs):
        return True

    def is_device_connected(self):
        return (True, True, True)

    def apply_laser_power(self):
        return True

    def start_configure_camera_sensors(self, request, *, on_complete_fn):
        on_complete_fn(SimpleNamespace(ok=True, error=""))
        return True

    def cancel_calibration(self):
        pass

    def start_calibration(self, request, *, on_complete_fn, on_progress_fn=None,
                          **kwargs):
        self.start_kwargs = kwargs
        hook = kwargs.get("on_override_fn")
        row = SimpleNamespace(
            side="right", cam_id=5, mean=62.1, avg_contrast=0.31, bfi=0.0,
            bvi=5.0, mean_test="FAIL", contrast_test="PASS", dark_test="PASS",
        )
        if not self.gate_fail:
            on_complete_fn(SimpleNamespace(
                outcome=SimpleNamespace(value="passed"), error="", rows=[],
                csv_path="cal.csv", json_path="cal.json", calibration_written=True,
            ))
            return True
        granted, justification = False, ""
        if hook is not None:
            on_progress_fn and on_progress_fn("override")
            self.hook_rows = [row]
            decision = hook([row])
            granted = bool(getattr(decision, "accepted", decision))
            justification = getattr(decision, "justification", "") or ""
        if granted:
            on_complete_fn(SimpleNamespace(
                outcome=SimpleNamespace(value="overridden"), error="", rows=[row],
                csv_path="cal.csv", json_path="cal.json", calibration_written=True,
                override_granted=True, override_justification=justification,
            ))
        else:
            on_complete_fn(SimpleNamespace(
                outcome=SimpleNamespace(value="failed"),
                error="calibration scan below threshold on R6; nothing written",
                rows=[row], csv_path="cal.csv", json_path="cal.json",
                calibration_written=False,
            ))
        return True


def _run_measurement(tmp_path, monkeypatch, *, extra=(), answers=(), gate_fail=False):
    script = load_wi15_script(
        wi15_script_path("wi15_measurement_calibration.py"),
        "wi15_measurement_override_under_test",
    )
    fake = _MeasurementInterface(gate_fail=gate_fail)
    monkeypatch.setattr(script, "interface_factory", lambda **kwargs: fake)
    lines = []
    code = script.main(
        ["--output-dir", str(tmp_path), "--operator", "op", "--fixture-id",
         "fx", "--side", "right", "--phantom-confirmed", *extra],
        input_func=_answers(*answers),
        output_func=lines.append,
    )
    return code, fake, lines


def test_measurement_script_call_is_unchanged_without_override(tmp_path, monkeypatch):
    code, fake, lines = _run_measurement(tmp_path, monkeypatch, gate_fail=True)

    assert code == 1
    assert fake.start_kwargs == {}
    assert "Final result: FAIL" in lines


def test_measurement_script_override_asks_at_the_gate_and_exits_3(
    tmp_path, monkeypatch, test_password
):
    code, fake, lines = _run_measurement(
        tmp_path, monkeypatch, extra=["--allow-override"],
        answers=[test_password, "yes", "known dim unit, bench only"],
        gate_fail=True,
    )

    assert code == 3
    assert "on_override_fn" in fake.start_kwargs
    assert "Problem: Scan mean/contrast below the limits on R6." in lines
    assert "  R6 mean: 62.1" in lines
    assert "Final result: OVERRIDE" in lines
    assert any("UNDER OPERATOR OVERRIDE: known dim unit, bench only" in l for l in lines)
    assert any("Asking about the override" in l for l in lines)


def test_measurement_script_override_declined_fails_like_before(
    tmp_path, monkeypatch, test_password
):
    code, fake, lines = _run_measurement(
        tmp_path, monkeypatch, extra=["--allow-override"],
        answers=[test_password, "no"], gate_fail=True,
    )

    assert code == 1
    assert "Override declined. Nothing will be written." in lines
    assert "Nothing was saved to the console." in lines
    assert "Final result: FAIL" in lines


def test_measurement_script_bad_password_never_touches_hardware(
    tmp_path, monkeypatch, test_password
):
    code, fake, lines = _run_measurement(
        tmp_path, monkeypatch, extra=["--allow-override"],
        answers=["x", "y", "z"], gate_fail=True,
    )

    assert code == 1
    assert fake.start_kwargs is None
    assert lines[-1] == "Final result: FAIL"


# ------------------------------------------------------------------ engine


def _run_engine(interface, request, **kwargs):
    done = threading.Event()
    holder = {}
    assert interface.start_calibration(
        request, on_complete_fn=lambda r: (holder.update(r=r), done.set()), **kwargs
    )
    assert done.wait(30)
    return holder["r"]


def test_engine_override_consent_continues_to_validation_then_writes(
    interface, request_obj
):
    from dataclasses import replace

    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock(
        side_effect=lambda *a, **k: interface.get_calibration()
    )
    scans = []
    fake_start = interface.scan_workflow.start_scan
    interface.scan_workflow.start_scan = (
        lambda req: (scans.append(1), fake_start(req))[1]
    )
    asked = []

    def on_override(rows):
        asked.append(rows)
        return OverrideDecision(
            OverrideRequest("calibration_gate", "below"), True, "op", "dim unit"
        )

    result = _run_engine(
        interface, replace(request_obj, thresholds=_gate_failing_thresholds()),
        on_override_fn=on_override,
    )

    assert len(asked) == 1 and all(r.mean_test == "FAIL" for r in asked[0])
    assert len(scans) == 2, "validation scan must still run under override"
    interface.write_calibration.assert_called_once()
    assert result.outcome is CalibrationOutcome.OVERRIDDEN
    assert result.calibration_written is True
    assert result.override_granted is True
    assert result.override_justification == "dim unit"
    with open(result.json_path, encoding="utf-8") as handle:
        manifest = json.load(handle)
    assert manifest["outcome"] == "overridden"
    assert manifest["override_granted"] is True
    assert manifest["override_justification"] == "dim unit"


def test_engine_override_declined_is_the_usual_gate_failure(interface, request_obj):
    from dataclasses import replace

    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock()

    result = _run_engine(
        interface, replace(request_obj, thresholds=_gate_failing_thresholds()),
        on_override_fn=lambda rows: False,
    )

    interface.write_calibration.assert_not_called()
    assert result.outcome is CalibrationOutcome.FAILED
    assert result.override_granted is False
    assert "nothing written" in result.error


def test_engine_override_cannot_write_past_an_ambient_dark_failure(
    interface, request_obj
):
    from dataclasses import replace

    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock()
    failing = replace(
        _gate_failing_thresholds(), max_dark_per_camera=[-1.0] * 8
    )

    result = _run_engine(
        interface, replace(request_obj, thresholds=failing),
        on_override_fn=lambda rows: True,
    )

    interface.write_calibration.assert_not_called()
    assert result.outcome is CalibrationOutcome.FAILED
    assert result.override_granted is True
    assert result.calibration_written is False
    assert "ambient-dark" in result.error


def test_engine_without_the_hook_fails_the_gate_as_before(interface, request_obj):
    from dataclasses import replace

    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock()

    result = _run_engine(
        interface, replace(request_obj, thresholds=_gate_failing_thresholds())
    )

    interface.write_calibration.assert_not_called()
    assert result.outcome is CalibrationOutcome.FAILED
    assert result.override_granted is False


# ----------------------------------------------------------------- reports


def test_single_report_renders_the_override_as_its_own_status():
    html = HtmlRunReport("unused").render(single_request(), _overridden_single_result())

    assert 'class="status status-overridden"' in html
    assert "Status: overridden" in html
    assert "Operator override mode settings" in html
    assert "Operator override decision" in html
    assert "accepted for bench" in html
    assert "Tuned User Configuration written under operator override" in html
    assert "Minimum accepted energy uJ" in html


def test_dual_report_renders_the_override_sections():
    now = datetime.now(timezone.utc)
    request = OverrideRequest("pair_energy_band", "after three complete cross-checks",
                              {"left_mean_uj": 299.0}, "300 to 400 uJ", "300 to 400 uJ")
    result = DualSensorLaserCalibrationResult(
        status=ProcedureStatus.OVERRIDDEN, sdk_version="test-sdk",
        started_at=now, ended_at=now,
        override=OverrideSettings(authorized_by="op"),
        override_decision=OverrideDecision(request, True, "op", "bench unit"),
    )

    html = DualSensorHtmlRunReport("unused").render(dual_request(), result)

    assert "status-overridden" in html
    assert "Operator override decision" in html
    assert "bench unit" in html
