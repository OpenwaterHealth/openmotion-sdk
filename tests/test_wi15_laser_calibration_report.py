import json
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path

from omotion.calibration.laser import (
    CriterionResult,
    DeviceIdentity,
    FailureKind,
    FinalSettingCheck,
    OphirIdentity,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
)
from omotion.calibration.reporting import HtmlRunReport, JsonRunRecorder
from omotion.calibration.single_sensor_laser import (
    OphirEvidenceApplicability,
    OphirSettingEvidence,
    ProcedureEvent,
    SingleSensorLaserCalibrationRequest,
    SingleSensorLaserCalibrationResult,
    SingleSensorLaserCalibrationWorkflow,
    TuningCandidate,
    TuningSelection,
)
from wi15_builders import valid_measurement as _measurement


class _Mode(str, Enum):
    READY = "ready"


@dataclass(frozen=True)
class _SerializationEvidence:
    location: Path
    readings: tuple[float, float, float, float]
    mode: _Mode


class _AbortBeforePreflightBench:
    def stop_trigger(self):
        pass


def _request(**changes):
    values = {
        "side": "left",
        "side_confirmed": True,
        "fixture_confirmed": True,
        "operator": "operator",
        "build_id": "build-1",
        "fixture_id": "fixture-1",
        "procedure_id": "WI-00015",
        "output_root": "unused-by-report",
        "run_id": "run-1",
    }
    values.update(changes)
    return SingleSensorLaserCalibrationRequest(**values)


def _result(**changes):
    measurement = _measurement()
    values = {
        "status": ProcedureStatus.PASSED,
        "side": "left",
        "topology": TopologySnapshot(True, True, False),
        "identities": (
            DeviceIdentity("console", "console-1", "firmware-1", "console-hw"),
            DeviceIdentity("sensor", "sensor-1", "firmware-2", "sensor-hw"),
        ),
        "ophir_identity": OphirIdentity(
            "meter", "meter-1", "energy-sensor", "ophir-1", "2027-01-01"
        ),
        "ophir_setting_evidence": (
            OphirSettingEvidence(
                "measurement_mode",
                "Energy",
                "Energy",
                OphirEvidenceApplicability.APPLICABLE,
                True,
            ),
        ),
        "pre_existing_config": {"TA_CURRENT_DRV": 4900.0, "TA_PULSE_WIDTH": 500.0},
        "requested_default_config": {"TA_CURRENT_DRV": 5000.0, "TA_PULSE_WIDTH": 500.0},
        "default_config_readback": {"TA_CURRENT_DRV": 5000.0, "TA_PULSE_WIDTH": 500.0},
        "configurations": (SettingReadback("TA_CURRENT_DRV", 5000.0, 4999.0),),
        "measurements": (measurement,),
        "measurement_criteria": (
            (CriterionResult("pulse_count", True, "Pulse count must be > 25."),),
        ),
        "adjustments": (SettingReadback("TA_CURRENT_DRV", 4950.0, 4950.0),),
        "candidates": (TuningCandidate(4950.0, 4950.0, 500.0, 500.0, measurement),),
        "selection": TuningSelection(
            "downward_current",
            "closest_candidate",
            True,
            4950.0,
            500.0,
            350.0,
            "closest",
        ),
        "requested_final_config": {"TA_CURRENT_DRV": 4950.0, "TA_PULSE_WIDTH": 500.0},
        "final_config_readback": {"TA_CURRENT_DRV": 4950.0, "TA_PULSE_WIDTH": 500.0},
        "final_setting_checks": (
            FinalSettingCheck(
                "TA_CURRENT_DRV", 4950.0, 4949.0, 1.0, 0.020202, 2.0, True
            ),
            FinalSettingCheck(
                "TA_PULSE_WIDTH", 500.0, 510.0, 10.0, 2.0, 2.0, True
            ),
        ),
        "active_default_restore": (SettingReadback("TA_CURRENT_DRV", 5000.0, 5000.0),),
        "trigger_cleanup_failure": "cleanup note",
        "active_default_restore_failure": "restore note",
        "events": (
            ProcedureEvent(
                datetime(2026, 8, 12, tzinfo=timezone.utc), "preflight", "ready"
            ),
        ),
        "report_paths": (Path("evidence.txt"),),
    }
    values.update(changes)
    return SingleSensorLaserCalibrationResult(**values)


def test_recorder_creates_unique_directory_and_serializes_checkpoint_data(tmp_path):
    """A serialization regression must not make state files non-standard JSON."""
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "run-1")
    event = ProcedureEvent(
        datetime(2026, 8, 12, tzinfo=timezone.utc),
        "preflight",
        "event",
        {
            "evidence": _SerializationEvidence(
                Path("meter/readback"),
                (1.0, float("nan"), float("inf"), float("-inf")),
                _Mode.READY,
            )
        },
    )

    recorder.record(event)

    payload = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    evidence = payload["events"][0]["data"]["evidence"]
    assert evidence == {
        "location": "meter/readback",
        "mode": "ready",
        "readings": [1.0, "NaN", "Infinity", "-Infinity"],
    }
    assert "NaN" not in recorder.json_path.read_text(encoding="utf-8").replace(
        '"NaN"', ""
    )
    first_checkpoint = recorder.json_path.read_text(encoding="utf-8")

    other = JsonRunRecorder(tmp_path, "WI-00015", "run-1")

    assert recorder.run_directory != other.run_directory
    assert recorder.run_directory.parent == tmp_path
    assert other.run_directory.parent == tmp_path
    assert recorder.json_path.read_text(encoding="utf-8") == first_checkpoint


def test_recorder_checkpoints_each_event_and_terminal_failure_without_overwriting(
    tmp_path,
):
    """Missing a checkpoint would lose the only evidence from an aborted run."""
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "aborted")
    first = ProcedureEvent(
        datetime(2026, 8, 12, tzinfo=timezone.utc), "confirmation", "one"
    )
    second = ProcedureEvent(
        datetime(2026, 8, 12, tzinfo=timezone.utc), "preflight", "two"
    )

    recorder.record(first)
    first_payload = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    recorder.record(second)
    second_payload = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    result = SingleSensorLaserCalibrationWorkflow(
        _AbortBeforePreflightBench(), recorder
    ).run(_request(side=None, side_confirmed=False))

    terminal = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    assert [item["message"] for item in first_payload["events"]] == ["one"]
    assert [item["message"] for item in second_payload["events"]] == ["one", "two"]
    assert result.status is ProcedureStatus.FAILED
    assert terminal["status"] == "failed"
    assert (
        terminal["failure_reason"]
        == "Confirm the selected sensor side before continuing."
    )
    assert not list(recorder.run_directory.glob("*.tmp"))


def test_event_after_rich_checkpoint_preserves_all_structured_state(tmp_path):
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "rich-state")
    result = _result()
    later_event = ProcedureEvent(
        datetime(2026, 8, 12, 1, tzinfo=timezone.utc), "later", "still durable"
    )

    recorder.checkpoint(result)
    recorder.record(later_event)

    payload = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    assert payload["measurements"]
    assert payload["measurement_criteria"]
    assert payload["adjustments"]
    assert payload["candidates"]
    assert payload["configurations"]
    assert payload["requested_default_config"]
    assert payload["final_setting_checks"]
    assert [event["stage"] for event in payload["events"]] == ["preflight", "later"]


def test_html_report_escapes_and_shows_complete_passing_evidence(tmp_path):
    """Unescaped or omitted evidence would make a passing record unsafe to review."""
    request = _request(operator="<operator & co>")
    result = _result(
        identities=(DeviceIdentity("sensor", "<device & serial>", "fw", "hw"),)
    )
    report = HtmlRunReport(tmp_path).write(request, result, tmp_path / "run.json")

    text = report.read_text(encoding="utf-8")
    assert "&lt;operator &amp; co&gt;" in text
    assert "&lt;device &amp; serial&gt;" in text
    assert "<operator & co>" not in text
    assert "passed" in text
    for expected in (
        "Topology",
        "Ophir settings",
        "measurement_mode",
        "pulse_count",
        "TA_CURRENT_DRV",
        "closest",
        "cleanup note",
        "restore note",
        "evidence.txt",
    ):
        assert expected in text
    assert 'href="run.json"' in text
    assert text.count('class="changed"') == 1


def test_html_report_omits_all_unreached_stage_headings_after_earliest_failure(
    tmp_path,
):
    """Empty headings would falsely imply the procedure reached later stages."""
    result = _result(
        status=ProcedureStatus.FAILED,
        failure_kind=FailureKind.SETUP,
        failure_reason="Fixture confirmation was not supplied.",
        topology=None,
        identities=(),
        ophir_identity=None,
        ophir_setting_evidence=(),
        pre_existing_config=None,
        requested_default_config=None,
        default_config_readback=None,
        configurations=(),
        measurements=(),
        measurement_criteria=(),
        adjustments=(),
        candidates=(),
        selection=None,
        requested_final_config=None,
        final_config_readback=None,
        active_default_restore=(),
        trigger_cleanup_failure=None,
        active_default_restore_failure=None,
    )

    text = (
        HtmlRunReport(tmp_path)
        .write(_request(), result, "run.json")
        .read_text(encoding="utf-8")
    )

    assert "Status: failed" in text
    assert "Fixture confirmation was not supplied." in text
    assert 'href="run.json"' in text
    for unreached in (
        "Topology",
        "Device identities",
        "Ophir identity",
        "Ophir settings",
        "Pre-existing User Configuration",
        "Requested default User Configuration",
        "Default User Configuration readback",
        "Measurements",
        "Active configuration readbacks",
        "Adjustments",
        "Active default restoration",
        "Cleanup diagnostics",
        "Final User Configuration readback",
    ):
        assert unreached not in text


def test_html_report_gates_sections_by_reached_evidence_and_keeps_cleanup_failure(
    tmp_path,
):
    """Stage headings must track records, while real cleanup diagnostics remain visible."""
    result = _result(
        status=ProcedureStatus.FAILED,
        failure_kind=FailureKind.MEASUREMENT,
        failure_reason="Initial measurement was invalid.",
        requested_final_config=None,
        final_config_readback=None,
        adjustments=(),
        candidates=(),
        selection=None,
        active_default_restore=(),
        trigger_cleanup_failure="Trigger stop failed.",
        active_default_restore_failure=None,
    )

    text = (
        HtmlRunReport(tmp_path)
        .write(_request(), result, "run.json")
        .read_text(encoding="utf-8")
    )

    assert "Measurement 1" in text
    assert "Active configuration readbacks" in text
    assert "Cleanup diagnostics" in text
    assert "Trigger stop failed." in text
    for unreached in (
        "Adjustments",
        "Tuning candidates",
        "Active default restoration",
        "Passing tuned User Configuration",
        "Final User Configuration readback",
    ):
        assert unreached not in text


