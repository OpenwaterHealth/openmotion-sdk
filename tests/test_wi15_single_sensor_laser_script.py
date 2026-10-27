import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import pytest

import omotion
from omotion.calibration.laser import FailureKind, ProcedureStatus
from omotion.calibration.single_sensor_laser import (
    ReportArtifactStatus,
    SingleSensorLaserCalibrationResult,
)
from wi15_script_harness import (
    FakeBench,
    FakeMeter,
    FakeRecorder,
    FakeReport,
    complete_args,
    load_wi15_script,
    wi15_script_path,
)


SCRIPT_PATH = wi15_script_path("wi15_single_sensor_laser_calibration.py")


def load_script():
    return load_wi15_script(SCRIPT_PATH, "wi15_script_under_test")


@dataclass(frozen=True)
class FakeResult:
    status: ProcedureStatus
    failure_kind: FailureKind | None = None
    failure_reason: str | None = None
    report_paths: tuple[Path, ...] = ()
    report_artifact: object | None = None
    resource_cleanup_failure: str | None = None


class FakeWorkflow:
    def __init__(self, result):
        self.result = result
        self.requests = []

    def run(self, request):
        self.requests.append(request)
        return self.result


def configured_script(monkeypatch, tmp_path, result=FakeResult(ProcedureStatus.PASSED)):
    script = load_script()
    recorder = FakeRecorder(tmp_path)
    meter = FakeMeter()
    bench = FakeBench(meter)
    workflow = FakeWorkflow(result)
    report = FakeReport(recorder.run_directory)
    monkeypatch.setattr(script, "recorder_factory", lambda *args: recorder)
    monkeypatch.setattr(script, "meter_factory", lambda: meter)
    monkeypatch.setattr(script, "bench_factory", lambda value: bench)
    monkeypatch.setattr(script, "workflow_factory", lambda *args: workflow)
    monkeypatch.setattr(
        script, "report_factory", lambda directory, filename: report
    )
    return script, recorder, meter, bench, workflow, report


def answers(*values):
    values = iter(values)
    return lambda _prompt: next(values)


def test_script_source_is_ascii_decodable_and_does_not_use_legacy_tuning():
    """A non-ASCII console script or old runner import would break supported operation."""
    source = SCRIPT_PATH.read_bytes()
    source.decode("ascii")
    script = load_script()
    assert "omotion.tuning" not in script.__dict__.get("__doc__", "")
    assert "omotion.tuning" not in SCRIPT_PATH.read_text(encoding="ascii")


def test_main_reprompts_side_and_requires_all_confirmations(monkeypatch, tmp_path):
    """An unconfirmed or ambiguous module selection could fire the wrong fixture."""
    script, recorder, meter, bench, workflow, report = configured_script(monkeypatch, tmp_path)
    messages = []

    exit_code = script.main(
        complete_args(tmp_path),
        input_func=answers("middle", "right", "yes", "yes", "yes"),
        output_func=messages.append,
    )

    assert exit_code == 0
    assert workflow.requests[0].side == "right"
    assert workflow.requests[0].side_confirmed is True
    assert workflow.requests[0].fixture_confirmed is True
    assert workflow.requests[0].sdk_version == omotion.__version__
    assert workflow.requests[0].started_at is not None
    assert any("Please answer left or right." in message for message in messages)
    assert bench.closed == 1
    assert meter.closed == 0
    assert report.writes[0][1].report_paths == (recorder.json_path, report.report_path)
    assert recorder.checkpoints[-1].report_paths == (recorder.json_path, report.report_path)


def test_bench_close_failure_downgrades_a_pass_and_is_recorded(monkeypatch, tmp_path):
    """Cleanup problems are evidence (parity with the dual and safety runs)."""
    script, recorder, _meter, bench, _workflow, report = configured_script(
        monkeypatch, tmp_path
    )

    def fail_close():
        bench.closed += 1
        raise RuntimeError("Motion shutdown transport failed")

    bench.close = fail_close

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    assert exit_code == 1
    assert bench.closed == 1
    terminal = recorder.checkpoints[-1]
    assert terminal.status is ProcedureStatus.FAILED
    assert terminal.failure_reason == "Hardware resource cleanup failed."
    assert terminal.resource_cleanup_failure == "Motion shutdown transport failed"
    assert report.writes[-1][1].resource_cleanup_failure == (
        "Motion shutdown transport failed"
    )


def test_fixture_confirmation_requires_only_ophir_zero_cm_placement(monkeypatch, tmp_path):
    script, _recorder, _meter, _bench, workflow, _report = configured_script(
        monkeypatch, tmp_path
    )
    prompts = []
    replies = iter(("left", "yes", "yes", "yes"))

    def capture_prompt(prompt):
        prompts.append(prompt)
        return next(replies)

    exit_code = script.main(complete_args(tmp_path), input_func=capture_prompt)

    assert exit_code == 0
    assert prompts[-1] == (
        "Is the sensor in the 0 cm fixture? (yes/no): "
    )
    assert all("containment" not in prompt.lower() for prompt in prompts)
    assert workflow.requests[0].fixture_confirmed is True


def test_single_sensor_connected_confirmation_is_asked_and_notes_disconnection(
    monkeypatch, tmp_path
):
    """The operator must be told to disconnect the other sensor and confirm it."""
    script, _recorder, _meter, _bench, _workflow, _report = configured_script(
        monkeypatch, tmp_path
    )
    prompts = []
    messages = []
    replies = iter(("left", "yes", "yes", "yes"))

    def capture_prompt(prompt):
        prompts.append(prompt)
        return next(replies)

    exit_code = script.main(
        complete_args(tmp_path),
        input_func=capture_prompt,
        output_func=messages.append,
    )

    assert exit_code == 0
    assert "Is only the left sensor connected to the system? (yes/no): " in prompts
    assert any(
        "Disconnect the other sensor module" in message for message in messages
    )


def test_declining_single_sensor_confirmation_cancels_before_hardware(
    monkeypatch, tmp_path
):
    """A second or wrong-side sensor caught by the operator must stop the run early."""
    script = load_script()
    constructed = []
    monkeypatch.setattr(
        script, "recorder_factory", lambda *args: constructed.append("recorder")
    )
    monkeypatch.setattr(script, "meter_factory", lambda: constructed.append("meter"))

    exit_code = script.main(
        complete_args(tmp_path),
        input_func=answers("left", "yes", "no"),
    )

    assert exit_code != 0
    assert constructed == []


def test_main_uses_explicit_sdk_fallback_only_when_runtime_version_is_unavailable(
    monkeypatch, tmp_path
):
    """Operator build metadata must never masquerade as the runtime SDK version."""
    script, _recorder, _meter, _bench, workflow, _report = configured_script(
        monkeypatch, tmp_path
    )
    monkeypatch.delattr(script.omotion, "__version__", raising=False)

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    assert exit_code == 0
    assert workflow.requests[0].build_id == "build-7"
    assert workflow.requests[0].sdk_version == "unavailable"


@pytest.mark.parametrize("reply", ["no", EOFError()])
def test_cancel_or_eof_before_hardware_returns_nonzero_without_constructing_hardware(
    monkeypatch, tmp_path, reply
):
    """Cancellation before consent must never acquire or configure hardware."""
    script = load_script()
    constructed = []
    monkeypatch.setattr(script, "recorder_factory", lambda *args: constructed.append("recorder"))
    monkeypatch.setattr(script, "meter_factory", lambda: constructed.append("meter"))

    def prompt(_prompt):
        if not hasattr(prompt, "side_given"):
            prompt.side_given = True
            return "left"
        if isinstance(reply, Exception):
            raise reply
        return reply

    exit_code = script.main(complete_args(tmp_path), input_func=prompt)

    assert exit_code != 0
    assert constructed == []


@pytest.mark.parametrize(
    "status, expected_exit",
    [
        (ProcedureStatus.PASSED, 0),
        (ProcedureStatus.FAILED, 1),
        (ProcedureStatus.FAILED_NCR, 1),
        (ProcedureStatus.CANCELED, 1),
    ],
)
def test_terminal_status_controls_exit_code(monkeypatch, tmp_path, status, expected_exit):
    """A non-passing terminal result must be visible to calling automation."""
    script, _recorder, _meter, _bench, _workflow, _report = configured_script(
        monkeypatch, tmp_path, FakeResult(status)
    )

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    assert exit_code == expected_exit


def test_terminal_failure_prints_the_structured_category_and_exact_reason(
    monkeypatch, tmp_path
):
    terminal_result = SingleSensorLaserCalibrationResult(
        status=ProcedureStatus.FAILED,
        side="left",
        failure_kind=FailureKind.MEASUREMENT,
        failure_reason="Adjustment energy measurement failed quality criteria.",
    )
    script, _recorder, _meter, _bench, _workflow, _report = configured_script(
        monkeypatch, tmp_path, terminal_result
    )
    messages = []

    exit_code = script.main(
        complete_args(tmp_path),
        input_func=answers("left", "yes", "yes", "yes"),
        output_func=messages.append,
    )

    assert exit_code == 1
    assert "Final result: FAIL" in messages
    assert "# procedure status: failed" in messages
    assert "Problem type: measurement" in messages
    assert (
        "Problem: Adjustment energy measurement failed quality criteria."
        in messages
    )


def test_main_generates_one_run_id_for_recorder_and_workflow_request(
    monkeypatch, tmp_path
):
    """A crossed second must not split request evidence from its artifact directory."""
    script, recorder, _meter, _bench, workflow, _report = configured_script(
        monkeypatch, tmp_path
    )
    generated_ids = iter(("first-run-id", "second-run-id"))
    recorder_run_ids = []
    monkeypatch.setattr(script, "_run_id", lambda: next(generated_ids))
    monkeypatch.setattr(
        script,
        "recorder_factory",
        lambda output_dir, procedure_id, run_id: recorder_run_ids.append(run_id)
        or recorder,
    )

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    assert exit_code == 0
    assert recorder_run_ids == ["first-run-id"]
    assert workflow.requests[0].run_id == "first-run-id"


def test_metadata_prompts_complete_before_hardware_and_cleanup_survives_workflow_error(
    monkeypatch, tmp_path
):
    """Metadata must be attributable before hardware use and cleanup must survive errors."""
    script = load_script()
    calls = []
    meter = FakeMeter()
    bench = FakeBench(meter)
    monkeypatch.setattr(script, "recorder_factory", lambda *args: calls.append("recorder") or FakeRecorder(tmp_path))
    monkeypatch.setattr(script, "meter_factory", lambda: calls.append("meter") or meter)
    monkeypatch.setattr(script, "bench_factory", lambda value: calls.append("bench") or bench)

    class RaisingWorkflow:
        def run(self, request):
            raise RuntimeError("workflow boom")

    monkeypatch.setattr(script, "workflow_factory", lambda *args: RaisingWorkflow())
    monkeypatch.setattr(script, "report_factory", FakeReport)

    exit_code = script.main(
        ["--output-dir", str(tmp_path)],
        input_func=answers(
            "operator",
            "build-7",
            "fixture-2",
            "current",
            "left",
            "yes",
            "yes",
            "yes",
        ),
    )

    assert exit_code != 0
    assert calls == ["recorder", "meter", "bench"]
    assert bench.closed == 1
    assert meter.closed == 0


def _configure_real_artifact_main(
    monkeypatch, script, run_id, *, obstruct_report=False, identities=()
):
    meter = FakeMeter()
    bench = FakeBench(meter)
    monkeypatch.setattr(script, "_run_id", lambda: run_id)
    monkeypatch.setattr(script, "meter_factory", lambda: meter)
    monkeypatch.setattr(script, "bench_factory", lambda value: bench)

    class ArtifactWorkflow:
        def __init__(self, _bench, recorder):
            self.recorder = recorder

        def run(self, request):
            result = SingleSensorLaserCalibrationResult(
                status=ProcedureStatus.PASSED,
                side=request.side,
                sdk_version=request.sdk_version,
                started_at=request.started_at,
                ended_at=datetime.now(timezone.utc),
                identities=identities,
            )
            self.recorder.checkpoint(result)
            if obstruct_report:
                (
                    self.recorder.run_directory / "single-laser-cal-report.html"
                ).mkdir()
            return result

    monkeypatch.setattr(script, "workflow_factory", ArtifactWorkflow)
    return meter, bench


def test_production_main_finalizes_real_json_and_html_before_claiming_report(
    monkeypatch, tmp_path
):
    """The production orchestration must publish only an existing atomic report."""
    script = load_script()
    _configure_real_artifact_main(monkeypatch, script, "real-artifacts")

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    run_directory = tmp_path / "single-laser-cal-real-artifacts"
    payload = json.loads(
        (run_directory / "single-laser-cal-run.json").read_text(encoding="utf-8")
    )
    assert exit_code == 0
    assert not (tmp_path / "WI-00015-real-artifacts").exists()
    assert (run_directory / "single-laser-cal-report.html").is_file()
    assert payload["status"] == "passed"
    assert payload["report_artifact"]["status"] == ReportArtifactStatus.FINALIZED.value
    assert [Path(item).name for item in payload["report_paths"]] == [
        "single-laser-cal-run.json",
        "single-laser-cal-report.html",
    ]


def test_artifact_names_carry_the_console_serial_and_test_type(
    monkeypatch, tmp_path
):
    """A run whose preflight read the console serial names its files with it,
    so a folder of runs is tellable apart without opening anything (#268)."""
    from omotion.calibration.laser import DeviceIdentity

    script = load_script()
    _configure_real_artifact_main(
        monkeypatch,
        script,
        "named-artifacts",
        identities=(
            DeviceIdentity(
                role="console",
                serial="CS 01/A",
                firmware="1.0",
                hardware_id="hw",
            ),
        ),
    )

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    # The run directory is renamed serial-first too, so listings sort by
    # unit; the serial is sanitized into a safe filename component.
    run_directory = tmp_path / "CS-01-A-single-laser-cal-named-artifacts"
    assert exit_code == 0
    assert not (tmp_path / "WI-00015-named-artifacts").exists()
    json_path = run_directory / "CS-01-A-single-laser-cal-run.json"
    report_path = run_directory / "CS-01-A-single-laser-cal-report.html"
    assert json_path.is_file()
    assert report_path.is_file()
    assert not (run_directory / "run.json").exists()
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert [Path(item).name for item in payload["report_paths"]] == [
        json_path.name,
        report_path.name,
    ]
    # The report's raw-evidence link points at the renamed JSON.
    assert f'href="{json_path.name}"' in report_path.read_text(encoding="utf-8")


def test_production_main_persists_explicit_incomplete_artifact_during_render(
    monkeypatch, tmp_path
):
    """The durable pre-render state must identify HTML as incomplete, not absent."""
    script = load_script()
    _configure_real_artifact_main(monkeypatch, script, "render-in-progress")
    observed = {}

    class InspectingRealReport:
        def __init__(self, directory, filename):
            self._real = script.HtmlRunReport(directory, filename)
            self.report_path = self._real.report_path

        def write(self, request, result, json_path):
            observed.update(json.loads(Path(json_path).read_text(encoding="utf-8")))
            assert not self.report_path.exists()
            return self._real.write(request, result, json_path)

    monkeypatch.setattr(script, "report_factory", InspectingRealReport)

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    assert exit_code == 0
    assert observed["status"] == "passed"
    assert observed["report_artifact"]["status"] == "incomplete"
    assert [Path(item).name for item in observed["report_paths"]] == [
        "single-laser-cal-run.json"
    ]


def test_production_main_report_failure_checkpoints_failed_incomplete_artifact(
    monkeypatch, tmp_path
):
    """Renderer failure must replace any prior pass with durable failure evidence."""
    script = load_script()
    _configure_real_artifact_main(
        monkeypatch,
        script,
        "report-failure",
        obstruct_report=True,
    )

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    run_directory = tmp_path / "single-laser-cal-report-failure"
    payload = json.loads(
        (run_directory / "single-laser-cal-run.json").read_text(encoding="utf-8")
    )
    assert exit_code == 1
    assert payload["status"] == "failed"
    assert payload["failure_kind"] == FailureKind.REPORT.value
    assert payload["report_artifact"]["status"] == ReportArtifactStatus.FAILED.value
    assert payload["report_artifact"]["path"].endswith(
        "single-laser-cal-report.html"
    )
    assert payload["report_artifact"]["failure"]
    assert [Path(item).name for item in payload["report_paths"]] == [
        "single-laser-cal-run.json"
    ]
    assert not (run_directory / "single-laser-cal-report.html").is_file()


def test_production_main_report_factory_failure_replaces_prior_pass(monkeypatch, tmp_path):
    """Report construction failure must also replace a durable workflow pass."""
    script = load_script()
    _configure_real_artifact_main(monkeypatch, script, "report-factory-failure")

    def failing_report_factory(_directory, _filename):
        raise RuntimeError("report construction failed")

    monkeypatch.setattr(script, "report_factory", failing_report_factory)

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    run_directory = tmp_path / "single-laser-cal-report-factory-failure"
    payload = json.loads(
        (run_directory / "single-laser-cal-run.json").read_text(encoding="utf-8")
    )
    assert exit_code == 1
    assert payload["status"] == "failed"
    assert payload["failure_kind"] == FailureKind.REPORT.value
    assert payload["report_artifact"]["failure"] == "report construction failed"
    assert Path(payload["report_artifact"]["path"]) == (
        run_directory / "single-laser-cal-report.html"
    )
    assert (
        payload["report_artifact"]["status"]
        == ReportArtifactStatus.FAILED.value
    )


def test_main_requires_the_claimed_report_path_to_exist(monkeypatch, tmp_path):
    """A writer returning a different file cannot finalize the claimed artifact."""
    script = load_script()
    _configure_real_artifact_main(monkeypatch, script, "misdirected-report")

    class MisdirectedReport:
        def __init__(self, directory, filename):
            self.report_path = Path(directory) / filename

        def write(self, *_args):
            other_path = self.report_path.with_name("other.html")
            other_path.write_text("other", encoding="ascii")
            return other_path

    monkeypatch.setattr(script, "report_factory", MisdirectedReport)

    exit_code = script.main(
        complete_args(tmp_path), input_func=answers("left", "yes", "yes", "yes")
    )

    run_directory = tmp_path / "single-laser-cal-misdirected-report"
    payload = json.loads(
        (run_directory / "single-laser-cal-run.json").read_text(encoding="utf-8")
    )
    assert exit_code == 1
    assert payload["status"] == "failed"
    assert payload["report_artifact"]["status"] == "failed"
    assert not (run_directory / "single-laser-cal-report.html").exists()
