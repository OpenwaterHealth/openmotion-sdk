import json
from dataclasses import replace
from datetime import datetime, timezone

import pytest

from omotion.calibration.dual_sensor_laser import (
    DualSensorLaserCalibrationResult,
    PlacementChangeRequest,
)
from omotion.calibration.laser import FailureKind, ProcedureStatus
from omotion.calibration.single_sensor_laser import ReportArtifactStatus
from wi15_script_harness import (
    FakeBench,
    FakeMeter,
    FakeRecorder,
    FakeReport,
    complete_args,
    load_wi15_script,
    wi15_script_path,
)


SCRIPT_PATH = wi15_script_path("wi15_dual_sensor_laser_calibration.py")


def load_script():
    return load_wi15_script(SCRIPT_PATH, "wi15_dual_script_under_test")


def result(status=ProcedureStatus.PASSED):
    return DualSensorLaserCalibrationResult(
        status=status,
        sdk_version="test-sdk",
        started_at=datetime.now(timezone.utc),
        ended_at=datetime.now(timezone.utc),
    )


def configured_script(monkeypatch, tmp_path, terminal_result=None, *, request_placements=False):
    script = load_script()
    recorder = FakeRecorder(tmp_path)
    meter = FakeMeter()
    bench = FakeBench(meter)
    captured = {}

    class FakeWorkflow:
        def __init__(self, workflow_bench, workflow_recorder, placement_callback):
            # main() hands the workflow narration wrappers around the exact
            # bench and recorder it constructed.
            assert workflow_bench.wrapped is bench
            assert workflow_recorder.wrapped is recorder
            self.placement_callback = placement_callback

        def run(self, request):
            captured["request"] = request
            if request_placements:
                captured["placement_results"] = [
                    self.placement_callback(
                        PlacementChangeRequest(
                            None,
                            "left",
                            "LEFT-001",
                            "Initial paired measurement",
                            "Initial paired measurement - place the left sensor in the Ophir 0 cm fixture",
                        )
                    ),
                    self.placement_callback(
                        PlacementChangeRequest(
                            "left",
                            "right",
                            "RIGHT-001",
                            "Initial paired measurement",
                            "Initial paired measurement - switch to the right sensor",
                        )
                    ),
                ]
            return terminal_result or result()

    monkeypatch.setattr(script, "recorder_factory", lambda *args: recorder)
    monkeypatch.setattr(script, "meter_factory", lambda: meter)
    monkeypatch.setattr(script, "bench_factory", lambda value: bench)
    monkeypatch.setattr(script, "workflow_factory", FakeWorkflow)
    monkeypatch.setattr(script, "report_factory", FakeReport)
    return script, recorder, meter, bench, captured


def test_placement_callback_is_courteous_and_names_side_serial_and_zero_cm_fixture(
    monkeypatch, tmp_path
):
    script, _recorder, _meter, bench, captured = configured_script(
        monkeypatch, tmp_path, request_placements=True
    )
    prompts = []
    messages = []
    replies = iter(("yes", "yes"))

    def prompt(text):
        prompts.append(text)
        return next(replies)

    exit_code = script.main(
        complete_args(tmp_path), input_func=prompt, output_func=messages.append
    )

    assert exit_code == 0
    assert captured["placement_results"] == [True, True]
    assert "left sensor (serial LEFT-001)" in prompts[0]
    assert "right sensor (serial RIGHT-001)" in prompts[1]
    assert all("0 cm fixture" in prompt for prompt in prompts)
    assert any("Initial paired measurement" in message for message in messages)
    assert all("_measure" not in text for text in prompts + messages)
    assert bench.closed == 1


def test_declined_placement_is_passed_to_workflow_as_false(monkeypatch, tmp_path):
    script, _recorder, _meter, _bench, captured = configured_script(
        monkeypatch,
        tmp_path,
        terminal_result=result(ProcedureStatus.CANCELED),
        request_placements=True,
    )
    replies = iter(("yes", "no"))
    exit_code = script.main(complete_args(tmp_path), input_func=lambda _: next(replies))
    assert exit_code == 1
    assert captured["placement_results"] == [True, False]


def test_hardware_close_failure_replaces_pending_pass_before_report_finalization(
    monkeypatch, tmp_path
):
    script, recorder, _meter, bench, _captured = configured_script(
        monkeypatch, tmp_path
    )
    reported_results = []

    def fail_close():
        bench.closed += 1
        raise RuntimeError("Motion shutdown transport failed")

    class CapturingReport(FakeReport):
        def write(self, request, result, json_path):
            reported_results.append(result)
            return super().write(request, result, json_path)

    bench.close = fail_close
    monkeypatch.setattr(script, "report_factory", CapturingReport)

    exit_code = script.main(complete_args(tmp_path), input_func=lambda _: "yes")

    assert exit_code == 1
    assert bench.closed == 1
    terminal = recorder.checkpoints[-1]
    assert terminal.status is ProcedureStatus.FAILED
    assert terminal.failure_kind is FailureKind.MEASUREMENT
    assert terminal.failure_reason == "Hardware resource cleanup failed."
    assert terminal.resource_cleanup_failure == "Motion shutdown transport failed"
    assert reported_results[-1].resource_cleanup_failure == (
        "Motion shutdown transport failed"
    )
