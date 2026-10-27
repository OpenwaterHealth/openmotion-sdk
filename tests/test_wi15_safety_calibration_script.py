import json
from datetime import datetime, timezone

import pytest

from omotion.calibration.laser import FailureKind, ProcedureStatus
from omotion.calibration.safety import ShippingTopology
from omotion.calibration.safety_workflow import SafetyCalibrationResult
from omotion.calibration.single_sensor_laser import ReportArtifactStatus
from wi15_fakes import FakeClock
from wi15_script_harness import (
    FakeRecorder,
    FakeReport,
    load_wi15_script,
    wi15_script_path,
)


SCRIPT_PATH = wi15_script_path("wi15_safety_calibration.py")


def load_script():
    return load_wi15_script(SCRIPT_PATH, "wi15_safety_script_under_test")


class FakeBench:
    def __init__(self, coordinator):
        self.coordinator = coordinator
        self.closed = 0
        self.close_error = None

    def close(self):
        self.closed += 1
        if self.close_error:
            raise self.close_error


def _result(status=ProcedureStatus.PASSED, **changes):
    values = {
        "status": status,
        "shipping_topology": ShippingTopology.SINGLE_LEFT,
        "sdk_version": "test-sdk",
        "started_at": datetime.now(timezone.utc),
        "ended_at": datetime.now(timezone.utc),
    }
    values.update(changes)
    return SafetyCalibrationResult(**values)


def _complete_args(tmp_path):
    return [
        "--output-dir",
        str(tmp_path),
        "--operator",
        "operator",
        "--build-revision",
        "build-7",
        "--fixture-id",
        "bench-2",
    ]


def _configured_script(monkeypatch, tmp_path, terminal_result=None):
    script = load_script()
    recorder = FakeRecorder(tmp_path)
    captured = {}

    def make_bench(*, power_cycle_coordinator):
        bench = FakeBench(power_cycle_coordinator)
        captured["bench"] = bench
        return bench

    class FakeWorkflow:
        def __init__(self, bench, workflow_recorder):
            # main() hands the workflow narration wrappers around the exact
            # bench and recorder it constructed.
            assert bench.wrapped is captured["bench"]
            assert workflow_recorder.wrapped is recorder

        def run(self, request):
            captured["request"] = request
            return terminal_result or _result(
                shipping_topology=request.shipping_topology
            )

    monkeypatch.setattr(script, "recorder_factory", lambda *args: recorder)
    monkeypatch.setattr(script, "bench_factory", make_bench)
    monkeypatch.setattr(script, "workflow_factory", FakeWorkflow)
    monkeypatch.setattr(script, "report_factory", FakeReport)
    return script, recorder, captured


def test_metadata_topology_and_connection_confirmation_finish_before_hardware_construction(
    monkeypatch, tmp_path
):
    script = load_script()
    calls = []
    recorder = FakeRecorder(tmp_path)

    monkeypatch.setattr(
        script,
        "recorder_factory",
        lambda *args: calls.append("recorder") or recorder,
    )
    monkeypatch.setattr(
        script,
        "bench_factory",
        lambda **kwargs: calls.append("bench") or FakeBench(kwargs["power_cycle_coordinator"]),
    )

    class FakeWorkflow:
        def __init__(self, *_args):
            pass

        def run(self, request):
            calls.append(("workflow", request.shipping_topology))
            return _result(shipping_topology=request.shipping_topology)

    monkeypatch.setattr(script, "workflow_factory", FakeWorkflow)
    monkeypatch.setattr(script, "report_factory", FakeReport)
    replies = iter(("operator", "bench"))

    exit_code = script.main(
        ["--output-dir", str(tmp_path)],
        input_func=lambda prompt: calls.append(("prompt", prompt)) or next(replies),
    )

    assert exit_code == 0
    bench_index = calls.index("bench")
    prompts = [item for item in calls[:bench_index] if isinstance(item, tuple)]
    assert all("topology" not in prompt.lower() for _, prompt in prompts)
    assert calls[-1] == ("workflow", ShippingTopology.CONSOLE_ONLY)


def test_power_cycle_is_observed_without_confirmation_prompts():
    """The operator just flips power; the coordinator only observes."""
    script = load_script()
    clock = FakeClock()
    outputs = []

    def connected():
        return clock.now < 1.0 or clock.now >= 3.0

    def forbidden_input(_prompt):
        raise AssertionError("the power-cycle flow must not prompt for input")

    coordinator = script.ManualPowerCycleCoordinator(
        input_func=forbidden_input,
        output_func=outputs.append,
        clock=clock,
        wall_clock=lambda: 2_000.0 + clock.now,
        sleep=clock.sleep,
        poll_interval_s=0.5,
        disconnect_timeout_s=5.0,
        reconnect_timeout_s=5.0,
    )

    evidence = coordinator.perform(
        minimum_off_s=1.0,
        expected_console_serial="C-1",
        is_console_connected=connected,
        read_console_serial=lambda: "C-1",
    )

    assert evidence.disconnect_observed
    assert evidence.reconnect_observed
    assert evidence.restart_proven
    assert evidence.off_duration_s == pytest.approx(2.0)
    assert evidence.on_allowed_at <= evidence.reconnect_observed_at
    assert evidence.console_serial_before == "C-1"
    assert evidence.console_serial_after == "C-1"
    assert "without operator confirmation gates" in evidence.restart_proof
    assert any(
        "turn the console power off, then back on" in message.lower()
        for message in outputs
    )


def test_too_fast_power_cycle_measures_the_short_dwell_for_rejection():
    """A cycle quicker than the minimum dwell must be measurable as such."""
    script = load_script()
    clock = FakeClock()
    outputs = []

    def connected():
        return clock.now < 1.0 or clock.now >= 1.5

    coordinator = script.ManualPowerCycleCoordinator(
        input_func=lambda _prompt: "unused",
        output_func=outputs.append,
        clock=clock,
        wall_clock=lambda: 2_000.0 + clock.now,
        sleep=clock.sleep,
        poll_interval_s=0.25,
        disconnect_timeout_s=5.0,
        reconnect_timeout_s=5.0,
    )

    evidence = coordinator.perform(
        minimum_off_s=15.0,
        expected_console_serial="C-1",
        is_console_connected=connected,
        read_console_serial=lambda: "C-1",
    )

    assert evidence.disconnect_observed and evidence.reconnect_observed
    assert evidence.off_duration_s < 15.0
    assert evidence.on_allowed_at > evidence.reconnect_observed_at


def test_unobserved_disconnect_returns_failed_evidence_without_prompts():
    script = load_script()
    clock = FakeClock()
    outputs = []

    def forbidden_input(_prompt):
        raise AssertionError("the power-cycle flow must not prompt for input")

    coordinator = script.ManualPowerCycleCoordinator(
        input_func=forbidden_input,
        output_func=outputs.append,
        clock=clock,
        wall_clock=lambda: 2_000.0 + clock.now,
        sleep=clock.sleep,
        poll_interval_s=0.25,
        disconnect_timeout_s=1.0,
        reconnect_timeout_s=1.0,
    )

    evidence = coordinator.perform(
        minimum_off_s=1.0,
        expected_console_serial="C-1",
        is_console_connected=lambda: True,
        read_console_serial=lambda: "C-1",
    )

    assert not evidence.disconnect_observed
    assert not evidence.reconnect_observed
    assert not evidence.restart_proven
    assert evidence.off_duration_s is None
    assert any(
        "did not turn off" in message.lower()
        for message in outputs
    )


def test_runner_declares_console_only_topology_and_finalizes_artifacts(
    monkeypatch, tmp_path
):
    script, recorder, captured = _configured_script(monkeypatch, tmp_path)
    messages = []

    exit_code = script.main(
        _complete_args(tmp_path),
        input_func=lambda _prompt: "unused",
        output_func=messages.append,
    )

    assert exit_code == 0
    assert captured["request"].shipping_topology is ShippingTopology.CONSOLE_ONLY
    assert captured["bench"].coordinator is not None
    assert captured["bench"].closed == 1
    assert recorder.checkpoints[-1].report_artifact.status is ReportArtifactStatus.FINALIZED
    assert "Final result: PASS" in messages
    assert any(message.startswith("Saved data (JSON): ") for message in messages)
    assert any(message.startswith("Saved report (HTML): ") for message in messages)


