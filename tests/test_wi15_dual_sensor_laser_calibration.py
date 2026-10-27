from collections import deque
from dataclasses import replace
from pathlib import Path

import pytest

from omotion.calibration.dual_sensor_laser import (
    DualPreflightSnapshot,
    DualSensorLaserCalibrationRequest,
    DualSensorLaserCalibrationWorkflow,
)
from omotion.calibration.laser import (
    DeviceIdentity,
    FailureKind,
    OphirIdentity,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
    default_user_configuration,
)
from wi15_builders import (
    FPGA_REVISIONS,
    valid_measurement_for_mean as valid_measurement,
    valid_ophir_setting_evidence as valid_ophir_evidence,
)
from wi15_fakes import FakeRecorder


class FakeDualBench:
    def __init__(
        self,
        measurements=(),
        *,
        topology=TopologySnapshot(True, True, True),
        console_serial="CONSOLE-001",
        left_serial="LEFT-001",
        right_serial="RIGHT-001",
        fpga_revisions=FPGA_REVISIONS,
        ophir_ready=True,
        config_write_results=(),
        register_read_queues=None,
    ):
        self.calls = []
        self.measurements = deque(measurements)
        self.topology = topology
        self.config_write_results = deque(config_write_results)
        self.register_read_queues = {
            name: deque(values)
            for name, values in (register_read_queues or {}).items()
        }
        self.user_configuration_writes = []
        self.register_writes = []
        self.stop_count = 0
        self.pre_existing_config = {"customer_key": 17}
        self.registers = {
            "TA_CURRENT_DRV": 5000.0,
            "TA_PULSE_WIDTH": 500.0,
            "SEED_CW_GAIN": 140.0,
            "EE_PULSE_WIDTH_UL": 550.0,
            "OPT_PULSE_WIDTH_UL": 550.0,
        }
        self.snapshot = DualPreflightSnapshot(
            topology=topology,
            console_identity=DeviceIdentity(
                "console",
                console_serial,
                "console-fw",
                "console-hw",
                "fpga-fw",
                fpga_revisions,
            ),
            left_sensor_identity=DeviceIdentity(
                "left sensor", left_serial, "left-fw", "left-hw"
            ),
            right_sensor_identity=DeviceIdentity(
                "right sensor", right_serial, "right-fw", "right-hw"
            ),
            console_responsive=True,
            ophir_identity=OphirIdentity(
                "Ophir meter", "METER-001", "Ophir sensor", "HEAD-001", "2027-01-01"
            ),
            ophir_ready=ophir_ready,
            ophir_setting_evidence=valid_ophir_evidence(),
            ophir_failure_reason=None if ophir_ready else "Ophir preflight failed.",
        )

    def preflight_dual(self):
        self.calls.append("preflight_dual")
        return self.snapshot

    def revalidate_dual_topology(self):
        self.calls.append("revalidate_dual_topology")
        return self.topology

    def read_user_configuration(self):
        self.calls.append("read_user_configuration")
        return dict(self.pre_existing_config)

    def write_user_configuration(self, configuration):
        written = dict(configuration)
        phase = "default" if not self.user_configuration_writes else "final"
        self.calls.append(f"write_user_configuration:{phase}")
        self.user_configuration_writes.append(written)
        if self.config_write_results:
            result = self.config_write_results.popleft()
            if isinstance(result, Exception):
                raise result
            return dict(result) if isinstance(result, dict) else result
        return written

    def bring_up_laser_configuration(self):
        self.calls.append("bring_up_laser_configuration")

    def read_register(self, name):
        self.calls.append(f"read_register:{name}")
        queued = self.register_read_queues.get(name)
        if queued:
            return queued.popleft()
        return self.registers[name]

    def write_register(self, name, value):
        requested = float(value)
        self.calls.append(f"write_register:{name}:{requested:g}")
        self.register_writes.append((name, requested))
        self.registers[name] = requested
        return SettingReadback(name, requested, requested)

    def read_trigger_rate_hz(self):
        self.calls.append("read_trigger_rate_hz")
        return 40.0

    def write_trigger_rate_hz(self, rate_hz):
        self.calls.append(f"write_trigger_rate_hz:{rate_hz:g}")
        return SettingReadback("trigger_rate_hz_write", rate_hz, rate_hz)

    def measure_energy(self):
        self.calls.append("measure_energy")
        if not self.measurements:
            raise AssertionError("Test did not queue enough energy measurements")
        return self.measurements.popleft()

    def stop_trigger(self):
        self.calls.append("stop_trigger")
        self.stop_count += 1


class PlacementResponses:
    def __init__(self, responses=()):
        self.requests = []
        self.responses = deque(responses)

    def __call__(self, request):
        self.requests.append(request)
        response = self.responses.popleft() if self.responses else True
        if isinstance(response, BaseException):
            raise response
        return response


def valid_request(tmp_path=Path("run-output")):
    return DualSensorLaserCalibrationRequest(
        operator="Operator A",
        build_id="BUILD-001",
        fixture_id="OPHIR-0CM-001",
        fixture_calibration_status="Current",
        procedure_id="WI-00015",
        output_root=tmp_path,
        run_id="DUAL-001",
        sdk_version="test-sdk",
    )


def run_workflow(
    bench,
    *,
    responses=(),
    target_energy_uj=350.0,
    minimum_accepted_energy_uj=300.0,
    maximum_accepted_energy_uj=400.0,
):
    recorder = FakeRecorder()
    placements = PlacementResponses(responses)
    result = DualSensorLaserCalibrationWorkflow(
        bench,
        recorder,
        placements,
        target_energy_uj=target_energy_uj,
        minimum_accepted_energy_uj=minimum_accepted_energy_uj,
        maximum_accepted_energy_uj=maximum_accepted_energy_uj,
    ).run(valid_request())
    return result, recorder, placements


@pytest.mark.parametrize(
    ("changes", "reason_fragment"),
    [
        (
            {"topology": TopologySnapshot(True, True, False)},
            "Only the left sensor is connected; use Single-Sensor Laser Calibration",
        ),
        (
            {"topology": TopologySnapshot(True, False, True)},
            "Only the right sensor is connected; use Single-Sensor Laser Calibration",
        ),
        ({"console_serial": None}, "Console serial"),
        ({"left_serial": "  "}, "Left-sensor serial"),
        ({"right_serial": None}, "Right-sensor serial"),
        ({"fpga_revisions": ()}, "TA, SEED, SAFETY_EE, and SAFETY_OPT"),
        ({"ophir_ready": False}, "Ophir preflight failed"),
    ],
)
def test_dual_preflight_failure_prevents_configuration_and_firing(
    changes, reason_fragment
):
    bench = FakeDualBench(**changes)
    result, recorder, placements = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.SETUP
    assert reason_fragment in result.failure_reason
    assert bench.user_configuration_writes == []
    assert "measure_energy" not in bench.calls
    assert placements.requests == []
    assert recorder.checkpoints[-1] == result


def test_exact_100_initial_pair_continues_with_side_change_only_prompts():
    bench = FakeDualBench(
        [
            valid_measurement(300),
            valid_measurement(400),
            valid_measurement(300),
            valid_measurement(400),
        ]
    )
    result, _, placements = run_workflow(bench)
    assert result.status is ProcedureStatus.PASSED
    assert result.initial_pair.metrics.difference_uj == 100
    assert [item.to_side for item in placements.requests] == [
        "left",
        "right",
        "left",
        "right",
    ]
    assert [item.sensor_serial for item in placements.requests[:2]] == [
        "LEFT-001",
        "RIGHT-001",
    ]
    assert len(result.crosschecks) == 1
    assert result.crosschecks[0].accepted


def test_initial_differential_above_100_is_terminal_ncr_without_final_write():
    bench = FakeDualBench([valid_measurement(299), valid_measurement(400)])
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    assert result.initial_pair.metrics.difference_uj == 101
    assert len(bench.user_configuration_writes) == 1
    assert result.requested_final_config is None
    assert result.crosschecks == ()


def test_declined_sensor_switch_cancels_before_the_associated_measurement():
    bench = FakeDualBench([valid_measurement(350)])
    result, _, placements = run_workflow(bench, responses=[True, False])
    assert result.status is ProcedureStatus.CANCELED
    assert result.failure_kind is FailureKind.CANCELED
    assert [item.to_side for item in placements.requests] == ["left", "right"]
    assert bench.calls.count("measure_energy") == 1
    assert len(bench.user_configuration_writes) == 1


def test_canceled_sensor_switch_acknowledgement_cancels_the_run():
    """Ctrl-C / EOF at the placement prompt is a cancel, not a crash."""
    bench = FakeDualBench([valid_measurement(350)])
    result, _, placements = run_workflow(
        bench, responses=[True, KeyboardInterrupt()]
    )
    assert result.status is ProcedureStatus.CANCELED
    assert result.failure_kind is FailureKind.CANCELED
    assert "canceled the requested switch to the right sensor" in result.failure_reason
    assert bench.calls.count("measure_energy") == 1
    assert len(bench.user_configuration_writes) == 1


def test_final_configuration_write_failure_fails_without_pass_claim():
    default_config = default_user_configuration()
    bench = FakeDualBench(
        [
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360),
        ],
        config_write_results=[
            default_config,
            RuntimeError("EEPROM write refused"),
        ],
    )
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert "write or readback failed" in result.failure_reason
    assert result.final_config_readback is None


def test_downward_tuning_targets_higher_side_without_reprompting_during_sweep():
    bench = FakeDualBench(
        [
            valid_measurement(360),
            valid_measurement(420),
            valid_measurement(400),
            valid_measurement(375),
            valid_measurement(330),
            valid_measurement(370),
        ]
    )
    result, _, placements = run_workflow(bench)
    assert result.status is ProcedureStatus.PASSED
    round_one = result.tuning_rounds[0]
    assert round_one.direction == "downward_current"
    assert round_one.selected_side == "right"
    assert round_one.target_uj == 380
    assert "higher energy reading" in round_one.reason
    assert [write for write in bench.register_writes if write[0] == "TA_CURRENT_DRV"] == [
        ("TA_CURRENT_DRV", 4950.0),
        ("TA_CURRENT_DRV", 4900.0),
    ]
    assert [item.to_side for item in placements.requests] == [
        "left",
        "right",
        "left",
        "right",
    ]
    assert result.requested_final_config["TA_CURRENT_DRV"] == 4900
    assert result.requested_final_config["EE_PULSE_WIDTH_UL"] == 550


def test_upward_tuning_targets_lower_side_and_retains_660_limits():
    bench = FakeDualBench(
        [
            valid_measurement(270),
            valid_measurement(330),
            valid_measurement(290),
            valid_measurement(310),
            valid_measurement(325),
            valid_measurement(325),
            valid_measurement(375),
        ]
    )
    result, _, placements = run_workflow(bench)
    assert result.status is ProcedureStatus.PASSED
    round_one = result.tuning_rounds[0]
    assert round_one.direction == "upward_pulse"
    assert round_one.selected_side == "left"
    assert round_one.target_uj == 320
    assert "lower energy reading" in round_one.reason
    assert bench.register_writes[:2] == [
        ("EE_PULSE_WIDTH_UL", 660.0),
        ("OPT_PULSE_WIDTH_UL", 660.0),
    ]
    assert [write for write in bench.register_writes if write[0] == "TA_PULSE_WIDTH"] == [
        ("TA_PULSE_WIDTH", 510.0),
        ("TA_PULSE_WIDTH", 520.0),
        ("TA_PULSE_WIDTH", 530.0),
    ]
    assert [item.to_side for item in placements.requests] == [
        "left",
        "right",
        "left",
        "right",
    ]
    assert result.requested_final_config["TA_PULSE_WIDTH"] == 530
    assert result.requested_final_config["EE_PULSE_WIDTH_UL"] == 660
    assert result.requested_final_config["OPT_PULSE_WIDTH_UL"] == 660


def test_three_complete_nonpassing_crosschecks_fail_ncr_and_never_start_fourth():
    means = [260, 340, 290, 310, 299, 370, 315, 299, 380, 310, 299, 390]
    bench = FakeDualBench([valid_measurement(mean) for mean in means])
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED_NCR
    assert [item.number for item in result.crosschecks] == [1, 2, 3]
    assert not any(item.accepted for item in result.crosschecks)
    assert bench.calls.count("measure_energy") == len(means)
    assert len(bench.user_configuration_writes) == 1


@pytest.mark.parametrize(
    ("means", "passing_crosscheck"),
    [
        ([260, 340, 290, 310, 300, 370], 1),
        ([260, 340, 290, 310, 299, 370, 315, 300, 380], 2),
        (
            [260, 340, 290, 310, 299, 370, 315, 299, 380, 310, 300, 390],
            3,
        ),
    ],
)
def test_dual_workflow_may_pass_on_any_of_three_complete_crosschecks(
    means, passing_crosscheck
):
    bench = FakeDualBench([valid_measurement(mean) for mean in means])

    result, _, _ = run_workflow(bench)

    assert result.status is ProcedureStatus.PASSED
    assert len(result.crosschecks) == passing_crosscheck
    assert result.crosschecks[-1].accepted is True
    assert bench.calls.count("measure_energy") == len(means)


def test_invalid_partial_crosscheck_does_not_create_a_complete_crosscheck():
    bench = FakeDualBench(
        [
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360, n=25),
        ]
    )
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert result.crosschecks == ()
    assert len(result.observations) == 4
    assert len(bench.user_configuration_writes) == 1


def test_final_active_setting_outside_two_percent_prevents_passing_config_write():
    bench = FakeDualBench(
        [
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360),
        ],
        register_read_queues={"TA_CURRENT_DRV": [5000.0, 5100.001]},
    )
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.final_setting_checks[-1].passed is False
    assert len(bench.user_configuration_writes) == 1


def test_final_configuration_requires_exact_immediate_complete_readback():
    default_config = default_user_configuration()
    mismatched_final = dict(default_config)
    mismatched_final["TA_CURRENT_DRV"] = 4999
    bench = FakeDualBench(
        [
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360),
        ],
        config_write_results=[default_config, mismatched_final],
    )
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.requested_final_config["TA_CURRENT_DRV"] == 5000
    assert result.final_config_readback["TA_CURRENT_DRV"] == 4999
    assert len(bench.user_configuration_writes) == 2


def test_every_observation_has_an_auditor_facing_label_side_and_serial():
    bench = FakeDualBench(
        [
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360),
        ]
    )
    result, recorder, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.PASSED
    assert all("sensor" in item.label.lower() for item in result.observations)
    assert [(item.side, item.sensor_serial) for item in result.observations] == [
        ("left", "LEFT-001"),
        ("right", "RIGHT-001"),
        ("left", "LEFT-001"),
        ("right", "RIGHT-001"),
    ]
    assert any("because" in event.message for event in recorder.events if event.stage == "tuning")


def test_failed_sweep_retains_round_target_reason_and_completed_steps():
    """Losing the active round on a later write failure would leave an audit gap."""

    class FailingSecondCurrentWriteBench(FakeDualBench):
        def write_register(self, name, value):
            if name == "TA_CURRENT_DRV" and sum(
                written_name == name for written_name, _ in self.register_writes
            ) == 1:
                raise RuntimeError("current transport failed")
            return super().write_register(name, value)

    bench = FailingSecondCurrentWriteBench(
        [
            valid_measurement(360),
            valid_measurement(420),
            valid_measurement(400),
        ]
    )
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert len(result.tuning_rounds) == 1
    active_round = result.tuning_rounds[0]
    assert active_round.selected_side == "right"
    assert active_round.target_uj == 380
    assert "higher energy reading" in active_round.reason
    assert len(active_round.steps) == 1
    assert active_round.steps[0].requested_value == 4950


def test_later_upward_round_at_600_below_300_is_immediate_ncr():
    """A second round must not attempt another cross-check beyond the pulse bound."""
    means = [250, 350, 255, 260, 265, 270, 275, 280, 285, 290, 295, 300, 299, 350]
    bench = FakeDualBench([valid_measurement(mean) for mean in means])
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    assert len(result.crosschecks) == 1
    assert bench.calls.count("measure_energy") == len(means)
    assert len(bench.user_configuration_writes) == 1


def test_topology_change_immediately_before_default_write_prevents_mutation():
    bench = FakeDualBench()
    bench.topology = TopologySnapshot(True, True, False)
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.SETUP
    assert "changed before configuration mutation" in result.failure_reason
    assert bench.user_configuration_writes == []
    assert "measure_energy" not in bench.calls


def test_default_configuration_requires_exact_complete_immediate_readback():
    incomplete = default_user_configuration()
    incomplete.pop("TEC_TRIP")
    bench = FakeDualBench(config_write_results=[incomplete])
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.default_config_readback == incomplete
    assert "bring_up_laser_configuration" not in bench.calls
    assert "measure_energy" not in bench.calls


def test_downward_current_floor_without_acceptable_candidate_is_ncr():
    bench = FakeDualBench(
        [valid_measurement(410), valid_measurement(390)]
        + [valid_measurement(450) for _ in range(60)]
    )
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    current_writes = [value for name, value in bench.register_writes if name == "TA_CURRENT_DRV"]
    assert current_writes[0] == 4950
    assert 2000 in current_writes
    assert all(value >= 2000 for value in current_writes)
    assert len(bench.user_configuration_writes) == 1


def test_upward_pulse_ceiling_below_300_is_ncr_and_never_exceeds_600():
    means = [250, 350, 255, 260, 265, 270, 275, 280, 285, 290, 295, 299]
    bench = FakeDualBench([valid_measurement(mean) for mean in means])
    result, _, _ = run_workflow(bench)
    assert result.status is ProcedureStatus.FAILED_NCR
    pulse_writes = [value for name, value in bench.register_writes if name == "TA_PULSE_WIDTH"]
    assert 600 in pulse_writes
    assert all(value <= 600 for value in pulse_writes)
    assert result.crosschecks == ()
    assert len(bench.user_configuration_writes) == 1


def test_non_boolean_placement_response_cancels_fail_closed():
    bench = FakeDualBench()
    result, _, placements = run_workflow(bench, responses=["yes"])
    assert result.status is ProcedureStatus.CANCELED
    assert len(placements.requests) == 1
    assert "measure_energy" not in bench.calls
    assert len(bench.user_configuration_writes) == 1


def test_reused_workflow_starts_each_execution_with_an_unseated_fixture():
    bench = FakeDualBench(
        [
            valid_measurement(350),
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360),
        ]
    )
    recorder = FakeRecorder()
    placements = PlacementResponses([True, False, True, True, True, True])
    workflow = DualSensorLaserCalibrationWorkflow(bench, recorder, placements)

    first = workflow.run(valid_request())
    second = workflow.run(replace(valid_request(), run_id="DUAL-002"))

    assert first.status is ProcedureStatus.CANCELED
    assert second.status is ProcedureStatus.PASSED
    assert [item.to_side for item in placements.requests] == [
        "left",
        "right",
        "left",
        "right",
        "left",
        "right",
    ]


def test_final_trigger_stop_failure_changes_pending_pass_to_failed():
    class FailingFinalTriggerStopBench(FakeDualBench):
        def stop_trigger(self):
            self.calls.append("stop_trigger")
            self.stop_count += 1
            if self.stop_count == 5:
                raise RuntimeError("trigger transport failed")

    bench = FailingFinalTriggerStopBench(
        [
            valid_measurement(340),
            valid_measurement(360),
            valid_measurement(340),
            valid_measurement(360),
        ]
    )

    result, recorder, _ = run_workflow(bench)

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert result.trigger_cleanup_failure == "Trigger stop failed."
    assert result.requested_final_config is not None
    assert recorder.checkpoints[-1] == result
