from collections import deque
from dataclasses import replace
from datetime import datetime, timedelta, timezone
import math

import pytest

from omotion.calibration.laser import (
    DeviceIdentity,
    FailureKind,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
)
from omotion.calibration.safety import (
    NormalScanEvidence,
    SafetyWarningEvidence,
    ShippingTopology,
)
from omotion.calibration.safety_workflow import (
    AdcSamplingPolicy,
    ConsolePreflightSnapshot,
    SafetyCalibrationRequest,
    SafetyCalibrationWorkflow,
)
from wi15_builders import FPGA_REVISIONS, valid_power_cycle, valid_safety_config
from wi15_fakes import FakeRecorder


NOW = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)


def _valid_config(**changes):
    config = valid_safety_config(FACTORY_NOTE=7)
    config.update(changes)
    return config


def _clear_warning(index=0):
    return SafetyWarningEvidence(
        NOW + timedelta(milliseconds=index),
        True,
        True,
        (),
        {"so": 0, "se": 0},
    )


def _valid_scan(topology=ShippingTopology.SINGLE_LEFT):
    topology_snapshot = {
        ShippingTopology.SINGLE_LEFT: TopologySnapshot(True, True, False),
        ShippingTopology.SINGLE_RIGHT: TopologySnapshot(True, False, True),
        ShippingTopology.DUAL: TopologySnapshot(True, True, True),
    }[topology]
    identities = []
    if topology_snapshot.left_connected:
        identities.append(DeviceIdentity("left sensor", "L-1", "1", "HL"))
    else:
        identities.append(DeviceIdentity("left sensor", None, None, None))
    if topology_snapshot.right_connected:
        identities.append(DeviceIdentity("right sensor", "R-1", "1", "HR"))
    else:
        identities.append(DeviceIdentity("right sensor", None, None, None))
    return NormalScanEvidence(
        declared_topology=topology,
        requested_duration_s=30.0,
        actual_duration_s=30.0,
        started=True,
        completed=True,
        canceled=False,
        error=None,
        topology=topology_snapshot,
        identities=tuple(identities),
        overrides={},
        safety_observations=(_clear_warning(),),
        warnings=(),
    )


def _valid_power_cycle():
    return valid_power_cycle(
        NOW, restart_proof="firmware uptime reset from 734 s to 2 s"
    )


class FakeSafetyBench:
    def __init__(self, *, topology=ShippingTopology.SINGLE_LEFT):
        self.preflight = ConsolePreflightSnapshot(
            topology=TopologySnapshot(True, True, False),
            console_identity=DeviceIdentity(
                "console", "C-1", "1.2", "HC", "FPGA", FPGA_REVISIONS
            ),
            console_responsive=True,
        )
        self.current_config = _valid_config()
        self.active_registers = {
            "TA_CURRENT_DRV": 5000.0,
            "TA_PULSE_WIDTH": 500.0,
        }
        self.trigger_rates = deque([40.0])
        self.trigger_write_result = SettingReadback("trigger_rate_hz_write", 40.0, 40.0)
        self.adc = {
            "SAFETY_OPT": deque([100.0] * 10),
            "SAFETY_EE": deque([200.0] * 10),
        }
        self.warnings = deque(_clear_warning(index) for index in range(40))
        self.write_result = None
        self.written_config = None
        self.power_cycle_result = _valid_power_cycle()
        self.post_restart_config = None
        self.scan_result = _valid_scan(topology)
        self.calls = []
        self.trigger_starts = 0
        self.trigger_stops = 0
        self.raise_stop = False
        self.mutations = []
        self.scan_requests = []

    def preflight_console(self):
        self.calls.append("preflight_console")
        return self.preflight

    def read_user_configuration(self):
        self.calls.append("read_user_configuration")
        if self.post_restart_config is not None and "power_cycle" in self.calls:
            return dict(self.post_restart_config)
        if self.written_config is not None and "power_cycle" in self.calls:
            return dict(self.written_config)
        return dict(self.current_config)

    def bring_up_laser_configuration(self):
        self.calls.append("bring_up_laser_configuration")

    def read_register(self, name):
        self.calls.append(f"read_register:{name}")
        return self.active_registers[name]

    def read_trigger_rate_hz(self):
        self.calls.append("read_trigger_rate_hz")
        return self.trigger_rates.popleft() if len(self.trigger_rates) > 1 else self.trigger_rates[0]

    def write_trigger_rate_hz(self, value):
        self.calls.append(f"write_trigger_rate_hz:{value}")
        self.mutations.append(("trigger", value))
        return self.trigger_write_result

    def start_trigger(self):
        self.calls.append("start_trigger")
        self.trigger_starts += 1

    def read_adc_ma(self, controller):
        self.calls.append(f"read_adc_ma:{controller}")
        values = self.adc[controller]
        value = values.popleft() if len(values) > 1 else values[0]
        if isinstance(value, BaseException):
            raise value
        return value

    def read_safety_warning(self):
        self.calls.append("read_safety_warning")
        value = self.warnings.popleft() if len(self.warnings) > 1 else self.warnings[0]
        if isinstance(value, BaseException):
            raise value
        return value

    def stop_trigger(self):
        self.calls.append("stop_trigger")
        self.trigger_stops += 1
        if self.raise_stop:
            raise RuntimeError("stop relay did not acknowledge")

    def write_user_configuration(self, configuration):
        requested = dict(configuration)
        self.calls.append("write_user_configuration")
        self.mutations.append(("configuration", requested))
        if self.write_result is None:
            self.written_config = requested
            return requested
        if isinstance(self.write_result, BaseException):
            raise self.write_result
        return dict(self.write_result)

    def power_cycle(self, *, minimum_off_s, expected_console_serial):
        self.calls.append("power_cycle")
        self.mutations.append(("power_cycle", minimum_off_s, expected_console_serial))
        return self.power_cycle_result

    def run_normal_scan(self, declared_topology, *, duration_s):
        self.calls.append("run_normal_scan")
        self.scan_requests.append((declared_topology, duration_s))
        return self.scan_result


def _request(topology=ShippingTopology.SINGLE_LEFT):
    return SafetyCalibrationRequest(
        shipping_topology=topology,
        operator="Ada",
        build_id="BUILD-1",
        fixture_id="BENCH-1",
        procedure_id="WI-00015 Safety Calibration",
        output_root="runs",
        run_id="RUN-1",
        started_at=NOW,
    )


def _run(bench=None, *, topology=ShippingTopology.SINGLE_LEFT, policy=None):
    bench = bench or FakeSafetyBench(topology=topology)
    recorder = FakeRecorder()
    workflow = SafetyCalibrationWorkflow(
        bench,
        recorder,
        sampling_policy=policy or AdcSamplingPolicy(interval_s=0.0),
        sleep_func=lambda _seconds: None,
        now_func=lambda: NOW,
    )
    return workflow.run(_request(topology)), bench, recorder


@pytest.mark.parametrize(
    "preflight",
    [
        ConsolePreflightSnapshot(
            TopologySnapshot(False, False, False),
            DeviceIdentity("console", "C-1", None, None),
            False,
        ),
        ConsolePreflightSnapshot(
            TopologySnapshot(True, False, False),
            DeviceIdentity("console", None, None, None),
            True,
        ),
        ConsolePreflightSnapshot(
            TopologySnapshot(True, False, False),
            DeviceIdentity("console", "  ", None, None),
            True,
        ),
        ConsolePreflightSnapshot(
            TopologySnapshot(True, False, False),
            DeviceIdentity("console", "C-1", None, None),
            False,
        ),
    ],
)
def test_setup_failure_prevents_bringup_adc_write_and_scan(preflight):
    bench = FakeSafetyBench()
    bench.preflight = preflight

    result, bench, _ = _run(bench)

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.SETUP
    assert result.failure_reason
    assert bench.mutations == []
    assert bench.trigger_starts == 0
    assert bench.scan_requests == []


def test_missing_console_fpga_revisions_prevents_safety_calibration_mutation():
    bench = FakeSafetyBench()
    bench.preflight = replace(
        bench.preflight,
        console_identity=replace(
            bench.preflight.console_identity,
            fpga_firmware_revisions=(),
        ),
    )

    result, bench, _ = _run(bench)

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.SETUP
    assert "TA, SEED, SAFETY_EE, and SAFETY_OPT" in result.failure_reason
    assert bench.mutations == []
    assert bench.trigger_starts == 0


def test_sensor_modules_are_not_a_precondition_for_console_adc_calibration():
    bench = FakeSafetyBench()
    bench.preflight = replace(
        bench.preflight,
        topology=TopologySnapshot(True, False, False),
    )

    result, bench, _ = _run(bench)

    assert result.status is ProcedureStatus.PASSED
    assert bench.trigger_starts == 1


def test_current_configuration_is_checkpointed_before_invalid_field_fails():
    bench = FakeSafetyBench()
    bench.current_config = _valid_config(TA_CURRENT_DRV=0)

    result, bench, recorder = _run(bench)

    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.current_configuration["TA_CURRENT_DRV"] == 0
    assert any(
        checkpoint.current_configuration is not None
        for checkpoint in recorder.checkpoints
    )
    assert bench.trigger_starts == 0
    assert not any(item[0] == "configuration" for item in bench.mutations)


@pytest.mark.parametrize(
    ("name", "actual", "passed"),
    [
        ("TA_CURRENT_DRV", 4900.0, True),
        ("TA_CURRENT_DRV", 5100.0, True),
        ("TA_CURRENT_DRV", 4899.999, False),
        ("TA_CURRENT_DRV", 5100.001, False),
        ("TA_PULSE_WIDTH", 490.0, True),
        ("TA_PULSE_WIDTH", 510.0, True),
        ("TA_PULSE_WIDTH", 489.999, False),
        ("TA_PULSE_WIDTH", 510.001, False),
    ],
)
def test_active_ta_settings_use_inclusive_two_percent_gate(name, actual, passed):
    bench = FakeSafetyBench()
    bench.active_registers[name] = actual

    result, bench, _ = _run(bench)

    assert (result.status is ProcedureStatus.PASSED) is passed
    check = next(item for item in result.active_setting_checks if item.name == name)
    assert check.passed is passed
    if not passed:
        assert bench.trigger_starts == 0


def test_only_out_of_range_trigger_rate_is_corrected_and_reverified():
    bench = FakeSafetyBench()
    bench.trigger_rates = deque([37.0, 40.0])

    result, bench, _ = _run(bench)

    assert result.status is ProcedureStatus.PASSED
    assert result.trigger_readbacks == (
        SettingReadback("trigger_rate_hz_initial", 40.0, 37.0),
        SettingReadback("trigger_rate_hz_write", 40.0, 40.0),
        SettingReadback("trigger_rate_hz_final", 40.0, 40.0),
    )
    assert bench.mutations[0] == ("trigger", 40.0)


@pytest.mark.parametrize(
    "write_result",
    [
        None,
        SettingReadback("wrong_name", 40.0, 40.0),
        SettingReadback("trigger_rate_hz_write", 39.0, 40.0),
        SettingReadback("trigger_rate_hz_write", 40.0, math.nan),
    ],
)
def test_malformed_immediate_trigger_write_fails_before_firing(write_result):
    bench = FakeSafetyBench()
    bench.trigger_rates = deque([37.0, 40.0])
    bench.trigger_write_result = write_result

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.CONFIGURATION
    assert "trigger" in result.failure_reason.lower()
    assert bench.trigger_starts == 0


def test_adc_reads_are_interleaved_and_use_all_ten_scaled_ma_values():
    bench = FakeSafetyBench()
    bench.adc["SAFETY_OPT"] = deque(float(value) for value in range(100, 110))
    bench.adc["SAFETY_EE"] = deque(float(value) for value in range(200, 210))

    result, bench, _ = _run(bench)

    assert result.status is ProcedureStatus.PASSED
    assert [
        call for call in bench.calls if call.startswith("read_adc_ma")
    ][:4] == [
        "read_adc_ma:SAFETY_OPT",
        "read_adc_ma:SAFETY_EE",
        "read_adc_ma:SAFETY_OPT",
        "read_adc_ma:SAFETY_EE",
    ]
    assert result.opt_calculation.sample_count == 10
    assert result.opt_calculation.samples_ma == tuple(float(value) for value in range(100, 110))
    assert result.opt_calculation.mean_ma == 104.5
    assert result.opt_calculation.rounded_limit_ma == 136
    assert result.ee_calculation.mean_ma == 204.5
    assert result.ee_calculation.rounded_limit_ma == 225


@pytest.mark.parametrize(
    "rejected",
    [None, True, math.nan, math.inf, RuntimeError("ADC unavailable")],
)
def test_adc_rejections_are_recorded_and_retried_within_bound(rejected):
    bench = FakeSafetyBench()
    bench.adc["SAFETY_OPT"] = deque([rejected, *([100.0] * 10)])

    result, bench, recorder = _run(bench)

    assert result.status is ProcedureStatus.PASSED
    opt_reads = [read for read in result.adc_reads if read.controller == "SAFETY_OPT"]
    assert len(opt_reads) == 11
    assert not opt_reads[0].accepted
    assert opt_reads[0].rejection_reason
    assert sum(read.accepted for read in opt_reads) == 10
    assert any(len(item.adc_reads) == 1 for item in recorder.checkpoints)


def test_fewer_than_ten_adc_samples_fails_without_configuration_write():
    bench = FakeSafetyBench()
    bench.adc["SAFETY_OPT"] = deque([None] * 30)

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.MEASUREMENT
    assert "10 valid" in result.failure_reason
    assert len([read for read in result.adc_reads if read.controller == "SAFETY_OPT"]) == 30
    assert result.opt_calculation is None
    assert not any(item[0] == "configuration" for item in bench.mutations)
    assert bench.trigger_stops == 1


def test_adc_exception_and_stop_failure_preserve_evidence_and_cannot_pass():
    bench = FakeSafetyBench()
    bench.adc["SAFETY_OPT"] = deque([RuntimeError("read broke")] + [100.0] * 10)
    bench.raise_stop = True

    result, bench, _ = _run(bench)

    assert result.status is ProcedureStatus.FAILED
    assert "stop" in result.failure_reason.lower()
    assert result.trigger_cleanup_failure == "stop relay did not acknowledge"
    assert any(not read.accepted for read in result.adc_reads)


def test_warning_transport_and_trigger_stop_failures_are_both_preserved():
    bench = FakeSafetyBench()
    bench.warnings = deque([RuntimeError("telemetry transport failed")])
    bench.raise_stop = True

    result, _, _ = _run(bench)

    assert result.status is ProcedureStatus.FAILED
    assert "telemetry transport failed" in result.failure_reason
    assert "stop relay did not acknowledge" in result.failure_reason
    assert result.trigger_cleanup_failure == "stop relay did not acknowledge"


def test_known_safety_fault_during_adc_firing_fails_before_write():
    bench = FakeSafetyBench()
    bench.warnings = deque(
        [
            SafetyWarningEvidence(
                NOW,
                True,
                False,
                ("OPT current limit",),
                {"so": 1, "se": 0},
            )
        ]
    )

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.MEASUREMENT
    assert "OPT current limit" in result.failure_reason
    assert bench.trigger_stops == 1
    assert not any(item[0] == "configuration" for item in bench.mutations)


def test_adc_firing_requires_at_least_one_known_safety_telemetry_observation():
    bench = FakeSafetyBench()
    bench.warnings = deque(
        [SafetyWarningEvidence(NOW, False, True, (), {"error": "not sampled"})]
    )

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.MEASUREMENT
    assert "telemetry" in result.failure_reason.lower()
    assert bench.trigger_stops == 1
    assert not any(item[0] == "configuration" for item in bench.mutations)


def test_complete_intended_config_changes_only_four_derived_limits():
    result, bench, _ = _run()

    assert result.status is ProcedureStatus.PASSED
    intended = dict(result.intended_configuration)
    current = dict(result.current_configuration)
    assert intended == {
        **current,
        "OPT_DRIVE_CL": 130,
        "EE_DRIVE_CL": 220,
        "OPT_PULSE_WIDTH_UL": 550,
        "EE_PULSE_WIDTH_UL": 550,
    }
    assert dict(result.immediate_configuration_readback) == intended
    assert dict(result.post_restart_configuration) == intended


@pytest.mark.parametrize(
    "write_result",
    [
        RuntimeError("write transport failed"),
        {**_valid_config(), "OPT_DRIVE_CL": 129},
        {key: value for key, value in _valid_config().items() if key != "FACTORY_NOTE"},
        {**_valid_config(), "UNREQUESTED": 1},
    ],
)
def test_write_failure_or_complete_readback_mismatch_prevents_power_cycle(write_result):
    bench = FakeSafetyBench()
    bench.write_result = write_result

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.CONFIGURATION
    assert "configuration" in result.failure_reason.lower()
    assert "power_cycle" not in bench.calls
    assert result.post_restart_configuration is None


@pytest.mark.parametrize(
    ("changes", "reason_fragment"),
    [
        ({"disconnect_observed": False}, "disconnect"),
        ({"off_duration_s": 0.999}, "too quickly"),
        ({"reconnect_observed": False}, "reconnect"),
        ({"restart_proven": False, "restart_proof": None}, "restart"),
        ({"console_serial_after": "C-2"}, "serial"),
    ],
)
def test_power_cycle_requires_observed_disconnect_dwell_reconnect_and_restart(
    changes, reason_fragment
):
    bench = FakeSafetyBench()
    bench.power_cycle_result = replace(_valid_power_cycle(), **changes)

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.CONFIGURATION
    assert reason_fragment in result.failure_reason.lower()
    assert bench.scan_requests == []


@pytest.mark.parametrize(
    "changes",
    [
        {"disconnect_observed_at": None},
        {"on_allowed_at": NOW + timedelta(seconds=1.5)},
        {"on_requested_at": NOW + timedelta(seconds=15)},
        {"reconnect_observed_at": NOW + timedelta(seconds=15)},
    ],
)
def test_power_cycle_requires_complete_chronological_timestamp_evidence(changes):
    bench = FakeSafetyBench()
    bench.power_cycle_result = replace(_valid_power_cycle(), **changes)

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.CONFIGURATION
    assert "timestamp" in result.failure_reason.lower()
    assert bench.scan_requests == []


def test_post_restart_complete_config_mismatch_prevents_scan():
    bench = FakeSafetyBench()
    bench.post_restart_config = {
        **_valid_config(),
        "OPT_DRIVE_CL": 129,
        "EE_DRIVE_CL": 220,
    }

    result, bench, _ = _run(bench)

    assert result.failure_kind is FailureKind.CONFIGURATION
    assert "post-restart" in result.failure_reason.lower()
    assert bench.scan_requests == []


@pytest.mark.parametrize(
    ("topology", "snapshot", "left_serial", "right_serial"),
    [
        (ShippingTopology.SINGLE_LEFT, TopologySnapshot(True, True, False), "L-1", None),
        (ShippingTopology.SINGLE_RIGHT, TopologySnapshot(True, False, True), None, "R-1"),
        (ShippingTopology.DUAL, TopologySnapshot(True, True, True), "L-1", "R-1"),
    ],
)
def test_final_scan_accepts_each_exact_declared_shipping_topology(
    topology, snapshot, left_serial, right_serial
):
    bench = FakeSafetyBench(topology=topology)
    identities = (
        DeviceIdentity("left sensor", left_serial, None, None),
        DeviceIdentity("right sensor", right_serial, None, None),
    )
    bench.scan_result = replace(
        _valid_scan(topology), topology=snapshot, identities=identities
    )

    result, bench, _ = _run(bench, topology=topology)

    assert result.status is ProcedureStatus.PASSED
    assert bench.scan_requests == [(topology, 30.0)]
    assert result.normal_scan.overrides == {}


@pytest.mark.parametrize(
    ("changes", "reason_fragment"),
    [
        ({"started": False}, "start"),
        ({"completed": False}, "complete"),
        ({"canceled": True}, "cancel"),
        ({"actual_duration_s": 29.999}, "30"),
        ({"requested_duration_s": 29.0}, "requested"),
        ({"error": "laser safety interlock"}, "interlock"),
        ({"safety_observations": ()}, "telemetry"),
        ({"warnings": ("laser safety warning",)}, "warning"),
        ({"overrides": {"trigger": 40}}, "override"),
        ({"topology": TopologySnapshot(True, True, True)}, "topology"),
    ],
)
def test_final_scan_fails_closed_for_each_acceptance_gate(changes, reason_fragment):
    bench = FakeSafetyBench()
    bench.scan_result = replace(_valid_scan(), **changes)

    result, _, _ = _run(bench)

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert reason_fragment in result.failure_reason.lower()


def test_final_scan_known_fault_fails_even_when_warning_summary_is_empty():
    bench = FakeSafetyBench()
    warning = SafetyWarningEvidence(
        NOW,
        True,
        False,
        ("EE current limit",),
        {"so": 0, "se": 1},
    )
    bench.scan_result = replace(
        _valid_scan(), safety_observations=(warning,), warnings=()
    )

    result, _, _ = _run(bench)

    assert result.failure_kind is FailureKind.MEASUREMENT
    assert "EE current limit" in result.failure_reason


def test_passing_workflow_records_auditor_readable_stage_labels_and_terminal_reason():
    result, _, recorder = _run()

    assert result.status is ProcedureStatus.PASSED
    assert result.failure_kind is None
    assert result.failure_reason is None
    assert result.ended_at == NOW
    stages = [event.stage for event in result.events]
    assert stages == [
        "1. Console-only safety calibration preflight",
        "2. Persisted operating configuration verification",
        "3. Safety-controller ADC acquisition",
        "4. Derived safety-limit calculation",
        "5. Complete configuration write and immediate verification",
        "6. Measured power-cycle persistence verification",
        "7. Normal 30-second scan with persisted values",
        "8. Procedure completion",
    ]
    assert recorder.checkpoints[-1] == result


def test_console_only_declaration_skips_the_normal_scan_and_passes():
    """A console-only bench run has no modules, so stage 7 must not demand one."""
    bench = FakeSafetyBench()

    result, bench, recorder = _run(bench, topology=ShippingTopology.CONSOLE_ONLY)

    assert result.status is ProcedureStatus.PASSED
    assert result.normal_scan is None
    assert bench.scan_requests == []
    assert any(
        "not applicable" in event.stage.lower() for event in result.events
    )
