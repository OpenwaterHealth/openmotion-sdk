from dataclasses import replace
import importlib
import json
import math

import pytest

from omotion.calibration.reporting import HtmlRunReport, JsonRunRecorder
from omotion.calibration.single_sensor_laser import (
    OphirEvidenceApplicability,
    OphirSettingEvidence,
    PreflightSnapshot,
    SingleSensorLaserCalibrationRequest,
    SingleSensorLaserCalibrationWorkflow,
)
from omotion.calibration.laser import (
    DEFAULT_USER_CONFIG,
    DeviceIdentity,
    FailureKind,
    OphirIdentity,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
)
from wi15_builders import (
    FPGA_REVISIONS,
    valid_measurement as _valid_measurement,
    valid_ophir_setting_evidence as _valid_ophir_setting_evidence,
)


_DEFAULT_WRITE_RESULT = object()
_DEFAULT_REGISTER_WRITE_RESULT = object()
_DEFAULT_TRIGGER_WRITE_RESULT = object()


class FakeLaserBench:
    def __init__(
        self,
        preflight_queue=(),
        measurements=(),
        user_configuration_reads=None,
        write_result=_DEFAULT_WRITE_RESULT,
        active_registers=None,
        trigger_rate=40.0,
        mutate_written_configuration=False,
        stop_trigger_outcomes=(),
        register_readbacks=None,
        register_write_result=_DEFAULT_REGISTER_WRITE_RESULT,
        user_configuration_write_outcomes=(),
        register_write_outcomes=(),
        topology_rechecks=(),
        trigger_write_result=_DEFAULT_TRIGGER_WRITE_RESULT,
    ):
        self.calls = []
        self.preflight_queue = list(preflight_queue)
        self.measurements = list(
            measurements or [_valid_measurement(), _valid_measurement()]
        )
        self.user_configuration_reads = list(
            user_configuration_reads
            if user_configuration_reads is not None
            else [{"LEGACY_SETTING": 1}, dict(DEFAULT_USER_CONFIG)]
        )
        self.write_result = write_result
        self.active_registers = dict(active_registers or DEFAULT_USER_CONFIG)
        self.trigger_rate = trigger_rate
        self.mutate_written_configuration = mutate_written_configuration
        self.stop_trigger_outcomes = list(stop_trigger_outcomes)
        self.register_readbacks = {
            name: list(values) for name, values in (register_readbacks or {}).items()
        }
        self.register_write_result = register_write_result
        self.user_configuration_write_outcomes = list(
            user_configuration_write_outcomes
        )
        self.register_write_outcomes = list(register_write_outcomes)
        self.topology_rechecks = list(topology_rechecks)
        self.trigger_write_result = trigger_write_result
        self.last_preflight = None
        self.written_user_configurations = []

    def preflight(self, side):
        self.calls.append(f"preflight:{side}")
        outcome = self.preflight_queue.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        self.last_preflight = outcome
        return outcome

    def revalidate_topology(self, side):
        self.calls.append(f"revalidate_topology:{side}")
        if self.topology_rechecks:
            outcome = self.topology_rechecks.pop(0)
            if isinstance(outcome, Exception):
                raise outcome
            return outcome
        return self.last_preflight.topology

    def write_user_configuration(self, configuration):
        self.calls.append("write_user_configuration")
        if self.mutate_written_configuration:
            configuration["TA_CURRENT_DRV"] = 1
            self.user_configuration_reads[0] = dict(configuration)
        self.written_user_configuration = dict(configuration)
        self.written_user_configurations.append(dict(configuration))
        if self.user_configuration_write_outcomes:
            outcome = self.user_configuration_write_outcomes.pop(0)
            if isinstance(outcome, Exception):
                raise outcome
            return outcome
        if self.write_result is _DEFAULT_WRITE_RESULT:
            if self.user_configuration_reads:
                return self.user_configuration_reads.pop(0)
            return dict(configuration)
        return self.write_result

    def read_user_configuration(self):
        self.calls.append("read_user_configuration")
        if self.user_configuration_reads:
            return self.user_configuration_reads.pop(0)
        return dict(self.written_user_configuration)

    def bring_up_laser_configuration(self):
        self.calls.append("bring_up_laser_configuration")

    def read_register(self, name):
        self.calls.append(f"read_register:{name}")
        if name in self.register_readbacks and self.register_readbacks[name]:
            outcome = self.register_readbacks[name].pop(0)
            if isinstance(outcome, Exception):
                raise outcome
            return outcome
        return self.active_registers[name]

    def write_register(self, name, value):
        self.calls.append(f"write_register:{name}:{value}")
        self.active_registers[name] = value
        if self.register_write_outcomes:
            outcome = self.register_write_outcomes.pop(0)
            if isinstance(outcome, Exception):
                raise outcome
            if isinstance(outcome, SettingReadback):
                self.active_registers[name] = outcome.actual
            return outcome
        if self.register_write_result is _DEFAULT_REGISTER_WRITE_RESULT:
            actual = value
            if name in self.register_readbacks and self.register_readbacks[name]:
                actual = self.register_readbacks[name].pop(0)
                if isinstance(actual, Exception):
                    raise actual
            self.active_registers[name] = actual
            return SettingReadback(name, value, actual)
        return self.register_write_result

    def power_cycle(self):
        self.calls.append("power_cycle")

    def read_trigger_rate_hz(self):
        self.calls.append("read_trigger_rate_hz")
        return self.trigger_rate

    def write_trigger_rate_hz(self, rate_hz):
        self.calls.append(f"write_trigger_rate_hz:{rate_hz}")
        self.trigger_rate = rate_hz
        if self.trigger_write_result is not _DEFAULT_TRIGGER_WRITE_RESULT:
            return self.trigger_write_result
        return SettingReadback("trigger_rate_hz_write", rate_hz, rate_hz)

    def measure_energy(self):
        self.calls.append("measure_energy")
        return self.measurements.pop(0)

    def stop_trigger(self):
        self.calls.append("stop_trigger")
        if self.stop_trigger_outcomes:
            outcome = self.stop_trigger_outcomes.pop(0)
            if isinstance(outcome, Exception):
                raise outcome


class FakeRecorder:
    def __init__(self):
        self.events = []
        self.checkpoints = []
        self.states = []

    def record(self, event):
        self.events.append(event)

    def checkpoint(self, result):
        self.states.append(result)
        if result.status is not ProcedureStatus.IN_PROGRESS:
            self.checkpoints.append(result)


def _request(**changes):
    values = {
        "side": "left",
        "side_confirmed": True,
        "fixture_confirmed": True,
        "operator": "operator",
        "build_id": "build-1",
        "fixture_id": "fixture-1",
        "procedure_id": "WI-00015",
        "output_root": "output",
        "run_id": "run-1",
    }
    values.update(changes)
    return SingleSensorLaserCalibrationRequest(**values)


def _preflight(
    *,
    topology=TopologySnapshot(True, True, False),
    console_serial="console-1",
    console_fpga_revisions=FPGA_REVISIONS,
    sensor_serial="sensor-1",
    console_responsive=True,
    ophir_ready=True,
    ophir_identity=OphirIdentity("meter", "meter-1", "sensor", "ophir-1", "2027-01-01"),
    ophir_setting_evidence=None,
    ophir_failure_reason=None,
):
    if ophir_setting_evidence is None:
        ophir_setting_evidence = _valid_ophir_setting_evidence()
    return PreflightSnapshot(
        topology=topology,
        console_identity=DeviceIdentity(
            "console",
            console_serial,
            "1.0",
            "console-hw",
            fpga_firmware_revisions=console_fpga_revisions,
        ),
        selected_sensor_identity=DeviceIdentity("sensor", sensor_serial, "1.0", "sensor-hw"),
        ophir_identity=ophir_identity,
        ophir_ready=ophir_ready,
        console_responsive=console_responsive,
        ophir_setting_evidence=ophir_setting_evidence,
        ophir_failure_reason=ophir_failure_reason,
    )


@pytest.mark.parametrize(
    ("side", "side_confirmed"),
    [("left", False), ("neither", True)],
)
def test_preflight_rejects_an_unconfirmed_or_invalid_sensor_side_before_bench_access(
    side, side_confirmed
):
    """Allowing an unconfirmed side would let a run target the wrong module."""
    bench = FakeLaserBench()
    recorder = FakeRecorder()
    request = _request(side=side, side_confirmed=side_confirmed)

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(request)

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == "Confirm the selected sensor side before continuing."
    assert bench.calls == ["stop_trigger"]
    assert recorder.checkpoints == [result]


def test_preflight_requires_confirmed_fixture_placement_before_bench_access():
    """Skipping fixture confirmation could measure the sensor at the wrong position."""
    bench = FakeLaserBench()
    recorder = FakeRecorder()
    request = _request(side="right", fixture_confirmed=False)

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(request)

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == "Confirm fixture placement before continuing."
    assert bench.calls == ["stop_trigger"]
    assert recorder.checkpoints == [result]


def test_preflight_rejects_non_exact_topology_before_any_configuration_or_measurement():
    """A second or wrong-side sensor makes laser calibration ambiguous."""
    snapshot = _preflight(topology=TopologySnapshot(True, True, True))
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.topology == TopologySnapshot(True, True, True)
    assert result.failure_reason == (
        "Both sensors are connected; use Dual-Sensor Laser Calibration "
        "instead, or disconnect the sensor not being calibrated to "
        "continue with Single-Sensor Laser Calibration."
    )
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    ("console_serial", "sensor_serial", "reason"),
    [
        ("  ", "sensor-1", "Console serial must be nonblank text."),
        ("console-1", None, "Selected-sensor serial must be nonblank text."),
    ],
)
def test_preflight_rejects_blank_console_or_selected_sensor_serial(
    console_serial, sensor_serial, reason
):
    """Blank identity evidence would make a completed calibration untraceable."""
    snapshot = _preflight(
        console_serial=console_serial,
        sensor_serial=sensor_serial,
    )
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == reason
    assert result.identities == (snapshot.console_identity, snapshot.selected_sensor_identity)
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


def test_preflight_requires_all_four_console_fpga_revisions_before_configuration():
    snapshot = _preflight(console_fpga_revisions=())
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.SETUP
    assert "TA, SEED, SAFETY_EE, and SAFETY_OPT" in result.failure_reason
    assert bench.calls == ["preflight:left", "stop_trigger"]


def test_preflight_returns_an_ophir_failure_as_a_structured_setup_result():
    """A missing Ophir meter must stop before configuration or laser firing."""
    snapshot = _preflight(
        ophir_ready=False,
        ophir_failure_reason="No Ophir energy meter was found.",
    )
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == "No Ophir energy meter was found."
    assert result.ophir_identity == snapshot.ophir_identity
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize("console_responsive", [False, None])
def test_preflight_requires_an_explicitly_responsive_console(console_responsive):
    """A connected-but-unresponsive console cannot safely authorize a run."""
    snapshot = _preflight(console_responsive=console_responsive)
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == "Console must be responsive before continuing."
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    ("changes", "reason"),
    [
        ({"ophir_identity": None}, "Ophir identity must be present."),
        (
            {
                "ophir_identity": OphirIdentity(
                    "meter", None, "sensor", "ophir-1", "2027-01-01"
                )
            },
            "Ophir identity fields must be nonblank text.",
        ),
    ],
)
def test_preflight_requires_complete_ophir_identity(changes, reason):
    """Incomplete Ophir evidence must not authorize configuration or firing."""
    snapshot = _preflight(**changes)
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == reason
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


def test_preflight_accepts_exact_complete_ophir_setting_evidence():
    """Changing or omitting any approved Ophir setting must fail closed."""
    snapshot = _preflight()
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.ophir_setting_evidence == snapshot.ophir_setting_evidence
    assert bench.calls.count("stop_trigger") == 2
    assert bench.calls[bench.calls.index("measure_energy") + 1] == "stop_trigger"
    assert "write_user_configuration" in bench.calls
    assert recorder.checkpoints == [result]


def test_right_side_success_mirrors_left_at_the_workflow_level():
    """The declared side is data, not logic: a right-only unit passes end to end."""
    snapshot = _preflight(topology=TopologySnapshot(True, False, True))
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(
        _request(side="right")
    )

    assert result.status is ProcedureStatus.PASSED
    assert result.side == "right"
    assert bench.calls[0] == "preflight:right"
    assert "write_user_configuration" in bench.calls
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    "evidence",
    [
        _valid_ophir_setting_evidence()[:-1],
        _valid_ophir_setting_evidence()
        + (
            OphirSettingEvidence(
                "extra_setting",
                1,
                1,
                OphirEvidenceApplicability.APPLICABLE,
                True,
            ),
        ),
        _valid_ophir_setting_evidence()[:-1] + (_valid_ophir_setting_evidence()[0],),
        _valid_ophir_setting_evidence()[:1]
        + (replace(_valid_ophir_setting_evidence()[1], actual=3.0),)
        + _valid_ophir_setting_evidence()[2:],
        (replace(_valid_ophir_setting_evidence()[0], passed=False),)
        + _valid_ophir_setting_evidence()[1:],
        (replace(_valid_ophir_setting_evidence()[0], name="arbitrary_name"),)
        + _valid_ophir_setting_evidence()[1:],
        (
            replace(
                _valid_ophir_setting_evidence()[0],
                actual=None,
                applicability=OphirEvidenceApplicability.NOT_APPLICABLE,
            ),
        )
        + _valid_ophir_setting_evidence()[1:],
    ],
    ids=[
        "missing",
        "extra",
        "duplicate",
        "mismatched",
        "failed",
        "arbitrary-name",
        "wrongly-applicable",
    ],
)
def test_preflight_rejects_invalid_ophir_setting_evidence(evidence):
    """Incomplete or unverified Ophir setup must not authorize laser work."""
    snapshot = _preflight(ophir_setting_evidence=evidence)
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == "Ophir setting evidence is incomplete or invalid."
    assert result.ophir_setting_evidence == evidence
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


def test_preflight_exception_becomes_a_checkpointed_setup_failure_and_stops_trigger():
    """Leaking a bench preflight exception would skip the procedure evidence."""
    bench = FakeLaserBench([RuntimeError("meter startup failed")])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_reason == (
        "Bench preflight failed (RuntimeError: meter startup failed)."
    )
    assert bench.calls == ["preflight:left", "stop_trigger"]
    assert recorder.checkpoints == [result]


def test_completed_ophir_preflight_precedes_later_configuration_or_measurement_work():
    """Configuration or firing before an Ophir-ready result violates fail-closed order."""
    snapshot = _preflight()
    bench = FakeLaserBench([snapshot])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.topology == snapshot.topology
    assert result.identities == (snapshot.console_identity, snapshot.selected_sensor_identity)
    assert result.ophir_identity == snapshot.ophir_identity
    assert result.ophir_setting_evidence == snapshot.ophir_setting_evidence
    assert [event.stage for event in result.events] == [
        "confirmation",
        "preflight",
        "default_configuration",
        "laser_configuration_bringup",
        "tuning",
        "final_configuration",
    ]
    assert bench.calls == [
        "preflight:left",
        "read_user_configuration",
        "revalidate_topology:left",
        "write_user_configuration",
        "bring_up_laser_configuration",
        "read_register:TA_CURRENT_DRV",
        "read_register:TA_PULSE_WIDTH",
        "read_register:SEED_CW_GAIN",
        "read_register:EE_PULSE_WIDTH_UL",
        "read_register:OPT_PULSE_WIDTH_UL",
        "read_trigger_rate_hz",
        "measure_energy",
        "stop_trigger",
        "measure_energy",
        "stop_trigger",
        "read_register:TA_CURRENT_DRV",
        "read_register:TA_PULSE_WIDTH",
        "write_user_configuration",
    ]


def test_topology_is_revalidated_immediately_before_the_first_configuration_mutation():
    """A late opposite sensor must invalidate an otherwise exact preflight snapshot."""
    bench = FakeLaserBench(
        [_preflight()],
        topology_rechecks=[TopologySnapshot(True, True, True)],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.SETUP
    assert result.topology_revalidation == TopologySnapshot(True, True, True)
    assert result.failure_reason == (
        "Exact single-sensor topology changed before configuration mutation."
    )
    assert bench.calls[:3] == [
        "preflight:left",
        "read_user_configuration",
        "revalidate_topology:left",
    ]
    assert "write_user_configuration" not in bench.calls
    assert "measure_energy" not in bench.calls


def test_checked_default_configuration_preserves_prior_config_and_requires_exact_readback():
    """Skipping any configuration evidence can conceal a partial default write."""
    snapshot = _preflight()
    prior = {"TA_PULSE_WIDTH": 470, "SITE_MARKER": 3}
    bench = FakeLaserBench(
        [snapshot],
        user_configuration_reads=[prior, dict(DEFAULT_USER_CONFIG)],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.pre_existing_config == prior
    assert result.requested_default_config == DEFAULT_USER_CONFIG
    assert result.default_config_readback == DEFAULT_USER_CONFIG
    assert bench.written_user_configuration == DEFAULT_USER_CONFIG
    assert bench.calls.index("bring_up_laser_configuration") > bench.calls.index(
        "write_user_configuration"
    )
    assert bench.calls.count("read_user_configuration") == 1


def test_default_write_without_a_checked_result_fails_before_bringup_or_measurement():
    """Treating a missing SDK write result as success could fire unknown laser settings."""
    bench = FakeLaserBench([_preflight()], write_result=None)
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.pre_existing_config == {"LEGACY_SETTING": 1}
    assert result.requested_default_config == DEFAULT_USER_CONFIG
    assert result.default_config_readback is None
    assert "bring_up_laser_configuration" not in bench.calls
    assert "measure_energy" not in bench.calls
    assert recorder.checkpoints == [result]


def test_laser_configuration_bringup_is_checkpointed_before_active_reads():
    """A completed bring-up mutation must be durable before the next observation."""

    class StopAfterBringup(FakeRecorder):
        def checkpoint(self, result):
            super().checkpoint(result)
            if result.events and result.events[-1].stage == "laser_configuration_bringup":
                raise KeyboardInterrupt

    bench = FakeLaserBench([_preflight()])
    recorder = StopAfterBringup()

    with pytest.raises(KeyboardInterrupt):
        SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert "bring_up_laser_configuration" in bench.calls
    assert not any(call.startswith("read_register:") for call in bench.calls)
    assert recorder.states[-1].events[-1].message == (
        "Laser configuration bring-up completed."
    )


def test_default_write_immediate_mismatch_cannot_be_erased_by_matching_later_read():
    """The adapter-returned complete readback is the authoritative write evidence."""
    immediate_mismatch = {**DEFAULT_USER_CONFIG, "TA_CURRENT_DRV": 4999}
    bench = FakeLaserBench(
        [_preflight()],
        write_result=immediate_mismatch,
        user_configuration_reads=[
            {"LEGACY_SETTING": 1},
            dict(DEFAULT_USER_CONFIG),
        ],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.default_config_readback == immediate_mismatch
    assert bench.calls.count("read_user_configuration") == 1
    assert "bring_up_laser_configuration" not in bench.calls


@pytest.mark.parametrize(
    "readback",
    [
        {key: value for key, value in DEFAULT_USER_CONFIG.items() if key != "TEC_TRIP"},
        {**DEFAULT_USER_CONFIG, "UNAPPROVED_KEY": 1},
        {**DEFAULT_USER_CONFIG, "TA_CURRENT_DRV": 4999},
    ],
    ids=["missing-key", "extra-key", "mismatched-value"],
)
def test_default_readback_mismatch_fails_before_bringup_or_measurement(readback):
    """A non-exact readback leaves the persisted default configuration unproven."""
    bench = FakeLaserBench(
        [_preflight()],
        user_configuration_reads=[{"LEGACY_SETTING": 1}, readback],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.default_config_readback == readback
    assert "bring_up_laser_configuration" not in bench.calls
    assert "measure_energy" not in bench.calls


def test_mutating_bench_write_cannot_change_the_approved_default_readback_contract():
    """A mutable SDK write argument must not redefine the approved default object."""
    bench = FakeLaserBench(
        [_preflight()],
        mutate_written_configuration=True,
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.requested_default_config == DEFAULT_USER_CONFIG
    assert result.default_config_readback == {**DEFAULT_USER_CONFIG, "TA_CURRENT_DRV": 1}
    assert "bring_up_laser_configuration" not in bench.calls
    assert "measure_energy" not in bench.calls
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    ("actual_current", "expected_status"),
    [(5100.0, ProcedureStatus.PASSED), (5101.0, ProcedureStatus.FAILED)],
)
def test_active_default_registers_allow_only_inclusive_two_percent_quantization(
    actual_current, expected_status
):
    """A wider active-current tolerance would authorize a materially wrong laser drive."""
    bench = FakeLaserBench(
        [_preflight()], active_registers={**DEFAULT_USER_CONFIG, "TA_CURRENT_DRV": actual_current}
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is expected_status
    assert result.configurations[0].name == "TA_CURRENT_DRV"
    assert result.configurations[0].requested == 5000
    assert result.configurations[0].actual == actual_current
    if expected_status is ProcedureStatus.FAILED:
        assert result.failure_kind is FailureKind.CONFIGURATION
        assert "measure_energy" not in bench.calls


@pytest.mark.parametrize(
    ("name", "actual"),
    [
        ("TA_CURRENT_DRV", 5150.0),
        ("TA_PULSE_WIDTH", 515.0),
        ("SEED_CW_GAIN", 143.0),
        ("EE_PULSE_WIDTH_UL", 567.0),
        ("OPT_PULSE_WIDTH_UL", 567.0),
    ],
)
def test_each_required_active_default_register_must_stay_within_two_percent(
    name, actual
):
    """Skipping an active default register could fire with an unsafe laser limit."""
    bench = FakeLaserBench(
        [_preflight()], active_registers={**DEFAULT_USER_CONFIG, name: actual}
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.configurations[-1].name == name
    assert result.configurations[-1].actual == actual
    assert "measure_energy" not in bench.calls


def test_corrects_only_trigger_rate_to_40_hz_and_verifies_before_measurement():
    """Measuring before a checked 40 Hz correction would invalidate the energy rate gate."""
    bench = FakeLaserBench([_preflight()], trigger_rate=37.0)

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert bench.calls.index("write_trigger_rate_hz:40.0") < bench.calls.index(
        "measure_energy"
    )
    assert bench.calls.count("read_trigger_rate_hz") == 2
    trigger_readback = [
        item for item in result.configurations if item.name == "trigger_rate_hz"
    ][-1]
    assert trigger_readback.actual == 40.0


def test_initial_trigger_observation_is_checkpointed_before_correction():
    """The observed nondefault rate must be durable before its corrective write."""

    class StopAtInitialTrigger(FakeRecorder):
        def checkpoint(self, result):
            super().checkpoint(result)
            if (
                result.configurations
                and result.configurations[-1].name == "trigger_rate_hz_initial"
            ):
                raise KeyboardInterrupt

    bench = FakeLaserBench([_preflight()], trigger_rate=37.0)
    recorder = StopAtInitialTrigger()

    with pytest.raises(KeyboardInterrupt):
        SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert recorder.states[-1].configurations[-1] == SettingReadback(
        "trigger_rate_hz_initial", 40.0, 37.0
    )
    assert "write_trigger_rate_hz:40.0" not in bench.calls


def test_trigger_write_requires_typed_identity_preserving_readback():
    bench = FakeLaserBench(
        [_preflight()],
        trigger_rate=37.0,
        trigger_write_result=40.0,
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.configurations[-1] == SettingReadback(
        "trigger_rate_hz_initial", 40.0, 37.0
    )
    assert "malformed readback evidence" in result.failure_reason
    assert bench.calls.count("read_trigger_rate_hz") == 1


def test_nonfinite_trigger_write_evidence_is_checkpointed_before_later_matching_read():
    bench = FakeLaserBench(
        [_preflight()],
        trigger_rate=37.0,
        trigger_write_result=SettingReadback(
            "trigger_rate_hz_write", 40.0, float("nan")
        ),
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.configurations[-1].name == "trigger_rate_hz_write"
    assert result.configurations[-1].requested == 40.0
    assert math.isnan(result.configurations[-1].actual)
    assert any(
        state.configurations
        and state.configurations[-1].name == "trigger_rate_hz_write"
        and math.isnan(state.configurations[-1].actual)
        for state in recorder.states
    )
    assert bench.trigger_rate == 40.0
    assert bench.calls.count("read_trigger_rate_hz") == 1


@pytest.mark.parametrize(
    "measurement",
    [
        _valid_measurement(n=25),
        _valid_measurement(stdev_uj=40.0),
        _valid_measurement(rate_hz=38.9),
        _valid_measurement(mean_uj=float("nan")),
    ],
    ids=["pulse-count", "stdev", "rate", "non-finite"],
)
def test_invalid_initial_measurement_is_recorded_checkpointed_and_never_advances_to_tuning(
    measurement,
):
    """Using an invalid Ophir observation for tuning would bypass the quality gates."""
    bench = FakeLaserBench([_preflight()], measurements=[measurement, measurement])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert result.measurements == (measurement,)
    failed_names = {
        criterion.name
        for criterion in result.measurement_criteria[0]
        if not criterion.passed
    }
    assert failed_names
    assert all(name in result.failure_reason for name in failed_names)
    assert "Observed n=" in result.failure_reason
    assert "rate=" in result.failure_reason
    assert {criterion.name for criterion in result.measurement_criteria[0]} == {
        "n",
        "pulse_count",
        "discarded",
        "mean_uj",
        "stdev_uj",
        "rate_hz",
        "min_uj",
        "max_uj",
        "duration_s",
    }
    assert "tuning" not in [event.stage for event in result.events]
    assert bench.calls[bench.calls.index("measure_energy") + 1] == "stop_trigger"
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert max(
        index for index, call in enumerate(bench.calls) if call == "stop_trigger"
    ) < restore_index
    assert recorder.checkpoints == [result]


def test_valid_initial_measurement_records_criteria_and_advances_to_tuning():
    """Dropping valid measurement evidence would leave the next tuning stage unauditable."""
    measurement = _valid_measurement()
    bench = FakeLaserBench([_preflight()], measurements=[measurement, measurement])

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.measurements == (measurement, measurement)
    assert all(criterion.passed for criterion in result.measurement_criteria[0])
    assert "tuning" in [event.stage for event in result.events]
    assert bench.calls[bench.calls.index("measure_energy") + 1] == "stop_trigger"


def test_measurement_exception_still_stops_trigger_and_becomes_a_measurement_failure():
    """A meter exception must not leave the trigger running or lose terminal state."""
    bench = FakeLaserBench([_preflight()], measurements=[RuntimeError("meter failed")])
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert bench.calls[bench.calls.index("measure_energy") + 1] == "stop_trigger"
    assert len(result.active_default_restore) == 5
    assert recorder.checkpoints == [result]


def test_persistent_measurement_stop_failure_is_reported_without_losing_failure_state():
    """A second stop failure must remain reportable after the best-effort retry."""
    bench = FakeLaserBench(
        [_preflight()],
        stop_trigger_outcomes=[
            RuntimeError("first stop failed"),
            RuntimeError("retry stop failed"),
        ],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert result.failure_reason == (
        "Energy measurement failed (RuntimeError: first stop failed)."
    )
    assert result.trigger_cleanup_failure == "Trigger stop failed."
    assert result.events[-2].stage == "trigger_cleanup"
    assert bench.calls.count("stop_trigger") == 2
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert max(
        index for index, call in enumerate(bench.calls) if call == "stop_trigger"
    ) < restore_index
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize("final_mean", [300.0, 400.0], ids=["lower", "upper"])
def test_exact_350_uses_a_distinct_inclusive_final_measurement_without_tuning_writes(
    final_mean,
):
    """Reusing the initial sample or excluding an endpoint would misstate acceptance."""
    initial = _valid_measurement(mean_uj=350.0)
    final = _valid_measurement(mean_uj=final_mean)
    bench = FakeLaserBench([_preflight()], measurements=[initial, final])

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.measurements == (initial, final)
    assert bench.calls.count("measure_energy") == 2
    assert not any(call.startswith("write_register:") for call in bench.calls)
    assert len(bench.written_user_configurations) == 2
    assert result.requested_final_config == DEFAULT_USER_CONFIG
    assert result.final_config_readback == DEFAULT_USER_CONFIG
    assert "power_cycle" not in bench.calls


@pytest.mark.parametrize("final_mean", [299.999, 400.001], ids=["below", "above"])
def test_exact_350_rejects_a_distinct_final_measurement_outside_300_to_400(
    final_mean,
):
    """Persisting settings outside the inclusive energy window would falsely pass."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=350.0), _valid_measurement(mean_uj=final_mean)],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    assert bench.calls.count("measure_energy") == 2
    assert len(bench.written_user_configurations) == 1
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert bench.calls[restore_index - 1] == "stop_trigger"
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    ("name", "final_actual", "expected_status"),
    [
        ("TA_CURRENT_DRV", 5100.0, ProcedureStatus.PASSED),
        ("TA_CURRENT_DRV", 5100.001, ProcedureStatus.FAILED),
        ("TA_PULSE_WIDTH", 490.0, ProcedureStatus.PASSED),
        ("TA_PULSE_WIDTH", 489.999, ProcedureStatus.FAILED),
    ],
)
def test_final_requested_ta_settings_require_inclusive_two_percent_active_readback(
    name, final_actual, expected_status
):
    """Skipping the final active-setting check could persist an unproved setting."""
    requested = DEFAULT_USER_CONFIG[name]
    bench = FakeLaserBench(
        [_preflight()],
        register_readbacks={name: [requested, final_actual]},
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is expected_status
    final_readback = [item for item in result.configurations if item.name == name][-1]
    assert final_readback.requested == requested
    assert final_readback.actual == final_actual
    final_check = [item for item in result.final_setting_checks if item.name == name][-1]
    assert final_check.requested == requested
    assert final_check.actual == final_actual
    assert final_check.absolute_difference == pytest.approx(abs(final_actual - requested))
    assert final_check.percent_difference == pytest.approx(
        abs(final_actual - requested) / requested * 100.0
    )
    assert final_check.tolerance_percent == 2.0
    assert final_check.passed is (expected_status is ProcedureStatus.PASSED)
    assert len(bench.written_user_configurations) == (
        2 if expected_status is ProcedureStatus.PASSED else 1
    )
    if expected_status is ProcedureStatus.FAILED:
        assert result.failure_kind is FailureKind.CONFIGURATION
        assert len(result.active_default_restore) == 5
        restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
        assert max(
            index for index, call in enumerate(bench.calls) if call == "stop_trigger"
        ) < restore_index
        assert recorder.checkpoints == [result]


def test_downward_tuning_uses_50_ma_steps_and_reapplies_the_closer_prior_candidate():
    """Stopping on the crossing or changing pulse width would miss the closest safe setting."""
    measurements = [
        _valid_measurement(mean_uj=380.0),
        _valid_measurement(mean_uj=360.0),
        _valid_measurement(mean_uj=330.0),
        _valid_measurement(mean_uj=355.0),
    ]
    bench = FakeLaserBench([_preflight()], measurements=measurements)

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    register_writes = [call for call in bench.calls if call.startswith("write_register:")]
    assert register_writes == [
        "write_register:TA_CURRENT_DRV:4950",
        "write_register:TA_CURRENT_DRV:4900",
        "write_register:TA_CURRENT_DRV:4950",
    ]
    for write in register_writes[:2]:
        write_index = bench.calls.index(write)
        assert bench.calls[write_index + 1] == "measure_energy"
    assert all(item.name == "TA_CURRENT_DRV" for item in result.adjustments)
    assert [item.requested for item in result.adjustments] == [4950, 4900, 4950]
    assert [candidate.measurement.mean_uj for candidate in result.candidates] == [
        380.0,
        360.0,
        330.0,
    ]
    assert result.selection is not None
    assert result.selection.direction == "downward_current"
    assert result.selection.selected_requested_current_ma == 4950
    assert result.selection.selected_mean_uj == 360.0
    assert result.requested_final_config == {
        **DEFAULT_USER_CONFIG,
        "TA_CURRENT_DRV": 4950,
    }


def test_injected_validation_target_exercises_downward_current_without_changing_default():
    """A bench-only target seam must drive every branch and closest-setting choice."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            _valid_measurement(mean_uj=325.0),
            _valid_measurement(mean_uj=315.0),
            _valid_measurement(mean_uj=300.0),
            _valid_measurement(mean_uj=302.0),
        ],
    )

    result = SingleSensorLaserCalibrationWorkflow(
        bench, FakeRecorder(), target_energy_uj=300.0
    ).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert [item.requested for item in result.adjustments] == [4950, 4900]
    assert result.selection is not None
    assert result.selection.direction == "downward_current"
    assert result.selection.selected_requested_current_ma == 4900
    assert result.selection.selected_mean_uj == 300.0
    assert "closest to 300 uJ" in result.selection.rationale
    assert result.requested_final_config == {
        **DEFAULT_USER_CONFIG,
        "TA_CURRENT_DRV": 4900,
    }


def test_downward_tuning_can_pass_with_the_only_in_range_candidate_at_2000_ma():
    """Treating the floor itself as failure would discard an acceptable reachable result."""
    tuning_measurements = [
        _valid_measurement(mean_uj=450.0) for _ in range(59)
    ] + [_valid_measurement(mean_uj=320.0)]
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=450.0), *tuning_measurements, _valid_measurement(mean_uj=325.0)],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    current_writes = [
        call for call in bench.calls if call.startswith("write_register:TA_CURRENT_DRV:")
    ]
    assert current_writes[-1] == "write_register:TA_CURRENT_DRV:2000"
    assert len(current_writes) == 60
    assert result.selection is not None
    assert result.selection.selected_requested_current_ma == 2000
    assert result.requested_final_config["TA_CURRENT_DRV"] == 2000


def test_downward_tuning_fails_ncr_at_2000_when_every_candidate_is_out_of_range():
    """Continuing below the conservative floor or persisting an unsafe candidate is forbidden."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=450.0)]
        + [_valid_measurement(mean_uj=410.0) for _ in range(60)],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    current_writes = [
        call for call in bench.calls if call.startswith("write_register:TA_CURRENT_DRV:")
    ]
    tuning_current_writes = [call for call in current_writes if not call.endswith(":5000")]
    assert tuning_current_writes[-1] == "write_register:TA_CURRENT_DRV:2000"
    assert not any(call.endswith(":1950") for call in tuning_current_writes)
    assert bench.calls.count("measure_energy") == 61
    assert len(bench.written_user_configurations) == 1
    assert len(result.active_default_restore) == 5
    assert result.active_default_restore_failure is None
    assert result.selection is not None
    assert result.selection.direction == "downward_current"
    assert result.selection.decision_kind == "closest_candidate"
    assert result.selection.accepted is False
    assert result.selection.selected_requested_current_ma == 2000
    assert result.selection.selected_requested_pulse_width_us == 500
    assert result.selection.selected_mean_uj == 410.0
    assert "closest" in result.selection.rationale.lower()
    assert "outside" in result.selection.rationale.lower()
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert bench.calls[restore_index - 1] == "stop_trigger"
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    ("measurements", "write_result", "expected_kind"),
    [
        (
            [_valid_measurement(mean_uj=380.0), _valid_measurement(n=25)],
            SettingReadback("TA_CURRENT_DRV", 4950, 4950),
            FailureKind.MEASUREMENT,
        ),
        (
            [_valid_measurement(mean_uj=380.0)],
            None,
            FailureKind.CONFIGURATION,
        ),
    ],
    ids=["invalid-measurement", "missing-write-result"],
)
def test_downward_tuning_quality_or_write_error_fails_without_passing_handoff(
    measurements, write_result, expected_kind
):
    """An unchecked adjustment cannot become a selected or persisted candidate."""
    bench = FakeLaserBench(
        [_preflight()], measurements=measurements, register_write_result=write_result
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is expected_kind
    assert len(bench.written_user_configurations) == 1
    assert recorder.checkpoints == [result]


@pytest.mark.parametrize(
    "immediate_evidence",
    [
        object(),
        SettingReadback("TA_PULSE_WIDTH", 4950, 4950),
        SettingReadback("TA_CURRENT_DRV", 4900, 4900),
        SettingReadback("TA_CURRENT_DRV", 4950, 4800),
    ],
    ids=["wrong-type", "wrong-name", "wrong-request", "wrong-actual"],
)
def test_adjustment_immediate_readback_is_validated_before_any_later_matching_read(
    immediate_evidence,
):
    """A malformed or mismatched first register readback must fail permanently."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=380.0)],
        register_write_outcomes=[immediate_evidence],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert "measure_energy" not in bench.calls[
        bench.calls.index("write_register:TA_CURRENT_DRV:4950") + 1 :
    ]
    if isinstance(immediate_evidence, SettingReadback):
        assert result.adjustments[-1] == immediate_evidence


def test_upward_tuning_stages_660_limits_uses_10_us_steps_and_reapplies_closer_prior():
    """Changing current or selecting the crossing instead of the closer prior pulse is wrong."""
    measurements = [
        _valid_measurement(mean_uj=320.0),
        _valid_measurement(mean_uj=340.0),
        _valid_measurement(mean_uj=370.0),
        _valid_measurement(mean_uj=345.0),
    ]
    bench = FakeLaserBench([_preflight()], measurements=measurements)

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    register_writes = [call for call in bench.calls if call.startswith("write_register:")]
    assert register_writes == [
        "write_register:EE_PULSE_WIDTH_UL:660",
        "write_register:OPT_PULSE_WIDTH_UL:660",
        "write_register:TA_PULSE_WIDTH:510",
        "write_register:TA_PULSE_WIDTH:520",
        "write_register:TA_PULSE_WIDTH:510",
    ]
    assert not any("TA_CURRENT_DRV" in call for call in register_writes)
    for name in ("EE_PULSE_WIDTH_UL", "OPT_PULSE_WIDTH_UL"):
        write_index = bench.calls.index(f"write_register:{name}:660")
        assert not bench.calls[write_index + 1].startswith(f"read_register:{name}")
    assert [item.requested for item in result.adjustments] == [660, 660, 510, 520, 510]
    assert [candidate.measurement.mean_uj for candidate in result.candidates] == [
        320.0,
        340.0,
        370.0,
    ]
    assert result.selection is not None
    assert result.selection.direction == "upward_pulse"
    assert result.selection.selected_requested_pulse_width_us == 510
    assert result.selection.selected_mean_uj == 340.0
    assert result.requested_final_config == {
        **DEFAULT_USER_CONFIG,
        "TA_PULSE_WIDTH": 510,
        "EE_PULSE_WIDTH_UL": 660,
        "OPT_PULSE_WIDTH_UL": 660,
    }


def test_upward_tuning_at_600_below_300_is_immediate_ncr_and_never_exceeds_ceiling():
    """Persisting or incrementing beyond an underpowered 600 us ceiling is unsafe."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=250.0)]
        + [_valid_measurement(mean_uj=290.0) for _ in range(10)],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    tuning_pulse_writes = [
        int(call.rsplit(":", 1)[1])
        for call in bench.calls
        if call.startswith("write_register:TA_PULSE_WIDTH:")
        and int(call.rsplit(":", 1)[1]) >= 510
    ]
    assert tuning_pulse_writes == list(range(510, 601, 10))
    assert max(tuning_pulse_writes) == 600
    assert bench.calls.count("measure_energy") == 11
    assert len(bench.written_user_configurations) == 1
    assert len(result.active_default_restore) == 5
    assert result.active_default_restore_failure is None
    assert result.selection is not None
    assert result.selection.direction == "upward_pulse"
    assert result.selection.decision_kind == "bound"
    assert result.selection.accepted is False
    assert result.selection.selected_requested_pulse_width_us == 600
    assert result.selection.selected_mean_uj == 290.0
    assert "600" in result.selection.rationale
    assert "below 300" in result.selection.rationale
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert bench.calls[restore_index - 1] == "stop_trigger"
    assert recorder.checkpoints == [result]


def test_upward_tuning_can_pass_with_an_in_range_candidate_at_600_us():
    """The pulse ceiling remains selectable when its measured energy is acceptable."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=250.0)]
        + [_valid_measurement(mean_uj=250.0) for _ in range(9)]
        + [_valid_measurement(mean_uj=320.0), _valid_measurement(mean_uj=325.0)],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.selection is not None
    assert result.selection.selected_requested_pulse_width_us == 600
    assert result.requested_final_config["TA_PULSE_WIDTH"] == 600
    assert result.requested_final_config["EE_PULSE_WIDTH_UL"] == 660
    assert result.requested_final_config["OPT_PULSE_WIDTH_UL"] == 660
    assert not any(call.endswith(":610") for call in bench.calls)


def test_upward_closest_outside_at_ceiling_retains_rejected_selection_evidence():
    """Rejecting an out-of-range closest pulse must not erase the decision trail."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=250.0)]
        + [_valid_measurement(mean_uj=280.0) for _ in range(9)]
        + [_valid_measurement(mean_uj=410.0)],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    assert len(result.candidates) == 11
    assert result.candidates[-1].requested_pulse_width_us == 600
    assert result.candidates[-1].measurement.mean_uj == 410.0
    assert result.selection is not None
    assert result.selection.direction == "upward_pulse"
    assert result.selection.decision_kind == "closest_candidate"
    assert result.selection.accepted is False
    assert result.selection.selected_requested_current_ma == 5000
    assert result.selection.selected_requested_pulse_width_us == 600
    assert result.selection.selected_mean_uj == 410.0
    assert "closest" in result.selection.rationale.lower()
    assert "outside" in result.selection.rationale.lower()
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert bench.calls[restore_index - 1] == "stop_trigger"
    assert len(bench.written_user_configurations) == 1


def test_upward_temporary_limit_readback_error_fails_before_any_pulse_adjustment():
    """Pulse tuning must not begin unless both temporary safety limits are active."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=320.0)],
        register_readbacks={"EE_PULSE_WIDTH_UL": [550, 674.0]},
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    writes_before_cleanup = bench.calls[: bench.calls.index("write_register:TA_CURRENT_DRV:5000")]
    assert "write_register:TA_PULSE_WIDTH:510" not in writes_before_cleanup
    assert len(bench.written_user_configurations) == 1
    assert recorder.checkpoints == [result]


def test_downward_handoff_persists_requested_current_after_final_acceptance_not_quantized_actual():
    """Writing the active quantized current would silently change the selected request."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            _valid_measurement(mean_uj=380.0),
            _valid_measurement(mean_uj=340.0),
            _valid_measurement(mean_uj=345.0),
        ],
        register_readbacks={"TA_CURRENT_DRV": [5000, 4949, 4949]},
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.candidates[-1].requested_current_ma == 4950
    assert result.candidates[-1].actual_current_ma == 4949
    assert result.requested_final_config["TA_CURRENT_DRV"] == 4950
    assert result.requested_final_config["EE_PULSE_WIDTH_UL"] == 550
    assert result.requested_final_config["OPT_PULSE_WIDTH_UL"] == 550
    assert result.final_config_readback == result.requested_final_config
    final_measure_index = [
        index for index, call in enumerate(bench.calls) if call == "measure_energy"
    ][-1]
    final_write_index = [
        index
        for index, call in enumerate(bench.calls)
        if call == "write_user_configuration"
    ][-1]
    assert bench.calls[final_measure_index + 1] == "stop_trigger"
    assert bench.calls[final_write_index - 2 : final_write_index] == [
        "read_register:TA_CURRENT_DRV",
        "read_register:TA_PULSE_WIDTH",
    ]
    assert final_write_index > final_measure_index
    assert "power_cycle" not in bench.calls


def test_upward_handoff_persists_requested_pulse_and_provisional_660_limits():
    """Persisting quantized pulse readback or default limits would corrupt upward tuning."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            _valid_measurement(mean_uj=320.0),
            _valid_measurement(mean_uj=360.0),
            _valid_measurement(mean_uj=350.0),
        ],
        register_readbacks={
            "TA_PULSE_WIDTH": [500, 509, 509],
            "EE_PULSE_WIDTH_UL": [550, 659],
            "OPT_PULSE_WIDTH_UL": [550, 659],
        },
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.PASSED
    assert result.candidates[-1].requested_pulse_width_us == 510
    assert result.candidates[-1].actual_pulse_width_us == 509
    assert result.requested_final_config["TA_PULSE_WIDTH"] == 510
    assert result.requested_final_config["EE_PULSE_WIDTH_UL"] == 660
    assert result.requested_final_config["OPT_PULSE_WIDTH_UL"] == 660
    assert result.final_config_readback == result.requested_final_config
    assert "power_cycle" not in bench.calls


def test_passing_configuration_write_failure_returns_failed_after_final_acceptance():
    """A missing final SDK write result cannot produce a passing tuned handoff."""
    bench = FakeLaserBench(
        [_preflight()],
        user_configuration_write_outcomes=[dict(DEFAULT_USER_CONFIG), None],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.requested_final_config == DEFAULT_USER_CONFIG
    assert result.final_config_readback is None
    assert len(bench.written_user_configurations) == 2
    assert bench.calls.count("measure_energy") == 2
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert max(
        index for index, call in enumerate(bench.calls) if call == "stop_trigger"
    ) < restore_index
    assert recorder.checkpoints == [result]


def test_passing_configuration_write_exception_is_a_configuration_failure():
    """A transport exception during persistence must not be mislabeled as measurement."""
    bench = FakeLaserBench(
        [_preflight()],
        user_configuration_write_outcomes=[
            dict(DEFAULT_USER_CONFIG),
            RuntimeError("USB write failed"),
        ],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.final_config_readback is None
    assert len(bench.written_user_configurations) == 2
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert max(
        index for index, call in enumerate(bench.calls) if call == "stop_trigger"
    ) < restore_index


@pytest.mark.parametrize(
    "final_readback",
    [
        {key: value for key, value in DEFAULT_USER_CONFIG.items() if key != "TEC_TRIP"},
        {**DEFAULT_USER_CONFIG, "UNAPPROVED_KEY": 1},
        {**DEFAULT_USER_CONFIG, "TA_PULSE_WIDTH": 501},
    ],
    ids=["missing-key", "extra-key", "mismatched-value"],
)
def test_passing_configuration_requires_exact_complete_immediate_readback(final_readback):
    """A partial or altered persisted object cannot become safety-calibration input."""
    bench = FakeLaserBench(
        [_preflight()],
        user_configuration_reads=[
            {"LEGACY_SETTING": 1},
            dict(DEFAULT_USER_CONFIG),
            final_readback,
        ],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert result.requested_final_config == DEFAULT_USER_CONFIG
    assert result.final_config_readback == final_readback
    assert len(bench.written_user_configurations) == 2
    assert len(result.active_default_restore) == 5
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert max(
        index for index, call in enumerate(bench.calls) if call == "stop_trigger"
    ) < restore_index
    assert "power_cycle" not in bench.calls
    assert recorder.checkpoints == [result]


def test_adjustment_failure_records_best_effort_active_default_restore_after_stop():
    """A failed tuning step must leave evidence of fail-closed active-register cleanup."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=380.0), _valid_measurement(n=25)],
        register_write_outcomes=[
            SettingReadback("TA_CURRENT_DRV", 4950, 4950),
            None,
            SettingReadback("TA_PULSE_WIDTH", 500, 500),
            SettingReadback("SEED_CW_GAIN", 140, 140),
            SettingReadback("EE_PULSE_WIDTH_UL", 550, 550),
            SettingReadback("OPT_PULSE_WIDTH_UL", 550, 550),
        ],
    )

    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.MEASUREMENT
    assert [item.name for item in result.active_default_restore] == [
        "TA_PULSE_WIDTH",
        "SEED_CW_GAIN",
        "EE_PULSE_WIDTH_UL",
        "OPT_PULSE_WIDTH_UL",
    ]
    assert "TA_CURRENT_DRV restore returned malformed evidence" in result.active_default_restore_failure
    cleanup_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert bench.calls[cleanup_index - 1] == "stop_trigger"


@pytest.mark.parametrize(
    "bench",
    [
        FakeLaserBench(
            [_preflight()],
            measurements=[_valid_measurement(mean_uj=380.0)],
            register_write_outcomes=[RuntimeError("register write failed")],
        ),
        FakeLaserBench(
            [_preflight()],
            register_readbacks={
                "TA_CURRENT_DRV": [5000, RuntimeError("register read failed")]
            },
        ),
    ],
    ids=["adjustment-write", "final-readback"],
)
def test_register_transport_exceptions_are_configuration_failures(bench):
    """Register transport errors must not inherit the surrounding measurement stage."""
    result = SingleSensorLaserCalibrationWorkflow(bench, FakeRecorder()).run(_request())

    assert result.status is ProcedureStatus.FAILED
    assert result.failure_kind is FailureKind.CONFIGURATION
    assert len(bench.written_user_configurations) == 1


def test_persistent_post_firing_restore_failures_do_not_replace_primary_ncr():
    """Cleanup diagnostics must survive without masking the final-energy NCR."""
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            _valid_measurement(mean_uj=350.0),
            _valid_measurement(mean_uj=299.0),
        ],
        register_write_outcomes=[None, None, None, None, None],
    )
    recorder = FakeRecorder()

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(_request())

    assert result.status is ProcedureStatus.FAILED_NCR
    assert result.failure_kind is FailureKind.NCR
    assert result.failure_reason == "Final energy must be between 300 and 400 uJ inclusive."
    assert result.active_default_restore_failure == (
        "TA_CURRENT_DRV restore returned malformed evidence; "
        "TA_PULSE_WIDTH restore returned malformed evidence; "
        "SEED_CW_GAIN restore returned malformed evidence; "
        "EE_PULSE_WIDTH_UL restore returned malformed evidence; "
        "OPT_PULSE_WIDTH_UL restore returned malformed evidence"
    )
    assert result.active_default_restore == ()
    restore_index = bench.calls.index("write_register:TA_CURRENT_DRV:5000")
    assert bench.calls[restore_index - 1] == "stop_trigger"
    assert len(bench.written_user_configurations) == 1
    assert recorder.checkpoints == [result]


def test_end_to_end_downward_pass_persists_and_reports_the_final_handoff(tmp_path):
    """Losing the terminal pass or reordering firing and persistence would hide unsafe evidence."""
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "downward-pass")
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            _valid_measurement(mean_uj=380.0),
            _valid_measurement(mean_uj=360.0),
            _valid_measurement(mean_uj=330.0),
            _valid_measurement(mean_uj=355.0),
        ],
    )
    incremental_payloads = []
    real_preflight = bench.preflight

    def observe_incremental_json(side):
        incremental_payloads.append(
            json.loads(recorder.json_path.read_text(encoding="utf-8"))
        )
        return real_preflight(side)

    bench.preflight = observe_incremental_json
    request = _request(output_root=tmp_path, run_id="downward-pass")

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(request)

    assert incremental_payloads[0]["events"][0]["stage"] == "confirmation"
    persisted_result = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    assert persisted_result["status"] == "passed"
    assert persisted_result["selection"]["direction"] == "downward_current"
    assert persisted_result["selection"]["selected_requested_current_ma"] == 4950
    assert bench.calls == [
        "preflight:left",
        "read_user_configuration",
        "revalidate_topology:left",
        "write_user_configuration",
        "bring_up_laser_configuration",
        "read_register:TA_CURRENT_DRV",
        "read_register:TA_PULSE_WIDTH",
        "read_register:SEED_CW_GAIN",
        "read_register:EE_PULSE_WIDTH_UL",
        "read_register:OPT_PULSE_WIDTH_UL",
        "read_trigger_rate_hz",
        "measure_energy",
        "stop_trigger",
        "write_register:TA_CURRENT_DRV:4950",
        "measure_energy",
        "stop_trigger",
        "write_register:TA_CURRENT_DRV:4900",
        "measure_energy",
        "stop_trigger",
        "write_register:TA_CURRENT_DRV:4950",
        "measure_energy",
        "stop_trigger",
        "read_register:TA_CURRENT_DRV",
        "read_register:TA_PULSE_WIDTH",
        "write_user_configuration",
    ]
    expected_handoff = {**DEFAULT_USER_CONFIG, "TA_CURRENT_DRV": 4950}
    assert bench.written_user_configurations == [
        dict(DEFAULT_USER_CONFIG),
        expected_handoff,
    ]
    report = HtmlRunReport(recorder.run_directory)
    terminal_result = replace(
        result,
        report_paths=(recorder.json_path, report.report_path),
    )
    recorder.checkpoint(terminal_result)
    report.write(request, terminal_result, recorder.json_path)
    recorder.checkpoint(terminal_result)

    final_json = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    html = report.report_path.read_text(encoding="utf-8")
    assert final_json["requested_final_config"] == expected_handoff
    assert final_json["final_config_readback"] == expected_handoff
    assert final_json["sdk_version"]
    assert final_json["started_at"]
    assert final_json["ended_at"]
    assert final_json["final_setting_checks"] == [
        {
            "absolute_difference": 0,
            "actual": 4950,
            "name": "TA_CURRENT_DRV",
            "passed": True,
            "percent_difference": 0.0,
            "requested": 4950,
            "tolerance_percent": 2.0,
        },
        {
            "absolute_difference": 0,
            "actual": 500,
            "name": "TA_PULSE_WIDTH",
            "passed": True,
            "percent_difference": 0.0,
            "requested": 500,
            "tolerance_percent": 2.0,
        },
    ]
    assert [path.rsplit("/", 1)[-1] for path in final_json["report_paths"]] == [
        "run.json",
        "report.html",
    ]
    assert "Status: passed" in html
    assert final_json["sdk_version"] in html
    assert "Passing tuned User Configuration" in html
    assert "downward_current" in html
    assert "4950" in html
    assert 'href="run.json"' in html


def test_end_to_end_upward_ceiling_ncr_restores_defaults_without_handoff(tmp_path):
    """A ceiling NCR must remain terminal in bench calls, JSON, and the HTML report."""
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "upward-ceiling-ncr")
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[_valid_measurement(mean_uj=250.0)]
        + [_valid_measurement(mean_uj=290.0) for _ in range(10)],
    )
    incremental_payloads = []
    real_preflight = bench.preflight

    def observe_incremental_json(side):
        incremental_payloads.append(
            json.loads(recorder.json_path.read_text(encoding="utf-8"))
        )
        return real_preflight(side)

    bench.preflight = observe_incremental_json
    request = _request(output_root=tmp_path, run_id="upward-ceiling-ncr")

    result = SingleSensorLaserCalibrationWorkflow(bench, recorder).run(request)

    assert incremental_payloads[0]["events"][0]["stage"] == "confirmation"
    persisted_result = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    assert persisted_result["status"] == "failed_ncr"
    assert persisted_result["failure_kind"] == "ncr"
    assert persisted_result["requested_final_config"] is None
    assert persisted_result["final_config_readback"] is None
    assert persisted_result["selection"] == {
        "accepted": False,
        "decision_kind": "bound",
        "direction": "upward_pulse",
        "rationale": (
            "The 600 us pulse-width bound was reached with energy below 300 uJ, "
            "so closest-candidate selection was intentionally bypassed."
        ),
        "selected_mean_uj": 290.0,
        "selected_requested_current_ma": 5000,
        "selected_requested_pulse_width_us": 600,
    }
    assert bench.calls[:13] == [
        "preflight:left",
        "read_user_configuration",
        "revalidate_topology:left",
        "write_user_configuration",
        "bring_up_laser_configuration",
        "read_register:TA_CURRENT_DRV",
        "read_register:TA_PULSE_WIDTH",
        "read_register:SEED_CW_GAIN",
        "read_register:EE_PULSE_WIDTH_UL",
        "read_register:OPT_PULSE_WIDTH_UL",
        "read_trigger_rate_hz",
        "measure_energy",
        "stop_trigger",
    ]
    assert bench.calls[13:15] == [
        "write_register:EE_PULSE_WIDTH_UL:660",
        "write_register:OPT_PULSE_WIDTH_UL:660",
    ]
    pulse_calls = bench.calls[15:45]
    expected_pulse_calls = []
    for pulse_width in (510, 520, 530, 540, 550, 560, 570, 580, 590, 600):
        expected_pulse_calls.extend(
            [
                f"write_register:TA_PULSE_WIDTH:{pulse_width}",
                "measure_energy",
                "stop_trigger",
            ]
        )
    assert pulse_calls == expected_pulse_calls
    assert bench.calls[45:] == [
        "write_register:TA_CURRENT_DRV:5000",
        "write_register:TA_PULSE_WIDTH:500",
        "write_register:SEED_CW_GAIN:140",
        "write_register:EE_PULSE_WIDTH_UL:550",
        "write_register:OPT_PULSE_WIDTH_UL:550",
    ]
    assert bench.written_user_configurations == [dict(DEFAULT_USER_CONFIG)]
    report = HtmlRunReport(recorder.run_directory)
    terminal_result = replace(
        result,
        report_paths=(recorder.json_path, report.report_path),
    )
    recorder.checkpoint(terminal_result)
    report.write(request, terminal_result, recorder.json_path)
    recorder.checkpoint(terminal_result)

    final_json = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    html = report.report_path.read_text(encoding="utf-8")
    assert final_json["status"] == "failed_ncr"
    assert len(final_json["active_default_restore"]) == 5
    assert final_json["requested_final_config"] is None
    assert final_json["final_config_readback"] is None
    assert "Status: failed_ncr" in html
    assert "600 us pulse-width ceiling" in html
    assert "Active default restoration" in html
    assert "Passing tuned User Configuration" not in html
    assert "Final User Configuration readback" not in html
    assert 'href="run.json"' in html


class InterruptingJsonRecorder(JsonRunRecorder):
    def __init__(self, *args, stop_when, **kwargs):
        super().__init__(*args, **kwargs)
        self.stop_when = stop_when

    def checkpoint(self, result):
        super().checkpoint(result)
        if self.stop_when(result):
            raise KeyboardInterrupt


def test_downward_sweep_checkpoint_survives_interruption_with_complete_evidence(tmp_path):
    """Every checked observation and mutation must be durable before the next step."""
    recorder = InterruptingJsonRecorder(
        tmp_path,
        "WI-00015",
        "interrupted-downward",
        stop_when=lambda state: (
            len(state.adjustments) == 1
            and len(state.measurements) == 2
            and len(state.candidates) == 2
        ),
    )
    bench = FakeLaserBench(
        [_preflight()],
        measurements=[
            _valid_measurement(mean_uj=380.0),
            _valid_measurement(mean_uj=360.0),
        ],
    )

    with pytest.raises(KeyboardInterrupt):
        SingleSensorLaserCalibrationWorkflow(bench, recorder).run(
            _request(output_root=tmp_path, run_id="interrupted-downward")
        )

    payload = json.loads(recorder.json_path.read_text(encoding="utf-8"))
    assert payload["status"] == "in_progress"
    assert [item["mean_uj"] for item in payload["measurements"]] == [380.0, 360.0]
    assert len(payload["measurement_criteria"]) == 2
    assert payload["adjustments"] == [
        {"actual": 4950, "name": "TA_CURRENT_DRV", "requested": 4950}
    ]
    assert payload["candidates"][-1]["measurement"]["mean_uj"] == 360.0
    assert payload["requested_default_config"] == dict(DEFAULT_USER_CONFIG)


@pytest.mark.parametrize(
    "measurements, validation_call",
    [
        ([_valid_measurement(mean_uj=350.0)], 1),
        (
            [
                _valid_measurement(mean_uj=380.0),
                _valid_measurement(mean_uj=360.0),
            ],
            2,
        ),
        (
            [
                _valid_measurement(mean_uj=350.0),
                _valid_measurement(mean_uj=350.0),
            ],
            2,
        ),
    ],
    ids=["initial", "sweep-candidate", "final"],
)
def test_each_raw_measurement_path_is_durable_before_criterion_derivation(
    monkeypatch, tmp_path, measurements, validation_call
):
    workflow_module = importlib.import_module(
        "omotion.calibration.single_sensor_laser"
    )
    original_validator = workflow_module.validate_energy_measurement
    recorder = JsonRunRecorder(tmp_path, "WI-00015", f"raw-{validation_call}")
    observed_payloads = []
    calls = 0

    def interrupting_validator(measurement):
        nonlocal calls
        calls += 1
        if calls == validation_call:
            observed_payloads.append(
                json.loads(recorder.json_path.read_text(encoding="utf-8"))
            )
            raise KeyboardInterrupt
        return original_validator(measurement)

    monkeypatch.setattr(
        workflow_module, "validate_energy_measurement", interrupting_validator
    )
    bench = FakeLaserBench([_preflight()], measurements=measurements)

    with pytest.raises(KeyboardInterrupt):
        SingleSensorLaserCalibrationWorkflow(bench, recorder).run(
            _request(output_root=tmp_path, run_id=f"raw-{validation_call}")
        )

    payload = observed_payloads[0]
    assert len(payload["measurements"]) == validation_call
    assert len(payload["measurement_criteria"]) == validation_call - 1
