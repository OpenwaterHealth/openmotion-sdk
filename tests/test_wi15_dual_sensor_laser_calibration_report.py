from dataclasses import replace
from datetime import datetime, timezone

from omotion.calibration.dual_sensor_laser import (
    CrossCheck,
    DualSensorLaserCalibrationRequest,
    DualSensorLaserCalibrationResult,
    PairObservation,
    PlacementAcknowledgement,
    PlacementChangeRequest,
    SensorEnergyObservation,
    TuningRound,
    TuningSelection,
    TuningStep,
)
from omotion.calibration.dual_sensor_laser_report import DualSensorHtmlRunReport
from omotion.calibration.laser import (
    CriterionResult,
    DeviceIdentity,
    EnergyMeasurement,
    FailureKind,
    FpgaFirmwareRevision,
    FinalSettingCheck,
    PairMetrics,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
    default_user_configuration,
)
from omotion.calibration.single_sensor_laser import ProcedureEvent


NOW = datetime(2026, 8, 13, 12, 0, tzinfo=timezone.utc)


def request():
    return DualSensorLaserCalibrationRequest(
        operator="Operator <A>",
        build_id="BUILD-001",
        fixture_id="OPHIR-0CM-001",
        fixture_calibration_status="Current",
        procedure_id="WI-00015",
        output_root="runs",
        run_id="DUAL-001",
        sdk_version="sdk-test",
        started_at=NOW,
    )


def observation(side, serial, label, mean):
    measurement = EnergyMeasurement(
        26, 0, mean, 10.0, 40.0, mean - 10, mean + 10, 0.65
    )
    return SensorEnergyObservation(
        side=side,
        sensor_serial=serial,
        label=label,
        measurement=measurement,
        criteria=(CriterionResult("pulse_count", True, "Pulse count must be > 25."),),
    )


def pair(label, left_mean, right_mean, metrics=None):
    observation_phase = label.split(" — paired", maxsplit=1)[0]
    left = observation(
        "left", "LEFT-001", f"{observation_phase} — left sensor", left_mean
    )
    right = observation(
        "right", "RIGHT-001", f"{observation_phase} — right sensor", right_mean
    )
    return PairObservation(
        label=label,
        left=left,
        right=right,
        metrics=metrics
        or PairMetrics(
            left_mean,
            right_mean,
            abs(left_mean - right_mean),
            (left_mean + right_mean) / 2,
            abs((left_mean + right_mean) / 2 - 350),
            left_mean - 350,
            right_mean - 350,
        ),
    )


def passing_result():
    initial = pair("Initial paired measurement — paired baseline", 360, 420)
    adjusted = observation(
        "right", "RIGHT-001", "Midpoint adjustment round 1, step 1 — right sensor", 375
    )
    readback = SettingReadback("TA_CURRENT_DRV", 4900, 4900)
    step = TuningStep(
        number=1,
        label="Adjustment step 1 — reduced TA current from 4950 mA to 4900 mA for the right sensor",
        side="right",
        register_name="TA_CURRENT_DRV",
        requested_value=4900,
        readback=readback,
        observation=adjusted,
    )
    selection = TuningSelection(
        direction="downward_current",
        selected_side="right",
        target_uj=380,
        requested_current_ma=4900,
        requested_pulse_width_us=500,
        selected_mean_uj=375,
        rationale="Selected 4900 mA because its valid right-sensor reading was closest to the calculated 380 uJ target.",
    )
    tuning_round = TuningRound(
        number=1,
        label="Midpoint adjustment round 1 — selected right sensor because it had the higher energy reading; target 380 uJ.",
        input_pair=initial,
        direction="downward_current",
        selected_side="right",
        reason="the right sensor had the higher energy reading",
        target_uj=380,
        steps=(step,),
        selection=selection,
    )
    checked_pair = pair("Cross-check 1 — paired verification result", 330, 370)
    crosscheck = CrossCheck(
        1,
        "Cross-check 1 — paired result: both sensors were within the approved 300-400 uJ range.",
        checked_pair,
        True,
    )
    defaults = default_user_configuration()
    final = dict(defaults)
    final["TA_CURRENT_DRV"] = 4900
    placement = PlacementAcknowledgement(
        PlacementChangeRequest(
            None,
            "left",
            "LEFT-001",
            "Initial paired measurement",
            "Initial paired measurement — place the left sensor in the Ophir 0 cm fixture",
        ),
        True,
        NOW,
    )
    return DualSensorLaserCalibrationResult(
        status=ProcedureStatus.PASSED,
        sdk_version="sdk-test",
        started_at=NOW,
        ended_at=NOW,
        topology=TopologySnapshot(True, True, True),
        topology_revalidation=TopologySnapshot(True, True, True),
        identities=(
            DeviceIdentity(
                "console",
                "CONSOLE-001",
                "console-fw",
                "console-hw",
                fpga_firmware_revisions=tuple(
                    FpgaFirmwareRevision(controller, version)
                    for controller, version in (
                        ("TA", "1.2.3"),
                        ("SEED", "4.5.6"),
                        ("SAFETY_EE", "7.8.9"),
                        ("SAFETY_OPT", "10.11.12"),
                    )
                ),
            ),
            DeviceIdentity(
                "left sensor", "LEFT-001", "left-fw", "left-hw", "camera-fpga-left"
            ),
            DeviceIdentity(
                "right sensor", "RIGHT-001", "right-fw", "right-hw", "camera-fpga-right"
            ),
        ),
        pre_existing_config={"customer_key": 17},
        requested_default_config=defaults,
        default_config_readback=defaults,
        configurations=(SettingReadback("TA_CURRENT_DRV", 5000, 5000),),
        placements=(placement,),
        observations=(
            initial.left,
            initial.right,
            adjusted,
            checked_pair.left,
            checked_pair.right,
        ),
        initial_pair=initial,
        tuning_rounds=(tuning_round,),
        crosschecks=(crosscheck,),
        adjustments=(readback,),
        requested_final_config=final,
        final_config_readback=final,
        final_setting_checks=(
            FinalSettingCheck("TA_CURRENT_DRV", 4900, 4900, 0, 0, 2, True),
            FinalSettingCheck("TA_PULSE_WIDTH", 500, 500, 0, 0, 2, True),
        ),
        events=(ProcedureEvent(NOW, "crosscheck", crosscheck.label),),
    )


def test_dual_report_renders_complete_passing_evidence_with_audit_language(tmp_path):
    html = DualSensorHtmlRunReport(tmp_path).render(request(), passing_result(), "run.json")

    assert "WI-00015 Dual-Sensor Laser Calibration" in html
    assert "Target midpoint energy uJ" in html
    assert "Minimum accepted energy uJ" in html
    assert "Maximum accepted energy uJ" in html
    assert "Distance from 350 uJ" in html
    # Common audit metadata (addendum section 6): the operator identity must
    # reach the human report, HTML-escaped.
    assert "Request metadata" in html
    assert "Operator &lt;A&gt;" in html
    assert "CONSOLE-001" in html and "LEFT-001" in html and "RIGHT-001" in html
    for expected in (
        "TA FPGA firmware revision",
        "1.2.3",
        "SEED FPGA firmware revision",
        "4.5.6",
        "SAFETY_EE FPGA firmware revision",
        "7.8.9",
        "SAFETY_OPT FPGA firmware revision",
        "10.11.12",
    ):
        assert expected in html
    assert "camera-fpga-left" not in html
    assert "camera-fpga-right" not in html
    assert "Topology immediately before configuration mutation" not in html
    assert "Initial paired measurement — left sensor" in html
    assert "Initial differential and midpoint" in html
    assert "selected right sensor because it had the higher energy reading" in html
    assert "Adjustment step 1 — reduced TA current" in html
    assert "Cross-check 1" in html
    assert "Final configuration verification" in html
    assert "Final 2 percent setting checks" in html
    assert 'class="changed"' in html
    assert 'href="run.json"' in html


def test_dual_report_omits_unreached_sections_after_initial_differential_ncr(tmp_path):
    result = replace(
        passing_result(),
        status=ProcedureStatus.FAILED_NCR,
        failure_reason="Initial left/right energy differential exceeded 100 uJ.",
        initial_pair=pair("Initial paired measurement — paired baseline", 299, 400),
        tuning_rounds=(),
        crosschecks=(),
        adjustments=(),
        requested_final_config=None,
        final_config_readback=None,
        final_setting_checks=(),
    )
    html = DualSensorHtmlRunReport(tmp_path).render(request(), result, "run.json")

    assert "Initial differential and midpoint" in html
    assert "Initial left/right energy differential exceeded 100 uJ" in html
    assert "Midpoint adjustment rounds" not in html
    assert "Cross-check results" not in html
    assert "Passing tuned User Configuration" not in html
    assert "Final configuration verification" not in html


def test_dual_report_uses_supplied_pair_metrics_without_recalculating(tmp_path):
    inconsistent = PairMetrics(1, 2, 91, 92, 93, 94, 95)
    result = replace(
        passing_result(),
        initial_pair=pair("Initial paired measurement — paired baseline", 1, 2, inconsistent),
    )
    html = DualSensorHtmlRunReport(tmp_path).render(request(), result, "run.json")

    for supplied in (91, 92, 93, 94, 95):
        assert f">{supplied}<" in html


def test_dual_report_labels_requested_final_config_unconfirmed_after_write_failure(tmp_path):
    result = replace(
        passing_result(),
        status=ProcedureStatus.FAILED,
        failure_reason="Passing User Configuration readback did not match.",
        final_config_readback=None,
    )
    html = DualSensorHtmlRunReport(tmp_path).render(request(), result, "run.json")

    assert "Requested tuned User Configuration (unconfirmed)" in html
    assert "Passing tuned User Configuration" not in html


def test_dual_report_renders_hardware_resource_cleanup_failure(tmp_path):
    result = replace(
        passing_result(),
        status=ProcedureStatus.FAILED,
        failure_kind=FailureKind.MEASUREMENT,
        failure_reason="Hardware resource cleanup failed.",
        resource_cleanup_failure="Motion shutdown transport failed",
    )

    html = DualSensorHtmlRunReport(tmp_path).render(request(), result, "run.json")

    assert "Cleanup diagnostics" in html
    assert "resource_cleanup_failure" in html
    assert "Motion shutdown transport failed" in html
