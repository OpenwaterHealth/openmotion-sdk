from datetime import datetime, timedelta, timezone

from omotion.calibration.laser import (
    CriterionResult,
    DeviceIdentity,
    FailureKind,
    FinalSettingCheck,
    ProcedureStatus,
    SettingReadback,
    TopologySnapshot,
)
from omotion.calibration.safety import (
    ADC_ROUNDING_RULE,
    AdcReadEvidence,
    NormalScanEvidence,
    PulseLimitCalculation,
    SafetyLimitCalculation,
    SafetyWarningEvidence,
    ShippingTopology,
)
from omotion.calibration.safety_report import SafetyCalibrationHtmlRunReport
from omotion.calibration.safety_workflow import (
    SafetyCalibrationRequest,
    SafetyCalibrationResult,
)
from omotion.calibration.single_sensor_laser import ProcedureEvent
from wi15_builders import FPGA_REVISIONS, valid_power_cycle, valid_safety_config


NOW = datetime(2026, 8, 13, 16, 0, tzinfo=timezone.utc)


def _request(**changes):
    values = {
        "shipping_topology": ShippingTopology.SINGLE_LEFT,
        "operator": "Ada <QA>",
        "build_id": "BUILD-1",
        "fixture_id": "BENCH-1",
        "procedure_id": "WI-00015 Safety Calibration",
        "output_root": "runs",
        "run_id": "RUN-1",
        "sdk_version": "1.6.0-test",
        "started_at": NOW,
    }
    values.update(changes)
    return SafetyCalibrationRequest(**values)


def _warning(*, ok=True, faults=()):
    return SafetyWarningEvidence(
        NOW,
        True,
        ok,
        tuple(faults),
        {"safety_se": 0 if ok else 1, "safety_so": 0, "read_ok": True},
    )


def _current_config():
    return valid_safety_config()


def _result(**changes):
    current = _current_config()
    intended = {**current, "EE_DRIVE_CL": 220, "OPT_DRIVE_CL": 130}
    opt_calculation = SafetyLimitCalculation(
        "SAFETY_OPT",
        (100.0,) * 10,
        10,
        100.0,
        1.3,
        130.0,
        ADC_ROUNDING_RULE,
        130,
    )
    ee_calculation = SafetyLimitCalculation(
        "SAFETY_EE",
        (200.0,) * 10,
        10,
        200.0,
        1.1,
        220.00000000000003,
        ADC_ROUNDING_RULE,
        220,
    )
    pulse_calculation = PulseLimitCalculation(
        500,
        1.1,
        550.0,
        ADC_ROUNDING_RULE,
        550,
        550,
        550,
        "derived",
    )
    power_cycle = valid_power_cycle(NOW)
    scan = NormalScanEvidence(
        ShippingTopology.SINGLE_LEFT,
        30.0,
        30.2,
        True,
        True,
        False,
        None,
        TopologySnapshot(True, True, False),
        (
            DeviceIdentity("left sensor", "L-1", "left-fw", "left-hw"),
            DeviceIdentity("right sensor", None, None, None),
        ),
        {},
        (_warning(),),
        (),
    )
    values = {
        "status": ProcedureStatus.PASSED,
        "shipping_topology": ShippingTopology.SINGLE_LEFT,
        "sdk_version": "1.6.0-runtime",
        "started_at": NOW,
        "ended_at": NOW + timedelta(minutes=2),
        "initial_topology": TopologySnapshot(True, False, False),
        "console_identity": DeviceIdentity(
            "console",
            "C-1",
            "console-fw",
            "console-hw",
            "camera-fpga-omitted",
            FPGA_REVISIONS,
        ),
        "current_configuration": current,
        "configuration_criteria": (
            CriterionResult("ta_current", True, "finite and positive"),
        ),
        "active_setting_checks": (
            FinalSettingCheck(
                "TA_CURRENT_DRV", 5000.0, 4999.0, 1.0, 0.02, 2.0, True
            ),
            FinalSettingCheck(
                "TA_PULSE_WIDTH", 500.0, 500.0, 0.0, 0.0, 2.0, True
            ),
        ),
        "trigger_readbacks": (
            SettingReadback("trigger_rate_hz_initial", 40.0, 39.5),
            SettingReadback("trigger_rate_hz_write", 40.0, 40.0),
            SettingReadback("trigger_rate_hz_final", 40.0, 40.0),
        ),
        "adc_reads": (
            AdcReadEvidence("SAFETY_OPT", 1, NOW, False, None, "read unavailable"),
            *tuple(
                AdcReadEvidence("SAFETY_OPT", index + 2, NOW, True, 100.0, None)
                for index in range(10)
            ),
            *tuple(
                AdcReadEvidence("SAFETY_EE", index + 1, NOW, True, 200.0, None)
                for index in range(10)
            ),
        ),
        "safety_observations": (_warning(),),
        "opt_calculation": opt_calculation,
        "ee_calculation": ee_calculation,
        "pulse_calculation": pulse_calculation,
        "intended_configuration": intended,
        "immediate_configuration_readback": intended,
        "power_cycle": power_cycle,
        "post_restart_configuration": intended,
        "persistence_criteria": (
            CriterionResult(
                "complete_post_restart_configuration", True, "exact match"
            ),
        ),
        "normal_scan": scan,
        "events": (
            ProcedureEvent(NOW, "1. Console-only safety calibration preflight", "ready"),
        ),
    }
    values.update(changes)
    return SafetyCalibrationResult(**values)


def test_report_escapes_and_renders_complete_passing_audit_evidence(tmp_path):
    path = SafetyCalibrationHtmlRunReport(tmp_path).write(
        _request(), _result(), tmp_path / "run.json"
    )

    text = path.read_text(encoding="utf-8")
    assert "Ada &lt;QA&gt;" in text
    assert "Ada <QA>" not in text
    assert 'href="run.json"' in text
    for expected in (
        "WI-00015 Safety Calibration",
        "Status: passed",
        "1.6.0-runtime",
        "C-1",
        "TA FPGA firmware revision",
        "SAFETY_OPT FPGA firmware revision",
        "Current User Configuration",
        "Active TA setting checks",
        "trigger_rate_hz_initial",
        "All SAFETY_OPT and SAFETY_EE ADC reads",
        "read unavailable",
        "Scaled-mA safety-limit calculations",
        "nearest integer; exact halves round upward",
        "220.00000000000003",
        "Pulse-width-limit calculation",
        "Complete intended User Configuration",
        "Immediate complete configuration readback",
        "Power-cycle persistence evidence",
        "same Motion handle disconnected and reconnected",
        "Post-restart complete configuration",
        "Normal 30-second scan with persisted values",
        "single-left",
        "30.2",
        "No configuration overrides",
        "Procedure event timeline",
    ):
        assert expected in text
    assert "camera-fpga-omitted" not in text
    assert text.count('class="changed"') == 2


def test_report_omits_all_unreached_headings_after_earliest_setup_failure(tmp_path):
    result = SafetyCalibrationResult(
        status=ProcedureStatus.FAILED,
        shipping_topology=ShippingTopology.SINGLE_LEFT,
        failure_kind=FailureKind.SETUP,
        failure_reason="Console serial is blank.",
        started_at=NOW,
        ended_at=NOW,
    )

    text = SafetyCalibrationHtmlRunReport(tmp_path).render(
        _request(), result, "run.json"
    )

    assert "Console serial is blank." in text
    for absent in (
        "Current User Configuration",
        "Active TA setting checks",
        "ADC reads",
        "safety-limit calculations",
        "Complete intended User Configuration",
        "Power-cycle persistence evidence",
        "Normal 30-second scan",
    ):
        assert absent not in text


def test_report_shows_partial_adc_reads_but_not_unreached_calculation_or_write(tmp_path):
    result = _result(
        status=ProcedureStatus.FAILED,
        failure_kind=FailureKind.MEASUREMENT,
        failure_reason="SAFETY_OPT produced fewer than 10 valid samples.",
        adc_reads=(
            AdcReadEvidence("SAFETY_OPT", 1, NOW, False, None, "timeout"),
        ),
        safety_observations=(),
        opt_calculation=None,
        ee_calculation=None,
        pulse_calculation=None,
        intended_configuration=None,
        immediate_configuration_readback=None,
        power_cycle=None,
        post_restart_configuration=None,
        persistence_criteria=(),
        normal_scan=None,
    )

    text = SafetyCalibrationHtmlRunReport(tmp_path).render(
        _request(), result, "run.json"
    )

    assert "All SAFETY_OPT and SAFETY_EE ADC reads" in text
    assert "timeout" in text
    assert "Scaled-mA safety-limit calculations" not in text
    assert "Complete intended User Configuration" not in text
    assert "Power-cycle persistence evidence" not in text
    assert "Normal 30-second scan with persisted values" not in text


def test_failed_write_labels_intended_configuration_unconfirmed_and_omits_scan(tmp_path):
    result = _result(
        status=ProcedureStatus.FAILED,
        failure_kind=FailureKind.CONFIGURATION,
        failure_reason="Immediate readback mismatch.",
        immediate_configuration_readback=None,
        power_cycle=None,
        post_restart_configuration=None,
        persistence_criteria=(),
        normal_scan=None,
    )

    text = SafetyCalibrationHtmlRunReport(tmp_path).render(
        _request(), result, "run.json"
    )

    assert "Requested complete User Configuration (unconfirmed)" in text
    assert "Passing User Configuration" not in text
    assert "Power-cycle persistence evidence" not in text
    assert "Normal 30-second scan with persisted values" not in text
