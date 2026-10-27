"""Pure tests for the outcome resolver — no hardware, no threads."""
import pytest

from omotion.CalibrationWorkflow import CalibrationOutcome, _resolve_outcome


@pytest.mark.parametrize(
    "ok,passed,canceled,timed_out,expected",
    [
        (True,  True,  False, False, CalibrationOutcome.PASSED),
        (True,  False, False, False, CalibrationOutcome.FAILED),
        (False, False, True,  False, CalibrationOutcome.CANCELED),
        (False, False, False, False, CalibrationOutcome.ERROR),
        # Watchdog timeout wins over the canceled flag it sets as a side effect.
        (False, False, True,  True,  CalibrationOutcome.TIMED_OUT),
        # Timeout during an error still reports as timeout (the timeout caused it).
        (False, False, False, True,  CalibrationOutcome.TIMED_OUT),
    ],
)
def test_resolve_outcome(ok, passed, canceled, timed_out, expected):
    assert _resolve_outcome(
        ok=ok, passed=passed, canceled=canceled, timed_out=timed_out,
    ) is expected


def test_outcome_is_str_enum():
    # QML/JSON consumers rely on the string value.
    assert CalibrationOutcome.TIMED_OUT == "timed_out"
    assert f"{CalibrationOutcome.PASSED.value}" == "passed"


def test_exported_from_package():
    from omotion import CalibrationOutcome as exported
    assert exported is CalibrationOutcome


def test_legacy_construction_defaults_outcome_to_none():
    """A result built without outcome= (pre-outcome caller shape) must
    surface outcome=None so consumers fall back to the boolean triple —
    an ERROR default would misreport healthy legacy results as errors."""
    from omotion.CalibrationWorkflow import CalibrationResult, TestScanResult
    r = CalibrationResult(
        ok=True, passed=True, canceled=False, error="",
        csv_path="", json_path="", calibration=None, rows=[],
        calibration_scan_left_path="", calibration_scan_right_path="",
        validation_scan_left_path="", validation_scan_right_path="",
        started_timestamp="",
    )
    t = TestScanResult(
        ok=True, passed=True, canceled=False, error="",
        csv_path="", json_path="", rows=[],
        test_scan_left_path="", test_scan_right_path="",
        started_timestamp="",
    )
    assert r.outcome is None and r.calibration_written is False
    assert t.outcome is None
