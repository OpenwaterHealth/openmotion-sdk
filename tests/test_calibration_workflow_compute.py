"""Unit tests for CalibrationWorkflow pure helpers (no hardware)."""

import pytest

from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationResultRow,
    CalibrationThresholds,
)


def _thresholds():
    return CalibrationThresholds(
        min_mean_per_camera=[100.0]*8,
        min_contrast_per_camera=[0.2]*8,
        min_bfi_per_camera=[3.0]*8,
        min_bvi_per_camera=[3.0]*8,
    )


def test_request_requires_duration_sec():
    with pytest.raises(TypeError):
        # duration_sec is required, no default.
        CalibrationRequest(
            operator_id="op",
            output_dir="/tmp/x",
            left_camera_mask=0xFF,
            right_camera_mask=0xFF,
            thresholds=_thresholds(),
        )


def test_request_defaults():
    req = CalibrationRequest(
        operator_id="op",
        output_dir="/tmp/x",
        left_camera_mask=0xFF,
        right_camera_mask=0xFF,
        thresholds=_thresholds(),
        duration_sec=5,
    )
    assert req.scan_delay_sec == 1
    assert req.max_duration_sec == 600
    assert req.notes == ""


def test_thresholds_lengths_are_eight():
    t = _thresholds()
    assert len(t.min_mean_per_camera) == 8
    assert len(t.min_contrast_per_camera) == 8
    assert len(t.min_bfi_per_camera) == 8
    assert len(t.min_bvi_per_camera) == 8


def test_result_default_state_is_failed():
    r = CalibrationResult(
        ok=False, passed=False, canceled=False, error="",
        csv_path="", json_path="", calibration=None, rows=[],
        calibration_scan_left_path="", calibration_scan_right_path="",
        validation_scan_left_path="", validation_scan_right_path="",
        started_timestamp="",
    )
    assert r.ok is False
    assert r.passed is False


# ----- evaluate_passed -----

from omotion.CalibrationWorkflow import evaluate_passed


def test_evaluate_passed_empty_rows_returns_false():
    assert evaluate_passed([]) is False


# ----- write_result_csv -----

from omotion.CalibrationWorkflow import write_result_csv


def test_write_result_csv_round_trip(tmp_path):
    rows = [
        CalibrationResultRow(
            camera_index=0, side="left", cam_id=0,
            mean=200.0, avg_contrast=0.4, bfi=5.0, bvi=5.5, dark=0.0,
            mean_test="PASS", contrast_test="PASS",
            bfi_test="PASS", bvi_test="FAIL", dark_test="NA",
            security_id="sec-0", hwid="hw-x",
        ),
    ]
    out = tmp_path / "calibration-test.csv"
    write_result_csv(str(out), rows)
    assert out.exists()
    content = out.read_text(encoding="utf-8").splitlines()
    assert len(content) == 2
    header = content[0].split(",")
    assert header == [
        "camera_index", "side", "cam",
        "mean", "avg_contrast", "bfi", "bvi", "dark",
        "mean_test", "contrast_test", "bfi_test", "bvi_test", "dark_test",
        "security_id", "hwid",
    ]
    fields = content[1].split(",")
    # cam column should be 1-indexed (cam_id 0 → cam 1)
    assert fields[2] == "1"
    assert "left" in content[1]
    assert "FAIL" in content[1]


# ----- write_result_json -----

import json

from omotion.CalibrationWorkflow import write_result_json


class _FakeSensor:
    def __init__(self, hwid: str, fw: str):
        self._hwid = hwid
        self._fw = fw

    def get_cached_hardware_id(self) -> str: return self._hwid
    def get_hardware_id(self) -> str: return self._hwid
    def get_version(self) -> str: return self._fw


class _FakeConsole:
    def get_hardware_id(self) -> str: return "console-hwid-deadbeef"
    def get_version(self) -> str: return "v9.9.9"


class _FakeInterface:
    def __init__(self):
        self.console = _FakeConsole()
        self.left  = _FakeSensor("left-hwid-aaa", "v1.2.3")
        self.right = _FakeSensor("right-hwid-bbb", "v1.2.3")


def test_write_result_json_includes_full_provenance(tmp_path):
    rows = [
        CalibrationResultRow(
            camera_index=0, side="left", cam_id=0,
            mean=200.0, avg_contrast=0.4, bfi=5.0, bvi=5.5, dark=0.0,
            mean_test="PASS", contrast_test="PASS",
            bfi_test="PASS", bvi_test="FAIL", dark_test="NA",
            security_id="cam-uid-aaa", hwid="left-hwid-aaa",
        ),
    ]
    thr = CalibrationThresholds(
        min_mean_per_camera=[50.0]*8,
        min_contrast_per_camera=[0.25]*8,
        min_bfi_per_camera=[-0.25]*8,
        min_bvi_per_camera=[4.75]*8,
        max_bfi_per_camera=[0.25]*8,
        max_bvi_per_camera=[5.25]*8,
    )
    req = CalibrationRequest(
        operator_id="op", output_dir=str(tmp_path),
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        thresholds=thr, duration_sec=5,
    )
    out = tmp_path / "calibration-test.json"
    write_result_json(
        str(out),
        started_timestamp="20260502_130928",
        passed=True, canceled=False, error="",
        request=req, rows=rows, calibration=None,
        scan_paths={"calibration_left": "/tmp/cl.csv",
                    "calibration_right": "", "validation_left": "",
                    "validation_right": ""},
        interface=_FakeInterface(),
    )
    assert out.exists()
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["schema_version"] == 1
    assert data["passed"] is True
    assert data["console"]["hwid"] == "console-hwid-deadbeef"
    assert data["console"]["firmware_version"] == "v9.9.9"
    assert data["sensors"]["left"]["hwid"] == "left-hwid-aaa"
    assert data["sensors"]["left"]["firmware_version"] == "v1.2.3"
    assert data["sensors"]["right"]["hwid"] == "right-hwid-bbb"
    assert data["host"]["hostname"]   # populated, content is host-dependent
    assert data["sdk"]["version"]
    assert data["thresholds"]["min_mean_per_camera"] == [50.0]*8
    assert data["thresholds"]["max_bfi_per_camera"] == [0.25]*8
    assert len(data["cameras"]) == 1
    cam = data["cameras"][0]
    assert cam["cam"] == 1                    # 1-indexed
    assert cam["security_id"] == "cam-uid-aaa"
    assert cam["sensor_hwid"] == "left-hwid-aaa"
    assert cam["mean"] == 200.0
    assert cam["min_mean"] == 50.0
    assert cam["bvi_test"] == "FAIL"


def test_write_result_json_handles_missing_sensor(tmp_path):
    """Right sensor disconnected → manifest still written, marked not connected."""
    iface = _FakeInterface()
    iface.right = None
    req = CalibrationRequest(
        operator_id="op", output_dir=str(tmp_path),
        left_camera_mask=0xFF, right_camera_mask=0x00,
        thresholds=CalibrationThresholds(
            min_mean_per_camera=[0.0]*8, min_contrast_per_camera=[0.0]*8,
            min_bfi_per_camera=[-1.0]*8, min_bvi_per_camera=[-1.0]*8,
        ),
        duration_sec=5,
    )
    out = tmp_path / "calibration-no-right.json"
    write_result_json(
        str(out),
        started_timestamp="20260502_130928",
        passed=False, canceled=True, error="user canceled",
        request=req, rows=[], calibration=None,
        scan_paths={"calibration_left": "", "calibration_right": "",
                    "validation_left": "", "validation_right": ""},
        interface=iface,
    )
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["sensors"]["left"]["connected"] is True
    assert data["sensors"]["right"]["connected"] is False
    assert data["sensors"]["right"]["camera_mask"] == "0x00"
    assert data["canceled"] is True
    assert data["error"] == "user canceled"
    assert data["cameras"] == []


# ----- ft_max_dark_per_camera (#122) -----


def test_thresholds_max_dark_defaults_to_none():
    t = _thresholds()
    assert t.max_dark_per_camera is None


def test_thresholds_max_dark_accepts_list():
    t = CalibrationThresholds(
        min_mean_per_camera=[100.0] * 8,
        min_contrast_per_camera=[0.2] * 8,
        min_bfi_per_camera=[3.0] * 8,
        min_bvi_per_camera=[3.0] * 8,
        max_dark_per_camera=[3.0] * 8,
    )
    assert t.max_dark_per_camera == [3.0] * 8


def _dark_row(*, dark_test="NA", dark=0.0, mean_test="PASS",
              contrast_test="PASS", bfi_test="PASS", bvi_test="PASS"):
    return CalibrationResultRow(
        camera_index=0, side="left", cam_id=0,
        mean=100.0, avg_contrast=0.3, bfi=4.0, bvi=4.0, dark=dark,
        mean_test=mean_test, contrast_test=contrast_test,
        bfi_test=bfi_test, bvi_test=bvi_test, dark_test=dark_test,
        security_id="", hwid="",
    )


def test_result_row_has_dark_fields():
    r = _dark_row(dark=1.5, dark_test="PASS")
    assert r.dark == 1.5
    assert r.dark_test == "PASS"


def test_evaluate_passed_all_pass_including_dark():
    assert evaluate_passed([_dark_row(dark_test="PASS")]) is True


def test_evaluate_passed_dark_fail_overrides_all_other_pass():
    assert evaluate_passed([_dark_row(dark_test="FAIL")]) is False


def test_evaluate_passed_dark_na_does_not_gate():
    assert evaluate_passed([_dark_row(dark_test="NA")]) is True


# ----- _build_result_rows_from_samples dark-test path (#122) -----

import math

from omotion.CalibrationWorkflow import _build_result_rows_from_samples
from omotion.MotionProcessing import Sample


def _light(side, cam_id, *, mean=200.0, contrast=0.3, bfi=4.0, bvi=6.0,
           frame_id=10):
    return Sample(
        side=side, cam_id=cam_id,
        frame_id=frame_id, absolute_frame_id=frame_id,
        timestamp_s=0.0, row_sum=0, temperature_c=0.0,
        mean=mean, std_dev=mean * contrast, contrast=contrast,
        bfi=bfi, bvi=bvi,
        is_corrected=True, is_dark=False,
    )


def _dark(side, cam_id, *, mean=1.0, frame_id=0):
    # Dark samples come from the firmware's leading/trailing windows;
    # only the per-camera mean matters for the ambient gate.
    return Sample(
        side=side, cam_id=cam_id,
        frame_id=frame_id, absolute_frame_id=frame_id,
        timestamp_s=0.0, row_sum=0, temperature_c=0.0,
        mean=mean, std_dev=0.0, contrast=0.0,
        bfi=0.0, bvi=0.0,
        is_corrected=True, is_dark=True,
    )


def _full_thresholds(*, max_dark_per_camera=None):
    return CalibrationThresholds(
        min_mean_per_camera=[100.0] * 8,
        min_contrast_per_camera=[0.2] * 8,
        min_bfi_per_camera=[-1.0] * 8,
        min_bvi_per_camera=[5.0] * 8,
        max_dark_per_camera=max_dark_per_camera,
    )


def test_dark_test_pass_when_below_threshold():
    light = [_light(side, 0) for side in ("left", "right")]
    dark = [_dark(side, 0, mean=1.0) for side in ("left", "right")]
    rows = _build_result_rows_from_samples(
        light, dark_samples=dark,
        left_camera_mask=0x01, right_camera_mask=0x01,
        thresholds=_full_thresholds(max_dark_per_camera=[3.0] * 8),
        sensor_left=None, sensor_right=None,
    )
    assert len(rows) == 2
    assert all(r.dark_test == "PASS" for r in rows)
    assert all(r.dark == 1.0 for r in rows)


def test_dark_test_fail_when_above_threshold():
    light = [_light("left", 0)]
    dark = [_dark("left", 0, mean=5.0)]
    rows = _build_result_rows_from_samples(
        light, dark_samples=dark,
        left_camera_mask=0x01, right_camera_mask=0x00,
        thresholds=_full_thresholds(max_dark_per_camera=[3.0] * 8),
        sensor_left=None, sensor_right=None,
    )
    assert rows[0].dark == 5.0
    assert rows[0].dark_test == "FAIL"


def test_dark_test_na_when_threshold_missing():
    light = [_light("left", 0)]
    dark = [_dark("left", 0, mean=5.0)]
    rows = _build_result_rows_from_samples(
        light, dark_samples=dark,
        left_camera_mask=0x01, right_camera_mask=0x00,
        thresholds=_full_thresholds(max_dark_per_camera=None),
        sensor_left=None, sensor_right=None,
    )
    assert rows[0].dark_test == "NA"


def test_dark_value_present_on_passing_run():
    light = [_light("left", 0)]
    dark = [_dark("left", 0, mean=2.0)]
    rows = _build_result_rows_from_samples(
        light, dark_samples=dark,
        left_camera_mask=0x01, right_camera_mask=0x00,
        thresholds=_full_thresholds(max_dark_per_camera=[3.0] * 8),
        sensor_left=None, sensor_right=None,
    )
    assert rows[0].dark == 2.0
    assert rows[0].dark_test == "PASS"


def test_dark_test_fail_when_no_dark_samples_for_active_camera():
    light = [_light("left", 0)]
    rows = _build_result_rows_from_samples(
        light, dark_samples=[],
        left_camera_mask=0x01, right_camera_mask=0x00,
        thresholds=_full_thresholds(max_dark_per_camera=[3.0] * 8),
        sensor_left=None, sensor_right=None,
    )
    assert math.isnan(rows[0].dark)
    assert rows[0].dark_test == "FAIL"


import csv as _csv


def test_write_result_csv_includes_dark_columns(tmp_path):
    rows = [_dark_row(dark=1.5, dark_test="PASS")]
    path = str(tmp_path / "with-dark.csv")
    write_result_csv(path, rows)
    with open(path, newline="", encoding="utf-8") as f:
        reader = _csv.DictReader(f)
        assert "dark" in reader.fieldnames
        assert "dark_test" in reader.fieldnames
        row0 = next(reader)
        assert float(row0["dark"]) == 1.5
        assert row0["dark_test"] == "PASS"


def test_write_result_json_includes_dark(tmp_path):
    rows = [
        CalibrationResultRow(
            camera_index=0, side="left", cam_id=0,
            mean=200.0, avg_contrast=0.4, bfi=5.0, bvi=5.5, dark=1.5,
            mean_test="PASS", contrast_test="PASS",
            bfi_test="PASS", bvi_test="PASS", dark_test="PASS",
            security_id="cam-uid-aaa", hwid="left-hwid-aaa",
        ),
    ]
    thr = CalibrationThresholds(
        min_mean_per_camera=[50.0] * 8,
        min_contrast_per_camera=[0.25] * 8,
        min_bfi_per_camera=[-0.25] * 8,
        min_bvi_per_camera=[4.75] * 8,
        max_bfi_per_camera=[0.25] * 8,
        max_bvi_per_camera=[5.25] * 8,
        max_dark_per_camera=[3.0] * 8,
    )
    req = CalibrationRequest(
        operator_id="op", output_dir=str(tmp_path),
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        thresholds=thr, duration_sec=5,
    )
    out = tmp_path / "with-dark.json"
    write_result_json(
        str(out),
        started_timestamp="20260512_140000",
        passed=True, canceled=False, error="",
        request=req, rows=rows, calibration=None,
        scan_paths={"calibration_left": "", "calibration_right": "",
                    "validation_left": "", "validation_right": ""},
        interface=_FakeInterface(),
    )
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["thresholds"]["max_dark_per_camera"] == [3.0] * 8
    cam = data["cameras"][0]
    assert cam["dark"] == 1.5
    assert cam["max_dark"] == 3.0
    assert cam["dark_test"] == "PASS"


def test_request_average_full_scan_defaults_false():
    """The new request flag must default to False so existing callers
    (everything in the wild today) keep getting the rolling-window
    averaging they were getting before."""
    req = CalibrationRequest(
        operator_id="op",
        output_dir="/tmp",
        left_camera_mask=0x01,
        right_camera_mask=0x00,
        thresholds=_thresholds(),
        duration_sec=15,
    )
    assert req.average_full_scan is False


def test_request_average_full_scan_accepts_true():
    req = CalibrationRequest(
        operator_id="op",
        output_dir="/tmp",
        left_camera_mask=0x01,
        right_camera_mask=0x00,
        thresholds=_thresholds(),
        duration_sec=15,
        average_full_scan=True,
    )
    assert req.average_full_scan is True


def test_test_scan_result_default_mode_is_test():
    from omotion.CalibrationWorkflow import TestScanResult

    r = TestScanResult(
        ok=True, passed=True, canceled=False, error="",
        csv_path="", json_path="", rows=[],
        test_scan_left_path="", test_scan_right_path="",
        started_timestamp="20260521_000000",
    )
    assert r.mode == "test"


def test_write_result_json_records_mode():
    import json
    import tempfile
    from pathlib import Path

    from omotion.CalibrationWorkflow import write_result_json

    class _StubInterface:
        console = None
        left = None
        right = None

    req = CalibrationRequest(
        operator_id="op",
        output_dir="/tmp",
        left_camera_mask=0x01,
        right_camera_mask=0x00,
        thresholds=_thresholds(),
        duration_sec=15,
    )
    with tempfile.TemporaryDirectory() as td:
        path = str(Path(td) / "out.json")
        write_result_json(
            path,
            started_timestamp="20260521_000000",
            passed=True,
            canceled=False,
            error="",
            request=req,
            rows=[],
            calibration=None,
            scan_paths={},
            interface=_StubInterface(),
            mode="test",
        )
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        assert data["mode"] == "test"
