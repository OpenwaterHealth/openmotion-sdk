"""Naming of the durable WI-15 run artifacts (#268).

Files are named ``<console-serial>-<procedure-slug>-run.json`` /
``-report.html`` so a folder of runs identifies its unit and test type
without opening anything. The WI id and timestamp stay on the run directory
and inside the report content.
"""

import json
from dataclasses import dataclass

from omotion.calibration.laser import DeviceIdentity
from omotion.calibration.reporting import JsonRunRecorder
from omotion.calibration.script_support import (
    run_artifact_stem,
    system_serial_number,
)


def _console(serial):
    return DeviceIdentity(
        role="console", serial=serial, firmware="1.0", hardware_id="hw"
    )


def _sensor(serial):
    return DeviceIdentity(
        role="left sensor", serial=serial, firmware="1.0", hardware_id="hw"
    )


@dataclass(frozen=True)
class LaserShapedResult:
    identities: tuple = ()


@dataclass(frozen=True)
class SafetyShapedResult:
    console_identity: DeviceIdentity | None = None


def test_system_serial_comes_from_the_console_identity_only():
    result = LaserShapedResult(identities=(_sensor("SN-L"), _console("CS-9")))
    assert system_serial_number(result) == "CS-9"


def test_safety_results_carry_the_console_identity_at_top_level():
    assert system_serial_number(SafetyShapedResult(_console(" CS-7 "))) == "CS-7"


def test_missing_or_blank_serial_yields_no_system_serial():
    assert system_serial_number(LaserShapedResult()) is None
    assert system_serial_number(LaserShapedResult((_console(None),))) is None
    assert system_serial_number(LaserShapedResult((_console("  "),))) is None
    assert system_serial_number(SafetyShapedResult(None)) is None


def test_stem_is_serial_then_slug_and_sanitized():
    result = LaserShapedResult((_console("CS 01/A"),))
    assert run_artifact_stem("safety-cal", result) == "CS-01-A-safety-cal"


def test_stem_without_a_serial_is_the_slug_alone():
    assert run_artifact_stem("dual-laser-cal", LaserShapedResult()) == (
        "dual-laser-cal"
    )


def test_rename_evidence_moves_the_written_file_and_future_writes(tmp_path):
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "run-1")
    recorder.checkpoint({"status": "in_progress"})
    original = recorder.run_directory / "run.json"
    assert original.is_file()

    renamed = recorder.rename_evidence("CS-9-safety-cal-run.json")

    assert renamed == recorder.run_directory / "CS-9-safety-cal-run.json"
    assert recorder.json_path == renamed
    assert not original.exists()
    assert json.loads(renamed.read_text(encoding="utf-8")) == {
        "status": "in_progress"
    }
    recorder.checkpoint({"status": "passed"})
    assert json.loads(renamed.read_text(encoding="utf-8")) == {
        "status": "passed"
    }
    assert not original.exists()


def test_rename_evidence_before_any_write_only_repoints(tmp_path):
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "run-2")

    renamed = recorder.rename_evidence("stem-run.json")

    assert not (recorder.run_directory / "run.json").exists()
    recorder.checkpoint({"status": "passed"})
    assert renamed.is_file()


def test_rename_run_directory_moves_contents_and_recorder_paths(tmp_path):
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "run-3")
    recorder.checkpoint({"status": "in_progress"})
    original = recorder.run_directory

    target = recorder.rename_run_directory("CS-9-safety-cal-run-3")

    assert target == tmp_path / "CS-9-safety-cal-run-3"
    assert recorder.run_directory == target
    assert recorder.json_path == target / "run.json"
    assert not original.exists()
    assert json.loads(recorder.json_path.read_text(encoding="utf-8")) == {
        "status": "in_progress"
    }
    recorder.checkpoint({"status": "passed"})
    assert json.loads(recorder.json_path.read_text(encoding="utf-8")) == {
        "status": "passed"
    }


def test_rename_run_directory_collisions_get_a_numeric_suffix(tmp_path):
    (tmp_path / "CS-9-safety-cal-run-4").mkdir()
    recorder = JsonRunRecorder(tmp_path, "WI-00015", "run-4")

    target = recorder.rename_run_directory("CS-9-safety-cal-run-4")

    assert target == tmp_path / "CS-9-safety-cal-run-4-1"
    assert recorder.run_directory == target
