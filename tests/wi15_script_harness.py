"""Shared harness for the WI-00015 console-script tests.

Imported with absolute imports (``from wi15_script_harness import FakeMeter``)
by the ``test_wi15_*_script.py`` files. Each script test module keeps its own
``SCRIPT_PATH``/``load_script`` wrapper (built on ``wi15_script_path`` /
``load_wi15_script``) plus any script-specific fakes; only the identical
pieces live here.
"""

import importlib.util
import sys
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "omotion" / "scripts"


def wi15_script_path(filename):
    return SCRIPTS_DIR / filename


def load_wi15_script(script_path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    finally:
        sys.modules.pop(spec.name, None)
    return module


class FakeRecorder:
    """Run-directory recorder handed to the scripts' recorder_factory seam."""

    def __init__(self, root):
        self.run_directory = Path(root) / "run"
        self.run_directory.mkdir(parents=True)
        self.json_path = self.run_directory / "run.json"
        self.checkpoints = []

    def checkpoint(self, result):
        self.checkpoints.append(result)

    def rename_run_directory(self, basename):
        # Mirrors JsonRunRecorder's path bookkeeping. No disk move: this
        # fake never writes, and prebuilt FakeReports keep working against
        # whichever directory they were handed.
        self.run_directory = self.run_directory.parent / basename
        self.json_path = self.run_directory / self.json_path.name
        return self.run_directory

    def rename_evidence(self, filename):
        # Mirrors JsonRunRecorder: repoint (and move, were anything written).
        self.json_path = self.run_directory / filename
        return self.json_path


class FakeMeter:
    def __init__(self):
        self.closed = 0

    def close(self):
        self.closed += 1


class FakeBench:
    """Meter-owning laser bench (the safety script's bench differs)."""

    def __init__(self, meter):
        self.meter = meter
        self.closed = 0

    def close(self):
        self.closed += 1


class FakeReport:
    def __init__(self, directory, filename="report.html"):
        self.report_path = Path(directory) / filename
        self.writes = []

    def write(self, request, result, json_path):
        self.writes.append((request, result, Path(json_path)))
        # The run directory may have been renamed (a path-only move in
        # FakeRecorder) after this report was constructed.
        self.report_path.parent.mkdir(parents=True, exist_ok=True)
        self.report_path.write_text("report", encoding="ascii")
        return self.report_path


def complete_args(tmp_path):
    return [
        "--output-dir",
        str(tmp_path),
        "--operator",
        "operator",
        "--build-revision",
        "build-7",
        "--fixture-id",
        "fixture-2",
        "--fixture-calibration-status",
        "current",
    ]
