"""Integration tests for CalibrationWorkflow with mocked ScanWorkflow.

Patches MotionInterface.scan_workflow.start_scan to immediately invoke
on_complete_fn with a fake ScanResult pointing at fixture CSVs.
Patches MotionInterface.write_calibration to record arguments without
hitting hardware.
"""
import os
import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from omotion import MotionInterface
from omotion.Calibration import Calibration
from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationThresholds,
)
from omotion.ScanWorkflow import ScanResult


_FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
_LEFT  = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
_RIGHT = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")


def _have_fixtures() -> bool:
    return os.path.exists(_LEFT) and os.path.exists(_RIGHT)


@pytest.fixture
def thresholds():
    # The fixture CSV is truncated mid-scan, so the corrected stream's
    # dark-baseline endpoint is not a real laser-off frame and BFI/BVI
    # values come out unrealistic. Use very permissive thresholds — this
    # test exercises workflow plumbing, not the science values.
    return CalibrationThresholds(
        min_mean_per_camera=[0.0]*8,
        min_contrast_per_camera=[0.0]*8,
        min_bfi_per_camera=[-1e9]*8,
        min_bvi_per_camera=[-1e9]*8,
    )


@pytest.fixture
def request_obj(tmp_path, thresholds):
    return CalibrationRequest(
        operator_id="opX",
        output_dir=str(tmp_path),
        left_camera_mask=0xFF,
        right_camera_mask=0xFF,
        thresholds=thresholds,
        duration_sec=2,
        scan_delay_sec=0,
        max_duration_sec=60,
    )


@pytest.fixture
def interface():
    iface = MotionInterface(demo_mode=True)

    # CalibrationWorkflow now flashes sensors at phase 0 via
    # start_configure_camera_sensors. In demo mode there are no real
    # sensors to configure, so we stub it to immediately succeed.
    from omotion.ScanWorkflow import ConfigureResult

    def _fake_configure(req, *, on_complete_fn=None, on_log_fn=None, **kw):
        def _run():
            time.sleep(0.02)
            if on_complete_fn:
                on_complete_fn(ConfigureResult(ok=True, error=""))
        threading.Thread(target=_run, daemon=True).start()
        return True

    iface.start_configure_camera_sensors = _fake_configure
    return iface


def _make_fake_scan(left, right):
    """Return a function that mimics scan_workflow.start_scan.

    Spawns a thread that:
      1. Synthesises a corrected CorrectedBatch with one Sample per
         (side, cam) and emits it via on_corrected_batch_fn (matching
         the new live-capture flow used by CalibrationWorkflow).
      2. Calls on_complete_fn with a ScanResult pointing at fixture
         paths so the workflow has CSV artifacts.
    """
    from omotion.MotionProcessing import CorrectedBatch, Sample

    def _make_batch():
        samples = []
        for side in ("left", "right"):
            for cam_id in range(8):
                # Multiple samples per camera so per-frame averaging
                # has something to average.
                for fid in range(50, 100):
                    samples.append(Sample(
                        side=side, cam_id=cam_id, frame_id=fid,
                        absolute_frame_id=fid, timestamp_s=fid / 40.0,
                        row_sum=1000, temperature_c=25.0,
                        mean=200.0, std_dev=80.0, contrast=0.4,
                        bfi=5.0, bvi=5.0,
                        is_corrected=True, is_dark=False,
                    ))
        return CorrectedBatch(
            dark_frame_start=10, dark_frame_end=240, samples=samples,
        )

    def _fake(req, *, on_complete_fn=None, on_log_fn=None,
              on_corrected_batch_fn=None, **kw):
        def _run():
            time.sleep(0.05)
            if on_corrected_batch_fn:
                on_corrected_batch_fn(_make_batch())
            res = ScanResult(
                ok=True, error="", left_path=left, right_path=right,
                canceled=False, scan_timestamp="20260501_000000",
            )
            if on_complete_fn:
                on_complete_fn(res)
        threading.Thread(target=_run, daemon=True).start()
        return True
    return _fake


def test_happy_path_produces_csv_and_passes(interface, request_obj):
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    interface.scan_workflow.start_scan = _make_fake_scan(_LEFT, _RIGHT)
    interface.write_calibration = MagicMock(
        return_value=Calibration(
            c_min=np.zeros((2, 8)), c_max=np.full((2, 8), 0.5),
            i_min=np.zeros((2, 8)), i_max=np.full((2, 8), 200.0),
            source="console",
        )
    )

    done = threading.Event()
    result_box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (result_box.append(r), done.set()),
    )
    assert done.wait(timeout=60.0), "calibration didn't complete"
    r = result_box[0]
    assert r.ok is True
    assert r.passed is True
    assert r.canceled is False
    assert os.path.exists(r.csv_path)
    assert r.calibration is not None
    interface.write_calibration.assert_called_once()


def test_cancel_during_phase_1(interface, request_obj):
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    started = threading.Event()
    cancel_called = threading.Event()

    def _slow_scan(req, *, on_complete_fn=None, **kw):
        def _run():
            started.set()
            cancel_called.wait(timeout=5.0)
            if on_complete_fn:
                on_complete_fn(ScanResult(
                    ok=False, error="canceled", left_path="", right_path="",
                    canceled=True, scan_timestamp="x",
                ))
        threading.Thread(target=_run, daemon=True).start()
        return True

    interface.scan_workflow.start_scan = _slow_scan
    interface.scan_workflow.cancel_scan = MagicMock(
        side_effect=lambda **kw: cancel_called.set()
    )
    interface.write_calibration = MagicMock()

    done = threading.Event()
    box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert started.wait(timeout=5.0)
    interface.cancel_calibration()
    assert done.wait(timeout=15.0)
    r = box[0]
    assert r.ok is False
    assert r.canceled is True
    assert r.csv_path == ""
    interface.write_calibration.assert_not_called()


def test_phase1_scan_failure_aborts_before_write(interface, request_obj):
    def _fail_scan(req, *, on_complete_fn=None, **kw):
        threading.Thread(
            target=lambda: on_complete_fn(ScanResult(
                ok=False, error="USB lost", left_path="", right_path="",
                canceled=False, scan_timestamp="x",
            )),
            daemon=True,
        ).start()
        return True

    interface.scan_workflow.start_scan = _fail_scan
    interface.write_calibration = MagicMock()

    done = threading.Event()
    box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert done.wait(timeout=10.0)
    r = box[0]
    assert r.ok is False
    assert "USB lost" in r.error
    interface.write_calibration.assert_not_called()
