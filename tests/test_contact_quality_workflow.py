"""Tests for ContactQualityWorkflow — SDK-owned CQ check procedure.

These tests are pure-software and require no hardware.

Task 14 of the pipeline cutover (Phase D).
"""

import numpy as np
import pytest
from dataclasses import dataclass, field
from typing import Optional
from unittest.mock import MagicMock

from omotion.ContactQualityWorkflow import (
    ContactQualityWorkflow,
    CamCQResult,
    ContactQualityResult,
    _ContactQualitySink,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

@dataclass
class _FakeFrameBatch:
    """Minimal FrameBatch stub for sink unit tests."""
    bfi_rolling: np.ndarray        # shape (n_frames, 2, 8)
    frame_type:  Optional[np.ndarray] = None


def _bfi_batch(
    n_frames: int,
    bfi_value: float,
    frame_types=None,
) -> _FakeFrameBatch:
    """Return a FrameBatch-like stub with uniform BFI across all cams."""
    if frame_types is None:
        frame_types = ["light"] * n_frames
    return _FakeFrameBatch(
        bfi_rolling=np.full((n_frames, 2, 8), bfi_value, dtype=np.float32),
        frame_type=np.array(frame_types, dtype="<U8"),
    )


# ---------------------------------------------------------------------------
# _ContactQualitySink unit tests
# ---------------------------------------------------------------------------

def test_cq_sink_marks_camera_ok_when_avg_within_range():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("rolling", _bfi_batch(4, 5.0))
    result = sink.result(left_mask=(1 << 2), right_mask=0, duration_sec=1.0)

    assert ("left", 2) in result.per_camera
    cam = result.per_camera[("left", 2)]
    assert cam.passed is True
    assert cam.reason == "ok"
    assert cam.avg_bfi == pytest.approx(5.0, abs=1e-4)
    assert result.passed is True


def test_cq_sink_fails_camera_below_dark_threshold():
    sink = _ContactQualitySink(
        dark_thresholds=[2.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("rolling", _bfi_batch(2, 1.0))   # below 2.0
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "below_dark"


def test_cq_sink_fails_camera_above_light_threshold():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[7.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("rolling", _bfi_batch(2, 9.0))   # above 7.0
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "above_light"


def test_cq_sink_no_signal_when_no_data_collected():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    # No batch consumed.
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "no_signal"


def test_cq_sink_skips_warmup_and_stale_frames():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    # All frames are warmup/stale — none should be accumulated.
    batch = _bfi_batch(4, 5.0, frame_types=["warmup", "stale", "warmup", "stale"])
    sink.consume("rolling", batch)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "no_signal"


def test_cq_sink_ignores_non_rolling_channel():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    # Feed on "final" channel — should be silently ignored.
    sink.consume("final", _bfi_batch(4, 5.0))
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.reason == "no_signal"


def test_cq_sink_overall_passed_requires_all_cams():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    # Two cams active; one above threshold (cam 0 via bit 0, cam 1 via bit 1).
    # Supply data: cam 0 ok, cam 1 above light.
    # bfi_rolling shape: (1, 2, 8) — set cam 0 to 5.0, cam 1 to 20.0
    arr = np.full((1, 2, 8), 5.0, dtype=np.float32)
    arr[0, 0, 1] = 20.0   # left cam 1 above light
    batch = _FakeFrameBatch(
        bfi_rolling=arr,
        frame_type=np.array(["light"], dtype="<U8"),
    )
    sink.consume("rolling", batch)
    result = sink.result(left_mask=0x03, right_mask=0, duration_sec=1.0)

    assert result.per_camera[("left", 0)].passed is True
    assert result.per_camera[("left", 1)].passed is False
    assert result.passed is False


def test_cq_sink_on_scan_start_clears_accumulated_data():
    sink = _ContactQualitySink(
        dark_thresholds=[1.0] * 8,
        light_thresholds=[8.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("rolling", _bfi_batch(4, 5.0))
    # Second scan start — should clear previous data.
    sink.on_scan_start(None)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    cam = result.per_camera[("left", 0)]
    assert cam.reason == "no_signal"


# ---------------------------------------------------------------------------
# ContactQualityWorkflow end-to-end tests
# ---------------------------------------------------------------------------

def test_cq_workflow_check_drives_scan_and_returns_result():
    """Drive ContactQualityWorkflow.check end-to-end with a faked ScanWorkflow.

    The fake start_scan finds the _ContactQualitySink in the request,
    feeds it synthetic rolling values, and returns.  check() should return
    a ContactQualityResult with passed=True for all active cameras.
    """
    fake_scan = MagicMock()
    fake_scan.await_complete = MagicMock()

    def _drive_scan(request):
        # Find the contact-quality sink in the request.
        sink = request.sinks[0]
        sink.on_scan_start(None)
        sink.consume("rolling", _bfi_batch(10, 5.0))
        sink.on_complete()
        return True

    fake_scan.start_scan.side_effect = _drive_scan

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    result = cq.check(
        duration_sec=1.0,
        rolling_window=10,
        dark_threshold_per_camera=[1.0] * 8,
        light_threshold_per_camera=[10.0] * 8,
        left_camera_mask=0xFF,
        right_camera_mask=0,
    )

    assert isinstance(result, ContactQualityResult)
    assert result.passed is True
    # 8 left cameras active, 0 right cameras.
    assert len(result.per_camera) == 8
    for key, cam in result.per_camera.items():
        assert cam.side == "left"
        assert cam.passed is True
        assert cam.reason == "ok"


def test_cq_workflow_check_uses_skip_default_storage():
    """The ScanRequest submitted by check() must have skip_default_storage=True
    and sinks containing a _ContactQualitySink.
    """
    from omotion.ContactQualityWorkflow import _ContactQualitySink as CQSink

    captured: dict = {}

    def _capture(request):
        captured["req"] = request
        # Feed the sink so result() doesn't crash.
        request.sinks[0].on_scan_start(None)
        request.sinks[0].on_complete()
        return True

    fake_scan = MagicMock()
    fake_scan.start_scan.side_effect = _capture
    fake_scan.await_complete = MagicMock()

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    cq.check(
        duration_sec=0.5,
        rolling_window=5,
        dark_threshold_per_camera=[1.0] * 8,
        light_threshold_per_camera=[10.0] * 8,
        left_camera_mask=0x01,
        right_camera_mask=0,
    )

    req = captured.get("req")
    assert req is not None, "start_scan was not called"
    assert req.skip_default_storage is True
    assert req.rolling_avg_enabled is True
    cq_sinks = [s for s in req.sinks if isinstance(s, CQSink)]
    assert len(cq_sinks) == 1


def test_cq_workflow_calls_await_complete():
    """check() must call await_complete so it blocks until the scan finishes."""
    fake_scan = MagicMock()
    fake_scan.start_scan = MagicMock(return_value=True)

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    cq.check(
        duration_sec=0.5,
        rolling_window=5,
        dark_threshold_per_camera=[1.0] * 8,
        light_threshold_per_camera=[10.0] * 8,
        left_camera_mask=0x01,
        right_camera_mask=0,
    )
    fake_scan.await_complete.assert_called_once()


def test_cq_workflow_fails_when_below_threshold():
    """check() returns passed=False when signal is below the dark threshold."""
    fake_scan = MagicMock()
    fake_scan.await_complete = MagicMock()

    def _drive_scan(request):
        sink = request.sinks[0]
        sink.on_scan_start(None)
        sink.consume("rolling", _bfi_batch(5, 0.5))   # below dark=1.0
        sink.on_complete()
        return True

    fake_scan.start_scan.side_effect = _drive_scan

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    result = cq.check(
        duration_sec=1.0,
        rolling_window=5,
        dark_threshold_per_camera=[1.0] * 8,
        light_threshold_per_camera=[10.0] * 8,
        left_camera_mask=0x01,
        right_camera_mask=0,
    )

    assert result.passed is False
    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "below_dark"
