"""Tests for ContactQualityWorkflow — SDK-owned CQ check procedure.

These tests are pure-software and require no hardware. Thresholds and
results are in **background-subtracted DN** scale (display_mean), matching
the legacy ContactQuality module semantics.
"""

import numpy as np
import pytest
from dataclasses import dataclass
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
    """Minimal FrameBatch stub for sink unit tests.

    The CQ sink reads ``display_mean`` for dark frames and ``mean_dc_rt``
    for light frames; the fixture seeds both with the same DN value so a
    single ``dn_value`` argument exercises whichever branch the test cares
    about.
    """
    display_mean: np.ndarray            # shape (n_frames, 2, 8) — used for dark frames
    mean_dc_rt:   np.ndarray            # shape (n_frames, 2, 8) — used for light frames
    std_raw:      Optional[np.ndarray] = None
    frame_type:   Optional[np.ndarray] = None


def _dn_batch(
    n_frames: int,
    dn_value: float,
    frame_types=None,
) -> _FakeFrameBatch:
    """Return a FrameBatch-like stub with uniform DN across all cams.

    Both display_mean and mean_dc_rt get the same value so the test stays
    valid regardless of which the sink reads for a given frame_type.
    """
    if frame_types is None:
        frame_types = ["light"] * n_frames
    arr = np.full((n_frames, 2, 8), dn_value, dtype=np.float32)
    return _FakeFrameBatch(
        display_mean=arr,
        mean_dc_rt=arr.copy(),
        std_raw=np.full((n_frames, 2, 8), 2.5, dtype=np.float32),
        frame_type=np.array(frame_types, dtype="<U8"),
    )


# ---------------------------------------------------------------------------
# _ContactQualitySink unit tests
# ---------------------------------------------------------------------------

def test_cq_sink_marks_camera_ok_when_light_above_threshold_and_no_dark():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("live", _dn_batch(4, 20.0))   # well above light threshold
    result = sink.result(left_mask=(1 << 2), right_mask=0, duration_sec=1.0)

    assert ("left", 2) in result.per_camera
    cam = result.per_camera[("left", 2)]
    assert cam.passed is True
    assert cam.reason == "ok"
    assert cam.light_avg_dn == pytest.approx(20.0, abs=1e-4)
    assert result.passed is True


def test_cq_sink_fails_camera_below_light_threshold():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("live", _dn_batch(4, 5.0))    # below light threshold 15.0
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "poor_contact"


def test_cq_sink_records_light_std_without_thresholding_it():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    batch = _FakeFrameBatch(
        display_mean=np.full((2, 2, 8), 20.0, dtype=np.float32),
        mean_dc_rt=np.full((2, 2, 8), 20.0, dtype=np.float32),
        std_raw=np.full((2, 2, 8), 500.0, dtype=np.float32),
        frame_type=np.array(["light", "light"], dtype="<U8"),
    )
    sink.consume("live", batch)

    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is True
    assert cam.reason == "ok"
    assert cam.light_std_dn == pytest.approx(500.0)


def test_cq_sink_fails_camera_when_dark_frame_exceeds_threshold():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    # First good light frames, then a dark frame above threshold:
    sink.consume("live", _dn_batch(4, 20.0))
    sink.consume("live", _dn_batch(2, 8.0, frame_types=["dark", "dark"]))
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "ambient_light"
    assert cam.dark_max_dn == pytest.approx(8.0, abs=1e-4)


def test_cq_sink_no_signal_when_no_data_collected():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.passed is False
    assert cam.reason == "no_signal"


def test_cq_sink_skips_warmup_and_stale_frames():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    batch = _dn_batch(4, 20.0, frame_types=["warmup", "stale", "warmup", "stale"])
    sink.consume("live", batch)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.reason == "no_signal"


def test_cq_sink_ignores_non_live_channel():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("rolling", _dn_batch(4, 20.0))
    sink.consume("final",   _dn_batch(4, 20.0))
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)

    cam = result.per_camera[("left", 0)]
    assert cam.reason == "no_signal"


def test_cq_sink_uses_rolling_window_not_cumulative_mean():
    """Light rolling window with size 3: only the last 3 light frames count."""
    sink = _ContactQualitySink(
        dark_thresholds=[100.0] * 8,
        light_thresholds=[15.0] * 8,
        rolling_window=3,
    )
    sink.on_scan_start(None)
    # Feed: 5.0, 5.0, 5.0, 25.0, 25.0, 25.0 — rolling avg should be 25.0 not 15.0
    for v in (5.0, 5.0, 5.0, 25.0, 25.0, 25.0):
        sink.consume("live", _dn_batch(1, v))
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    cam = result.per_camera[("left", 0)]
    assert cam.light_avg_dn == pytest.approx(25.0, abs=1e-4)
    assert cam.reason == "ok"


def test_cq_sink_overall_passed_requires_all_cams():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    arr = np.full((1, 2, 8), 20.0, dtype=np.float32)
    arr[0, 0, 1] = 5.0   # left cam 1 below light threshold
    batch = _FakeFrameBatch(
        display_mean=arr,
        mean_dc_rt=arr.copy(),
        frame_type=np.array(["light"], dtype="<U8"),
    )
    sink.consume("live", batch)
    result = sink.result(left_mask=0x03, right_mask=0, duration_sec=1.0)

    assert result.per_camera[("left", 0)].passed is True
    assert result.per_camera[("left", 1)].passed is False
    assert result.passed is False


def test_cq_sink_on_scan_start_clears_accumulated_data():
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
    )
    sink.on_scan_start(None)
    sink.consume("live", _dn_batch(4, 20.0))
    sink.on_scan_start(None)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    cam = result.per_camera[("left", 0)]
    assert cam.reason == "no_signal"


# ---------------------------------------------------------------------------
# ContactQualityWorkflow end-to-end tests
# ---------------------------------------------------------------------------

def test_cq_workflow_check_drives_scan_and_returns_result():
    fake_scan = MagicMock()
    fake_scan.await_complete = MagicMock()

    def _drive_scan(request):
        sink = request.sinks[0]
        sink.on_scan_start(None)
        sink.consume("live", _dn_batch(10, 20.0))   # well above light threshold
        sink.on_complete()
        return True

    fake_scan.start_scan.side_effect = _drive_scan

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    result = cq.check(
        duration_sec=1.0,
        rolling_window=10,
        dark_threshold_per_camera=[3.0] * 8,
        light_threshold_per_camera=[15.0] * 8,
        left_camera_mask=0xFF,
        right_camera_mask=0,
    )

    assert isinstance(result, ContactQualityResult)
    assert result.passed is True
    assert len(result.per_camera) == 8
    for key, cam in result.per_camera.items():
        assert cam.side == "left"
        assert cam.passed is True
        assert cam.reason == "ok"


def test_cq_workflow_check_uses_skip_default_storage():
    from omotion.ContactQualityWorkflow import _ContactQualitySink as CQSink

    captured: dict = {}

    def _capture(request):
        captured["req"] = request
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
        dark_threshold_per_camera=[3.0] * 8,
        light_threshold_per_camera=[15.0] * 8,
        left_camera_mask=0x01,
        right_camera_mask=0,
    )

    req = captured.get("req")
    assert req is not None
    assert req.skip_default_storage is True
    assert req.rolling_avg_enabled is False
    cq_sinks = [s for s in req.sinks if isinstance(s, CQSink)]
    assert len(cq_sinks) == 1


def test_cq_workflow_calls_await_complete():
    fake_scan = MagicMock()
    fake_scan.start_scan = MagicMock(return_value=True)

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    cq.check(
        duration_sec=0.5,
        rolling_window=5,
        dark_threshold_per_camera=[3.0] * 8,
        light_threshold_per_camera=[15.0] * 8,
        left_camera_mask=0x01,
        right_camera_mask=0,
    )
    fake_scan.await_complete.assert_called_once()


def test_cq_workflow_fails_when_below_light_threshold():
    fake_scan = MagicMock()
    fake_scan.await_complete = MagicMock()

    def _drive_scan(request):
        sink = request.sinks[0]
        sink.on_scan_start(None)
        sink.consume("live", _dn_batch(5, 5.0))   # below light=15.0
        sink.on_complete()
        return True

    fake_scan.start_scan.side_effect = _drive_scan

    cq = ContactQualityWorkflow(scan_workflow=fake_scan)
    result = cq.check(
        duration_sec=1.0,
        rolling_window=5,
        dark_threshold_per_camera=[3.0] * 8,
        light_threshold_per_camera=[15.0] * 8,
        left_camera_mask=0x01,
        right_camera_mask=0,
    )

    assert result.passed is False
