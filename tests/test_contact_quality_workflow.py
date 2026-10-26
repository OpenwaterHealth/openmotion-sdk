"""Tests for ContactQualityWorkflow — SDK-owned CQ check procedure.

These tests are pure-software and require no hardware. Thresholds and
results are in **background-subtracted DN** scale (subtracted_mean), matching
the legacy ContactQuality module semantics.
"""

import numpy as np
import pytest
from unittest.mock import MagicMock

from omotion.ContactQualityWorkflow import (
    ContactQualityWorkflow,
    CamCQResult,
    ContactQualityResult,
    _ContactQualitySink,
)

from _cq_helpers import _dn_batch


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
    batch = _dn_batch(2, 20.0)
    batch.std_raw[:] = 500.0
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
    sink.consume("raw",   _dn_batch(4, 20.0))
    sink.consume("final", _dn_batch(4, 20.0))
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
    batch = _dn_batch(1, 20.0)
    batch.subtracted_mean[:, 0, 1] = 5.0   # left cam 1 below light threshold
    batch.mean_dc_rt[:, 0, 1] = 5.0
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
# Unlit cameras: frames arrived, no light in them (sdk#343)
# ---------------------------------------------------------------------------

def _unlit(batch, side_idx=None, cam_ids=None):
    """Mark light rows unlit the way DarkCorrectionStage does: low_light_rt
    True, mean_dc_rt left NaN, subtracted_mean still a real near-zero
    reading. Restricted to one side's ``cam_ids`` when given."""
    low = np.zeros(batch.mean_dc_rt.shape, dtype=bool)
    sides = range(2) if side_idx is None else (side_idx,)
    cams = range(8) if cam_ids is None else cam_ids
    for i, s, c, _ft in batch.iter_rows():
        if s in sides and c in cams:
            low[i, s, c] = True
            batch.mean_dc_rt[i, s, c] = np.nan
            batch.subtracted_mean[i, s, c] = 0.2
    batch.low_light_rt = low
    return batch


def _sink(rolling_window=5):
    sink = _ContactQualitySink(
        dark_thresholds=[3.0] * 8,
        light_thresholds=[15.0] * 8,
        rolling_window=rolling_window,
    )
    sink.on_scan_start(None)
    return sink


def test_cq_sink_tells_an_unlit_camera_from_one_that_sent_nothing():
    """The 2026-10-26 bench shape (app#731 retest): L3-L5 streamed but every
    light frame was unlit, R8 delivered no usable frames, the rest were lit.
    Unlit is poor contact, as the live monitor reports it; only the camera
    that sent nothing is no_signal."""
    sink = _sink()
    sink.consume("live", _dn_batch(1, 0.0, frame_types=["dark"]))
    lights = _unlit(_dn_batch(10, 40.0), side_idx=0, cam_ids=(2, 3, 4))
    r8 = (lights.side_ids == 1) & (lights.cam_ids == 7)
    lights.frame_type[r8] = "stale"           # R8: nothing usable arrived
    sink.consume("live", lights)

    result = sink.result(left_mask=0xFF, right_mask=0xFF, duration_sec=1.0)
    reasons = {k: v.reason for k, v in result.per_camera.items()}

    for cam_id in (2, 3, 4):
        cam = result.per_camera[("left", cam_id)]
        assert cam.reason == "poor_contact"
        assert cam.passed is False
        assert np.isnan(cam.light_avg_dn)     # no light reading to report
        assert cam.dark_max_dn == pytest.approx(0.0)
    assert reasons[("right", 7)] == "no_signal"
    others = {k: r for k, r in reasons.items()
              if k not in {("left", 2), ("left", 3), ("left", 4), ("right", 7)}}
    assert set(others.values()) == {"ok"}
    assert result.passed is False


def test_cq_sink_terminal_unlit_frame_is_not_evidence():
    """The firmware's terminal laser-off frame reaches every camera unlit.
    A lit camera stays ok, and a camera whose only frame was that one is
    still no_signal: fewer unlit frames than a rolling window proves
    nothing."""
    sink = _sink(rolling_window=5)
    sink.consume("live", _dn_batch(6, 40.0))
    sink.consume("live", _unlit(_dn_batch(1, 40.0)))
    lit = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    assert lit.per_camera[("left", 0)].reason == "ok"

    sink = _sink(rolling_window=5)
    sink.consume("live", _unlit(_dn_batch(4, 40.0)))
    few = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    assert few.per_camera[("left", 0)].reason == "no_signal"

    sink.consume("live", _unlit(_dn_batch(1, 40.0)))   # 5 = one full window
    enough = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    assert enough.per_camera[("left", 0)].reason == "poor_contact"


def test_cq_sink_nan_light_without_low_light_flag_stays_no_signal():
    """A NaN light reading without low_light_rt means no dark baseline yet,
    not an unlit frame — no measurement exists, so no_signal stands. Covers
    both a batch with no low_light_rt array and one where it reads False."""
    for with_flags in (False, True):
        sink = _sink()
        batch = _dn_batch(10, 40.0)
        batch.mean_dc_rt[:] = np.nan
        if with_flags:
            batch.low_light_rt = np.zeros(batch.mean_dc_rt.shape, dtype=bool)
        sink.consume("live", batch)
        result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
        assert result.per_camera[("left", 0)].reason == "no_signal"


def test_cq_sink_unlit_camera_with_ambient_dark_reports_ambient_light():
    """Ambient light outranks poor contact for an unlit camera too."""
    sink = _sink()
    sink.consume("live", _dn_batch(1, 9.0, frame_types=["dark"]))
    sink.consume("live", _unlit(_dn_batch(10, 40.0)))
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    assert result.per_camera[("left", 0)].reason == "ambient_light"


def test_cq_sink_on_scan_start_clears_unlit_counts():
    sink = _sink()
    sink.consume("live", _unlit(_dn_batch(10, 40.0)))
    sink.on_scan_start(None)
    result = sink.result(left_mask=0x01, right_mask=0, duration_sec=1.0)
    assert result.per_camera[("left", 0)].reason == "no_signal"


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
