"""TimestampRepairStage -- divergence detection, re-anchoring, NaN-fill."""

import logging

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.timestamp_repair import TimestampRepairStage


def _make_batch(cam_ids, frame_ids, side_ids, timestamps, abs_frame_ids=None,
                frame_types=None):
    """Build a minimal FrameBatch for testing the repair stage."""
    n = len(cam_ids)
    batch = FrameBatch(
        cam_ids=np.array(cam_ids, dtype=np.int8),
        frame_ids=np.array(frame_ids, dtype=np.uint8),
        side_ids=np.array(side_ids, dtype=np.int8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array(timestamps, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
    if abs_frame_ids is not None:
        batch.abs_frame_ids = np.array(abs_frame_ids, dtype=np.int64)
    if frame_types is not None:
        batch.frame_type = np.array(frame_types, dtype="<U14")
    return batch


def test_clean_passthrough():
    """Clean frames: timestamps unchanged, all quality='ok', batch size unchanged."""
    stage = TimestampRepairStage()
    # 4 clean frames from cam 0, side 0, 25ms apart
    ts = [0.025, 0.050, 0.075, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    result = stage.process(batch)
    np.testing.assert_allclose(result.timestamp_s, ts, atol=1e-9)
    np.testing.assert_array_equal(result.quality, ["ok", "ok", "ok", "ok"])
    assert len(result.cam_ids) == 4


def test_condition1_bad_timestamp_gets_corrected():
    """A frame with timestamp beyond tolerance is corrected via re-anchoring."""
    stage = TimestampRepairStage()
    # Frame 13 (index 2) has a bad timestamp: jumped to 0.130 instead of ~0.075.
    # Frame 14 (index 3) at 0.100 is good and serves as the re-anchor.
    #   frame 11 @0.025 (ok), frame 12 @0.050 (ok),
    #   frame 13 @0.130 (BAD: expected_dt=25ms, actual_dt=80ms, off by 55ms),
    #   frame 14 @0.100 (gap=2 from last_good 12, expected_dt=50ms, actual=50ms => OK)
    # Note: frames 13 and 14 must have DIFFERENT timestamps to avoid cond2 trigger.
    ts = [0.025, 0.050, 0.130, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    result = stage.process(batch)
    # Frames 11 and 12 pass through unchanged
    assert result.quality[0] == "ok"
    assert result.quality[1] == "ok"
    # Frame 13 was bad -- corrected via re-anchoring
    assert result.quality[2] == "ts_corrected"
    # Frame 14 is the re-anchor (good)
    assert result.quality[3] == "ok"
    # Corrected timestamp for frame 13: interpolated between frame 12 (0.050)
    # and frame 14 (0.100). frame 13 is 1/2 of the way from 12 to 14.
    expected_ts_13 = 0.050 + (13 - 12) / (14 - 12) * (0.100 - 0.050)  # = 0.075
    assert abs(result.timestamp_s[2] - expected_ts_13) < 1e-9


def test_condition2_frame_id_disagreement():
    """Cameras at the same timestamp with different frame_ids are flagged bad."""
    stage = TimestampRepairStage()
    # Two cameras at t=0.050 disagree: cam0 says frame_id 12, cam1 says frame_id 13
    # Then a good frame (cam0, frame 14) at t=0.100 re-anchors.
    # For cam0: last_good=(11, 0.025), frame 14 ts=0.100.
    #   fid_gap=3, expected_dt=0.075, actual_dt=0.075 => within tolerance => re-anchor.
    batch = _make_batch(
        cam_ids=[0, 0, 1, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=[0.025, 0.050, 0.050, 0.100],
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    result = stage.process(batch)
    # Both frames at t=0.050 are bad (frame_id disagreement) and same-side
    # as the re-anchor, so both get re-anchored as ts_corrected
    assert result.quality[1] == "ts_corrected"
    assert result.quality[2] == "ts_corrected"


def test_condition2_frame_id_disagreement_is_per_side():
    """Equal timestamps on different modules do not imply a shared packet."""
    stage = TimestampRepairStage()
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 21, 22],
        side_ids=[0, 0, 1, 1],
        timestamps=[0.025, 0.050, 0.025, 0.050],
        abs_frame_ids=[11, 12, 21, 22],
        frame_types=["light", "light", "light", "light"],
    )

    result = stage.process(batch)

    np.testing.assert_array_equal(result.quality, ["ok", "ok", "ok", "ok"])


def test_nan_fill_for_missing_frames():
    """Missing abs_frame_ids get synthetic NaN-fill rows inserted."""
    stage = TimestampRepairStage()
    # Frame 11 then frame 14 -- frames 12 and 13 are missing
    batch = _make_batch(
        cam_ids=[0, 0],
        frame_ids=[11, 14],
        side_ids=[0, 0],
        timestamps=[0.025, 0.100],
        abs_frame_ids=[11, 14],
        frame_types=["light", "light"],
    )
    result = stage.process(batch)
    # Should have 4 rows: original 2 + 2 NaN fills
    assert len(result.cam_ids) == 4
    # Check quality flags
    assert result.quality[0] == "ok"       # frame 11
    assert result.quality[1] == "nan_filled"  # frame 12 (synthetic)
    assert result.quality[2] == "nan_filled"  # frame 13 (synthetic)
    assert result.quality[3] == "ok"       # frame 14
    # Synthetic rows have zero histograms
    assert np.all(result.raw_histograms[1] == 0)
    assert np.all(result.raw_histograms[2] == 0)
    # Timestamps are interpolated
    assert result.timestamp_s[0] == pytest.approx(0.025)
    assert result.timestamp_s[3] == pytest.approx(0.100)
    assert result.timestamp_s[1] > 0.025
    assert result.timestamp_s[2] > result.timestamp_s[1]
    assert result.timestamp_s[2] < 0.100


def test_nan_fill_for_missing_frames_across_process_calls():
    """Missing abs_frame_ids are detected across source batch boundaries."""
    stage = TimestampRepairStage()
    first = _make_batch(
        cam_ids=[0],
        frame_ids=[11],
        side_ids=[0],
        timestamps=[0.025],
        abs_frame_ids=[11],
        frame_types=["light"],
    )
    second = _make_batch(
        cam_ids=[0],
        frame_ids=[13],
        side_ids=[0],
        timestamps=[0.075],
        abs_frame_ids=[13],
        frame_types=["light"],
    )

    stage.process(first)
    result = stage.process(second)

    assert len(result.cam_ids) == 2
    np.testing.assert_array_equal(result.abs_frame_ids, [12, 13])
    np.testing.assert_array_equal(result.quality, ["nan_filled", "ok"])
    assert result.timestamp_s[0] == pytest.approx(0.050)
    assert result.timestamp_s[1] == pytest.approx(0.075)


def test_default_tolerance_catches_ten_ms_timestamp_error():
    """Default condition1 tolerance catches the observed 10ms EMI offset."""
    stage = TimestampRepairStage()
    batch = _make_batch(
        cam_ids=[0, 0, 0],
        frame_ids=[10, 11, 12],
        side_ids=[0, 0, 0],
        timestamps=[0.250, 0.285, 0.300],
        abs_frame_ids=[10, 11, 12],
        frame_types=["light", "light", "light"],
    )

    result = stage.process(batch)

    assert result.quality[1] == "ts_corrected"
    assert result.timestamp_s[1] == pytest.approx(0.275)


def test_default_tolerance_allows_two_ms_device_jitter():
    """Condition1 does not rewrite ordinary 2ms timestamp quantization."""
    stage = TimestampRepairStage()
    batch = _make_batch(
        cam_ids=[0, 0, 0],
        frame_ids=[10, 11, 12],
        side_ids=[0, 0, 0],
        timestamps=[0.250, 0.277, 0.300],
        abs_frame_ids=[10, 11, 12],
        frame_types=["light", "light", "light"],
    )

    result = stage.process(batch)

    np.testing.assert_array_equal(result.quality, ["ok", "ok", "ok"])
    np.testing.assert_allclose(result.timestamp_s, [0.250, 0.277, 0.300])


def test_buffer_force_flush_at_max():
    """When buffer fills without a re-anchor, force-flush using nominal period."""
    stage = TimestampRepairStage(max_buffer_frames=4)
    # 1 good frame, then 5 bad frames (buffer size 4, so force-flush at frame 4)
    ts = [0.025]
    fids = [11]
    # Bad frames: all have timestamp 0.050 (way off from expected 50ms, 75ms, 100ms, 125ms, 150ms)
    for i in range(5):
        ts.append(0.050)
        fids.append(12 + i)
    batch = _make_batch(
        cam_ids=[0] * 6,
        frame_ids=fids,
        side_ids=[0] * 6,
        timestamps=ts,
        abs_frame_ids=fids,
        frame_types=["light"] * 6,
    )
    result = stage.process(batch)
    # The first 4 bad frames should be force-flushed as ts_corrected
    corrected_count = sum(1 for q in result.quality if q == "ts_corrected")
    assert corrected_count >= 4


def test_logging_one_warning_per_window(caplog):
    """One WARNING per misalignment window, not per frame."""
    stage = TimestampRepairStage()
    # Frame 13 bad (jumps to 0.130), frame 14 at 0.100 re-anchors.
    ts = [0.025, 0.050, 0.130, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.process(batch)
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1
    assert "Misalignment window" in warnings[0].message


def test_logging_scan_summary(caplog):
    """End-of-scan summary emitted at on_scan_stop."""
    stage = TimestampRepairStage()
    # Frame 13 bad, frame 14 re-anchors
    ts = [0.025, 0.050, 0.130, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    stage.process(batch)

    flush_batch = _make_batch([], [], [], [], abs_frame_ids=[], frame_types=[])
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.on_scan_stop(flush_batch)
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    summary = [w for w in warnings if "Scan summary" in w.message]
    assert len(summary) == 1


def test_logging_summary_for_nan_fill_without_timestamp_correction(caplog):
    """Clean-cadence frame loss still appears in the end-of-scan summary."""
    stage = TimestampRepairStage()
    batch = _make_batch(
        cam_ids=[0, 0, 0],
        frame_ids=[11, 12, 15],
        side_ids=[0, 0, 0],
        timestamps=[0.025, 0.050, 0.125],
        abs_frame_ids=[11, 12, 15],
        frame_types=["light", "light", "light"],
    )

    result = stage.process(batch)
    np.testing.assert_array_equal(
        result.quality,
        ["ok", "ok", "nan_filled", "nan_filled", "ok"],
    )

    flush_batch = _make_batch([], [], [], [], abs_frame_ids=[], frame_types=[])
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.on_scan_stop(flush_batch)

    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    summary = [w for w in warnings if "Scan summary" in w.message]
    assert len(summary) == 1
    assert "0 frames re-timestamped, 2 frames NaN-filled" in summary[0].message


def test_no_logging_on_clean_scan(caplog):
    """Clean scan produces zero log output."""
    stage = TimestampRepairStage()
    ts = [0.025, 0.050, 0.075, 0.100]
    batch = _make_batch(
        cam_ids=[0, 0, 0, 0],
        frame_ids=[11, 12, 13, 14],
        side_ids=[0, 0, 0, 0],
        timestamps=ts,
        abs_frame_ids=[11, 12, 13, 14],
        frame_types=["light", "light", "light", "light"],
    )
    with caplog.at_level(logging.WARNING, logger="openmotion.sdk.pipeline.stages.timestamp_repair"):
        stage.process(batch)
        flush_batch = _make_batch([], [], [], [], abs_frame_ids=[], frame_types=[])
        stage.on_scan_stop(flush_batch)
    warnings = [r for r in caplog.records if r.levelno >= logging.WARNING]
    assert len(warnings) == 0


def test_warmup_and_stale_frames_pass_through_untouched():
    """Warmup and stale frames are not subject to divergence detection."""
    stage = TimestampRepairStage()
    batch = _make_batch(
        cam_ids=[0, 0, 0],
        frame_ids=[1, 2, 10],
        side_ids=[0, 0, 0],
        timestamps=[0.0, 0.025, 0.250],
        abs_frame_ids=[1, 2, 10],
        frame_types=["warmup", "warmup", "dark"],
    )
    result = stage.process(batch)
    assert len(result.cam_ids) == 3
    np.testing.assert_array_equal(result.quality, ["ok", "ok", "ok"])
