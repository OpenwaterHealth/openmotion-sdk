"""DarkCorrectionStage — orchestrator: dual-output realtime + batch."""

import logging

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, IntervalClosed
from omotion.pipeline.stages.dark import (
    DarkCorrectionStage, HybridRealtimePredictor, LinearInterpolation,
    CorrectedFrame, CorrectedInterval,
)


def _batch(n_frames, frame_types, abs_ids, *, mean_raw, std_raw, u2=None,
           side_ids=None):
    """Build a minimal FrameBatch — only side=0 cam=0 populated."""
    n = n_frames
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1   # marker for the only active camera

    if side_ids is None:
        side_ids_arr = np.zeros(n, dtype=np.int8)
    else:
        side_ids_arr = np.asarray(side_ids, dtype=np.int8)

    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=side_ids_arr,
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array(abs_ids, dtype=np.int64),
        frame_type=np.array(frame_types, dtype="<U8"),
        mean_raw=mean_raw, std_raw=std_raw,
    )
    return batch


def test_no_realtime_emit_before_first_dark():
    """Light frames before the first dark have no baseline to subtract — NaN result."""
    mean = np.full((3, 2, 8), 500.0, dtype=np.float32)
    std  = np.full((3, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch(3, ["light", "light", "light"], [11, 12, 13],
                   mean_raw=mean, std_raw=std)

    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)
    assert np.isnan(batch.mean_dc_rt[0, 0, 0])


def test_emits_interval_closed_event_when_two_darks_bracket_lights():
    n = 4
    mean = np.array([[100.0], [500.0], [510.0], [105.0]], dtype=np.float32).reshape(4, 1, 1) * np.ones((1, 2, 8))
    std  = np.array([[10.0],  [20.0],  [22.0],  [11.0]], dtype=np.float32).reshape(4, 1, 1) * np.ones((1, 2, 8))
    batch = _batch(n, ["dark", "light", "light", "dark"], [10, 11, 12, 14],
                   mean_raw=mean.astype(np.float32), std_raw=std.astype(np.float32))

    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)

    events = [e for e in batch.events if isinstance(e, IntervalClosed)]
    assert len(events) > 0


def test_realtime_dark_frame_reuses_previous_live_corrected_values():
    n = 3
    mean = np.array([[100.0], [500.0], [105.0]], dtype=np.float32).reshape(3, 1, 1) * np.ones((1, 2, 8))
    std = np.array([[10.0], [20.0], [11.0]], dtype=np.float32).reshape(3, 1, 1) * np.ones((1, 2, 8))
    batch = _batch(n, ["dark", "light", "dark"], [10, 11, 12],
                   mean_raw=mean.astype(np.float32), std_raw=std.astype(np.float32))

    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)

    assert batch.mean_dc_rt[1, 0, 0] == pytest.approx(400.0)
    assert batch.mean_dc_rt[2, 0, 0] == pytest.approx(batch.mean_dc_rt[1, 0, 0])
    assert batch.std_dc_rt[2, 0, 0] == pytest.approx(batch.std_dc_rt[1, 0, 0])
    assert batch.dark_baseline_rt[2, 0, 0] == pytest.approx(batch.dark_baseline_rt[1, 0, 0])


def test_emits_corrected_interval():
    """DarkCorrectionStage emits raw CorrectedInterval (enrichment is done
    by downstream stages ShotNoiseCorrectionStage and BfiBviStage)."""
    n = 4
    mean = np.array([[100.0], [500.0], [510.0], [105.0]], dtype=np.float32).reshape(4, 1, 1) * np.ones((1, 2, 8))
    std  = np.array([[10.0],  [20.0],  [22.0],  [11.0]], dtype=np.float32).reshape(4, 1, 1) * np.ones((1, 2, 8))
    batch = _batch(n, ["dark", "light", "light", "dark"], [10, 11, 12, 14],
                   mean_raw=mean.astype(np.float32), std_raw=std.astype(np.float32))

    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)

    events = [e for e in batch.events if isinstance(e, IntervalClosed)]
    assert len(events) > 0

    interval = events[0].corrected_batch
    assert isinstance(interval, CorrectedInterval), (
        f"Expected CorrectedInterval, got {type(interval)}"
    )
    assert len(interval.frames) > 0
    f = interval.frames[0]
    assert isinstance(f, CorrectedFrame)
    assert f.side == "left"
    assert f.cam_id == 0
    assert isinstance(f.mean, float)
    assert isinstance(f.std, float)


def test_zero_filled_row_routes_to_source_assigned_right_side():
    """A right-side row whose raw_histogram is all zeros (dropped frame /
    USB stall) must accumulate into the right-side dark history, not left.

    Before the side_ids fix, dark.py used
    ``np.argmax(raw_histograms[i].sum(axis=(-2, -1)))`` to infer the side,
    which silently defaults to 0 for an all-zero row.
    """
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    # One right-side dark frame whose raw_histogram is all zeros (the
    # source still sets side_ids=1 because it knows which endpoint the
    # frame came from).
    batch = FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        side_ids=np.array([1], dtype=np.int8),  # right
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([10], dtype=np.int64),
        frame_type=np.array(["dark"], dtype="<U8"),
        mean_raw=mean, std_raw=std,
    )
    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    stage.process(batch)

    # The dark observation should have landed in the right-side history
    # for cam 0, not the left-side history.
    assert ("right", 0) in stage._pending
    assert ("left", 0) not in stage._pending


def test_reset_clears_dark_history_and_pending():
    stage = DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch1 = _batch(1, ["dark"], [10], mean_raw=mean, std_raw=std)
    stage.process(batch1)

    stage.reset()

    batch2 = _batch(1, ["light"], [11],
                    mean_raw=np.full((1, 2, 8), 500.0, dtype=np.float32),
                    std_raw=np.full((1, 2, 8), 20.0, dtype=np.float32))
    stage.process(batch2)
    assert np.isnan(batch2.mean_dc_rt[0, 0, 0])


def _make_stage_with_cal():
    """Return a DarkCorrectionStage (no longer carries calibration — that's
    now handled by downstream stages)."""
    return DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
    )


def test_emits_raw_corrected_interval_without_stencil():
    """DarkCorrectionStage emits raw CorrectedInterval — no stencil, no
    enrichment. The quadratic stencil is now handled by DarkFrameHoldStage.

    Scenario: dark@10, lights@11-12, dark@30, lights@31-32, dark@50.
    Two intervals should close: [10,30] and [30,50], each containing only
    light CorrectedFrames (no dark-frame rows).
    """
    stage = _make_stage_with_cal()
    n = 7
    types   = ["dark",  "light", "light", "dark",  "light", "light", "dark"]
    abs_ids = [10,      11,      12,      30,      31,      32,      50]
    ts      = [i * 0.025 for i in range(n)]
    mean_v  = [100.0,  500.0,  510.0,  102.0,  505.0,  508.0,  101.0]
    std_v   = [10.0,    20.0,   21.0,   10.5,   20.5,   21.5,   10.2]

    mean_raw = np.zeros((n, 2, 8), dtype=np.float32)
    std_raw  = np.zeros((n, 2, 8), dtype=np.float32)
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1

    for i in range(n):
        mean_raw[i, 0, 0] = mean_v[i]
        std_raw[i, 0, 0]  = std_v[i]

    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array(ts, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array(abs_ids, dtype=np.int64),
        frame_type=np.array(types, dtype="<U8"),
        mean_raw=mean_raw, std_raw=std_raw,
    )

    stage.process(batch)

    intervals = [e.corrected_batch for e in batch.events
                 if isinstance(e, IntervalClosed)]
    assert len(intervals) == 2

    # Both should be raw CorrectedInterval (not enriched)
    for iv in intervals:
        assert isinstance(iv, CorrectedInterval)

    # Interval [10,30]: lights at 11, 12 only (no dark-frame row)
    iv0_ids = [f.abs_frame_id for f in intervals[0].frames]
    assert iv0_ids == [11, 12]
    assert intervals[0].left_abs == 10

    # Interval [30,50]: lights at 31, 32 only
    iv1_ids = [f.abs_frame_id for f in intervals[1].frames]
    assert iv1_ids == [31, 32]
    assert intervals[1].left_abs == 30


def test_terminal_flush_does_not_emit_terminal_dark_as_light():
    """on_scan_stop: the last buffered light is the terminal dark — it should
    NOT appear as a corrected frame in the emitted interval.

    Scenario: dark@10, lights@11-12, then scan ends. The last frame (12) is
    promoted to the terminal dark boundary. Since no lights remain after
    removing 12, the interval emits only the stencil value for dark@10.
    """
    stage = _make_stage_with_cal()
    n = 3
    types   = ["dark", "light", "light"]
    abs_ids = [10, 11, 12]
    mean_raw = np.array([[[65.0]*8]*2, [[500.0]*8]*2, [[66.0]*8]*2], dtype=np.float32)
    std_raw  = np.array([[[10.0]*8]*2,  [[20.0]*8]*2,  [[21.0]*8]*2],  dtype=np.float32)
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1

    process_batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array(abs_ids, dtype=np.int64),
        frame_type=np.array(types, dtype="<U8"),
        mean_raw=mean_raw, std_raw=std_raw,
    )
    stage.process(process_batch)

    # No interval should have closed during process (only 1 dark seen so far)
    closed_in_process = [e for e in process_batch.events
                         if isinstance(e, IntervalClosed)]
    assert len(closed_in_process) == 0

    # Fire terminal flush
    stop_batch = FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        side_ids=np.zeros(1, dtype=np.int8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.zeros(1, dtype=np.int64),
        frame_type=np.array(["dark"], dtype="<U8"),
        mean_raw=np.zeros((1, 2, 8), dtype=np.float32),
        std_raw=np.zeros((1, 2, 8), dtype=np.float32),
    )
    stage.on_scan_stop(stop_batch)

    closed_on_stop = [e for e in stop_batch.events
                      if isinstance(e, IntervalClosed)]
    assert len(closed_on_stop) == 1, (
        f"Expected 1 IntervalClosed from on_scan_stop, got {len(closed_on_stop)}"
    )
    iv = closed_on_stop[0].corrected_batch
    abs_ids_in_iv = [f.abs_frame_id for f in iv.frames]

    # abs_id=12 was the terminal dark — must NOT appear as a light frame
    assert 12 not in abs_ids_in_iv, (
        f"Terminal dark abs_id=12 should not be emitted as a corrected light frame. "
        f"abs_ids in interval: {abs_ids_in_iv}"
    )
    # abs_id=11 was a genuine light frame — should appear
    assert 11 in abs_ids_in_iv, (
        f"Light frame abs_id=11 should be in the corrected interval. "
        f"abs_ids in interval: {abs_ids_in_iv}"
    )


def test_terminal_flush_discards_trailing_dark_like_tail():
    """A stop-trigger drain can contain several laser-off-looking frames.

    The whole trailing dark-like tail should be removed from the pending light
    list, with the last tail frame used as the synthetic terminal boundary.
    """
    stage = _make_stage_with_cal()
    n = 5
    mean_raw = np.array(
        [
            [[65.0] * 8] * 2,
            [[500.0] * 8] * 2,
            [[510.0] * 8] * 2,
            [[66.0] * 8] * 2,
            [[65.0] * 8] * 2,
        ],
        dtype=np.float32,
    )
    std_raw = np.array(
        [
            [[10.0] * 8] * 2,
            [[20.0] * 8] * 2,
            [[21.0] * 8] * 2,
            [[11.0] * 8] * 2,
            [[12.0] * 8] * 2,
        ],
        dtype=np.float32,
    )
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1

    process_batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([10, 11, 12, 13, 14], dtype=np.int64),
        frame_type=np.array(["dark", "light", "light", "light", "light"], dtype="<U8"),
        mean_raw=mean_raw,
        std_raw=std_raw,
    )
    stage.process(process_batch)

    stop_batch = _batch(
        0, [], [], mean_raw=np.zeros((0, 2, 8), dtype=np.float32),
        std_raw=np.zeros((0, 2, 8), dtype=np.float32)
    )
    stage.on_scan_stop(stop_batch)

    closed = [e.corrected_batch for e in stop_batch.events if isinstance(e, IntervalClosed)]
    assert len(closed) == 1
    abs_ids_in_iv = [f.abs_frame_id for f in closed[0].frames]
    assert 11 in abs_ids_in_iv
    assert 12 in abs_ids_in_iv
    assert 13 not in abs_ids_in_iv
    assert 14 not in abs_ids_in_iv


def test_terminal_flush_uses_actual_terminal_dark_moments():
    stage = _make_stage_with_cal()
    n = 3
    mean_raw = np.array([[[65.0]*8]*2, [[500.0]*8]*2, [[66.0]*8]*2], dtype=np.float32)
    std_raw = np.array([[[10.0]*8]*2, [[20.0]*8]*2, [[12.0]*8]*2], dtype=np.float32)
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1

    process_batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([0.0, 0.025, 0.050], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([10, 11, 12], dtype=np.int64),
        frame_type=np.array(["dark", "light", "light"], dtype="<U8"),
        mean_raw=mean_raw, std_raw=std_raw,
    )
    stage.process(process_batch)

    stop_batch = _batch(
        0, [], [], mean_raw=np.zeros((0, 2, 8), dtype=np.float32),
        std_raw=np.zeros((0, 2, 8), dtype=np.float32)
    )
    stage.on_scan_stop(stop_batch)

    closed = [e.corrected_batch for e in stop_batch.events if isinstance(e, IntervalClosed)]
    assert len(closed) == 1
    light = next(f for f in closed[0].frames if f.abs_frame_id == 11)
    # darks at u1=65 (frame 10) and u1=66 (synthetic terminal at frame 12);
    # linear interp at frame 11 = 65.5; corrected_mean = 500 - 65.5 = 434.5
    assert light.mean == pytest.approx(434.5)


def test_terminal_flush_logs_and_skips_when_no_terminal_dark_found(caplog):
    stage = _make_stage_with_cal()
    n = 3
    mean_raw = np.array([[[80.0]*8]*2, [[500.0]*8]*2, [[510.0]*8]*2], dtype=np.float32)
    std_raw = np.array([[[10.0]*8]*2, [[20.0]*8]*2, [[21.0]*8]*2], dtype=np.float32)
    raw_hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    raw_hist[:, 0, 0, 0] = 1

    process_batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=raw_hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([0.0, 0.025, 0.050], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([10, 11, 12], dtype=np.int64),
        frame_type=np.array(["dark", "light", "light"], dtype="<U8"),
        mean_raw=mean_raw, std_raw=std_raw,
    )
    stage.process(process_batch)

    stop_batch = _batch(
        0, [], [], mean_raw=np.zeros((0, 2, 8), dtype=np.float32),
        std_raw=np.zeros((0, 2, 8), dtype=np.float32)
    )
    with caplog.at_level(logging.ERROR, logger="openmotion.sdk.pipeline.stages.dark"):
        stage.on_scan_stop(stop_batch)

    assert [e for e in stop_batch.events if isinstance(e, IntervalClosed)] == []
    assert "TERMINAL DARK MISSING" in caplog.text

    # Should also emit a TerminalDarkResult diagnostic event with found=False
    from omotion.pipeline.batch import TerminalDarkResult
    tdr = [e for e in stop_batch.events if isinstance(e, TerminalDarkResult)]
    assert len(tdr) > 0
    assert tdr[0].found is False
