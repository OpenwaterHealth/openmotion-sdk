"""SideAverageStage corrected path — dark-corrected per-side average (DB record).

DarkCorrectionStage emits one IntervalClosed(EnrichedCorrectedInterval) per
(side, cam). The stage gathers those per-camera intervals across the active
cameras of a side, groups by frame_id, spatially averages the selected cameras,
and emits one synthetic IntervalClosed per side whose EnrichedCorrectedFrames
carry cam_id=-1 — the side-average convention on the "final" channel.
"""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, IntervalClosed
from omotion.pipeline.stages.dark import EnrichedCorrectedFrame, EnrichedCorrectedInterval
from omotion.pipeline.stages.side_avg import CorrectedSideAverageStage


def _ef(fid, t, side, cam, bfi, bvi, mean=100.0, contrast=0.3):
    return EnrichedCorrectedFrame(
        abs_frame_id=fid, t=t, side=side, cam_id=cam,
        mean=mean, std=mean * contrast, contrast=contrast, bfi=bfi, bvi=bvi,
    )


def _interval(left_abs, right_abs, frames):
    return EnrichedCorrectedInterval(left_abs=left_abs, right_abs=right_abs, frames=frames)


def _batch(intervals=()):
    b = FrameBatch(
        cam_ids=np.zeros(0, dtype=np.int8), frame_ids=np.zeros(0, dtype=np.uint8),
        side_ids=np.zeros(0, dtype=np.int8),
        raw_histograms=np.zeros((0, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((0, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(0, dtype=np.float64), pdc=None, tcm=None, tcl=None,
    )
    for ci in intervals:
        b.events.append(IntervalClosed(corrected_batch=ci))
    return b


def _avg_frames(batch):
    """All cam_id=-1 EnrichedCorrectedFrames from synthetic side-average
    IntervalClosed events appended by the stage."""
    out = []
    for e in batch.events:
        if not isinstance(e, IntervalClosed):
            continue
        for f in getattr(e.corrected_batch, "frames", []):
            if int(getattr(f, "cam_id", -99)) == -1:
                out.append(f)
    return out


def _stage(mask=0x03):  # cams 0, 1
    return CorrectedSideAverageStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)


def test_disabled_stage_emits_nothing():
    stage = CorrectedSideAverageStage(enabled=False, left_camera_mask=0x03, right_camera_mask=0x03)
    b = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])])
    stage.process(b)
    stage.on_scan_stop(b)
    assert _avg_frames(b) == []


def test_emits_per_frame_spatial_average_across_cameras():
    """Both cameras' intervals close in one batch → watermark advances and
    the averages flush during process() itself."""
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0),
                           _ef(13, 5.025, "left", 0, 4.0, 40.0)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0),
                           _ef(13, 5.025, "left", 1, 8.0, 80.0)]),
    ])
    stage.process(b)
    by_fid = {f.abs_frame_id: f for f in _avg_frames(b)}
    assert isinstance(by_fid[12], EnrichedCorrectedFrame)
    assert by_fid[12].side == "left"
    assert by_fid[12].cam_id == -1
    assert by_fid[12].bfi == pytest.approx(4.0)   # mean(2, 6)
    assert by_fid[12].bvi == pytest.approx(40.0)  # mean(20, 60)
    assert by_fid[13].bfi == pytest.approx(6.0)   # mean(4, 8)
    assert by_fid[12].t == pytest.approx(5.0)


def test_flushes_only_below_cross_camera_watermark():
    """cam0 has progressed to 30 but cam1 only to 20 → captures ≥ 20 stay
    buffered until cam1 catches up (or scan stop)."""
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0)]),
        _interval(20, 30, [_ef(22, 5.5, "left", 0, 1.0, 10.0)]),
    ])
    stage.process(b)
    by_fid = {f.abs_frame_id: f for f in _avg_frames(b)}
    assert 12 in by_fid and by_fid[12].bfi == pytest.approx(4.0)  # mean(2, 6)
    assert 22 not in by_fid  # cam1's matching interval hasn't closed yet


def test_no_flush_until_every_enabled_camera_reports():
    """With cams 0 and 1 enabled, cam0's interval alone must not flush —
    flushing early would emit a partial average and later duplicate it."""
    stage = _stage()  # mask 0x03
    b = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])])
    stage.process(b)
    assert _avg_frames(b) == []


def test_staggered_interval_closes_emit_each_capture_exactly_once():
    """cam0 closes in batch 1, cam1 in batch 2: the capture flushes once,
    with both cameras' contributions."""
    stage = _stage()
    b1 = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])])
    stage.process(b1)
    assert _avg_frames(b1) == []
    b2 = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0)])])
    stage.process(b2)
    frames = _avg_frames(b2)
    assert len(frames) == 1
    assert frames[0].bfi == pytest.approx(4.0)  # mean(2, 6) — both cams


def test_missed_dark_spanning_interval_no_duplicates():
    """cam1 misses the dark at 20 so its interval spans (10, 30) while cam0
    closes (10,20) then (20,30). Every capture must be emitted exactly once,
    averaged over both cameras."""
    stage = _stage()
    b1 = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)]),
        _interval(20, 30, [_ef(22, 5.5, "left", 0, 4.0, 40.0)]),
    ])
    stage.process(b1)
    assert _avg_frames(b1) == []  # cam1 hasn't reported at all yet
    b2 = _batch([
        _interval(10, 30, [_ef(12, 5.0, "left", 1, 6.0, 60.0),
                           _ef(22, 5.5, "left", 1, 8.0, 80.0)]),
    ])
    stage.process(b2)
    frames = _avg_frames(b2)
    fids = [f.abs_frame_id for f in frames]
    assert fids == [12, 22]  # each exactly once
    by_fid = {f.abs_frame_id: f for f in frames}
    assert by_fid[12].bfi == pytest.approx(4.0)  # mean(2, 6)
    assert by_fid[22].bfi == pytest.approx(6.0)  # mean(4, 8)


def test_synthetic_interval_carries_flush_bounds():
    stage = _stage(mask=0x01)  # cam 0 only
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)]),
        _interval(20, 30, [_ef(22, 5.5, "left", 0, 1.0, 10.0)]),
    ])
    stage.process(b)
    synth = [e.corrected_batch for e in b.events
             if isinstance(e, IntervalClosed)
             and any(int(getattr(f, "cam_id", 0)) == -1
                     for f in getattr(e.corrected_batch, "frames", []))]
    assert len(synth) == 1
    # left = first flushed capture, right = the watermark (cam0 progress).
    assert (synth[0].left_abs, synth[0].right_abs) == (12, 30)


def test_averages_mean_and_contrast_too():
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0, mean=100.0, contrast=0.2)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0, mean=200.0, contrast=0.4)]),
    ])
    stage.process(b)
    f = _avg_frames(b)[0]
    assert f.mean == pytest.approx(150.0)      # mean(100, 200)
    assert f.contrast == pytest.approx(0.3)    # mean(0.2, 0.4)


def test_only_selected_cameras_averaged():
    stage = _stage(mask=0x01)  # cam 0 only
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 999.0, 999.0)]),  # cam 1 not selected
    ])
    stage.process(b)
    assert _avg_frames(b)[0].bfi == pytest.approx(2.0)  # only cam 0


def test_left_and_right_independent():
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0), _ef(12, 5.0, "left", 1, 4.0, 40.0)]),
        _interval(10, 20, [_ef(12, 5.0, "right", 0, 6.0, 60.0), _ef(12, 5.0, "right", 1, 8.0, 80.0)]),
    ])
    stage.process(b)
    by_side = {f.side: f for f in _avg_frames(b)}
    assert by_side["left"].bfi == pytest.approx(3.0)   # left mean(2,4)
    assert by_side["right"].bfi == pytest.approx(7.0)  # right mean(6,8)


def test_on_scan_stop_flushes_buffered_captures():
    """Captures still below the watermark at scan end flush via on_scan_stop,
    including intervals appended to the flush batch by the terminal-dark
    flush of upstream stages."""
    stage = _stage()
    b = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])])
    stage.process(b)
    flush = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0)])])
    stage.on_scan_stop(flush)
    frames = _avg_frames(flush)
    assert len(frames) == 1
    assert frames[0].bfi == pytest.approx(4.0)  # both cams contributed


def test_worst_contributing_quality_propagates():
    stage = _stage()
    good = _ef(12, 5.0, "left", 0, 2.0, 20.0)
    bad = _ef(12, 5.0, "left", 1, 6.0, 60.0)
    bad.quality = "nan_filled"
    b = _batch([_interval(10, 20, [good]), _interval(10, 20, [bad])])
    stage.process(b)
    assert _avg_frames(b)[0].quality == "nan_filled"


def test_reset_clears_pending_window():
    stage = _stage()
    stage.process(_batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])]))
    stage.reset()
    flush = _batch()
    stage.on_scan_stop(flush)
    assert _avg_frames(flush) == []
