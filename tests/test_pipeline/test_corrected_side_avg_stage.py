"""CorrectedSideAverageStage — dark-corrected per-side average (DB record).

DarkCorrectionStage emits one IntervalClosed(EnrichedCorrectedInterval) per
(side, cam). This stage gathers those per-camera intervals across the active
cameras of a side, groups by frame_id, spatially averages the selected cameras,
and emits one LiveEmit(channel="final_side", SideAverageSample) per capture.
"""

import numpy as np
import pytest
from omotion.pipeline.batch import (
    FrameBatch, IntervalClosed, LiveEmit, SideAverageSample,
)
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


def _final_side(batch):
    return [e.payload for e in batch.events
            if isinstance(e, LiveEmit) and e.channel == "final_side"]


def _stage(mask=0x03):  # cams 0, 1
    return CorrectedSideAverageStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)


def test_disabled_stage_emits_nothing():
    stage = CorrectedSideAverageStage(enabled=False, left_camera_mask=0x03, right_camera_mask=0x03)
    b = _batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])])
    stage.process(b)
    stage.on_scan_stop(b)
    assert _final_side(b) == []


def test_emits_per_frame_spatial_average_across_cameras():
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0),
                           _ef(13, 5.025, "left", 0, 4.0, 40.0)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0),
                           _ef(13, 5.025, "left", 1, 8.0, 80.0)]),
    ])
    stage.process(b)
    flush = _batch()
    stage.on_scan_stop(flush)  # window finalizes at scan stop
    by_fid = {p.frame_id: p for p in _final_side(flush)}
    assert isinstance(by_fid[12], SideAverageSample)
    assert by_fid[12].side == 0
    assert by_fid[12].bfi == pytest.approx(4.0)   # mean(2, 6)
    assert by_fid[12].bvi == pytest.approx(40.0)  # mean(20, 60)
    assert by_fid[13].bfi == pytest.approx(6.0)   # mean(4, 8)
    assert by_fid[12].t == pytest.approx(5.0)


def test_window_finalizes_when_next_window_begins():
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0)]),
        _interval(20, 30, [_ef(22, 5.5, "left", 0, 1.0, 10.0)]),  # new window → finalize prev
    ])
    stage.process(b)
    by_fid = {p.frame_id: p for p in _final_side(b)}
    assert 12 in by_fid and by_fid[12].bfi == pytest.approx(4.0)  # mean(2, 6)
    assert 22 not in by_fid  # window 2 still open until next window / stop


def test_averages_mean_and_contrast_too():
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0, mean=100.0, contrast=0.2)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 6.0, 60.0, mean=200.0, contrast=0.4)]),
    ])
    stage.process(b)
    flush = _batch()
    stage.on_scan_stop(flush)
    p = _final_side(flush)[0]
    assert p.mean == pytest.approx(150.0)      # mean(100, 200)
    assert p.contrast == pytest.approx(0.3)    # mean(0.2, 0.4)


def test_only_selected_cameras_averaged():
    stage = _stage(mask=0x01)  # cam 0 only
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)]),
        _interval(10, 20, [_ef(12, 5.0, "left", 1, 999.0, 999.0)]),  # cam 1 not selected
    ])
    stage.process(b)
    flush = _batch()
    stage.on_scan_stop(flush)
    assert _final_side(flush)[0].bfi == pytest.approx(2.0)  # only cam 0


def test_left_and_right_independent():
    stage = _stage()
    b = _batch([
        _interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0), _ef(12, 5.0, "left", 1, 4.0, 40.0)]),
        _interval(10, 20, [_ef(12, 5.0, "right", 0, 6.0, 60.0), _ef(12, 5.0, "right", 1, 8.0, 80.0)]),
    ])
    stage.process(b)
    flush = _batch()
    stage.on_scan_stop(flush)
    by_side = {p.side: p for p in _final_side(flush)}
    assert by_side[0].bfi == pytest.approx(3.0)  # left mean(2,4)
    assert by_side[1].bfi == pytest.approx(7.0)  # right mean(6,8)


def test_reset_clears_pending_window():
    stage = _stage()
    stage.process(_batch([_interval(10, 20, [_ef(12, 5.0, "left", 0, 2.0, 20.0)])]))
    stage.reset()
    flush = _batch()
    stage.on_scan_stop(flush)
    assert _final_side(flush) == []
