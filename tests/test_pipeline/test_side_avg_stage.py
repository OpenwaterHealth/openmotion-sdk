"""LiveSideAverageStage — per-capture spatial side average for reduced mode.

The live USB path delivers ONE camera per frame row, so a capture's cameras
arrive as consecutive rows sharing a frame_id. The stage gathers them and emits
one purely-spatial average per capture per side via LiveEmit(channel="live_side",
SideAverageSample) — no temporal carry, flushed at on_scan_stop.
"""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, LiveEmit, SideAverageSample
from omotion.pipeline.stages.side_avg import LiveSideAverageStage, spatial_side_average


# ── spatial_side_average (pure helper) ───────────────────────────────────────

def test_spatial_side_average_means_selected_cameras():
    vals = np.array([2.0, 4.0, np.nan, np.nan, np.nan, np.nan, 6.0, 8.0])
    cams = np.array([0, 1, 6, 7], dtype=np.int8)
    assert spatial_side_average(vals, cams) == pytest.approx(5.0)  # mean(2,4,6,8)


def test_spatial_side_average_ignores_unselected_cameras():
    vals = np.array([2.0, 4.0, 999.0, np.nan, np.nan, np.nan, 6.0, 8.0])
    cams = np.array([0, 1, 6, 7], dtype=np.int8)
    assert spatial_side_average(vals, cams) == pytest.approx(5.0)


def test_spatial_side_average_is_nan_aware():
    vals = np.array([2.0, np.nan, np.nan, np.nan, np.nan, np.nan, 6.0, np.nan])
    cams = np.array([0, 1, 6, 7], dtype=np.int8)
    assert spatial_side_average(vals, cams) == pytest.approx(4.0)  # mean(2,6)


def test_spatial_side_average_all_nan_returns_nan():
    assert np.isnan(spatial_side_average(np.full(8, np.nan), np.array([0, 1], dtype=np.int8)))


def test_spatial_side_average_empty_selection_returns_nan():
    assert np.isnan(spatial_side_average(np.ones(8), np.array([], dtype=np.int8)))


# ── LiveSideAverageStage ─────────────────────────────────────────────────────

_MASK = 0xC3  # cams 0, 1, 6, 7 (matches reduced-mode default)


def _live_batch(rows):
    """rows: (frame_id, side, cam, bfi, bvi, t). One frame row each, exactly one
    camera finite per row — the live USB layout after MomentsStage NaNs the
    zero-histogram positions."""
    n = len(rows)
    bfi = np.full((n, 2, 8), np.nan, dtype=np.float32)
    bvi = np.full((n, 2, 8), np.nan, dtype=np.float32)
    side_ids = np.array([r[1] for r in rows], dtype=np.int8)
    cam_ids = np.array([r[2] for r in rows], dtype=np.int8)
    abs_ids = np.array([r[0] for r in rows], dtype=np.int64)
    ts = np.array([r[5] for r in rows], dtype=np.float64)
    for i, (_fid, side, cam, b, v, _t) in enumerate(rows):
        bfi[i, side, cam] = b
        bvi[i, side, cam] = v
    return FrameBatch(
        cam_ids=cam_ids, frame_ids=(abs_ids % 256).astype(np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=ts, pdc=None, tcm=None, tcl=None,
        side_ids=side_ids, abs_frame_ids=abs_ids,
        bfi_live=bfi, bvi_live=bvi,
    )


def _empty_flush_batch():
    return FrameBatch(
        cam_ids=np.zeros(0, dtype=np.int8), frame_ids=np.zeros(0, dtype=np.uint8),
        side_ids=np.zeros(0, dtype=np.int8),
        raw_histograms=np.zeros((0, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((0, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(0, dtype=np.float64), pdc=None, tcm=None, tcl=None,
    )


def _side_emits(batch):
    return [e.payload for e in batch.events
            if isinstance(e, LiveEmit) and e.channel == "live_side"]


def _stage(mask=_MASK):
    return LiveSideAverageStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)


def test_disabled_stage_emits_nothing():
    stage = LiveSideAverageStage(enabled=False, left_camera_mask=_MASK, right_camera_mask=_MASK)
    b = _live_batch([(200, 0, 0, 2.0, 20.0, 5.0), (201, 0, 0, 1.0, 10.0, 5.025)])
    stage.process(b)
    assert _side_emits(b) == []


def test_emits_per_capture_spatial_mean():
    b = _live_batch([
        (200, 0, 0, 2.0, 20.0, 5.0),
        (200, 0, 1, 4.0, 40.0, 5.0),
        (200, 0, 6, 6.0, 60.0, 5.0),
        (200, 0, 7, 8.0, 80.0, 5.0),
        (201, 0, 0, 1.0, 10.0, 5.025),  # next capture → emit capture 200
    ])
    _stage().process(b)
    emits = _side_emits(b)
    assert len(emits) == 1
    p = emits[0]
    assert isinstance(p, SideAverageSample)
    assert p.frame_id == 200 and p.side == 0
    assert p.bfi == pytest.approx(5.0) and p.bvi == pytest.approx(50.0)
    assert p.t == pytest.approx(5.0)  # the capture's own timestamp, not 201's


def test_only_selected_cameras_averaged():
    stage = _stage(mask=0x03)  # cams 0, 1
    b = _live_batch([
        (200, 0, 0, 2.0, 20.0, 5.0),
        (200, 0, 1, 4.0, 40.0, 5.0),
        (200, 0, 6, 999.0, 999.0, 5.0),  # not selected
        (201, 0, 0, 1.0, 10.0, 5.025),
    ])
    stage.process(b)
    assert _side_emits(b)[0].bfi == pytest.approx(3.0)  # mean(2,4); 6 ignored


def test_no_temporal_carry_across_captures():
    """Capture 201 has only cam 0 — its average must be cam0 alone, NOT a blend
    with capture 200's other cameras held over."""
    b = _live_batch([
        (200, 0, 0, 2.0, 20.0, 5.0), (200, 0, 1, 4.0, 40.0, 5.0),
        (200, 0, 6, 6.0, 60.0, 5.0), (200, 0, 7, 8.0, 80.0, 5.0),
        (201, 0, 0, 1.0, 10.0, 5.025),   # only cam 0 this capture
        (202, 0, 0, 0.5, 5.0, 5.05),     # triggers emit of 201
    ])
    _stage().process(b)
    by_fid = {p.frame_id: p for p in _side_emits(b)}
    assert by_fid[200].bfi == pytest.approx(5.0)
    assert by_fid[201].bfi == pytest.approx(1.0)  # NOT mean(1,4,6,8)


def test_capture_straddles_batches():
    stage = _stage()
    b1 = _live_batch([(200, 0, 0, 2.0, 20.0, 5.0), (200, 0, 1, 4.0, 40.0, 5.0)])
    stage.process(b1)
    assert _side_emits(b1) == []  # capture 200 not complete yet
    b2 = _live_batch([
        (200, 0, 6, 6.0, 60.0, 5.0), (200, 0, 7, 8.0, 80.0, 5.0),  # 200 continues
        (201, 0, 0, 1.0, 10.0, 5.025),                              # 201 → emit 200
    ])
    stage.process(b2)
    emits = _side_emits(b2)
    assert len(emits) == 1
    assert emits[0].frame_id == 200
    assert emits[0].bfi == pytest.approx(5.0)
    assert emits[0].t == pytest.approx(5.0)


def test_flush_emits_final_capture_on_scan_stop():
    stage = _stage()
    b = _live_batch([
        (200, 0, 0, 2.0, 20.0, 5.0), (200, 0, 1, 4.0, 40.0, 5.0),
        (200, 0, 6, 6.0, 60.0, 5.0), (200, 0, 7, 8.0, 80.0, 5.0),
    ])
    stage.process(b)
    assert _side_emits(b) == []  # 200 still open
    flush = _empty_flush_batch()
    stage.on_scan_stop(flush)
    emits = _side_emits(flush)
    assert len(emits) == 1
    assert emits[0].frame_id == 200 and emits[0].bfi == pytest.approx(5.0)


def test_left_and_right_tracked_independently():
    stage = _stage()
    b = _live_batch([
        (200, 0, 0, 2.0, 20.0, 5.0), (200, 1, 0, 3.0, 30.0, 5.0),
        (200, 0, 1, 4.0, 40.0, 5.0), (200, 1, 1, 5.0, 50.0, 5.0),
        (201, 0, 0, 0.0, 0.0, 5.025), (201, 1, 0, 0.0, 0.0, 5.025),
    ])
    stage.process(b)
    by_side = {p.side: p for p in _side_emits(b)}
    assert by_side[0].frame_id == 200 and by_side[0].bfi == pytest.approx(3.0)   # mean(2,4)
    assert by_side[1].frame_id == 200 and by_side[1].bfi == pytest.approx(4.0)   # mean(3,5)


def test_reset_clears_open_capture():
    stage = _stage()
    stage.process(_live_batch([(200, 0, 0, 2.0, 20.0, 5.0)]))
    stage.reset()
    flush = _empty_flush_batch()
    stage.on_scan_stop(flush)
    assert _side_emits(flush) == []  # reset dropped the open capture
