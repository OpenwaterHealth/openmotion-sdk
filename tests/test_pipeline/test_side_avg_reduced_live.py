"""SideAveragingStage under REALISTIC reduced-mode live input.

The live USB path delivers ONE camera per frame row (LiveUsbSource._build_batch
sets raw_hist[i, side, cam] for a single cam, zeros elsewhere). MomentsStage
then emits mean=NaN for the zero-histogram positions, which propagates to
bfi_live=NaN at every camera except the one that owns the row. So a per-row
nanmean over the active cameras is NOT a cross-camera average — it's whichever
single camera owns that frame.

These tests pin the bug and the fix: with one-cam-per-row input the stage must
still produce a smooth TRUE side average (across the active cameras' most
recent values), not the jumpy single-camera passthrough.
"""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.side_avg import SideAveragingStage


def _sparse_batch(rows):
    """rows: list of (side_idx, cam_id, bfi, bvi). Each becomes one frame row
    with ONLY [i, side, cam] finite (all other positions NaN), mirroring the
    live one-camera-per-frame layout after MomentsStage NaNs zero histograms."""
    n = len(rows)
    bfi = np.full((n, 2, 8), np.nan, dtype=np.float32)
    bvi = np.full((n, 2, 8), np.nan, dtype=np.float32)
    side_ids = np.zeros(n, dtype=np.int8)
    cam_ids = np.zeros(n, dtype=np.int8)
    for i, (s, c, b, v) in enumerate(rows):
        bfi[i, s, c] = b
        bvi[i, s, c] = v
        side_ids[i] = s
        cam_ids[i] = c
    return FrameBatch(
        cam_ids=cam_ids,
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        side_ids=side_ids,
        bfi_live=bfi, bvi_live=bvi,
    )


def test_one_cam_per_row_produces_true_running_side_average():
    """4 cams active per side (mask 0xC3 = cams 0,1,6,7). Feed the 4 cams of a
    single synced frame as 4 consecutive one-cam rows. By the time the last
    camera's row is processed, the side average must be the mean of all four
    cameras' values — NOT the last camera's value alone (the old behavior)."""
    mask = 0xC3  # cams 0, 1, 6, 7
    stage = SideAveragingStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)
    # Left side, one synced frame split across its 4 cameras.
    batch = _sparse_batch([
        (0, 0, 2.0, 20.0),
        (0, 1, 4.0, 40.0),
        (0, 6, 6.0, 60.0),
        (0, 7, 8.0, 80.0),
    ])
    stage.process(batch)
    # After all 4 cams reported, the running average = mean(2,4,6,8) = 5.0,
    # mean(20,40,60,80) = 50.0 — a true side average.
    assert batch.bfi_live_side[3, 0] == pytest.approx(5.0)
    assert batch.bvi_live_side[3, 0] == pytest.approx(50.0)
    # And it is NOT the jumpy single-camera passthrough (cam7's 8.0).
    assert batch.bfi_live_side[3, 0] != pytest.approx(8.0)


def test_running_average_is_smooth_not_jumpy():
    """The single-camera passthrough bug made the side average swing across
    each camera's value frame-to-frame. The running average must vary far
    less than the raw per-camera spread once all cameras have reported."""
    mask = 0xC3
    stage = SideAveragingStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)
    # Cameras have very different baselines: 0, 10, 0, 10. Cycle through them
    # for several synced frames.
    cam_vals = {0: 0.0, 1: 10.0, 6: 0.0, 7: 10.0}
    rows = []
    for _frame in range(3):
        for c in (0, 1, 6, 7):
            rows.append((0, c, cam_vals[c], cam_vals[c]))
    batch = _sparse_batch(rows)
    stage.process(batch)
    # Once warmed up (after the first full sweep of 4 cams), the side average
    # should sit near the true mean (5.0) and stay there, not bounce 0↔10.
    warm = batch.bfi_live_side[4:, 0]  # rows after the first full sweep
    assert np.all(np.isfinite(warm))
    assert np.ptp(warm) < 1.0, f"side average still jumpy: {warm}"
    assert warm.mean() == pytest.approx(5.0, abs=0.5)


def test_running_average_carries_across_batches():
    """Cameras report across batch boundaries; the latest-value cache must
    persist so the average reflects all cameras, not just this batch's."""
    mask = 0xC3
    stage = SideAveragingStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)
    # Batch 1: cams 0,1 only.
    b1 = _sparse_batch([(0, 0, 2.0, 20.0), (0, 1, 4.0, 40.0)])
    stage.process(b1)
    # Batch 2: cams 6,7. The average must now include cams 0,1 from batch 1.
    b2 = _sparse_batch([(0, 6, 6.0, 60.0), (0, 7, 8.0, 80.0)])
    stage.process(b2)
    assert b2.bfi_live_side[1, 0] == pytest.approx(5.0)  # mean(2,4,6,8)


def test_reset_clears_running_cache():
    mask = 0xC3
    stage = SideAveragingStage(enabled=True, left_camera_mask=mask, right_camera_mask=mask)
    stage.process(_sparse_batch([(0, 0, 2.0, 20.0), (0, 1, 4.0, 40.0)]))
    stage.reset()
    # After reset, only the newly-reported cam contributes (no stale carry).
    b = _sparse_batch([(0, 6, 6.0, 60.0)])
    stage.process(b)
    assert b.bfi_live_side[0, 0] == pytest.approx(6.0)


def test_dense_input_still_matches_per_row_mean():
    """Backward compat: when every active camera is finite in a row (dense
    input, as the older tests assume), the running average equals the plain
    per-row mean across active cams."""
    bfi = np.arange(2 * 2 * 8, dtype=np.float32).reshape(2, 2, 8)
    bvi = bfi + 100
    n = bfi.shape[0]
    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        side_ids=np.zeros(n, dtype=np.int8),
        bfi_live=bfi.astype(np.float32), bvi_live=bvi.astype(np.float32),
    )
    SideAveragingStage(enabled=True, left_camera_mask=0xFF, right_camera_mask=0xFF).process(batch)
    np.testing.assert_allclose(batch.bfi_live_side, bfi.mean(axis=2), rtol=1e-6)
    np.testing.assert_allclose(batch.bvi_live_side, bvi.mean(axis=2), rtol=1e-6)
