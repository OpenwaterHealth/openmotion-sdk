"""SideAveragingStage — average BFI/BVI across cameras into per-side values."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.side_avg import SideAveragingStage, spatial_side_average


# ── spatial_side_average (pure helper) ───────────────────────────────────────

def test_spatial_side_average_means_selected_cameras():
    vals = np.array([2.0, 4.0, np.nan, np.nan, np.nan, np.nan, 6.0, 8.0])
    cams = np.array([0, 1, 6, 7], dtype=np.int8)
    assert spatial_side_average(vals, cams) == pytest.approx(5.0)  # mean(2,4,6,8)


def test_spatial_side_average_ignores_unselected_cameras():
    # cam 2 carries a large value but is not in the active selection.
    vals = np.array([2.0, 4.0, 999.0, np.nan, np.nan, np.nan, 6.0, 8.0])
    cams = np.array([0, 1, 6, 7], dtype=np.int8)
    assert spatial_side_average(vals, cams) == pytest.approx(5.0)


def test_spatial_side_average_is_nan_aware():
    vals = np.array([2.0, np.nan, np.nan, np.nan, np.nan, np.nan, 6.0, np.nan])
    cams = np.array([0, 1, 6, 7], dtype=np.int8)
    assert spatial_side_average(vals, cams) == pytest.approx(4.0)  # mean(2,6)


def test_spatial_side_average_all_nan_returns_nan():
    vals = np.full(8, np.nan)
    cams = np.array([0, 1], dtype=np.int8)
    assert np.isnan(spatial_side_average(vals, cams))


def test_spatial_side_average_empty_selection_returns_nan():
    vals = np.ones(8)
    cams = np.array([], dtype=np.int8)
    assert np.isnan(spatial_side_average(vals, cams))


# ── SideAveragingStage ───────────────────────────────────────────────────────

def _batch_with_bfi_bvi(bfi, bvi):
    n = bfi.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        bfi_live=bfi.astype(np.float32),
        bvi_live=bvi.astype(np.float32),
    )


def test_disabled_stage_is_noop():
    bfi = np.full((3, 2, 8), 5.0, dtype=np.float32)
    bvi = np.full((3, 2, 8), 7.0, dtype=np.float32)
    batch = _batch_with_bfi_bvi(bfi, bvi)
    SideAveragingStage(enabled=False, left_camera_mask=0xFF, right_camera_mask=0xFF).process(batch)
    assert batch.bfi_live_side is None
    assert batch.bvi_live_side is None


def test_enabled_with_full_mask_averages_all_8():
    bfi = np.arange(2 * 2 * 8, dtype=np.float32).reshape(2, 2, 8)
    bvi = bfi + 100
    batch = _batch_with_bfi_bvi(bfi, bvi)
    SideAveragingStage(enabled=True, left_camera_mask=0xFF, right_camera_mask=0xFF).process(batch)

    expected_bfi = bfi.mean(axis=2)
    expected_bvi = bvi.mean(axis=2)
    np.testing.assert_allclose(batch.bfi_live_side, expected_bfi, rtol=1e-6)
    np.testing.assert_allclose(batch.bvi_live_side, expected_bvi, rtol=1e-6)


def test_enabled_with_partial_mask_only_averages_active_cams():
    bfi = np.array([[1, 2, 3, 4, 5, 6, 7, 8]] * 2, dtype=np.float32).reshape(1, 2, 8)
    bvi = bfi * 10
    batch = _batch_with_bfi_bvi(bfi, bvi)
    SideAveragingStage(enabled=True, left_camera_mask=0x66, right_camera_mask=0x66).process(batch)

    # mask 0x66 = 01100110 → cams 1, 2, 5, 6 active → values 2, 3, 6, 7 → mean 4.5
    np.testing.assert_allclose(batch.bfi_live_side[0, 0], 4.5, rtol=1e-6)
    np.testing.assert_allclose(batch.bfi_live_side[0, 1], 4.5, rtol=1e-6)
