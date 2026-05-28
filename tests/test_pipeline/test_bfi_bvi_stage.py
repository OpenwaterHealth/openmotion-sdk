"""BfiBviStage — affine calibration map from (contrast, mean) to (BFI, BVI)."""

import numpy as np
from dataclasses import dataclass
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.bfi_bvi import BfiBviStage


@dataclass
class _Calibration:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


def _trivial_calibration():
    return _Calibration(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 100.0, dtype=np.float32),
    )


def _batch_with_live_values(mean_dc, contrast_sn):
    n = mean_dc.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        mean_dc_rt=mean_dc,
        contrast_sn_rt=contrast_sn,
    )


def test_calibration_maps_lower_extreme_to_zero():
    """BFI = 0 when K == C_max — legitimate "no blood flow" reading
    (e.g. during a clinical occlusion test). Passes through the
    sanity filter (lower bound is inclusive)."""
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 1.0, dtype=np.float32)
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    np.testing.assert_allclose(batch.bfi_live, 0.0, atol=1e-5)


def test_midpoint_contrast_maps_to_bfi_5():
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.5, dtype=np.float32)
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    np.testing.assert_allclose(batch.bfi_live, 5.0, atol=1e-5)


def test_bvi_uses_mean_with_i_min_i_max():
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.5, dtype=np.float32)  # midrange to avoid sanity filter
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    np.testing.assert_allclose(batch.bvi_live, 5.0, atol=1e-5)


def test_bfi_out_of_sanity_range_replaced_with_nan():
    """Last-frame-at-scan-stop artifact: when contrast K is huge (e.g.
    from near-zero mean after laser turn-off), BFI = (1 - K/span) * 10
    blows up massively negative. Values outside the [-2, 12] sanity
    range get NaN'd so consumers (LivePlotSink, ScanDBSink, viewer
    decimation) skip them naturally instead of plotting garbage."""
    # contrast = 200 → BFI = (1 - 200/1) * 10 = -1990 → NaN
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 200.0, dtype=np.float32)
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    assert np.all(np.isnan(batch.bfi_live))


def test_bvi_out_of_sanity_range_replaced_with_nan():
    """Same as above for BVI when mean is far above i_max."""
    # mean = 5000 with i_min=0, i_max=100 → BVI = (1 - 50) * 10 = -490 → NaN
    mean = np.full((1, 2, 8), 5000.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.5, dtype=np.float32)
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    assert np.all(np.isnan(batch.bvi_live))


def test_bfi_at_formula_upper_extreme_now_nan():
    """BFI = 10.0 EXACTLY when K == C_min EXACTLY is a degenerate-
    input marker — real measurements don't produce bitwise-identical
    floats from independent K and C_min. Filter it out so the
    bloodflow-app cell labels don't display the scan-stop artifact
    pattern (laser off → near-zero mean → tiny K → formula extreme)."""
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.0, dtype=np.float32)  # K = C_min
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    assert np.all(np.isnan(batch.bfi_live))


def test_bfi_near_extreme_passes():
    """BFI = 9.99 (near the top but not exact) is a legitimate
    high-blood-flow reading and passes through unfiltered."""
    # K = 0.001 → BFI = (1 - 0.001) * 10 = 9.99
    mean = np.full((1, 2, 8), 50.0, dtype=np.float32)
    contrast = np.full((1, 2, 8), 0.001, dtype=np.float32)
    batch = _batch_with_live_values(mean, contrast)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    np.testing.assert_allclose(batch.bfi_live, 9.99, atol=1e-4)
