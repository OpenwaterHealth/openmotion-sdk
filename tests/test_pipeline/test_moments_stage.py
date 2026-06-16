"""MomentsStage — vectorized mean, std, contrast from histograms."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.moments import MomentsStage


def _batch_with_histogram(hist):
    n = hist.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def _scalar_reference(h):
    bins = np.arange(1024)
    counts = h.sum()
    u1 = float(np.dot(h, bins) / counts) if counts > 0 else 0.0
    u2 = float(np.dot(h, bins ** 2) / counts) if counts > 0 else 0.0
    var = max(0.0, u2 - u1 ** 2)
    return u1, np.sqrt(var)


def test_moments_match_scalar_reference():
    np.random.seed(42)
    hist = np.random.poisson(1000, size=(3, 2, 8, 1024)).astype(np.uint32)
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)

    for n in range(3):
        for s in range(2):
            for c in range(8):
                ref_u1, ref_std = _scalar_reference(hist[n, s, c])
                np.testing.assert_allclose(batch.mean_raw[n, s, c], ref_u1,  rtol=1e-5)
                np.testing.assert_allclose(batch.std_raw[n, s, c],  ref_std, rtol=1e-5)


def test_moments_contrast_raw_is_none():
    """contrast_raw must not be computed in MomentsStage (pedestal-subtracted
    contrast is computed downstream in BfiBviStage)."""
    hist = np.random.poisson(1000, size=(2, 2, 8, 1024)).astype(np.uint32)
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)
    assert batch.contrast_raw is None


def test_variance_clamps_at_zero_for_pathological_input():
    hist = np.zeros((1, 2, 8, 1024), dtype=np.uint32)
    hist[..., 100] = 1000
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)
    assert np.all(batch.std_raw == 0.0)
    np.testing.assert_allclose(batch.mean_raw, 100.0, rtol=1e-6)


def test_zero_count_frame_produces_nan_mean_not_division_error():
    hist = np.zeros((1, 2, 8, 1024), dtype=np.uint32)
    batch = _batch_with_histogram(hist)
    MomentsStage().process(batch)
    assert np.all(np.isnan(batch.mean_raw))
