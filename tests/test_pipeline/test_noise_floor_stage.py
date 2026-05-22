"""NoiseFloorStage — zero histogram bins whose count is below threshold."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.noise_floor import NoiseFloorStage


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


def test_zeros_bins_below_threshold():
    hist = np.array([[1, 5, 9, 10, 11, 20]] * (2 * 8 * 1), dtype=np.uint32)
    hist = hist.reshape(1, 2, 8, 6)
    padded = np.zeros((1, 2, 8, 1024), dtype=np.uint32)
    padded[..., :6] = hist
    batch = _batch_with_histogram(padded)

    NoiseFloorStage(threshold=10).process(batch)

    expected_first6 = np.array([0, 0, 0, 10, 11, 20], dtype=np.uint32)
    np.testing.assert_array_equal(batch.raw_histograms[0, 0, 0, :6], expected_first6)


def test_threshold_zero_is_noop():
    hist = np.full((1, 2, 8, 1024), 5, dtype=np.uint32)
    batch = _batch_with_histogram(hist.copy())
    NoiseFloorStage(threshold=0).process(batch)
    np.testing.assert_array_equal(batch.raw_histograms, hist)


def test_vectorized_across_all_dims():
    hist = np.random.randint(0, 50, size=(7, 2, 8, 1024), dtype=np.uint32)
    batch = _batch_with_histogram(hist.copy())
    NoiseFloorStage(threshold=15).process(batch)
    expected = np.where(hist < 15, np.uint32(0), hist)
    np.testing.assert_array_equal(batch.raw_histograms, expected)
