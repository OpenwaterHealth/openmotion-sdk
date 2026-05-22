"""RollingAverageStage — smooth BFI/BVI over a configurable window."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.rolling_avg import RollingAverageStage


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


def test_rolling_avg_window_1_is_identity():
    bfi = np.arange(5 * 2 * 8, dtype=np.float32).reshape(5, 2, 8)
    bvi = bfi + 100
    batch = _batch_with_bfi_bvi(bfi, bvi)

    RollingAverageStage(window=1).process(batch)

    np.testing.assert_array_equal(batch.bfi_rolling, bfi)
    np.testing.assert_array_equal(batch.bvi_rolling, bvi)


def test_rolling_avg_window_3_averages_last_3():
    bfi = np.arange(5, dtype=np.float32).reshape(5, 1, 1) * np.ones((1, 2, 8), dtype=np.float32)
    bvi = bfi * 2
    batch = _batch_with_bfi_bvi(bfi, bvi)

    RollingAverageStage(window=3).process(batch)

    expected = np.array([0.0, 0.5, 1.0, 2.0, 3.0], dtype=np.float32).reshape(5, 1, 1) * np.ones((1, 2, 8))
    np.testing.assert_allclose(batch.bfi_rolling, expected, rtol=1e-6)


def test_window_state_persists_across_batches():
    stage = RollingAverageStage(window=3)

    bfi1 = np.array([[10.0], [20.0]], dtype=np.float32).reshape(2, 1, 1) * np.ones((1, 2, 8), dtype=np.float32)
    bvi1 = bfi1 * 2
    batch1 = _batch_with_bfi_bvi(bfi1, bvi1)
    stage.process(batch1)

    bfi2 = np.array([[30.0]], dtype=np.float32).reshape(1, 1, 1) * np.ones((1, 2, 8), dtype=np.float32)
    bvi2 = bfi2 * 2
    batch2 = _batch_with_bfi_bvi(bfi2, bvi2)
    stage.process(batch2)

    np.testing.assert_allclose(batch2.bfi_rolling[0], np.full((2, 8), 20.0), rtol=1e-6)


def test_reset_clears_window():
    stage = RollingAverageStage(window=3)
    bfi1 = np.full((2, 2, 8), 100.0, dtype=np.float32)
    bvi1 = bfi1 * 2
    stage.process(_batch_with_bfi_bvi(bfi1, bvi1))

    stage.reset()

    bfi2 = np.full((1, 2, 8), 50.0, dtype=np.float32)
    bvi2 = bfi2 * 2
    batch2 = _batch_with_bfi_bvi(bfi2, bvi2)
    stage.process(batch2)
    np.testing.assert_allclose(batch2.bfi_rolling, np.full((1, 2, 8), 50.0))
