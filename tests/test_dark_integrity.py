"""Unit tests for the SciencePipeline dark-integrity monitor.

Drives the pipeline directly with fake histograms so we can control
exactly what shows up at the "first dark" slot (frame 10 by default).
"""

import numpy as np

from omotion.MotionProcessing import (
    EXPECTED_HISTOGRAM_SUM,
    HISTO_SIZE_WORDS,
    PEDESTAL_HEIGHT,
    SciencePipeline,
)


def _hist_for_mean_std(mean: float, std: float) -> tuple[np.ndarray, int]:
    """Return a 1024-bin histogram whose population mean and std (over the
    bin centres 0..1023) are approximately the requested values, with
    total count == EXPECTED_HISTOGRAM_SUM (so the pipeline doesn't drop
    the frame as a sum-mismatch).
    """
    n_total = int(EXPECTED_HISTOGRAM_SUM)
    bins = np.arange(HISTO_SIZE_WORDS, dtype=float)
    weights = np.exp(-0.5 * ((bins - mean) / max(std, 1e-3)) ** 2)
    weights /= weights.sum()
    counts = np.round(weights * n_total).astype(np.int64)
    diff = n_total - int(counts.sum())
    centre = int(round(mean))
    if 0 <= centre < HISTO_SIZE_WORDS and counts[centre] + diff >= 0:
        counts[centre] += diff
    return counts.astype(np.uint32), int(counts.sum())


def _make_pipeline() -> SciencePipeline:
    p = SciencePipeline(
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        bfi_c_min=np.zeros((2, 8)), bfi_c_max=np.full((2, 8), 0.5),
        bfi_i_min=np.zeros((2, 8)), bfi_i_max=np.full((2, 8), 200.0),
        expected_row_sum=None,  # don't drop our synthetic frames
    )
    p.start()
    return p


def _drive_through_first_dark(pipeline: SciencePipeline, dark_mean: float, dark_std: float):
    """Feed warmup frames + one frame at the 'first dark' position with the
    provided dark statistics, then stop."""
    # First send 9 warmup frames with dummy light-frame histograms — they
    # get discarded before the dark check fires.
    light_h, light_sum = _hist_for_mean_std(180.0, 40.0)
    for fid in range(1, 10):
        pipeline.enqueue("left", 0, fid, fid * 0.025, light_h, light_sum, 25.0)
    # Frame 10 is the first scheduled dark per _is_dark_frame.
    dark_h, dark_sum = _hist_for_mean_std(dark_mean, dark_std)
    pipeline.enqueue("left", 0, 10, 10 * 0.025, dark_h, dark_sum, 25.0)
    pipeline.stop(timeout=5.0)


def test_dark_integrity_passes_for_real_dark():
    """A frame whose u1 ≈ pedestal and std ≈ 5 should pass the integrity
    check. No warnings produced."""
    p = _make_pipeline()
    _drive_through_first_dark(p, dark_mean=PEDESTAL_HEIGHT, dark_std=5.0)
    assert p.dark_integrity_warnings == []


def test_dark_integrity_flags_light_frame_in_dark_slot():
    """A frame at the 'first dark' position whose u1 is high and std is
    high (i.e. a real light frame, the firmware off-by-one symptom)
    must produce a warning."""
    p = _make_pipeline()
    _drive_through_first_dark(p, dark_mean=200.0, dark_std=40.0)
    warnings = p.dark_integrity_warnings
    assert len(warnings) >= 1
    msg = warnings[0]
    assert "DARK INTEGRITY FAILURE" in msg
    assert "left cam 0" in msg
    assert "frame 10" in msg


def test_dark_integrity_flags_high_u1_only():
    """Frame with elevated u1 but otherwise low std should still flag —
    the u1 threshold catches sensor pedestal drift / leakage."""
    p = _make_pipeline()
    _drive_through_first_dark(p, dark_mean=PEDESTAL_HEIGHT + 50.0, dark_std=2.0)
    assert len(p.dark_integrity_warnings) >= 1


def test_dark_integrity_flags_high_std_only():
    """Frame with pedestal-ish u1 but unexpectedly high std (light leak,
    speckle pattern) should flag."""
    p = _make_pipeline()
    _drive_through_first_dark(p, dark_mean=PEDESTAL_HEIGHT, dark_std=30.0)
    assert len(p.dark_integrity_warnings) >= 1
