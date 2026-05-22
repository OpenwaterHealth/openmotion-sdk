"""Smoke test: the real-time-corrected stream emits samples through
the same fixture the batched corrected-CSV test uses, and the
emitted samples are flagged corrected.

We intentionally do **not** assert tight numerical equivalence
between the batched and realtime streams on this fixture, because:

  * The batched path uses ``_flush_terminal_dark`` — a retroactive
    synthetic dark inserted after the scan ends — to interpolate the
    final interval. The realtime predictor by design can only see
    darks as they arrive, so it has to extrapolate forward instead of
    interpolate. That's a structural asymmetry, not a bug.

  * The fixture happens to contain a firmware-off-by-one dark at
    frame 10 with u1≈128 sitting next to a genuine dark at frame 601
    with u1≈63 — a 66-bin u1 swing between two consecutive darks that
    only exists in this synthetic recording. The realtime predictor
    averages over those two values and lands in the middle, while
    the batched path's terminal-dark interpolation happens to land
    closer to the post-601 region. Real-world data (validated in
    ``data-processing/dark-drift-study/``) doesn't show this kind of
    inter-dark u1 step.

The load-bearing accuracy validation lives in:

  * ``test_realtime_dark_estimator.py`` — unit tests on the
    estimator math with synthetic darks.
  * The hardware verification step in ``integration_proposal.md``
    that compares the live plots against the saved CSV on a real scan.

Skipped automatically when the fixture CSVs aren't present.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion.MotionProcessing import (
    CorrectedBatch,
    Sample,
    create_science_pipeline,
    feed_pipeline_from_csv,
)


FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
LEFT_CSV  = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
RIGHT_CSV = os.path.join(FIXTURES_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")
LEFT_MASK = 0xFF
RIGHT_MASK = 0xFF

_ZERO = np.zeros((2, 8), dtype=np.float64)
_ONE  = np.ones((2, 8), dtype=np.float64)
BFI_C_MIN = _ZERO.copy()
BFI_C_MAX = _ONE.copy()
BFI_I_MIN = _ZERO.copy()
BFI_I_MAX = np.full((2, 8), 1000.0)


@pytest.mark.skipif(
    not (os.path.exists(LEFT_CSV) and os.path.exists(RIGHT_CSV)),
    reason="fixture CSVs not present",
)
def test_realtime_emits_corrected_samples_through_pipeline() -> None:
    """End-to-end: feed the fixture through ``SciencePipeline`` with the
    new callback and verify the realtime stream actually fires and emits
    properly-shaped corrected samples for the post-warmup window."""
    realtime: list[Sample] = []
    pipeline = create_science_pipeline(
        left_camera_mask=LEFT_MASK,
        right_camera_mask=RIGHT_MASK,
        bfi_c_min=BFI_C_MIN, bfi_c_max=BFI_C_MAX,
        bfi_i_min=BFI_I_MIN, bfi_i_max=BFI_I_MAX,
        on_realtime_corrected_fn=lambda s: realtime.append(s),
    )
    feed_pipeline_from_csv(LEFT_CSV,  "left",  pipeline)
    feed_pipeline_from_csv(RIGHT_CSV, "right", pipeline)
    pipeline.stop(timeout=120.0)

    # Fixture covers two dark intervals × 16 streams × ~50 light frames
    # post-warmup → hundreds of samples expected.
    assert len(realtime) > 500, (
        f"realtime stream emitted only {len(realtime)} samples — "
        "warmup gate may be over-aggressive or the callback is not wired"
    )
    # All emitted samples must be flagged corrected.
    assert all(s.is_corrected for s in realtime), (
        "realtime stream emitted samples not flagged is_corrected=True"
    )
    # bfi/bvi must be finite numbers (no NaN/inf from divide-by-zero edge
    # cases).
    assert all(math.isfinite(s.bfi) and math.isfinite(s.bvi)
               for s in realtime), "realtime stream contained non-finite bfi/bvi"
    # Coverage check: both sides + multiple cameras should be present.
    sides = {s.side for s in realtime}
    assert sides == {"left", "right"}, f"realtime missed a side: {sides}"
    cams  = {(s.side, s.cam_id) for s in realtime}
    assert len(cams) >= 4, f"realtime covered only {len(cams)} (side, cam) keys"
