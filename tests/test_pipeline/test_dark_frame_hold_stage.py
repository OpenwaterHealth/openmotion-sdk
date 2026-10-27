"""DarkFrameHoldStage — dark frames repeat the last light frame's BFI/BVI."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.dark_frame_hold import DarkFrameHoldStage


def _batch(frame_types, side_ids, cam_ids, bfi_vals, bvi_vals):
    """One frame per entry; bfi/bvi placed at each frame's (side,cam) slot."""
    n = len(frame_types)
    bfi = np.full((n, 2, 8), np.nan, dtype=np.float32)
    bvi = np.full((n, 2, 8), np.nan, dtype=np.float32)
    for i in range(n):
        bfi[i, side_ids[i], cam_ids[i]] = bfi_vals[i]
        bvi[i, side_ids[i], cam_ids[i]] = bvi_vals[i]
    return FrameBatch(
        cam_ids=np.array(cam_ids, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.arange(n, dtype=np.int64),
        frame_type=np.array(frame_types, dtype="<U8"),
        side_ids=np.array(side_ids, dtype=np.int8),
        bfi_live=bfi, bvi_live=bvi,
    )


def test_dark_frame_holds_last_light_value():
    """A dark frame for a camera takes the previous light frame's BFI/BVI
    for that same camera, instead of its own (laser-off) value."""
    batch = _batch(
        frame_types=["light", "dark"],
        side_ids=[0, 0], cam_ids=[3, 3],
        bfi_vals=[0.42, 9.9],   # dark frame's own 9.9 is the spike
        bvi_vals=[5.1, 0.0],
    )
    DarkFrameHoldStage().process(batch)
    # Dark frame (row 1) now carries the light frame's values.
    assert batch.bfi_live[1, 0, 3] == pytest.approx(0.42)
    assert batch.bvi_live[1, 0, 3] == pytest.approx(5.1)
    # Light frame untouched.
    assert batch.bfi_live[0, 0, 3] == pytest.approx(0.42)


def test_hold_is_per_camera():
    """Each camera holds its OWN last light value — no cross-contamination."""
    batch = _batch(
        frame_types=["light", "light", "dark", "dark"],
        side_ids=[0, 1, 0, 1], cam_ids=[2, 5, 2, 5],
        bfi_vals=[0.30, 0.70, 8.0, 8.0],
        bvi_vals=[4.0, 6.0, 0.0, 0.0],
    )
    DarkFrameHoldStage().process(batch)
    assert batch.bfi_live[2, 0, 2] == pytest.approx(0.30)  # left cam2 held
    assert batch.bfi_live[3, 1, 5] == pytest.approx(0.70)  # right cam5 held
    assert batch.bvi_live[2, 0, 2] == pytest.approx(4.0)
    assert batch.bvi_live[3, 1, 5] == pytest.approx(6.0)


def test_dark_before_any_light_is_left_alone():
    """A dark frame with no prior light value to hold passes through
    unchanged (the consumer's NaN/skip handling deals with it)."""
    batch = _batch(
        frame_types=["dark"],
        side_ids=[0], cam_ids=[1],
        bfi_vals=[7.7], bvi_vals=[1.1],
    )
    DarkFrameHoldStage().process(batch)
    assert batch.bfi_live[0, 0, 1] == pytest.approx(7.7)  # unchanged


def test_light_frames_pass_through_unchanged():
    batch = _batch(
        frame_types=["light", "light"],
        side_ids=[0, 0], cam_ids=[0, 0],
        bfi_vals=[0.1, 0.2], bvi_vals=[5.0, 5.1],
    )
    DarkFrameHoldStage().process(batch)
    assert batch.bfi_live[0, 0, 0] == pytest.approx(0.1)
    assert batch.bfi_live[1, 0, 0] == pytest.approx(0.2)


def test_reset_clears_hold_state():
    stage = DarkFrameHoldStage()
    stage.process(_batch(["light"], [0], [4], [0.5], [5.0]))
    stage.reset()
    # After reset, a dark frame has nothing to hold → unchanged.
    batch = _batch(["dark"], [0], [4], [9.0], [0.0])
    stage.process(batch)
    assert batch.bfi_live[0, 0, 4] == pytest.approx(9.0)


def _enriched_light(abs_id, t, temp_c):
    from omotion.pipeline.stages.dark import EnrichedCorrectedFrame
    return EnrichedCorrectedFrame(
        abs_frame_id=abs_id, t=t, side="left", cam_id=0,
        mean=100.0, std=2.0, contrast=0.02, bfi=4.0, bvi=6.0,
        temp_c=temp_c,
    )


def _stencil_one_interval(frames):
    """Run one EnrichedCorrectedInterval through the stage's stencil path
    and return it (D_prev prepended on success)."""
    from omotion.pipeline.batch import IntervalClosed
    from omotion.pipeline.stages.dark import EnrichedCorrectedInterval
    eci = EnrichedCorrectedInterval(
        left_abs=10, right_abs=30, left_t=0.25, frames=frames,
    )
    batch = _batch(["light"], [0], [0], [0.1], [5.0])
    batch.events.append(IntervalClosed(corrected_batch=eci))
    DarkFrameHoldStage().process(batch)
    return eci


def test_stencilled_dark_row_temp_is_stencilled_from_neighbours():
    """The fabricated D_prev row's temp is interpolated by the same
    quadratic stencil as its other metrics (issue #221). With no left
    neighbours (first interval of a scan) the right_only fallback
    applies: (v(D+1) + v(D+2)) / 2."""
    eci = _stencil_one_interval(
        [_enriched_light(11, 0.275, 36.7), _enriched_light(12, 0.300, 36.8)])

    dark_row = eci.frames[0]
    assert dark_row.abs_frame_id == 10                    # D_prev prepended
    assert dark_row.temp_c == pytest.approx((36.7 + 36.8) / 2)
    assert eci.frames[1].temp_c == pytest.approx(36.7)    # lights untouched


def test_stencilled_dark_row_temp_none_when_neighbour_unstamped():
    """A right neighbour without a firmware stamp (temp_c None) leaves the
    dark row's temp None instead of making the stencil raise."""
    eci = _stencil_one_interval([_enriched_light(11, 0.275, None)])

    dark_row = eci.frames[0]
    assert dark_row.abs_frame_id == 10
    assert dark_row.temp_c is None
    assert dark_row.bfi == pytest.approx(4.0)  # other metrics still stencil
