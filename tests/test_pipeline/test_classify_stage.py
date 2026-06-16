"""FrameClassificationStage — abs_frame_id unwrap + frame_type labeling."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.classify import FrameClassificationStage


def _batch_with_raw_ids(raw_ids_per_side_cam):
    """raw_ids_per_side_cam: dict {(side_idx, cam_id): list_of_raw_frame_ids}."""
    rows = []
    for (side_idx, cam_id), raw_ids in raw_ids_per_side_cam.items():
        for raw_id in raw_ids:
            rows.append((side_idx, cam_id, raw_id))

    n = len(rows)
    cam_ids = np.array([r[1] for r in rows], dtype=np.int8)
    frame_ids = np.array([r[2] for r in rows], dtype=np.uint8)
    side_ids = np.array([r[0] for r in rows], dtype=np.int8)
    raw_hists = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    for i, (s, c, _) in enumerate(rows):
        raw_hists[i, s, c, 0] = 1
    return FrameBatch(
        cam_ids=cam_ids,
        frame_ids=frame_ids,
        side_ids=side_ids,
        raw_histograms=raw_hists,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_first_frame_with_raw_id_1_is_warmup_not_stale():
    batch = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    np.testing.assert_array_equal(batch.abs_frame_ids, [1, 2, 3])
    np.testing.assert_array_equal(batch.frame_type, ["warmup", "warmup", "warmup"])


def test_first_frame_with_raw_id_other_than_1_is_stale():
    batch = _batch_with_raw_ids({(0, 0): [42, 43, 44]})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    assert batch.frame_type[0] == "stale"


def test_warmup_range_marks_first_9_as_warmup():
    raw_ids = list(range(1, 15))
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    expected = ["warmup"] * 9 + ["dark"] + ["light"] * 4
    np.testing.assert_array_equal(batch.frame_type, expected)


def test_dark_schedule_fires_at_intervals():
    raw_ids = list(range(1, 25))
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=10).process(batch)

    expected_darks_at = {10, 11, 21}
    for i, raw_id in enumerate(raw_ids):
        is_dark = (i + 1) in expected_darks_at
        assert (batch.frame_type[i] == "dark") == is_dark, \
            f"abs={i+1}: got {batch.frame_type[i]}, expected dark={is_dark}"


def test_unwrap_handles_8bit_rollover():
    raw_ids = list(range(250, 256)) + list(range(0, 5))
    batch = _batch_with_raw_ids({(0, 0): raw_ids})
    FrameClassificationStage(discard_count=9, dark_interval=600).process(batch)
    expected_abs = [250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260]
    np.testing.assert_array_equal(batch.abs_frame_ids, expected_abs)


def test_zero_filled_row_keeps_source_assigned_side():
    """A row whose raw_histogram is all zeros (e.g. firmware-dropped frame)
    must still be routed to its source-assigned side.

    Before the side_ids fix, classify.py inferred side via
    ``np.argmax(raw_histograms[i].sum(axis=(-2, -1)))`` which silently
    defaults to 0 whenever the histogram is all zeros, so right-side dropped
    frames were misclassified as left.
    """
    # Two right-side rows, both with all-zero histograms — would have
    # defaulted to side=0 under the old logic.
    raw_ids = [1, 2]
    n = len(raw_ids)
    batch = FrameBatch(
        cam_ids=np.array([0, 0], dtype=np.int8),
        frame_ids=np.array(raw_ids, dtype=np.uint8),
        side_ids=np.array([1, 1], dtype=np.int8),  # right
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.arange(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )

    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    stage.process(batch)

    # The unwrapper key is (side_idx, cam_id) — verify state went to side=1.
    assert (1, 0) in stage._unwrappers
    assert (0, 0) not in stage._unwrappers


def test_reset_clears_unwrapper_state():
    stage = FrameClassificationStage(discard_count=9, dark_interval=600)
    batch1 = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    stage.process(batch1)
    assert batch1.abs_frame_ids[0] == 1

    stage.reset()
    batch2 = _batch_with_raw_ids({(0, 0): [1, 2, 3]})
    stage.process(batch2)
    assert batch2.abs_frame_ids[0] == 1
    assert batch2.frame_type[0] == "warmup"
