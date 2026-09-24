"""A frame the dark integrity guard flags is never a dark reference (#292).

It keeps its place in the dark schedule, so intervals still close on time,
but its level and noise come from the last clean dark (or, before the first
clean dark, the next one, or the pedestal). Frames corrected against a held
boundary are marked quality="dark_held".
"""

import numpy as np
import pytest

from omotion.pipeline.batch import (
    QUALITY_RANK, DarkIntegrityWarning, FrameBatch, IntervalClosed,
)
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.stages.dark import (
    DarkCorrectionStage, HybridRealtimePredictor, LinearInterpolation,
)

PEDESTAL = 64.0          # guard threshold = pedestal + 5 = 69
CLEAN, LIT, LIGHT = 65.0, 400.0, 500.0


def _stage():
    return DarkCorrectionStage(
        realtime_estimator=HybridRealtimePredictor(),
        batch_estimator=LinearInterpolation(),
        pedestals=SensorPedestals(left=PEDESTAL, right=PEDESTAL),
    )


def _batch(rows):
    """rows: [(frame_type, abs_id, u1)] for side 0, camera 0 (std 10)."""
    n = len(rows)
    mean = np.zeros((n, 2, 8), dtype=np.float32)
    std = np.full((n, 2, 8), 10.0, dtype=np.float32)
    for i, (_, _, u1) in enumerate(rows):
        mean[i, 0, 0] = u1
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        side_ids=np.zeros(n, dtype=np.int8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.array([a for _, a, _ in rows], dtype=np.float64) * 0.025,
        pdc=None, tcm=None, tcl=None,
        abs_frame_ids=np.array([a for _, a, _ in rows], dtype=np.int64),
        frame_type=np.array([t for t, _, _ in rows], dtype="<U8"),
        mean_raw=mean, std_raw=std,
    )


def _intervals(batch):
    return [e.corrected_batch for e in batch.events if isinstance(e, IntervalClosed)]


def test_flagged_dark_holds_the_last_clean_dark():
    batch = _batch([
        ("dark", 10, CLEAN), ("light", 11, LIGHT), ("light", 12, LIGHT),
        ("dark", 13, LIT),                       # lit frame on a dark slot
        ("light", 14, LIGHT), ("light", 15, LIGHT),
        ("dark", 16, 66.0),
    ])
    _stage().process(batch)

    first, second = _intervals(batch)
    # Both ends of the first interval use the clean 65, never the lit 400.
    assert [f.mean for f in first.frames] == pytest.approx([435.0, 435.0])
    # The second interval runs from the held 65 to the clean 66.
    assert [f.mean for f in second.frames] == pytest.approx(
        [LIGHT - (65 + 1 / 3), LIGHT - (65 + 2 / 3)])
    assert {f.quality for f in first.frames + second.frames} == {"dark_held"}
    assert sum(isinstance(e, DarkIntegrityWarning) for e in batch.events) == 1


def test_first_flagged_dark_takes_the_next_clean_dark():
    batch = _batch([
        ("dark", 10, LIT), ("light", 11, LIGHT), ("light", 12, LIGHT),
        ("dark", 13, 66.0),
    ])
    _stage().process(batch)

    (interval,) = _intervals(batch)
    assert [f.mean for f in interval.frames] == pytest.approx([434.0, 434.0])
    assert {f.quality for f in interval.frames} == {"dark_held"}


def test_no_clean_dark_falls_back_to_the_pedestal():
    batch = _batch([("dark", 10, LIT), ("light", 11, LIGHT), ("dark", 12, 380.0)])
    _stage().process(batch)

    (interval,) = _intervals(batch)
    assert interval.frames[0].mean == pytest.approx(LIGHT - PEDESTAL)
    assert interval.frames[0].quality == "dark_held"


def test_clean_intervals_stay_ok():
    batch = _batch([("dark", 10, CLEAN), ("light", 11, LIGHT), ("dark", 12, 66.0)])
    _stage().process(batch)
    (interval,) = _intervals(batch)
    assert interval.frames[0].quality == "ok"


def test_flagged_dark_stays_out_of_the_realtime_history():
    batch = _batch([
        ("dark", 10, CLEAN), ("light", 11, LIGHT),
        ("dark", 12, LIT), ("light", 13, LIGHT),
    ])
    _stage().process(batch)
    # Frame 13's live baseline is still the clean 65, not an average with 400.
    assert batch.dark_baseline_rt[3, 0, 0] == pytest.approx(CLEAN)
    assert batch.mean_dc_rt[3, 0, 0] == pytest.approx(LIGHT - CLEAN)


def test_realtime_uses_the_pedestal_until_the_first_clean_dark():
    batch = _batch([("dark", 10, LIT), ("light", 11, LIGHT)])
    _stage().process(batch)
    assert batch.mean_dc_rt[1, 0, 0] == pytest.approx(LIGHT - PEDESTAL)


def test_terminal_flush_closes_an_interval_whose_darks_were_all_flagged():
    stage = _stage()
    stage.process(_batch([
        ("dark", 10, LIT), ("light", 11, LIGHT), ("light", 12, LIGHT),
        ("light", 13, 64.5),                     # firmware terminal laser-off
    ]))
    stop = _batch([])
    stage.on_scan_stop(stop)

    (interval,) = _intervals(stop)
    # The held left end takes the terminal dark's level (64.5).
    assert [f.mean for f in interval.frames] == pytest.approx([435.5, 435.5])
    assert {f.quality for f in interval.frames} == {"dark_held"}


def test_dark_held_outranks_nan_filled():
    # A nan_filled camera drops out of an average; a dark_held one does not,
    # so it is the label a merged sample must keep.
    assert QUALITY_RANK["dark_held"] > QUALITY_RANK["nan_filled"] > QUALITY_RANK["ok"]
