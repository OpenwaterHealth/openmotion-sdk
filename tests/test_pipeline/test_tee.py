"""Tests for Tee — emits a LiveEmit event for the named channel."""

import numpy as np
from omotion.pipeline.batch import FrameBatch, LiveEmit
from omotion.pipeline.tee import Tee


def _batch_with_frame_types(types):
    n = len(types)
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        frame_type=np.array(types, dtype="<U8"),
    )


def test_tee_with_no_filter_emits_one_event_per_call():
    tee = Tee("raw", filter=None)
    batch = _batch_with_frame_types(["light"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
    assert emits[0].channel == "raw"
    assert emits[0].payload is batch


def test_tee_with_filter_skips_emit_when_filter_excludes_all_frames():
    tee = Tee("live", filter=lambda ft: ft != "warmup" and ft != "stale")
    batch = _batch_with_frame_types(["warmup", "stale", "warmup"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert emits == []


def test_tee_with_filter_emits_when_any_frame_passes():
    tee = Tee("live", filter=lambda ft: ft != "warmup" and ft != "stale")
    batch = _batch_with_frame_types(["warmup", "light", "warmup"])
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
    assert emits[0].channel == "live"


def test_tee_reset_is_a_noop():
    tee = Tee("raw", filter=None)
    tee.reset()


def test_tee_with_max_duration_s_emits_within_budget():
    tee = Tee("raw", filter=None, max_duration_s=60.0)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([0.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1


def test_tee_with_max_duration_s_skips_after_budget():
    tee = Tee("raw", filter=None, max_duration_s=60.0)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([65.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert emits == []


def test_tee_with_max_duration_s_none_means_unbounded():
    tee = Tee("raw", filter=None, max_duration_s=None)
    batch = _batch_with_frame_types(["light"])
    batch.timestamp_s = np.array([9999.0], dtype=np.float64)
    tee.process(batch)
    emits = [e for e in batch.events if isinstance(e, LiveEmit)]
    assert len(emits) == 1
