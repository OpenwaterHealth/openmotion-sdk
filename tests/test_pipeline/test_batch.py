"""Tests for FrameBatch — the per-batch data carrier that flows through stages."""

import numpy as np
import pytest

from omotion.pipeline.batch import FrameBatch, BatchEvent, IntervalClosed


def test_framebatch_construction_minimum_fields():
    """A FrameBatch can be constructed with only the fields HistogramParseStage populates.
    All later-stage fields default to None."""
    n = 5
    batch = FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.linspace(0.0, 0.1, n, dtype=np.float64),
        pdc=None,
        tcm=None,
        tcl=None,
    )
    assert batch.raw_histograms.shape == (n, 2, 8, 1024)
    assert batch.mean_raw is None
    assert batch.bfi_live is None
    assert batch.events == []


def test_framebatch_events_list_is_mutable():
    batch = _trivial_batch(n=1)
    batch.events.append(IntervalClosed(corrected_batch=None))
    assert len(batch.events) == 1
    assert isinstance(batch.events[0], IntervalClosed)


def test_framebatch_has_quality_field():
    """FrameBatch must expose an optional quality array (spec §5.1)."""
    batch = FrameBatch(
        cam_ids=np.array([0], dtype=np.int8),
        frame_ids=np.array([1], dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.array([0.0], dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
    assert batch.quality is None

    batch.quality = np.array(["ok"], dtype="<U14")
    assert batch.quality[0] == "ok"


def test_framebatch_snapshot_is_independent_copy():
    """snapshot() deep-copies every numpy array so later in-place mutation
    of the original cannot reach the snapshot (raw-CSV faithfulness)."""
    batch = _trivial_batch(n=3)
    batch.raw_histograms[:] = 42
    batch.timestamp_s[:] = [1.0, 2.0, 3.0]
    batch.abs_frame_ids = np.array([10, 11, 12], dtype=np.int64)
    batch.frame_type = np.array(["light", "light", "light"], dtype="<U14")

    snap = batch.snapshot()

    # Mutating the original must not touch the snapshot.
    batch.raw_histograms[:] = 0
    batch.timestamp_s[0] = 99.0
    batch.abs_frame_ids[0] = -1

    assert np.all(snap.raw_histograms == 42)
    assert snap.timestamp_s[0] == 1.0
    assert snap.abs_frame_ids[0] == 10
    assert snap.raw_histograms is not batch.raw_histograms


def test_framebatch_snapshot_preserves_none_fields_and_fresh_events():
    """None-valued fields stay None; the snapshot gets its own empty
    events list (it is a passive payload, not a live batch)."""
    batch = _trivial_batch(n=1)
    batch.events.append(IntervalClosed(corrected_batch=None))

    snap = batch.snapshot()

    assert snap.pdc is None
    assert snap.mean_raw is None
    assert snap.events == []          # not carried over
    assert snap.events is not batch.events


def _trivial_batch(n: int) -> FrameBatch:
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
