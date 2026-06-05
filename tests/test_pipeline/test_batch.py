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


def _trivial_batch(n: int) -> FrameBatch:
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )
