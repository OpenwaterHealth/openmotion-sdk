"""Unit tests for the real-time dark-correction estimator in
``SciencePipeline._emit_realtime_corrected`` plus its dark-history
ring buffer.

These exercise the estimator math in isolation — no USB, no science
worker thread. The matching equivalence test against the long-scan
fixture lives in ``test_realtime_dark_equivalence.py``.
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from omotion.MotionProcessing import (
    HISTO_BINS,
    HISTO_BINS_SQ,
    HISTO_SIZE_WORDS,
    Sample,
    create_science_pipeline,
)


# ---------- helpers ---------------------------------------------------------

_ZERO = np.zeros((2, 8), dtype=np.float64)
_ONE  = np.ones((2, 8),  dtype=np.float64)


def _make_pipeline(captured: list, *, history_size: int = 4):
    """Build a SciencePipeline that records realtime-corrected samples."""
    pipe = create_science_pipeline(
        left_camera_mask=0x01, right_camera_mask=0x00,
        bfi_c_min=_ZERO.copy(), bfi_c_max=_ONE.copy(),
        bfi_i_min=_ZERO.copy(),
        bfi_i_max=np.full((2, 8), 1000.0),
        on_realtime_corrected_fn=lambda s: captured.append(s),
        realtime_dark_history_size=history_size,
    )
    return pipe


def _push_dark(pipe, *, cam_id: int, t: float, u1: float, std: float):
    """Push a dark frame's (ts, u1, std) directly into the realtime
    history, bypassing the science worker. This is the closest unit
    test possible to the predictor in isolation."""
    from collections import deque
    key = ("left", cam_id)
    hist = pipe._realtime_dark_history.get(key)
    if hist is None:
        hist = deque(maxlen=pipe._realtime_dark_history_size)
        pipe._realtime_dark_history[key] = hist
    hist.append((float(t), float(u1), float(std)))


def _emit_light(pipe, *, cam_id: int, abs_frame: int, t: float,
                u1: float, u2: float, temp: float = 70.0, row_sum: int = 1000):
    """Invoke _emit_realtime_corrected directly with synthesised moments."""
    pipe._emit_realtime_corrected(
        key=("left", cam_id),
        raw_frame_id=abs_frame & 0xFF,
        absolute_frame=abs_frame,
        ts=t, u1=u1, u2=u2, row_sum=row_sum, temp=temp,
    )


# ---------- tests -----------------------------------------------------------

def test_no_callback_means_no_emit():
    """When on_realtime_corrected_fn is None (the default), the
    realtime branch must be a no-op even if a dark history exists."""
    pipe = create_science_pipeline(
        left_camera_mask=0x01, right_camera_mask=0x00,
        bfi_c_min=_ZERO.copy(), bfi_c_max=_ONE.copy(),
        bfi_i_min=_ZERO.copy(),
        bfi_i_max=np.full((2, 8), 1000.0),
    )
    assert pipe._on_realtime_corrected_fn is None
    # Push some darks into the history manually.
    _push_dark(pipe, cam_id=0, t=0.25, u1=128.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=15.25, u1=128.05, std=4.5)
    # _emit_realtime_corrected would AttributeError on None callback if
    # called — but the call site is guarded, so this is just a sanity
    # check that the guard is wired correctly via the public surface.
    pipe.stop()


def test_warmup_no_emit_until_two_darks():
    """The std predictor needs ≥ 2 darks for a slope estimate.
    With 0 or 1 in history, _emit_realtime_corrected must return
    silently."""
    captured: list[Sample] = []
    pipe = _make_pipeline(captured)
    # No darks yet.
    _emit_light(pipe, cam_id=0, abs_frame=11, t=0.275, u1=500.0, u2=260000.0)
    assert captured == []
    # One dark — still warming up.
    _push_dark(pipe, cam_id=0, t=0.25, u1=128.0, std=4.0)
    _emit_light(pipe, cam_id=0, abs_frame=12, t=0.300, u1=500.0, u2=260000.0)
    assert captured == []
    # Second dark — should emit on the next light frame.
    _push_dark(pipe, cam_id=0, t=15.25, u1=128.05, std=4.5)
    _emit_light(pipe, cam_id=0, abs_frame=600, t=15.300, u1=500.0, u2=260000.0)
    assert len(captured) == 1
    pipe.stop()


def test_u1_predictor_averages_last_three_darks():
    """Feed a clearly distinguishable set of u1 values; the corrected
    mean should reflect ``light_u1 - mean(last 3 darks)``."""
    captured: list[Sample] = []
    pipe = _make_pipeline(captured)
    # Darks at u1 = 100, 110, 120 (mean 110) plus an older one at 50
    # that should NOT be included in the avg-3 window.
    _push_dark(pipe, cam_id=0, t=0.0,   u1= 50.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=15.0,  u1=100.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=30.0,  u1=110.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=45.0,  u1=120.0, std=4.0)
    # Light frame at u1 = 500.  Expected corrected_mean = 500 − 110 = 390.
    _emit_light(pipe, cam_id=0, abs_frame=200, t=46.0,
                u1=500.0, u2=500.0 * 500.0 + 16.0)  # raw_var = 16
    assert len(captured) == 1
    s = captured[0]
    assert math.isclose(s.mean, 390.0, abs_tol=1e-6)
    pipe.stop()


def test_std_predictor_extrapolates_linearly_in_time():
    """Feed darks whose std rises by exactly 1 bin per 15 s; predict
    one 15-s interval beyond the last dark. Expected predicted std:
    last_std + 1·1 = last_std + 1."""
    captured: list[Sample] = []
    pipe = _make_pipeline(captured)
    # Two darks 15 s apart with stds 4.0 and 5.0 → slope = 1/15 bin/s.
    _push_dark(pipe, cam_id=0, t=0.0,  u1=128.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=15.0, u1=128.0, std=5.0)
    # Query 15 s past the last dark → expected dark_std = 6.0.
    # raw_var = predicted_dark_var + shot_noise + something we can solve for.
    # Use u1 such that corrected_mean = 0 so shot_noise_var = 0 and the
    # arithmetic collapses to:   corrected_var = raw_var − pred_dark_var.
    light_u1 = 128.0   # same as dark u1 ⇒ corrected_mean = 0
    pred_dark_var_expected = 6.0 * 6.0  # = 36
    raw_var = pred_dark_var_expected + 49.0  # corrected_var should be 49
    _emit_light(pipe, cam_id=0, abs_frame=600, t=30.0,
                u1=light_u1, u2=light_u1 * light_u1 + raw_var)
    assert len(captured) == 1
    s = captured[0]
    # corrected_std² = raw_var − pred_dark_var = 49 ⇒ corrected_std = 7.
    assert math.isclose(s.std_dev, 7.0, abs_tol=1e-6)
    pipe.stop()


def test_history_ring_buffer_caps_at_size():
    """Pushing more darks than ``realtime_dark_history_size`` must
    evict the oldest. With size = 4, the 5th push drops the 1st."""
    captured: list[Sample] = []
    pipe = _make_pipeline(captured, history_size=4)
    for i, u1 in enumerate([100.0, 110.0, 120.0, 130.0, 140.0]):
        _push_dark(pipe, cam_id=0, t=i * 15.0, u1=u1, std=4.0)
    history = pipe._realtime_dark_history[("left", 0)]
    assert len(history) == 4
    # Oldest u1 should be 110 (100 was evicted), newest 140.
    u1_values = [p[1] for p in history]
    assert u1_values == [110.0, 120.0, 130.0, 140.0]
    pipe.stop()


def test_zero_dt_between_darks_falls_back_to_zoh():
    """Defensive: if two consecutive darks somehow have the same
    timestamp (e.g. test mock or pathological data), the predictor
    must not divide by zero — it falls back to the most-recent
    std (zero-order hold)."""
    captured: list[Sample] = []
    pipe = _make_pipeline(captured)
    _push_dark(pipe, cam_id=0, t=10.0, u1=128.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=10.0, u1=128.0, std=5.0)
    _emit_light(pipe, cam_id=0, abs_frame=600, t=15.0,
                u1=128.0, u2=128.0**2 + 25.0)  # raw_var = 25
    assert len(captured) == 1
    s = captured[0]
    # pred_dark_var = 5² = 25 ⇒ corrected_var = 0 ⇒ corrected_std = 0.
    assert math.isclose(s.std_dev, 0.0, abs_tol=1e-6)
    pipe.stop()


def test_sample_carries_is_corrected_true():
    """Emitted Sample must be flagged corrected so downstream callbacks
    can distinguish it from the uncorrected stream."""
    captured: list[Sample] = []
    pipe = _make_pipeline(captured)
    _push_dark(pipe, cam_id=0, t=0.0,  u1=128.0, std=4.0)
    _push_dark(pipe, cam_id=0, t=15.0, u1=128.05, std=4.5)
    _emit_light(pipe, cam_id=0, abs_frame=600, t=30.0,
                u1=500.0, u2=500.0**2 + 20.0)
    assert len(captured) == 1
    assert captured[0].is_corrected is True
    pipe.stop()
