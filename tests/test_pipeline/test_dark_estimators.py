"""HybridRealtimePredictor — avg-of-3 u1 + linear-extrap std + ZOH fallback."""

import numpy as np
import pytest
from omotion.pipeline.stages.dark import (
    DarkHistory, HybridRealtimePredictor,
    PendingInterval, LinearInterpolation, DarkFrameQuadraticStencil,
    DarkObservation,
)


def _hist_with(entries):
    """entries: list of (t, u1, std)"""
    h = DarkHistory(max_darks=10)
    for t, u1, std in entries:
        h.append("left", 0, t=t, u1=u1, std=std)
    return h


def test_zoh_when_only_one_dark():
    h = _hist_with([(0.0, 100.0, 10.0)])
    pred = HybridRealtimePredictor()
    u1, std = pred.predict("left", 0, history=h, target_t=15.0)
    assert u1 == 100.0
    assert std == 10.0


def test_avg_of_2_when_only_two_darks():
    h = _hist_with([(0.0, 100.0, 10.0), (15.0, 200.0, 20.0)])
    pred = HybridRealtimePredictor()
    u1, std = pred.predict("left", 0, history=h, target_t=30.0)
    assert u1 == pytest.approx(150.0)
    assert std == pytest.approx(30.0)


def test_avg_of_3_when_three_or_more_darks():
    h = _hist_with([
        (0.0,  100.0, 10.0),
        (15.0, 110.0, 12.0),
        (30.0, 120.0, 14.0),
        (45.0, 130.0, 16.0),
    ])
    pred = HybridRealtimePredictor()
    u1, std = pred.predict("left", 0, history=h, target_t=60.0)
    assert u1 == pytest.approx(120.0)
    assert std == pytest.approx(18.0)


def test_returns_none_when_history_is_empty():
    h = DarkHistory(max_darks=4)
    pred = HybridRealtimePredictor()
    result = pred.predict("left", 0, history=h, target_t=15.0)
    assert result is None


def test_zoh_std_when_two_darks_have_same_timestamp():
    h = _hist_with([(0.0, 100.0, 10.0), (0.0, 200.0, 20.0)])
    pred = HybridRealtimePredictor()
    u1, std = pred.predict("left", 0, history=h, target_t=15.0)
    assert std == 20.0


def test_pending_interval_collects_frames_between_darks():
    pi = PendingInterval()
    pi.set_left_dark(DarkObservation(t=0.0, u1=100.0, std=10.0), abs_frame_id=10)
    pi.add_light(abs_frame_id=11, t=0.025, u1=500.0, u2=260_000.0)
    pi.add_light(abs_frame_id=12, t=0.050, u1=510.0, u2=265_000.0)
    pi.add_light(abs_frame_id=13, t=0.075, u1=520.0, u2=275_000.0)
    assert not pi.is_closed()

    pi.set_right_dark(DarkObservation(t=0.100, u1=105.0, std=11.0), abs_frame_id=14)
    assert pi.is_closed()
    interval = pi.flush()
    assert interval.left_abs == 10
    assert interval.right_abs == 14
    assert len(interval.light_frames) == 3


def test_linear_interpolation_dark_baseline_across_interval():
    pi = PendingInterval()
    pi.set_left_dark(DarkObservation(t=0.0, u1=100.0, std=10.0), abs_frame_id=10)
    pi.add_light(abs_frame_id=11, t=5.0, u1=500.0, u2=260_000.0)
    pi.set_right_dark(DarkObservation(t=10.0, u1=200.0, std=20.0), abs_frame_id=12)

    interval = pi.flush()
    interp = LinearInterpolation()
    corrected = interp.correct_interval(interval, side="left", cam_id=0)
    f = corrected.frames[0]
    assert f.abs_frame_id == 11
    assert f.side == "left"
    assert f.cam_id == 0
    assert f.mean == pytest.approx(350.0)
    raw_var = 260_000.0 - 500.0 ** 2
    # t_frac=0.5: baseline_var = 10**2 + 0.5*(20**2 - 10**2) = 100 + 150 = 250
    expected_var = max(0.0, raw_var - 250.0)
    assert f.std == pytest.approx(np.sqrt(expected_var))
    # Raw moment fields
    assert f.raw_u1 == pytest.approx(500.0)
    assert f.raw_var == pytest.approx(raw_var)
    assert f.dark_var == pytest.approx(250.0)


def test_linear_interpolation_uses_absolute_frame_position_not_timestamp():
    pi = PendingInterval()
    pi.set_left_dark(DarkObservation(t=0.0, u1=100.0, std=10.0), abs_frame_id=10)
    pi.add_light(abs_frame_id=15, t=1.0, u1=500.0, u2=260_000.0)
    pi.set_right_dark(DarkObservation(t=10.0, u1=200.0, std=20.0), abs_frame_id=20)

    interval = pi.flush()
    corrected = LinearInterpolation().correct_interval(interval, side="left", cam_id=0)

    f = corrected.frames[0]
    assert f.mean == pytest.approx(350.0)
    raw_var = 260_000.0 - 500.0 ** 2
    expected_dark_var = 10.0 ** 2 + 0.5 * (20.0 ** 2 - 10.0 ** 2)
    assert f.dark_var == pytest.approx(expected_dark_var)
    assert f.std == pytest.approx(np.sqrt(max(0.0, raw_var - expected_dark_var)))


def test_stencil_full_4_point_when_all_neighbours_present():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=1.0, v_minus_1=2.0, v_plus_1=4.0, v_plus_2=5.0,
    )
    assert v == pytest.approx(3.0)


def test_stencil_falls_back_to_right_only_when_no_left_neighbours():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=None, v_minus_1=None, v_plus_1=4.0, v_plus_2=5.0,
    )
    assert v == pytest.approx(4.5)


def test_stencil_falls_back_to_simple_avg_when_only_immediate_neighbours():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=None, v_minus_1=2.0, v_plus_1=4.0, v_plus_2=None,
    )
    assert v == pytest.approx(3.0)


def test_stencil_falls_back_to_repeat_right_when_only_right_available():
    stencil = DarkFrameQuadraticStencil()
    v = stencil.interpolate_dark_value(
        v_minus_2=None, v_minus_1=None, v_plus_1=4.0, v_plus_2=None,
    )
    assert v == pytest.approx(4.0)
