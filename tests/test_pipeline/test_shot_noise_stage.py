"""ShotNoiseCorrectionStage — subtract Poisson variance from corrected variance."""

import numpy as np
import pytest
from omotion.config import CAMERA_GAIN_MAP
from omotion.pipeline.batch import FrameBatch, IntervalClosed
from omotion.pipeline.pedestal import SensorPedestals, adc_gain_for_pedestal
from omotion.pipeline.stages.dark import CorrectedFrame, CorrectedInterval
from omotion.pipeline.stages.shot_noise import ShotNoiseCorrectionStage


# Tests pin to the legacy pedestal (64) so the expected values stay the same
# as before the constants moved out of factory.py.
PEDESTALS = SensorPedestals(left=64.0, right=64.0)
ADC_GAIN = adc_gain_for_pedestal(64.0)


def _batch_with_dc_rt(mean_dc, std_dc):
    n = mean_dc.shape[0]
    return FrameBatch(
        cam_ids=np.zeros(n, dtype=np.int8),
        frame_ids=np.arange(n, dtype=np.uint8),
        raw_histograms=np.zeros((n, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(n, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
        mean_dc_rt=mean_dc,
        std_dc_rt=std_dc,
    )


def test_subtracts_shot_noise_variance_per_camera():
    """Cameras whose shot term stays below the measured variance keep a real
    std; the gain-16 periphery (cams 0 and 7) drops below the Poisson floor at
    these values and is reported unresolvable (NaN) rather than zero."""
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    expected_shot_var = ADC_GAIN * 100.0 * CAMERA_GAIN_MAP
    expected_corr_var = 100.0 - expected_shot_var
    expected_std = np.where(expected_corr_var < 0, np.nan,
                            np.sqrt(np.maximum(0.0, expected_corr_var))).astype(np.float32)
    assert np.isnan(expected_std[0]) and np.isnan(expected_std[7]), "fixture must exercise both"
    assert np.all(np.isfinite(expected_std[1:7]))
    for s in range(2):
        np.testing.assert_allclose(batch.std_sn_rt[0, s], expected_std, rtol=1e-5)


def test_variance_below_poisson_floor_yields_nan_std():
    """A measured variance under the shot-noise floor is unphysical. It used to
    clamp to std 0 -> contrast 0 -> maximal BFI (issue #114)."""
    mean = np.full((1, 2, 8), 1000.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 1.0,    dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    assert np.all(np.isnan(batch.std_sn_rt))
    assert np.all(np.isnan(batch.contrast_sn_rt))


def test_contrast_computed_with_corrected_std_and_mean():
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    expected = batch.std_sn_rt / 100.0
    np.testing.assert_allclose(batch.contrast_sn_rt, expected, rtol=1e-5)


# ── Invalid-input handling (issue #114) ──────────────────────────────────────
# A frame with no usable signal must come out as "no measurement" (NaN), never
# as contrast 0.0 — which the calibration map would turn into a finite,
# super-maximal BFI indistinguishable from real top-of-scale flow.


def _corrected_interval_batch(frames):
    b = FrameBatch(
        cam_ids=np.zeros(0, dtype=np.int8), frame_ids=np.zeros(0, dtype=np.uint8),
        raw_histograms=np.zeros((0, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((0, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(0, dtype=np.float64), pdc=None, tcm=None, tcl=None,
    )
    b.events.append(IntervalClosed(corrected_batch=CorrectedInterval(
        left_abs=10, right_abs=20, frames=frames,
    )))
    return b


def _corrected_frame(mean, std, *, quality="ok"):
    return CorrectedFrame(
        abs_frame_id=12, t=5.0, side="left", cam_id=0,
        mean=mean, std=std, raw_u1=mean, raw_var=std ** 2 if std == std else float("nan"),
        dark_var=0.0, quality=quality,
    )


def test_batch_nan_mean_yields_nan_contrast():
    """A NaN-fill frame reaches the batch path with mean=std=NaN. The builtin
    max(0.0, NaN) used to return 0.0, and `NaN > 0` being False railed contrast
    to 0.0 — laundering missing data into a perfectly coherent speckle field."""
    nan = float("nan")
    batch = _corrected_interval_batch([_corrected_frame(nan, nan, quality="nan_filled")])

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    f = batch.events[0].corrected_batch.frames[0]
    assert np.isnan(f.contrast)
    assert np.isnan(f.std)


def test_batch_nonpositive_mean_yields_nan_contrast():
    """A signal-starved / covered camera dark-subtracts to mean <= 0. Contrast
    is undefined there, not zero."""
    batch = _corrected_interval_batch([_corrected_frame(-3.0, 2.0)])

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    assert np.isnan(batch.events[0].corrected_batch.frames[0].contrast)


def test_batch_positive_mean_still_computes_contrast():
    """Guard rail: the healthy path is untouched. std=30 keeps the signal
    variance above cam 0's shot term (gain 16), so nothing clamps."""
    batch = _corrected_interval_batch([_corrected_frame(100.0, 30.0)])

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    f = batch.events[0].corrected_batch.frames[0]
    expected_var = 30.0 ** 2 - ADC_GAIN * 100.0 * CAMERA_GAIN_MAP[0]
    assert expected_var > 0
    assert f.std == pytest.approx(expected_var ** 0.5)
    assert f.contrast == pytest.approx(expected_var ** 0.5 / 100.0)


def test_batch_clamped_variance_yields_nan_contrast():
    """Measured variance below the Poisson floor is unphysical — the speckle
    variance is unresolvable, not zero. Clamping it to 0 and dividing gave
    contrast exactly 0.0, which the calibration map reads as a perfectly
    coherent speckle field and reports as maximal flow. A pitch-dark scan
    produced 4151 such frames (issue #114, HIL 2026-07-21)."""
    # mean tiny but POSITIVE, so the mean<=0 guard does not fire; std so small
    # that the shot-noise term exceeds the measured variance.
    batch = _corrected_interval_batch([_corrected_frame(0.05, 0.001)])

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    f = batch.events[0].corrected_batch.frames[0]
    assert np.isnan(f.contrast), "clamped variance must not report contrast 0.0"
    assert np.isnan(f.std)


def test_realtime_clamped_variance_yields_nan_contrast():
    mean = np.full((1, 2, 8), 0.05,  dtype=np.float32)
    std  = np.full((1, 2, 8), 0.001, dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    assert np.all(np.isnan(batch.contrast_sn_rt))


def test_realtime_nan_mean_yields_nan_contrast():
    """Reference behavior — the realtime path already propagates NaN."""
    mean = np.full((1, 2, 8), np.nan, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    assert np.all(np.isnan(batch.contrast_sn_rt))


def test_realtime_nonpositive_mean_yields_nan_contrast():
    """A finite mean <= 0 railed the live contrast to 0.0 the same way the
    batch path did — both are now 'no measurement'."""
    mean = np.full((1, 2, 8), -3.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 2.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    assert np.all(np.isnan(batch.contrast_sn_rt))


# --- Per-scan gain maps (sdk#335: the laser optimizer writes its own gains) ---

def test_per_side_gain_map_applies_each_sides_gains_realtime():
    """A (2, 8) map: each side's Poisson term uses that side's gains."""
    gains = np.array([[16, 16, 16, 8, 8, 16, 16, 16],
                      [1, 2, 4, 8, 16, 8, 4, 2]], dtype=np.float32)
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std = np.full((1, 2, 8), 40.0, dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=gains).process(batch)

    for s in range(2):
        expected = np.sqrt(40.0 ** 2 - ADC_GAIN * 100.0 * gains[s])
        np.testing.assert_allclose(batch.std_sn_rt[0, s], expected, rtol=1e-5)


def test_per_side_gain_map_applies_on_the_event_path():
    gains = np.array([[16] * 8, [1] * 8], dtype=np.float32)
    left = _corrected_frame(100.0, 40.0)
    right = CorrectedFrame(
        abs_frame_id=12, t=5.0, side="right", cam_id=0, mean=100.0, std=40.0,
        raw_u1=100.0, raw_var=1600.0, dark_var=0.0, quality="ok",
    )
    batch = _corrected_interval_batch([left, right])

    ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=gains).process(batch)

    out = batch.events[0].corrected_batch.frames
    assert out[0].std == pytest.approx((1600.0 - ADC_GAIN * 100.0 * 16) ** 0.5)
    assert out[1].std == pytest.approx((1600.0 - ADC_GAIN * 100.0 * 1) ** 0.5)


def test_eight_value_map_still_applies_to_both_sides():
    """The stock (8,) map behaves exactly as before."""
    from omotion.pipeline.stages.shot_noise import gain_map_2x8
    np.testing.assert_array_equal(gain_map_2x8(CAMERA_GAIN_MAP),
                                  np.tile(CAMERA_GAIN_MAP, (2, 1)))


@pytest.mark.parametrize("bad", [[1, 2, 3], [[1] * 8] * 3, [0] * 8, [float("nan")] * 8])
def test_malformed_gain_map_is_refused(bad):
    with pytest.raises(ValueError):
        ShotNoiseCorrectionStage(pedestals=PEDESTALS, camera_gain_map=bad)
