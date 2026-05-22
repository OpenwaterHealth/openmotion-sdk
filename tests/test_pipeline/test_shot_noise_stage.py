"""ShotNoiseCorrectionStage — subtract Poisson variance from corrected variance."""

import numpy as np
from omotion.pipeline.batch import FrameBatch
from omotion.pipeline.stages.shot_noise import ShotNoiseCorrectionStage


ADC_GAIN = (1024 - 64) / 11_000
CAMERA_GAIN_MAP = np.array([16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float32)


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
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    expected_shot_var = ADC_GAIN * 100.0 * CAMERA_GAIN_MAP
    expected_corr_var = np.maximum(0.0, 100.0 - expected_shot_var)
    expected_std = np.sqrt(expected_corr_var).astype(np.float32)
    for s in range(2):
        np.testing.assert_allclose(batch.std_sn_rt[0, s], expected_std, rtol=1e-5)


def test_negative_corrected_variance_clamps_to_zero_std():
    mean = np.full((1, 2, 8), 1000.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 1.0,    dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    assert np.all(batch.std_sn_rt[0, 0, 0] == 0.0)


def test_contrast_computed_with_corrected_std_and_mean():
    mean = np.full((1, 2, 8), 100.0, dtype=np.float32)
    std  = np.full((1, 2, 8), 10.0,  dtype=np.float32)
    batch = _batch_with_dc_rt(mean, std)

    ShotNoiseCorrectionStage(adc_gain=ADC_GAIN, camera_gain_map=CAMERA_GAIN_MAP).process(batch)

    expected = batch.std_sn_rt / 100.0
    np.testing.assert_allclose(batch.contrast_sn_rt, expected, rtol=1e-5)
