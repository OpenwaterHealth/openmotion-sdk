"""The science pipeline recovers known physical truth.

The golden-replay and determinism tests pin the pipeline's output against its
own earlier output, so they would pass on a wrong formula as long as it stayed
wrong consistently. These tests check the arithmetic against an independent
answer instead: histograms are generated from a known sensor model
(``_speckle.py``: speckle of contrast K, Poisson shot noise, read noise around
a dark offset), run through ``default_pipeline()``, and the stored contrast,
mean, BFI and BVI are compared with the model's true values.

The cameras use the stock gain map. On the gain-16 outer positions at K = 0.1
the shot-noise variance is about three times the speckle variance, so a
missing or mis-scaled shot-noise correction, a wrong dark subtraction or a
wrong interpolation fails these tests by a wide margin.

What this does not show: that the model's constants (``ADC_GAIN``,
``CAMERA_GAIN_MAP``, the 11,000 e⁻ full well) match real cameras. The test
uses the pipeline's own constants to generate its data; validating them
against hardware needs a bench photon-transfer measurement.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from omotion.config import CAMERA_GAIN_MAP
from omotion.pipeline.pedestal import adc_gain_for_pedestal
from tests.test_pipeline._speckle import Camera, Scan, run

PEDESTAL = 128.0
ADC_GAIN = adc_gain_for_pedestal(PEDESTAL)
# Per position: read noise (DN) and signal above the dark offset (DN), close
# to a 2026-09 bench scan: the gain-16 outer cameras are read-noise heavy.
READ_SIGMA = [17.0, 4.4, 3.0, 2.5, 2.5, 3.0, 4.4, 17.0]
MEAN_DN = [40.0, 60.0, 80.0, 100.0, 100.0, 80.0, 60.0, 40.0]
C_MAX, I_MAX = 0.5, 300.0


@dataclass
class _Calibration:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


CAL = _Calibration(c_min=np.zeros((2, 8)), c_max=np.full((2, 8), C_MAX),
                   i_min=np.zeros((2, 8)), i_max=np.full((2, 8), I_MAX))


def _cameras(contrast: float) -> list[Camera]:
    return [Camera(cam_id=c, dn_per_e=ADC_GAIN * float(CAMERA_GAIN_MAP[c]),
                   read_sigma=READ_SIGMA[c], mean_dn=MEAN_DN[c], contrast=contrast)
            for c in range(8)]


def _stored_lights(result, scan: Scan):
    """Stored per-camera rows for light frames (the stencilled dark rows are
    interpolated, not measured, and are checked separately)."""
    return [f for f in result.final
            if f.cam_id >= 0 and not scan.is_dark(f.abs_frame_id)]


def _assert_recovers(frames, contrast: float, *, k_rel: float, mean_rel: float):
    assert frames, "no stored frames to check"
    for f in frames:
        assert f.quality == "ok"
        assert f.contrast == pytest.approx(contrast, rel=k_rel), (f.cam_id, f.abs_frame_id)
        assert f.mean == pytest.approx(MEAN_DN[f.cam_id], rel=mean_rel), (f.cam_id, f.abs_frame_id)
        # The stored BFI/BVI must equal the calibration map of the TRUE
        # contrast and mean, within what the tolerances above allow.
        true_mean = MEAN_DN[f.cam_id]
        assert f.bfi == pytest.approx(10 * (1 - contrast / C_MAX),
                                      abs=10 * contrast * k_rel / C_MAX + 1e-6)
        assert f.bvi == pytest.approx(10 * (1 - true_mean / I_MAX),
                                      abs=10 * true_mean * mean_rel / I_MAX + 1e-6)


@pytest.mark.parametrize("contrast", [0.1, 0.3, 0.5])
def test_stored_values_match_the_model_exactly_without_noise_floor(contrast):
    scan = Scan(_cameras(contrast), noise_floor_threshold=0)
    frames = _stored_lights(run(scan, CAL), scan)
    assert {f.cam_id for f in frames} == set(range(8))
    _assert_recovers(frames, contrast, k_rel=1e-3, mean_rel=1e-4)


@pytest.mark.parametrize("contrast", [0.1, 0.5])
def test_production_noise_floor_bias_stays_small(contrast):
    """NoiseFloorStage zeroes bins under 10 counts, trimming the histogram
    tails. Measured bias at these settings is about 0.2 % on contrast and
    0.04 % on the mean; fail if it grows past 0.5 % / 0.1 %."""
    scan = Scan(_cameras(contrast), noise_floor_threshold=10)
    _assert_recovers(_stored_lights(run(scan, CAL), scan), contrast,
                     k_rel=5e-3, mean_rel=1e-3)


def test_linear_dark_drift_is_removed_by_the_stored_path():
    """The dark level climbs 0.25 DN per frame (15 DN over the scan). The
    stored path interpolates the baseline linearly between the bounding
    darks, so a linear drift must cancel exactly."""
    scan = Scan(_cameras(0.3), noise_floor_threshold=0,
                dark_offset=lambda n: 112.0 + 0.25 * n)
    _assert_recovers(_stored_lights(run(scan, CAL), scan), 0.3,
                     k_rel=1e-3, mean_rel=1e-4)


def test_realtime_path_matches_the_model_with_a_steady_dark():
    """With a constant dark level the realtime predictor's baseline is exact
    too, so the live contrast and mean must match the model once the first
    dark has been seen."""
    scan = Scan(_cameras(0.3), noise_floor_threshold=0)
    checked = 0
    for batch in run(scan, CAL).live:
        for i in range(len(batch.cam_ids)):
            cam = int(batch.cam_ids[i])
            if str(batch.frame_type[i]) != "light":
                continue
            mean = float(batch.mean_dc_rt[i, 0, cam])
            contrast = float(batch.contrast_sn_rt[i, 0, cam])
            assert mean == pytest.approx(MEAN_DN[cam], rel=1e-4)
            assert contrast == pytest.approx(0.3, rel=1e-3)
            checked += 1
    assert checked > 8 * 50


def test_stencilled_dark_rows_sit_on_the_signal():
    """Each interval's leading dark frame is stored as an interpolation of
    its neighbours. With a steady signal it must equal the light frames."""
    scan = Scan(_cameras(0.3), noise_floor_threshold=0)
    darks = [f for f in run(scan, CAL).final
             if f.cam_id >= 0 and scan.is_dark(f.abs_frame_id)]
    assert {f.abs_frame_id for f in darks} == {3, 21, 41}
    for f in darks:
        assert f.contrast == pytest.approx(0.3, rel=1e-3)
        assert f.mean == pytest.approx(MEAN_DN[f.cam_id], rel=1e-4)
