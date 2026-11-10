"""What the stored record keeps for an unlit camera (issue #341).

A covered sensor or decoupled fiber delivers dark frames where light frames
are scheduled. The realtime path recognises them (``low_light_rt``) and
shows nothing. The stored path flags them ``low_light`` and then:

* keeps the mean (near 0, a real reading);
* keeps contrast and BFI as computed. The dark-subtracted mean scatters
  around zero, so wherever it lands slightly positive with residual
  variance, contrast = std / mean comes out far above any lit value and BFI
  far below zero. A fiber-unplugged bench scan stored about 2,100 such BFI
  values, down to -114,060. Ethan's rule (2026-11-09): store them anyway;
  the flag marks them;
* stores no BVI. Mapped from the ~0 mean it would read the top of the
  scale, 10.0, on every row.

In reduced mode the side average counts the unlit camera toward contrast,
BFI and mean, and leaves it out of BVI.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from omotion.config import CAMERA_GAIN_MAP
from omotion.pipeline.pedestal import adc_gain_for_pedestal
from tests.test_pipeline._speckle import Camera, Scan, run

ADC_GAIN = adc_gain_for_pedestal(128.0)
LIT_MEAN_DN = 60.0
COVERED = 7


@dataclass
class _Calibration:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


CAL = _Calibration(c_min=np.zeros((2, 8)), c_max=np.full((2, 8), 0.4),
                   i_min=np.zeros((2, 8)), i_max=np.full((2, 8), 300.0))


def _camera(cam_id: int, **kw) -> Camera:
    gain = float(CAMERA_GAIN_MAP[cam_id])
    return Camera(cam_id=cam_id, dn_per_e=ADC_GAIN * gain,
                  read_sigma=17.0 if gain == 16 else 4.4, **kw)


def _covered_scan(**kw) -> Scan:
    """Mask 0xC3. Camera 7's light frames are dark frames sitting 0.05 DN
    above its own dark level with a hair more noise — the case that slipped
    past the mean <= 0 and below-Poisson-floor guards of #114."""
    cams = [_camera(c, mean_dn=LIT_MEAN_DN) for c in (0, 1, 6)]
    cams.append(_camera(COVERED, mean_dn=0.0, light_offset_excess=0.05,
                        light_sigma_excess=0.02))
    return Scan(cams, **kw)


def _rows(result, cam_id):
    return [f for f in result.final if f.cam_id == cam_id]


def test_covered_camera_stores_no_bvi():
    rows = _rows(run(_covered_scan(), CAL), COVERED)
    assert rows, "the covered camera produced no stored rows"
    for f in rows:
        assert f.quality == "low_light", f.abs_frame_id
        assert np.isnan(f.bvi), f
        # The ~0 mean is a real reading and stays in the record.
        assert np.isfinite(f.mean) and abs(f.mean) < 1.0


def test_covered_camera_keeps_contrast_and_bfi_as_computed():
    """Contrast and BFI are not blanked: BFI follows the stored contrast
    through the calibration, however far out of range that puts it."""
    rows = _rows(run(_covered_scan(), CAL), COVERED)
    finite = [f for f in rows if np.isfinite(f.contrast)]
    assert finite, "no unlit frame kept a contrast"
    c_min, c_span = 0.0, 0.4
    for f in finite:
        assert f.bfi == pytest.approx((1 - (f.contrast - c_min) / c_span) * 10)
        assert f.contrast > 1.0   # far above any lit camera's ~0.3
    # Where contrast is undefined (mean <= 0 or below the shot-noise floor,
    # the #114 guards), BFI is too.
    for f in rows:
        assert np.isfinite(f.bfi) == np.isfinite(f.contrast), f


def test_lit_cameras_are_untouched():
    result = run(_covered_scan(), CAL)
    for cam in (0, 1, 6):
        rows = _rows(result, cam)
        assert rows
        for f in rows:
            assert f.quality == "ok"
            assert np.isfinite(f.bfi) and np.isfinite(f.bvi)
            assert abs(f.bfi) < 20


def test_side_average_drops_the_covered_camera_from_bvi_only():
    result = run(_covered_scan(reduced_mode=True), CAL)
    side = {f.abs_frame_id: f for f in _rows(result, -1)}
    assert side, "no reduced-mode side average was emitted"
    lit, every = {}, {}
    for cam in (0, 1, 6, COVERED):
        for f in _rows(result, cam):
            every.setdefault(f.abs_frame_id, []).append(f)
            if cam != COVERED:
                lit.setdefault(f.abs_frame_id, []).append(f)
    counted = 0
    for fid, s in side.items():
        assert s.bvi == pytest.approx(np.mean([f.bvi for f in lit[fid]]))
        assert s.mean == pytest.approx(np.mean([f.mean for f in every[fid]]))
        # nanmean: the covered camera counts wherever its contrast is defined.
        assert s.contrast == pytest.approx(np.nanmean([f.contrast for f in every[fid]]))
        assert s.bfi == pytest.approx(np.nanmean([f.bfi for f in every[fid]]))
        counted += np.isfinite([f.contrast for f in every[fid]]).sum() == len(every[fid])
        assert s.quality == "low_light"  # the unlit camera stays visible
    assert counted, "the covered camera never counted toward the side contrast"


def test_dim_but_lit_camera_still_reports():
    """Just above the dark-like threshold (u1 = pedestal + 5 = 133 DN): a
    camera 7 DN over a 127 DN dark level is lit and must keep its values."""
    scan = Scan([_camera(1, mean_dn=7.0)])
    rows = _rows(run(scan, CAL), 1)
    assert rows
    for f in rows:
        assert f.quality == "ok"
        assert np.isfinite(f.bfi) and np.isfinite(f.bvi)
        assert f.contrast == pytest.approx(0.3, rel=0.01)
