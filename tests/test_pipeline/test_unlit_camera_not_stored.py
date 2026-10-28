"""An unlit camera never stores a contrast or BFI (issue #341).

A covered sensor or decoupled fiber delivers dark frames where light frames
are scheduled. The realtime path recognises them (``low_light_rt``) and
shows nothing. The stored path used to correct them like any other frame:
the dark-subtracted mean scatters around zero, and whenever it landed
slightly positive with a residual variance left over, contrast = std / mean
came out in the hundreds and BFI in the -10^4 range, while BVI = map(~0 mean)
pinned at 10.0 on every row. A fiber-unplugged bench scan stored about 2,100
such BFI values (down to -114,060), and in reduced mode they were averaged
into the side average ScanDBSink persists.

BVI is still computed from the mean (near 10 for an unlit camera); the
``low_light`` quality flag is what marks the row.
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


def test_covered_camera_stores_no_contrast_or_bfi():
    rows = _rows(run(_covered_scan(), CAL), COVERED)
    assert rows, "the covered camera produced no stored rows"
    for f in rows:
        assert f.quality == "low_light", f.abs_frame_id
        assert np.isnan(f.contrast) and np.isnan(f.bfi), f
        # The ~0 mean is a real reading and stays in the record, and BVI is
        # still computed from it: near the top of the scale.
        assert np.isfinite(f.mean) and abs(f.mean) < 1.0
        assert f.bvi == pytest.approx(10.0, abs=0.05)


def test_lit_cameras_are_untouched():
    result = run(_covered_scan(), CAL)
    for cam in (0, 1, 6):
        rows = _rows(result, cam)
        assert rows
        for f in rows:
            assert f.quality == "ok"
            assert np.isfinite(f.bfi) and np.isfinite(f.bvi)
            assert abs(f.bfi) < 20


def test_side_average_drops_the_covered_camera_from_bfi_only():
    result = run(_covered_scan(reduced_mode=True), CAL)
    side = {f.abs_frame_id: f for f in _rows(result, -1)}
    assert side, "no reduced-mode side average was emitted"
    lit, every = {}, {}
    for cam in (0, 1, 6, COVERED):
        for f in _rows(result, cam):
            every.setdefault(f.abs_frame_id, []).append(f)
            if cam != COVERED:
                lit.setdefault(f.abs_frame_id, []).append(f)
    for fid, s in side.items():
        assert s.bfi == pytest.approx(np.mean([f.bfi for f in lit[fid]]))
        assert s.bvi == pytest.approx(np.mean([f.bvi for f in every[fid]]))
        assert s.mean == pytest.approx(np.mean([f.mean for f in every[fid]]))
        assert s.quality == "low_light"  # the unlit camera stays visible


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
