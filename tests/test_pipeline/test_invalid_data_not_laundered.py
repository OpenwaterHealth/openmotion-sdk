"""Invalid corrected frames must not become plausible BFI (issue #114).

End-to-end over the batch/corrected path — ShotNoiseCorrectionStage →
BfiBviStage → SideAverageStage — the three stages between a dark-corrected
interval and the reduced-mode record that ScanDBSink persists.

The failure this pins: a frame with no usable signal (NaN-fill row for a
dropped frame, or a covered / signal-starved camera whose dark-subtracted
mean is <= 0) used to leave shot-noise correction with contrast 0.0, which
the affine calibration map turns into BFI = (1 + c_min/c_span) * 10 — 10.0 at
default calibration, i.e. maximal blood flow. Nothing downstream gated on it,
so `nanmean` averaged that manufactured top-of-scale value into the side
average the clinical record is built from.
"""

from dataclasses import dataclass

import numpy as np
import pytest

from omotion.config import CAMERA_GAIN_MAP
from omotion.pipeline.batch import FrameBatch, IntervalClosed
from omotion.pipeline.pedestal import SensorPedestals
from omotion.pipeline.stages.bfi_bvi import BfiBviStage
from omotion.pipeline.stages.dark import CorrectedFrame, CorrectedInterval
from omotion.pipeline.stages.shot_noise import ShotNoiseCorrectionStage
from omotion.pipeline.stages.side_avg import SideAverageStage


PEDESTALS = SensorPedestals(left=64.0, right=64.0)
NAN = float("nan")


@dataclass
class _Calibration:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


def _trivial_calibration():
    return _Calibration(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 100.0, dtype=np.float32),
    )


def _cf(cam, mean, std, *, fid=12, quality="ok"):
    return CorrectedFrame(
        abs_frame_id=fid, t=5.0, side="left", cam_id=cam,
        mean=mean, std=std, raw_u1=mean, raw_var=0.0, dark_var=0.0,
        quality=quality,
    )


def _batch(*per_cam_frames):
    """One IntervalClosed per camera, mirroring DarkCorrectionStage's output."""
    b = FrameBatch(
        cam_ids=np.zeros(0, dtype=np.int8), frame_ids=np.zeros(0, dtype=np.uint8),
        side_ids=np.zeros(0, dtype=np.int8),
        raw_histograms=np.zeros((0, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((0, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(0, dtype=np.float64), pdc=None, tcm=None, tcl=None,
    )
    for frames in per_cam_frames:
        b.events.append(IntervalClosed(corrected_batch=CorrectedInterval(
            left_abs=10, right_abs=20, frames=list(frames),
        )))
    return b


def _run(batch):
    """Cameras 0 and 1 only, so the side average is over exactly two cams."""
    ShotNoiseCorrectionStage(pedestals=PEDESTALS,
                             camera_gain_map=CAMERA_GAIN_MAP).process(batch)
    BfiBviStage(calibration=_trivial_calibration()).process(batch)
    SideAverageStage(enabled=True, left_camera_mask=0x03,
                     right_camera_mask=0x03).process(batch)


def _per_cam(batch):
    return {f.cam_id: f
            for e in batch.events if isinstance(e, IntervalClosed)
            for f in getattr(e.corrected_batch, "frames", [])}


def _side_average(batch):
    cams = _per_cam(batch)
    assert -1 in cams, "no cam_id=-1 side-average frame was emitted"
    return cams[-1]


def test_nan_filled_camera_does_not_contribute_to_side_average():
    batch = _batch(
        [_cf(0, NAN, NAN, quality="nan_filled")],   # dropped frame
        [_cf(1, 100.0, 30.0)],                      # healthy camera
    )
    _run(batch)

    cams = _per_cam(batch)
    assert np.isnan(cams[0].bfi), "invalid frame must stay invalid"
    healthy_bfi = cams[1].bfi
    assert np.isfinite(healthy_bfi)
    assert _side_average(batch).bfi == pytest.approx(healthy_bfi)


def test_covered_camera_does_not_contribute_to_side_average():
    """mean <= 0 — the camera saw nothing above its own dark baseline."""
    batch = _batch(
        [_cf(0, -3.0, 2.0)],
        [_cf(1, 100.0, 30.0)],
    )
    _run(batch)

    cams = _per_cam(batch)
    assert np.isnan(cams[0].bfi)
    assert _side_average(batch).bfi == pytest.approx(cams[1].bfi)


def test_side_average_is_nan_when_every_camera_is_invalid():
    """No usable camera means no reading — not a number to plot."""
    batch = _batch(
        [_cf(0, NAN, NAN, quality="nan_filled")],
        [_cf(1, NAN, NAN, quality="nan_filled")],
    )
    _run(batch)

    assert np.isnan(_side_average(batch).bfi)


def test_side_average_still_carries_the_invalid_cameras_status():
    """The frame is dropped from the numeric average but its diagnostic flag
    must still reach the record."""
    batch = _batch(
        [_cf(0, NAN, NAN, quality="nan_filled")],
        [_cf(1, 100.0, 30.0)],
    )
    _run(batch)

    assert _side_average(batch).quality == "l1:nan_filled"
