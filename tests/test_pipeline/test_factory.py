"""default_pipeline factory — assembles the canonical 8-stage + 2-Tee chain."""

import numpy as np
from dataclasses import dataclass
from omotion.pipeline.factory import default_pipeline
from omotion.pipeline.sinks import ScanMetadata
from omotion.pipeline.pedestal import SensorPedestals


@dataclass
class _Cal:
    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray


def _trivial_calibration():
    return _Cal(
        c_min=np.zeros((2, 8), dtype=np.float32),
        c_max=np.ones((2, 8), dtype=np.float32),
        i_min=np.zeros((2, 8), dtype=np.float32),
        i_max=np.full((2, 8), 500.0, dtype=np.float32),
    )


def test_default_pipeline_has_expected_stages():
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=_trivial_calibration(),
        pedestals=SensorPedestals(left=64.0, right=64.0),
    )
    names = [stage.name for stage in pipeline.stages]
    assert names == [
        "frame_classification",
        "tee:raw",
        "noise_floor", "moments", "pedestal_subtraction",
        "dark_correction", "shot_noise_correction", "bfi_bvi",
        "dark_frame_hold",
        "side_average",
        "tee:live",
    ]


def test_default_pipeline_omits_raw_tee_when_duration_zero():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
        raw_save_max_duration_s=0,
    )
    names = [stage.name for stage in pipeline.stages]
    assert "tee:raw" not in names


def test_default_pipeline_includes_raw_tee_with_finite_duration():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
        raw_save_max_duration_s=60.0,
    )
    raw_tee = next(s for s in pipeline.stages if s.name == "tee:raw")
    assert raw_tee.max_duration_s == 60.0


def test_default_pipeline_has_no_telemetry_scaffolding():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(metadata=meta, calibration=cal,
                                pedestals=SensorPedestals(left=64.0, right=64.0))
    names = [s.name for s in pipeline.stages]
    assert "telemetry_ingest" not in names
    assert not hasattr(pipeline, "telemetry_aggregator")


def test_default_pipeline_includes_raw_tee_with_none_unbounded():
    cal = _trivial_calibration()
    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )
    pipeline = default_pipeline(
        metadata=meta, calibration=cal,
        pedestals=SensorPedestals(left=64.0, right=64.0),
        raw_save_max_duration_s=None,
    )
    raw_tee = next(s for s in pipeline.stages if s.name == "tee:raw")
    assert raw_tee.max_duration_s is None
