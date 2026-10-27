"""Pure tests for the ungated-run guard (#256) — no hardware, no threads.

Mean and contrast are non-negative quantities, so a threshold of 0 (or a
missing / non-numeric entry) can never fail: with such thresholds the #199
pre-write gate, the never-write rule, the rollback, and the PASS verdict
all become no-ops, and a far-below-spec calibration is written to the
console EEPROM and reported PASSED. ``start_calibration`` must refuse such
a request unless it explicitly opts in with ``allow_ungated=True``.
"""
import dataclasses
from unittest.mock import MagicMock

from omotion import factory_calibration_thresholds, ungated_cameras
from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationThresholds,
    CalibrationWorkflow,
)


def _zero_thresholds() -> CalibrationThresholds:
    return CalibrationThresholds(
        min_mean_per_camera=[0.0] * 8,
        min_contrast_per_camera=[0.0] * 8,
        min_bfi_per_camera=[0.0] * 8,
        min_bvi_per_camera=[0.0] * 8,
    )


def _request(thresholds, **kw) -> CalibrationRequest:
    return CalibrationRequest(
        operator_id="pytest",
        output_dir=".",
        left_camera_mask=0xFF,
        right_camera_mask=0xFF,
        thresholds=thresholds,
        duration_sec=1,
        **kw,
    )


# ── factory thresholds ────────────────────────────────────────────────────

def test_factory_thresholds_match_spec():
    t = factory_calibration_thresholds()
    assert t.min_mean_per_camera == [40.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 40.0]
    assert t.min_contrast_per_camera == [0.25] * 8
    assert t.min_bfi_per_camera == [-0.5] * 8
    assert t.max_bfi_per_camera == [0.5] * 8
    assert t.min_bvi_per_camera == [4.5] * 8
    assert t.max_bvi_per_camera == [5.5] * 8
    assert t.max_dark_per_camera == [3.0] * 8


def test_factory_thresholds_returns_fresh_instances():
    a = factory_calibration_thresholds()
    b = factory_calibration_thresholds()
    a.min_contrast_per_camera[0] = 0.0
    assert b.min_contrast_per_camera[0] == 0.25


def test_factory_thresholds_are_fully_gated():
    assert ungated_cameras(factory_calibration_thresholds(), 0xFF, 0xFF) == []


def test_wi15_factory_dict_matches_canonical():
    from omotion.scripts.wi15_measurement_calibration import FACTORY_THRESHOLDS
    assert FACTORY_THRESHOLDS == dataclasses.asdict(
        factory_calibration_thresholds()
    )


# ── ungated_cameras ───────────────────────────────────────────────────────

def test_zero_thresholds_flag_every_active_camera():
    assert ungated_cameras(_zero_thresholds(), 0xFF, 0x00) == [
        f"L{n}" for n in range(1, 9)
    ]
    assert ungated_cameras(_zero_thresholds(), 0x00, 0xFF) == [
        f"R{n}" for n in range(1, 9)
    ]


def test_only_active_cameras_are_flagged():
    # 0x03 = cameras 1 and 2 (cam_id 0 and 1)
    assert ungated_cameras(_zero_thresholds(), 0x03, 0x00) == ["L1", "L2"]


def test_either_gate_quantity_missing_flags_the_camera():
    # Real means but zero contrast: the incident shape — a bright camera
    # with below-spec contrast would still slip through.
    t = factory_calibration_thresholds()
    t.min_contrast_per_camera = [0.0] * 8
    assert ungated_cameras(t, 0x01, 0x00) == ["L1"]


def test_short_none_nan_and_negative_entries_are_ineffective():
    t = factory_calibration_thresholds()
    t.min_mean_per_camera = [40.0]          # covers only camera 1
    assert ungated_cameras(t, 0x03, 0x00) == ["L2"]

    t = factory_calibration_thresholds()
    t.min_contrast_per_camera[0] = None
    assert ungated_cameras(t, 0x01, 0x00) == ["L1"]

    t = factory_calibration_thresholds()
    t.min_contrast_per_camera[0] = float("nan")
    assert ungated_cameras(t, 0x01, 0x00) == ["L1"]

    t = factory_calibration_thresholds()
    t.min_mean_per_camera[0] = -5.0
    assert ungated_cameras(t, 0x01, 0x00) == ["L1"]


def test_none_threshold_list_is_fully_ineffective():
    t = factory_calibration_thresholds()
    t.min_mean_per_camera = None
    assert ungated_cameras(t, 0x01, 0x00) == ["L1"]


# ── start_calibration guard ───────────────────────────────────────────────

def test_start_calibration_refuses_ungated_request():
    wf = CalibrationWorkflow(MagicMock())
    logs: list[str] = []
    completions: list = []
    started = wf.start_calibration(
        _request(_zero_thresholds()),
        on_log_fn=logs.append,
        on_complete_fn=completions.append,
    )
    assert started is False
    # Refused synchronously: no worker, no completion callback, and the
    # running slot stays free for a corrected retry.
    assert completions == []
    assert wf._running is False
    assert any("allow_ungated" in m for m in logs)
    assert any("L1" in m and "R8" in m for m in logs)


def test_start_calibration_refusal_reports_only_active_cameras():
    wf = CalibrationWorkflow(MagicMock())
    logs: list[str] = []
    req = dataclasses.replace(
        _request(_zero_thresholds()),
        left_camera_mask=0x01, right_camera_mask=0x00,
    )
    started = wf.start_calibration(req, on_log_fn=logs.append)
    assert started is False
    assert any("L1" in m for m in logs)
    assert not any("L2" in m for m in logs)


def test_gated_thresholds_do_not_trip_the_guard():
    # Factory thresholds must not be refused. Patch the worker spawn out
    # so no thread runs against the MagicMock interface.
    wf = CalibrationWorkflow(MagicMock())
    req = _request(factory_calibration_thresholds())
    import threading as _threading
    from unittest.mock import patch

    class _InertThread:
        def __init__(self, *a, **k): ...
        daemon = True
        def start(self): ...

    with patch.object(_threading, "Thread", _InertThread):
        assert wf.start_calibration(req) is True
    assert wf._running is True  # took the slot — got past the guard


def test_allow_ungated_bypasses_the_guard():
    wf = CalibrationWorkflow(MagicMock())
    req = _request(_zero_thresholds(), allow_ungated=True)
    import threading as _threading
    from unittest.mock import patch

    class _InertThread:
        def __init__(self, *a, **k): ...
        daemon = True
        def start(self): ...

    with patch.object(_threading, "Thread", _InertThread):
        assert wf.start_calibration(req) is True
    assert wf._running is True
