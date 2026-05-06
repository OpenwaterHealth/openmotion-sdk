"""
Calibration procedure end-to-end test.

Drives ``MotionInterface.start_calibration`` through both sub-scans
(flash → trigger reset → calibration scan → compute → write → trigger
reset → validation scan → evaluate) and verifies the procedure
completes, writes a calibration to the console, produces a CSV, and
refreshes the SDK cache.

Skips gracefully when the console or both sensors are missing —
matches the existing ``test_console.py`` / ``test_sensor.py`` style.
"""

import os
import threading
import time

import pytest

from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationThresholds,
)


# Calibration drives the full ScanWorkflow path on real hardware. Tagged
# under ``sensor`` so it runs alongside the other hardware tests; bumped
# to a 180 s timeout because phase 0 flash + two short scans + the
# write-and-refresh round-trip can take a minute on real hardware.
pytestmark = [pytest.mark.sensor, pytest.mark.timeout(180)]


# Mirrors the trigger payload the bloodflow app's CQ + scan flows send
# (see ``BloodFlow.qml``).  Without this the firmware's
# ``LaserPulseSkipInterval`` may be zero and the laser never fires
# during the scan.
_TRIGGER_CONFIG = {
    "TriggerStatus": 2,                   # 2 = laser ON, 1 = OFF
    "TriggerFrequencyHz": 40,
    "TriggerPulseWidthUsec": 500,
    "LaserPulseDelayUsec": 100,
    "LaserPulseWidthUsec": 500,
    "LaserPulseSkipInterval": 600,
    "LaserPulseSkipDelayUsec": 1800,
    "EnableSyncOut": True,
    "EnableTaTrigger": True,
}


def _permissive_thresholds() -> CalibrationThresholds:
    """Thresholds chosen so any working camera passes — this test
    exercises workflow plumbing, not acceptance criteria."""
    return CalibrationThresholds(
        min_mean_per_camera=[0.0] * 8,
        min_contrast_per_camera=[0.0] * 8,
        min_bfi_per_camera=[-1e9] * 8,
        min_bvi_per_camera=[-1e9] * 8,
    )


def test_calibration_procedure_end_to_end(motion, tmp_path):
    # The session ``motion`` fixture only waits 3 s for connection;
    # if the hardware was just released by another process the SDK's
    # connection monitor may need a few more seconds to reattach.
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if motion.console.is_connected() and (
            motion.left.is_connected() or motion.right.is_connected()
        ):
            break
        time.sleep(0.25)

    if not motion.console.is_connected():
        pytest.skip("Console not connected")
    left_connected = motion.left.is_connected()
    right_connected = motion.right.is_connected()
    if not (left_connected or right_connected):
        pytest.skip("No sensors connected")

    request = CalibrationRequest(
        operator_id="pytest",
        output_dir=str(tmp_path),
        left_camera_mask=0xFF if left_connected else 0x00,
        right_camera_mask=0xFF if right_connected else 0x00,
        thresholds=_permissive_thresholds(),
        duration_sec=3,
        scan_delay_sec=1,
        max_duration_sec=150,
        trigger_config=_TRIGGER_CONFIG,
    )

    done = threading.Event()
    result_box: list[CalibrationResult] = []

    def _on_complete(r: CalibrationResult) -> None:
        result_box.append(r)
        done.set()

    started = motion.start_calibration(request, on_complete_fn=_on_complete)
    assert started is True, "start_calibration returned False"
    assert done.wait(timeout=160.0), "calibration didn't complete in 160 s"

    r = result_box[0]

    # Procedure ran clean
    assert r.canceled is False, f"unexpectedly canceled: {r.error}"
    assert r.ok is True, f"calibration failed: {r.error}"

    # Calibration was written to the console and refreshed into the cache
    assert r.calibration is not None, "result.calibration was None"
    assert r.calibration.source == "console"
    cached = motion.get_calibration()
    assert cached.source == "console", (
        f"SDK cache was not refreshed; source={cached.source!r}"
    )

    # CSV produced and accessible
    assert r.csv_path, "result.csv_path was empty"
    assert os.path.exists(r.csv_path), f"CSV missing at {r.csv_path}"

    # Per-camera rows for every active camera, no empties
    expected_rows = (
        bin(request.left_camera_mask).count("1")
        + bin(request.right_camera_mask).count("1")
    )
    assert len(r.rows) == expected_rows, (
        f"got {len(r.rows)} rows, expected {expected_rows} active cameras"
    )

    # Permissive thresholds → overall PASS
    assert r.passed is True, (
        f"permissive thresholds but result was FAIL — error={r.error!r}, "
        f"rows={r.rows!r}"
    )
