"""Integration tests for CalibrationWorkflow with mocked ScanWorkflow.

Patches MotionInterface.scan_workflow.start_scan so sub-scans don't hit
hardware, and MotionInterface.write_calibration to record arguments.
Sub-scans run via run_collection_scan, which calls start_scan(request) and
polls scan_workflow.running — there is no on_complete_fn / ScanResult callback.
"""
import os
import threading
import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from omotion import MotionInterface
from omotion.Calibration import Calibration
from omotion.CalibrationWorkflow import (
    CalibrationRequest,
    CalibrationResult,
    CalibrationThresholds,
)


_FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "fixtures")
_LEFT  = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_left_maskFF.csv")
_RIGHT = os.path.join(_FIXTURE_DIR, "scan_owC18EHALL_20251217_160949_right_maskFF.csv")


def _have_fixtures() -> bool:
    return os.path.exists(_LEFT) and os.path.exists(_RIGHT)


@pytest.fixture
def thresholds():
    # The fixture CSV is truncated mid-scan, so the corrected stream's
    # dark-baseline endpoint is not a real laser-off frame and BFI/BVI
    # values come out unrealistic. Use very permissive thresholds — this
    # test exercises workflow plumbing, not the science values.
    return CalibrationThresholds(
        min_mean_per_camera=[0.0]*8,
        min_contrast_per_camera=[0.0]*8,
        min_bfi_per_camera=[-1e9]*8,
        min_bvi_per_camera=[-1e9]*8,
    )


@pytest.fixture
def request_obj(tmp_path, thresholds):
    return CalibrationRequest(
        operator_id="opX",
        output_dir=str(tmp_path),
        left_camera_mask=0xFF,
        right_camera_mask=0xFF,
        thresholds=thresholds,
        duration_sec=2,
        scan_delay_sec=0,
        max_duration_sec=60,
        # The permissive thresholds above cannot fail the pre-write gate,
        # which start_calibration refuses by default (#256). These tests
        # exercise plumbing, so opt in to the ungated run explicitly.
        allow_ungated=True,
    )


@pytest.fixture
def interface():
    iface = MotionInterface(demo_mode=True)

    # CalibrationWorkflow now flashes sensors at phase 0 via
    # start_configure_camera_sensors. In demo mode there are no real
    # sensors to configure, so we stub it to immediately succeed.
    from omotion.ScanWorkflow import ConfigureResult

    def _fake_configure(req, *, on_complete_fn=None, on_log_fn=None, **kw):
        def _run():
            time.sleep(0.02)
            if on_complete_fn:
                on_complete_fn(ConfigureResult(ok=True, error=""))
        threading.Thread(target=_run, daemon=True).start()
        return True

    iface.start_configure_camera_sensors = _fake_configure
    return iface


def _same_calibration(a: Calibration, b: Calibration) -> bool:
    return (
        a.source == b.source
        and np.array_equal(a.c_min, b.c_min) and np.array_equal(a.c_max, b.c_max)
        and np.array_equal(a.i_min, b.i_min) and np.array_equal(a.i_max, b.i_max)
    )


def _console_block(*, left_c_max=0.31, left_i_max=222.0,
                   right_c_max=0.37, right_i_max=333.0) -> dict:
    """A stored calibration JSON block whose rows are distinguishable from
    the SDK defaults and from what the fake scan computes (C_max 0.4,
    I_max 400)."""
    return {"calibration": {
        "C_min": [[0.0] * 8, [0.0] * 8],
        "C_max": [[left_c_max] * 8, [right_c_max] * 8],
        "I_min": [[0.0] * 8, [0.0] * 8],
        "I_max": [[left_i_max] * 8, [right_i_max] * 8],
    }}


def _console_holds(interface, json_data) -> None:
    """Make the (demo-mode) console answer read_config with this JSON."""
    from omotion.MotionConfig import MotionConfig
    interface.console.read_config = MagicMock(
        side_effect=lambda: MotionConfig(json_data=dict(json_data)))


def _run_to_completion(interface, request, timeout=60.0) -> CalibrationResult:
    done = threading.Event()
    holder: dict = {}
    assert interface.start_calibration(
        request, on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(timeout=timeout), "calibration didn't complete"
    return holder["r"]


def _make_fake_scan_workflow(interface, left, right):
    """Patch interface.scan_workflow so start_scan synthesises corrected
    samples through the sink contract used by the new pipeline (Phase E).

    Drives the _CalibrationCollectorSink directly: on_scan_start ->
    consume("final", EnrichedCorrectedInterval) -> on_complete.
    Sets scan_workflow.running True briefly then False so the polling
    loop in _run_subscan_capture exits with last_scan_error=None.
    """
    from omotion.pipeline.stages.dark import (
        EnrichedCorrectedFrame, EnrichedCorrectedInterval,
    )

    def _make_interval():
        frames = []
        for side in ("left", "right"):
            for cam_id in range(8):
                for fid in range(50, 100):
                    frames.append(EnrichedCorrectedFrame(
                        abs_frame_id=fid, t=fid / 40.0, side=side, cam_id=cam_id,
                        mean=200.0, std=80.0, contrast=0.4, bfi=5.0, bvi=5.0,
                    ))
        return EnrichedCorrectedInterval(left_abs=10, right_abs=240, frames=frames)

    sw = interface.scan_workflow
    done_evt = threading.Event()

    def _fake_start_scan(req):
        sw._running = True
        sw._last_scan_error = None
        sw._last_scan_canceled = False

        def _run():
            time.sleep(0.05)
            for sink in req.sinks:
                try:
                    sink.on_scan_start(None)
                except Exception:
                    pass
                try:
                    sink.consume("final", _make_interval())
                except Exception:
                    pass
                try:
                    sink.on_complete()
                except Exception:
                    pass
            sw._running = False
            done_evt.set()

        threading.Thread(target=_run, daemon=True).start()
        return True

    def _fake_await(*, timeout_sec=None):
        done_evt.wait(timeout=timeout_sec)

    sw.start_scan = _fake_start_scan
    sw.await_complete = _fake_await
    # last_scan_error / last_scan_canceled / running come straight from
    # the real ScanWorkflow attrs we just twiddled above.


def test_happy_path_produces_csv_and_passes(interface, request_obj):
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock(
        return_value=Calibration(
            c_min=np.zeros((2, 8)), c_max=np.full((2, 8), 0.5),
            i_min=np.zeros((2, 8)), i_max=np.full((2, 8), 200.0),
            source="console",
        )
    )

    done = threading.Event()
    result_box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (result_box.append(r), done.set()),
    )
    assert done.wait(timeout=60.0), "calibration didn't complete"
    r = result_box[0]
    assert r.ok is True
    assert r.passed is True
    assert r.canceled is False
    assert os.path.exists(r.csv_path)
    assert r.calibration is not None
    interface.write_calibration.assert_called_once()


def test_cancel_during_phase_1(interface, request_obj):
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    sw = interface.scan_workflow
    started = threading.Event()
    cancel_called = threading.Event()

    def _slow_scan(req):
        sw._running = True
        sw._last_scan_error = None
        sw._last_scan_canceled = False

        def _run():
            started.set()
            cancel_called.wait(timeout=5.0)
            sw._last_scan_canceled = True
            sw._running = False

        threading.Thread(target=_run, daemon=True).start()
        return True

    sw.start_scan = _slow_scan
    sw.await_complete = lambda *, timeout_sec=None: (
        time.sleep(min(0.1, timeout_sec)) if timeout_sec else None
    )
    sw.cancel_scan = MagicMock(side_effect=lambda **kw: cancel_called.set())
    interface.write_calibration = MagicMock()

    done = threading.Event()
    box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert started.wait(timeout=5.0)
    interface.cancel_calibration()
    assert done.wait(timeout=15.0)
    r = box[0]
    assert r.ok is False
    assert r.canceled is True
    assert r.csv_path == ""
    interface.write_calibration.assert_not_called()


def test_phase1_scan_failure_aborts_before_write(interface, request_obj):
    """When the underlying scan worker raises, _run_subscan_capture should
    surface the error message via scan_workflow.last_scan_error so the
    workflow aborts before write_calibration is touched."""
    sw = interface.scan_workflow

    def _fail_scan(req):
        sw._running = True
        sw._last_scan_error = None
        sw._last_scan_canceled = False

        def _run():
            time.sleep(0.02)
            sw._last_scan_error = "USB lost"
            sw._running = False

        threading.Thread(target=_run, daemon=True).start()
        return True

    done_join = threading.Event()
    sw.start_scan = _fail_scan
    sw.await_complete = lambda *, timeout_sec=None: done_join.wait(timeout=timeout_sec)

    interface.write_calibration = MagicMock()

    done = threading.Event()
    box: list[CalibrationResult] = []
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    # Let the fake scan thread finish before the polling loop awaits.
    time.sleep(0.05)
    done_join.set()
    assert done.wait(timeout=10.0)
    r = box[0]
    assert r.ok is False
    assert "USB lost" in r.error
    interface.write_calibration.assert_not_called()


# ---------------------------------------------------------------------------
# Task 13: _run_subscan_capture passes collector sink + skip_default_storage
# ---------------------------------------------------------------------------

def test_subscan_uses_collector_sink_and_skip_default_storage(interface, request_obj):
    """_run_subscan_capture (called by start_calibration) must attach a
    _CalibrationCollectorSink to each ScanRequest and set
    skip_default_storage=True.  Verifies the new sink-based API shape
    added in Phase D of the pipeline cutover (Task 13).
    """
    from omotion.CalibrationWorkflow import _CalibrationCollectorSink

    captured_requests: list = []

    def _capture_and_complete(req, **kw):
        # run_collection_scan calls start_scan(request) then polls
        # scan_workflow.running (False here — no worker is started), so just
        # capture the request and report a successful launch.
        captured_requests.append(req)
        return True

    interface.scan_workflow.start_scan = _capture_and_complete
    interface.write_calibration = MagicMock(
        return_value=Calibration(
            c_min=np.zeros((2, 8)), c_max=np.full((2, 8), 0.5),
            i_min=np.zeros((2, 8)), i_max=np.full((2, 8), 200.0),
            source="console",
        )
    )

    done = threading.Event()
    box: list[CalibrationResult] = []

    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert done.wait(timeout=15.0), "calibration did not complete"

    # The calibration workflow makes at least 2 sub-scans (phase 1 + phase 4).
    assert len(captured_requests) >= 1, "No ScanRequests were captured"

    for req in captured_requests:
        assert req.skip_default_storage is True, (
            f"ScanRequest.skip_default_storage should be True; got {req.skip_default_storage}"
        )
        collector_sinks = [
            s for s in req.sinks
            if isinstance(s, _CalibrationCollectorSink)
        ]
        assert len(collector_sinks) >= 1, (
            f"Expected a _CalibrationCollectorSink in req.sinks; got {req.sinks}"
        )


# ---------------------------------------------------------------------------
# Task 16: start_test_scan also uses collector sink + skip_default_storage
# ---------------------------------------------------------------------------

def test_start_test_scan_uses_collector_sink_and_skip_default_storage(
    interface, request_obj
):
    """start_test_scan's sub-scan must also carry the collector sink and
    skip_default_storage=True.  Same shape as the calibration sub-scan
    (Task 16 of the pipeline cutover)."""
    from omotion.CalibrationWorkflow import _CalibrationCollectorSink, TestScanResult

    captured_requests: list = []

    def _capture_and_complete(req, **kw):
        captured_requests.append(req)
        return True

    interface.scan_workflow.start_scan = _capture_and_complete

    done = threading.Event()
    box: list[TestScanResult] = []
    interface.start_test_scan(
        request_obj,
        on_complete_fn=lambda r: (box.append(r), done.set()),
    )
    assert done.wait(timeout=15.0), "test scan did not complete"

    assert len(captured_requests) >= 1, "No ScanRequests were captured"

    for req in captured_requests:
        assert req.skip_default_storage is True, (
            f"ScanRequest.skip_default_storage should be True; got {req.skip_default_storage}"
        )
        collector_sinks = [
            s for s in req.sinks
            if isinstance(s, _CalibrationCollectorSink)
        ]
        assert len(collector_sinks) >= 1, (
            f"Expected a _CalibrationCollectorSink in req.sinks; got {req.sinks}"
        )


# ---------------------------------------------------------------------------
# Task 2: outcome wired through both workers (Refs #199)
# ---------------------------------------------------------------------------

def test_happy_path_outcome_is_passed(interface, request_obj):
    from omotion.CalibrationWorkflow import CalibrationOutcome
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock(
        side_effect=lambda cmin, cmax, imin, imax: Calibration(
            c_min=cmin, c_max=cmax, i_min=imin, i_max=imax, source="test"))
    done = threading.Event()
    holder = {}
    interface.start_calibration(
        request_obj, on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(30)
    assert holder["r"].outcome is CalibrationOutcome.PASSED


def test_cancel_outcome_is_canceled_not_timed_out(interface, request_obj):
    """Regression for the old finally-block back-fill that stamped ANY
    stop_evt as 'exceeded max_duration_sec' when the canceled flag hadn't
    been picked up by a phase boundary yet."""
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    from omotion.CalibrationWorkflow import CalibrationOutcome

    # (mirror the existing test_cancel_during_phase_1 setup, then:)
    sw = interface.scan_workflow
    started = threading.Event()
    cancel_called = threading.Event()

    def _slow_scan(req):
        sw._running = True
        sw._last_scan_error = None
        sw._last_scan_canceled = False

        def _run():
            started.set()
            cancel_called.wait(timeout=5.0)
            sw._last_scan_canceled = True
            sw._running = False

        threading.Thread(target=_run, daemon=True).start()
        return True

    sw.start_scan = _slow_scan
    sw.await_complete = lambda *, timeout_sec=None: (
        time.sleep(min(0.1, timeout_sec)) if timeout_sec else None
    )
    sw.cancel_scan = MagicMock(side_effect=lambda **kw: cancel_called.set())
    interface.write_calibration = MagicMock()

    # ... start_calibration, cancel mid-phase-1, wait for completion ...
    done = threading.Event()
    holder = {}
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (holder.update(r=r), done.set()),
    )
    assert started.wait(timeout=5.0)
    interface.cancel_calibration()
    assert done.wait(timeout=15.0)
    assert holder["r"].outcome is CalibrationOutcome.CANCELED
    assert "max_duration_sec" not in holder["r"].error


def test_fail_verdict_never_writes(interface, request_obj, thresholds):
    """A FAILED calibration must leave the console EEPROM untouched: the
    write happens only after a fully-passing validation, so a validation
    failure means write_calibration is never called at all, and the SDK's
    in-memory cache is restored to the console's own calibration.

    The failure is forced on BFI rather than mean. Mean and contrast are
    judged one scan earlier by the pre-write gate (#199), which fails the
    run before the validation scan even starts — BFI is a calibrated
    quantity, knowable only after the validation scan, which makes it the
    last threshold that can stop the write.
    """
    from dataclasses import replace
    from omotion.CalibrationWorkflow import (
        CalibrationOutcome, CalibrationThresholds,
    )
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    prior = interface.get_calibration()
    strict = CalibrationThresholds(
        min_mean_per_camera=[0.0] * 8,      # gate passes …
        min_contrast_per_camera=[0.0] * 8,  # … so validation is reached
        min_bfi_per_camera=[1e9] * 8,       # then validation fails
        min_bvi_per_camera=[-1e9] * 8,
    )
    req = replace(request_obj, thresholds=strict)
    interface.write_calibration = MagicMock()
    done = threading.Event(); holder = {}
    interface.start_calibration(
        req, on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(30)
    r = holder["r"]
    assert r.ok and not r.passed
    assert r.outcome is CalibrationOutcome.FAILED
    assert r.calibration_written is False
    interface.write_calibration.assert_not_called()
    # The proposed calibration lived in the in-memory cache for the
    # validation scan only; afterwards the cache must hold the console's
    # own calibration again.
    assert _same_calibration(interface.get_calibration(), prior)


def test_pass_verdict_writes_once_after_validation(interface, request_obj):
    # happy-path arrangement from test_happy_path_produces_csv_and_passes
    if not _have_fixtures():
        pytest.skip("fixture CSVs missing")

    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    # Record the order of the steps that matter: the in-memory apply must
    # precede the validation scan, and the single EEPROM write must be
    # the very last step.
    order = []
    sw = interface.scan_workflow
    real_set_rt = sw.set_realtime_calibration
    sw.set_realtime_calibration = (
        lambda *a, **kw: (order.append("apply_in_memory"),
                          real_set_rt(*a, **kw))[1]
    )
    fake_start = sw.start_scan   # installed by _make_fake_scan_workflow
    sw.start_scan = lambda req: (order.append("scan"), fake_start(req))[1]
    interface.write_calibration = MagicMock(
        side_effect=lambda cmin, cmax, imin, imax: (
            order.append("eeprom_write"),
            Calibration(c_min=cmin, c_max=cmax, i_min=imin, i_max=imax,
                        source="console"),
        )[1]
    )

    done = threading.Event()
    holder = {}
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (holder.update(r=r), done.set()),
    )
    assert done.wait(timeout=60.0), "calibration didn't complete"
    assert holder["r"].passed is True
    assert holder["r"].calibration_written is True
    interface.write_calibration.assert_called_once()
    assert order == ["scan", "apply_in_memory", "scan", "eeprom_write"]


def test_write_failure_after_pass_is_error_and_restores_cache(
    interface, request_obj,
):
    """If the post-validation EEPROM write raises (e.g. console USB died),
    the run completes as ERROR, reports calibration_written=False, and the
    in-memory cache is restored to the console calibration — the override
    applied for the validation scan must not outlive the run."""
    from omotion.CalibrationWorkflow import CalibrationOutcome
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    prior = interface.get_calibration()
    interface.write_calibration = MagicMock(
        side_effect=RuntimeError("usb gone"))

    done = threading.Event(); holder = {}
    interface.start_calibration(
        request_obj,
        on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(30)
    r = holder["r"]
    assert r.outcome is CalibrationOutcome.ERROR
    assert r.calibration_written is False
    assert "usb gone" in r.error
    assert _same_calibration(interface.get_calibration(), prior)


def test_watchdog_timeout_outcome_is_timed_out(interface, request_obj):
    from dataclasses import replace
    from omotion.CalibrationWorkflow import CalibrationOutcome
    # A fake scan workflow that never completes, and a 1-second watchdog.
    req = replace(request_obj, max_duration_sec=1)

    # (fake start_scan sets running=True and never flips it back; cancel_scan
    #  flips running=False so _run_subscan_capture's poll loop exits)
    sw = interface.scan_workflow

    def _never_completing_scan(r):
        sw._running = True
        sw._last_scan_error = None
        sw._last_scan_canceled = False
        return True

    def _fake_cancel_scan(**kw):
        sw._running = False
        sw._last_scan_canceled = True

    sw.start_scan = _never_completing_scan
    sw.cancel_scan = MagicMock(side_effect=_fake_cancel_scan)
    sw.await_complete = lambda *, timeout_sec=None: (
        time.sleep(min(0.05, timeout_sec)) if timeout_sec else None
    )
    interface.write_calibration = MagicMock()

    # ... start_calibration(req, ...), wait ...
    done = threading.Event()
    holder = {}
    interface.start_calibration(
        req, on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(timeout=15.0)
    assert holder["r"].outcome is CalibrationOutcome.TIMED_OUT
    assert "max_duration_sec" in holder["r"].error


# ── Pre-write gate (#199) ─────────────────────────────────────────────────
#
# If any camera misses its mean/contrast bar on the calibration scan the
# whole run FAILS right there: no validation scan, no operator override,
# and the console EEPROM is never touched.


def _gate_failing_thresholds():
    """Mean unreachable → the gate fails; everything else permissive so
    nothing *else* is what stopped the run."""
    return CalibrationThresholds(
        min_mean_per_camera=[1e9] * 8,
        min_contrast_per_camera=[0.0] * 8,
        min_bfi_per_camera=[-1e9] * 8,
        min_bvi_per_camera=[-1e9] * 8,
    )


def test_gate_failure_fails_run_and_never_writes(interface, request_obj):
    """One camera below the gate → the whole run is FAILED, the EEPROM is
    untouched, and the result still carries the measured rows + CSV so
    the operator can see exactly which cameras missed."""
    from dataclasses import replace
    from omotion.CalibrationWorkflow import CalibrationOutcome
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock()

    req = replace(request_obj, thresholds=_gate_failing_thresholds())
    done = threading.Event(); holder = {}
    interface.start_calibration(
        req, on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(30)

    r = holder["r"]
    interface.write_calibration.assert_not_called()
    assert r.outcome is CalibrationOutcome.FAILED
    assert "below threshold" in r.error
    assert "nothing written" in r.error
    assert r.calibration_written is False
    # The gate rows double as the run's result table so the UI can show
    # what missed; the evidence CSV is written like any other failure.
    assert r.rows, "gate failure must still deliver the measured rows"
    assert all(row.mean_test == "FAIL" for row in r.rows)
    assert r.csv_path and os.path.exists(r.csv_path)


def test_gate_failure_skips_validation_scan(interface, request_obj):
    """The run fails fast: after the calibration scan misses the gate no
    second (validation) scan is started and no in-memory calibration is
    applied."""
    from dataclasses import replace
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.write_calibration = MagicMock()
    sw = interface.scan_workflow
    scan_count = []
    fake_start = sw.start_scan   # installed by _make_fake_scan_workflow
    sw.start_scan = lambda req: (scan_count.append(1), fake_start(req))[1]
    applied = []
    real_set_rt = sw.set_realtime_calibration
    sw.set_realtime_calibration = (
        lambda *a, **kw: (applied.append(1), real_set_rt(*a, **kw))[1]
    )

    req = replace(request_obj, thresholds=_gate_failing_thresholds())
    done = threading.Event(); holder = {}
    interface.start_calibration(
        req, on_complete_fn=lambda r: (holder.update(r=r), done.set()))
    assert done.wait(30)

    assert len(scan_count) == 1, "validation scan ran despite gate failure"
    assert applied == [], "proposed calibration applied despite gate failure"
    assert holder["r"].passed is False


# ---------------------------------------------------------------------------
# One-side runs carry the other side forward from a FRESH console read,
# never from the SDK cache (#281: a right-only run was writing SDK
# defaults over the left module's stored calibration whenever the cache
# had not been loaded from the console, or had been reset to defaults by
# a transient read failure).
# ---------------------------------------------------------------------------

def _right_only(request_obj):
    from dataclasses import replace
    return replace(request_obj, left_camera_mask=0x00, right_camera_mask=0xFF)


def _capture_written(interface) -> dict:
    """Stub interface.write_calibration to record the arrays it is handed
    and hand back a console-sourced Calibration, like the real one."""
    captured: dict = {}

    def _write(c_min, c_max, i_min, i_max):
        captured.update(c_min=c_min, c_max=c_max, i_min=i_min, i_max=i_max)
        return Calibration(c_min=c_min, c_max=c_max, i_min=i_min, i_max=i_max,
                           source="console")

    interface.write_calibration = MagicMock(side_effect=_write)
    return captured


def test_right_only_run_carries_left_row_from_console_not_cache(
    interface, request_obj,
):
    """The user-reported case: the left module was calibrated earlier, the
    host never loaded the console calibration into the SDK cache (or the
    cache was reset to defaults), and the operator calibrates the right
    side only. The written block must keep the console's left row."""
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    _console_holds(interface, _console_block())
    assert interface.get_calibration().source == "default"   # cache never loaded
    captured = _capture_written(interface)

    r = _run_to_completion(interface, _right_only(request_obj))

    assert r.passed and r.calibration_written
    interface.write_calibration.assert_called_once()
    # Left row: exactly what the console stored, not the SDK defaults.
    np.testing.assert_array_equal(captured["c_max"][0], np.full(8, 0.31))
    np.testing.assert_array_equal(captured["i_max"][0], np.full(8, 222.0))
    # Right row: freshly computed from the fake scan (contrast 0.4, mean 200).
    np.testing.assert_allclose(captured["c_max"][1], np.full(8, 0.4))
    np.testing.assert_allclose(captured["i_max"][1], np.full(8, 400.0))


def test_right_only_run_prefers_fresh_console_read_over_stale_cache(
    interface, request_obj,
):
    """A cache that holds an older console calibration is not the baseline
    either: the EEPROM is re-read at run time."""
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    _console_holds(interface, _console_block(left_c_max=0.29, left_i_max=210.0))
    interface.refresh_calibration()                     # cache <- old values
    _console_holds(interface, _console_block(left_c_max=0.31, left_i_max=222.0))
    captured = _capture_written(interface)

    r = _run_to_completion(interface, _right_only(request_obj))

    assert r.passed
    np.testing.assert_array_equal(captured["c_max"][0], np.full(8, 0.31))
    np.testing.assert_array_equal(captured["i_max"][0], np.full(8, 222.0))


def test_right_only_run_on_never_calibrated_console_carries_sdk_defaults(
    interface, request_obj,
):
    """A console with no calibration block is the one case where SDK
    defaults are the right thing to carry forward for the left module."""
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    _console_holds(interface, {"EE_THRESH": [1, 2, 3]})   # no calibration key
    captured = _capture_written(interface)

    r = _run_to_completion(interface, _right_only(request_obj))

    assert r.passed
    defaults = Calibration.default()
    np.testing.assert_array_equal(captured["c_max"][0], defaults.c_max[0])
    np.testing.assert_array_equal(captured["i_max"][0], defaults.i_max[0])
    np.testing.assert_allclose(captured["c_max"][1], np.full(8, 0.4))


def test_right_only_run_refuses_when_console_calibration_unreadable(
    interface, request_obj,
):
    """If the console cannot be read, the run must not guess: it ends as
    ERROR before any scan and writes nothing — the alternative was writing
    SDK defaults over the left module's stored calibration."""
    from omotion.CalibrationWorkflow import CalibrationOutcome
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.console.read_config = MagicMock(return_value=None)  # device error
    scans = []
    sw = interface.scan_workflow
    fake_start = sw.start_scan
    sw.start_scan = lambda req: (scans.append(req), fake_start(req))[1]
    interface.write_calibration = MagicMock()

    r = _run_to_completion(interface, _right_only(request_obj))

    assert r.outcome is CalibrationOutcome.ERROR
    assert r.calibration_written is False
    assert "could not be read" in r.error
    interface.write_calibration.assert_not_called()
    assert scans == []                       # failed fast, before phase 1
    assert os.path.exists(r.json_path)       # manifest still records the run


def test_both_sides_run_also_refuses_when_console_unreadable(
    interface, request_obj,
):
    """Deliberately no mask special-casing: an unreadable console fails
    every run the same way, even one that would overwrite both rows."""
    from omotion.CalibrationWorkflow import CalibrationOutcome
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    interface.console.read_config = MagicMock(return_value=None)
    interface.write_calibration = MagicMock()

    r = _run_to_completion(interface, request_obj)   # masks 0xFF / 0xFF

    assert r.outcome is CalibrationOutcome.ERROR
    interface.write_calibration.assert_not_called()


def test_run_reads_console_before_first_scan(interface, request_obj):
    """The baseline read happens before the calibration scan so an
    unreadable console fails fast, and the compute step uses that read."""
    _make_fake_scan_workflow(interface, _LEFT, _RIGHT)
    order = []
    _console_holds(interface, _console_block())
    real_read = interface.console.read_config
    interface.console.read_config = MagicMock(
        side_effect=lambda: (order.append("console_read"), real_read())[1])
    sw = interface.scan_workflow
    fake_start = sw.start_scan
    sw.start_scan = lambda req: (order.append("scan"), fake_start(req))[1]
    interface.write_calibration = MagicMock(
        side_effect=lambda *a: (order.append("eeprom_write"),
                                Calibration(*a, source="console"))[1])

    r = _run_to_completion(interface, _right_only(request_obj))

    assert r.passed
    assert order[:2] == ["console_read", "scan"]
    assert order[-1] == "eeprom_write"
