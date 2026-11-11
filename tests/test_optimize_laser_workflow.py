"""Tests for omotion/OptimizeLaserWorkflow.py against a simulated rig.

Pure-software. The rig (tests/optimize_laser_fakes.py) models the cameras'
brightness from the registers the actuator wrote, and latches the safety
interlock (E-202) on a scan the safety limits don't allow, so these runs
check the write order as well as the result.
"""

import threading

import numpy as np
import pytest

from omotion.config import CAMERA_GAIN_MAP
from omotion.exposure.actuator import FW_DEFAULT_GAIN_CODES, LaserCameraActuator
from omotion.exposure.controller import OptimizeStatus
from omotion.exposure.settings import OptimizeLaserSettings
from omotion.OptimizeLaserWorkflow import (
    ACCEPT,
    CONTINUE,
    CSV_COLUMNS,
    DARK_SKIP_US,
    OptimizeLaserWorkflow,
    _OptimizeLaserSink,
)
from omotion.pipeline.batch import FrameBatch
from omotion.ScanWorkflow import ScanRequest

from optimize_laser_fakes import (
    CALIBRATED_REGISTERS,
    FakeConsole,
    FakeInterface,
    Rig,
    fake_apply_laser_power,
)

pytestmark = pytest.mark.unit

TARGET, BAND = 200.0, 20.0
LEFT = [(0, p) for p in range(8)]
# Middle cameras get the most light. At full laser output the outer pair
# reaches the target at ~12x gain, the middle pair at ~3x, so each camera
# gets there with its own gain.
SPREAD = {(0, p): c for p, c in enumerate([0.008, 0.012, 0.02, 0.03, 0.03, 0.02, 0.012, 0.008])}


def _actuator_factory(console, sensors, lock=None):
    return LaserCameraActuator(console, sensors, lock=lock, restore_laser=fake_apply_laser_power)


def _workflow(rig, **scan_kwargs):
    iface = FakeInterface(rig, **scan_kwargs)
    return OptimizeLaserWorkflow(iface, actuator_factory=_actuator_factory), iface


def _run(rig, settings=None, **kwargs):
    wf, iface = _workflow(rig, **{k: kwargs.pop(k) for k in ("refuse_at", "fail_at")
                                  if k in kwargs})
    result = wf.run(settings or OptimizeLaserSettings(), left_camera_mask=0xFF,
                    right_camera_mask=0, **kwargs)
    return result, wf, iface


def _first(console, name):
    return next(i for i, (n, _) in enumerate(console.writes) if n == name)


# ---- a run that passes ------------------------------------------------------


def test_run_reaches_target_and_leaves_the_settings_on_the_device(tmp_path):
    rig = Rig(SPREAD)
    result, _, iface = _run(rig, log_path=str(tmp_path / "opt.csv"))

    assert result.passed and result.status is OptimizeStatus.ON_TARGET, result
    assert not result.restored and rig.console.restores == 0
    s = result.settings
    for cam in LEFT:
        assert rig.left.gain_code(cam[1]) == s.gain_codes[cam]
        assert rig.left.exposure_rows(cam[1]) == 244
        assert abs(rig.signal(cam) - TARGET) <= BAND, (cam, rig.signal(cam))
    assert rig.console.get("TA_PULSE_WIDTH") == pytest.approx(s.pulse_width_us, abs=0.32)
    assert rig.console.get("TA_CURRENT_DRV") == pytest.approx(s.current_ma, abs=0.16)
    assert rig.console.get("EE_STATUS") == 0 and rig.console.get("OPT_STATUS") == 0
    for request in iface.scan_workflow.requests:
        assert request.trigger_config["LaserPulseSkipDelayUsec"] == DARK_SKIP_US
        assert request.skip_default_storage is True


def test_csv_log_has_the_settings_and_a_row_per_iteration_and_camera(tmp_path):
    path = tmp_path / "logs" / "opt.csv"
    result, _, _ = _run(Rig(SPREAD), log_path=str(path))

    text = path.read_text(encoding="utf-8").splitlines()
    comments = [line for line in text if line.startswith("#")]
    rows = [line for line in text if not line.startswith("#")]
    assert any("console_firmware: 1.8.1" in c for c in comments)
    assert any("dark_skip_us: 2400" in c for c in comments)
    assert any(c.startswith("# result: on_target") for c in comments)
    assert rows[0] == ",".join(CSV_COLUMNS)
    assert len(rows) - 1 == result.iterations * len(LEFT)
    assert result.csv_path == str(path)


def test_limits_are_loosened_before_the_laser_needs_them():
    """A laser-calibrated unit at 4400 mA, started at 2096 us / 5000 mA: the
    rig latches E-202 unless PULSE_WIDTH_UL, RATE_LL and DRIVE_CL moved
    first."""
    rig = Rig(SPREAD, console=FakeConsole(values=CALIBRATED_REGISTERS))
    result, _, _ = _run(rig)

    assert result.passed, result.error
    c = rig.console
    assert _first(c, "EE_PULSE_WIDTH_UL") < _first(c, "TA_PULSE_WIDTH")
    assert _first(c, "OPT_PULSE_WIDTH_UL") < _first(c, "TA_PULSE_WIDTH")
    assert _first(c, "EE_DRIVE_CL") < _first(c, "TA_CURRENT_DRV")
    assert _first(c, "OPT_DRIVE_CL") < _first(c, "TA_CURRENT_DRV")
    first_scan_writes = rig.scans[0][0]
    assert _first(c, "EE_RATE_LL") < first_scan_writes
    assert _first(c, "OPT_RATE_LL") < first_scan_writes


def test_without_loosening_the_first_scan_latches_and_the_run_restores(monkeypatch):
    """Negative control for the test above: the rig's interlock is real."""
    monkeypatch.setattr(LaserCameraActuator, "loosen_limits", lambda self, *a, **k: [])
    rig = Rig(SPREAD, console=FakeConsole(values=CALIBRATED_REGISTERS))
    result, _, _ = _run(rig)

    assert not result.passed
    assert "E-202" in result.error
    assert result.restored and rig.console.restores == 1


def test_too_dim_at_max_passes_and_keeps_the_maximum_settings():
    rig = Rig({cam: 1e-5 for cam in LEFT})
    result, _, _ = _run(rig)

    assert result.status is OptimizeStatus.TOO_DIM_AT_MAX and result.passed
    assert not result.restored
    assert result.settings.pulse_width_us == 2096.0 and result.settings.current_ma == 5000.0
    assert set(result.settings.gain_codes.values()) == {256}


def test_start_point_current_starts_from_the_device():
    rig = Rig(SPREAD)
    settings = OptimizeLaserSettings(start_point="current")
    initial = {cam: 2.0 for cam in LEFT}
    result, _, _ = _run(rig, settings, initial_gains=initial)

    first = result.records[0].settings
    assert first.pulse_width_us == 530.0                 # TA_PULSE_WIDTH on the bench unit
    assert first.current_ma == pytest.approx(5000.0, abs=0.16)
    assert set(first.gains.values()) == {2.0}
    assert result.passed


# ---- runs that don't pass ---------------------------------------------------


def test_envelope_firmware_is_refused_before_any_write():
    rig = Rig(SPREAD, console=FakeConsole(version="1.8.2-rc.4"))
    result, _, iface = _run(rig)

    assert not result.passed and "safety envelope" in result.error
    assert rig.console.writes == [] and rig.left.writes == []
    assert iface.scan_workflow.requests == []
    assert not result.restored and rig.console.restores == 0


def test_not_converged_restores_the_defaults():
    rig = Rig(SPREAD)
    result, _, _ = _run(rig, max_iterations=1)

    assert result.status is OptimizeStatus.NOT_CONVERGED and not result.passed
    assert result.settings is None and result.restored
    assert rig.console.restores == 1
    assert [rig.left.gain_code(p) for p in range(8)] == list(FW_DEFAULT_GAIN_CODES)


def test_cancel_between_iterations_restores_the_defaults():
    rig = Rig(SPREAD)
    stop = threading.Event()
    result, _, _ = _run(rig, stop_evt=stop, on_iteration=lambda record: stop.set())

    assert result.canceled and not result.passed and result.iterations == 1
    assert result.restored and rig.console.restores == 1


@pytest.mark.parametrize("scan_kwargs, match", [
    ({"fail_at": 2}, "sub-scan worker died"),
    ({"refuse_at": 1}, "refused start_scan.*previous scan is still active"),
])
def test_a_failed_scan_ends_the_run_and_restores(scan_kwargs, match):
    rig = Rig(SPREAD)
    result, _, _ = _run(rig, **scan_kwargs)

    assert not result.passed and result.error
    assert __import__("re").search(match, result.error), result.error
    assert result.restored and rig.console.restores == 1


def test_a_scan_with_no_usable_frames_is_an_error():
    rig = Rig(SPREAD)
    rig.batches = lambda *a, **k: iter(())
    result, _, _ = _run(rig)
    assert "no usable frames" in result.error and result.restored


def test_a_restore_failure_is_reported():
    rig = Rig(SPREAD, console=FakeConsole(restore_ok=False))
    result, _, _ = _run(rig, max_iterations=1)
    assert result.restored and result.restore_problems


def test_not_converged_asks_and_accept_keeps_the_last_settings(tmp_path):
    rig = Rig(SPREAD)
    asked = []
    result, wf, _ = _run(rig, max_iterations=1, log_path=str(tmp_path / "opt.csv"),
                         on_not_converged=lambda r: asked.append(r) or ACCEPT)
    assert len(asked) == 1 and asked[0].status is OptimizeStatus.NOT_CONVERGED
    assert result.passed and result.accepted
    assert result.status is OptimizeStatus.NOT_CONVERGED
    # Kept as the device holds them: what the last scan ran with, limits
    # still widened, nothing restored.
    assert result.settings == asked[0].settings
    assert not result.restored and rig.console.restores == 0
    assert rig.console.get("EE_RATE_LL") == pytest.approx(22400.0)
    wf.scan_overrides(result)
    assert "# result: not_converged (accepted)" in (tmp_path / "opt.csv").read_text()


def test_not_converged_continue_runs_more_iterations():
    rig = Rig(SPREAD)
    asked = []

    def ask(record):
        asked.append(record)
        return CONTINUE if len(asked) < 5 else "stop"

    result, _, _ = _run(rig, max_iterations=1, on_not_converged=ask)
    assert result.iterations > 1 and result.iterations == len(asked) + (1 if result.passed else 0)
    assert result.passed and result.status is OptimizeStatus.ON_TARGET and not result.accepted


@pytest.mark.parametrize("answer", ["stop", None, RuntimeError("dialog crashed")])
def test_not_converged_stop_restores(answer):
    rig = Rig(SPREAD)

    def ask(record):
        if isinstance(answer, Exception):
            raise answer
        return answer

    result, _, _ = _run(rig, max_iterations=1, on_not_converged=ask)
    assert not result.passed and not result.accepted
    assert result.restored and rig.console.restores == 1


def test_cancel_while_the_question_is_open_restores():
    rig = Rig(SPREAD)
    stop = threading.Event()

    def ask(record):
        stop.set()
        return ACCEPT

    result, _, _ = _run(rig, max_iterations=1, stop_evt=stop, on_not_converged=ask)
    assert result.canceled and not result.passed and result.restored


def test_a_failed_run_puts_the_stock_trigger_config_back():
    """The run's scans leave the 2400 us skip held on the console; after the
    RATE_LL restore it must not be what a direct trigger start would run."""
    rig = Rig(SPREAD)
    result, _, iface = _run(rig, max_iterations=1)
    assert result.restored and result.restore_problems == ()
    assert rig.console.trigger_configs == [iface.resolve_trigger_config()]
    assert rig.console.trigger_configs[-1]["LaserPulseSkipDelayUsec"] == 1800


def test_a_passed_run_leaves_the_trigger_config_alone():
    rig = Rig(SPREAD)
    result, _, _ = _run(rig)
    assert result.passed and rig.console.trigger_configs == []


def test_a_rejected_trigger_reset_is_a_restore_problem():
    rig = Rig(SPREAD)
    rig.console.reject_trigger = True
    result, _, _ = _run(rig, max_iterations=1)
    assert any(p.startswith("trigger config reset failed") for p in result.restore_problems)


def test_invalid_settings_raise_before_any_io():
    rig = Rig(SPREAD)
    wf, _ = _workflow(rig)
    with pytest.raises(ValueError):
        wf.run(OptimizeLaserSettings(passing_range=0.0), left_camera_mask=0xFF,
               right_camera_mask=0)
    with pytest.raises(ValueError, match="no cameras"):
        wf.run(OptimizeLaserSettings(), left_camera_mask=0, right_camera_mask=0)
    assert rig.console.writes == []


def test_a_second_run_is_refused_while_one_is_running():
    wf, _ = _workflow(Rig(SPREAD))
    wf._run_lock.acquire()
    try:
        assert wf.running
        with pytest.raises(RuntimeError, match="already running"):
            wf.run(OptimizeLaserSettings(), left_camera_mask=0xFF, right_camera_mask=0)
    finally:
        wf._run_lock.release()


def test_a_disconnected_sensor_is_refused():
    rig = Rig(SPREAD)
    rig.left.connected = False
    result, _, _ = _run(rig)
    assert "left sensor not connected" in result.error and rig.console.writes == []


# ---- after a run ------------------------------------------------------------


def test_scan_overrides_carry_the_trigger_gains_and_bvi_scale():
    result, wf, _ = _run(Rig(SPREAD))
    overrides = wf.scan_overrides(result)

    assert overrides["trigger_config"] == {
        "LaserPulseSkipDelayUsec": 2400,
        "LaserPulseWidthUsec": int(round(result.settings.pulse_width_us))}
    gains = overrides["camera_gains"]
    assert gains.shape == (2, 8)
    for (side, pos), gain in result.settings.gains.items():
        assert gains[side, pos] == gain
    np.testing.assert_array_equal(gains[1], CAMERA_GAIN_MAP)        # right side not scanned
    assert overrides["calibration_override"] == {"i_max": 2 * TARGET}
    ScanRequest(subject_id="s", duration_sec=1, left_camera_mask=0xFF,
                right_camera_mask=0, **overrides)


def test_reapply_puts_the_settings_back_after_a_reconnect():
    rig = Rig(SPREAD)
    result, wf, _ = _run(rig)
    fake_apply_laser_power(rig.console)         # what a console reconnect does
    for pos in range(8):
        rig.left.regs[pos][0x3508], rig.left.regs[pos][0x3509] = 16, 0

    wf.reapply(result, 0xFF, 0)

    s = result.settings
    assert rig.console.get("TA_PULSE_WIDTH") == pytest.approx(s.pulse_width_us, abs=0.32)
    assert rig.console.get("EE_RATE_LL") == pytest.approx(22400.0)
    assert rig.console.get("EE_PULSE_WIDTH_UL") == pytest.approx(2200.0)
    assert all(rig.left.gain_code(p) == s.gain_codes[(0, p)] for p in range(8))


def test_reapply_leaves_out_the_cameras_the_scan_powered_off():
    """A scan on fewer cameras than the run powers the others off, and a
    write to one fails (bench, 2026-11-02: run 0x66, scan 0x0F)."""
    rig = Rig(SPREAD)
    result, wf, _ = _run(rig)
    fake_apply_laser_power(rig.console)
    rig.left.fail = {(pos, reg) for pos in range(4, 8)
                     for reg in (0x3501, 0x3502, 0x3508, 0x3509)}
    rig.left.writes.clear()

    wf.reapply(result, 0x0F, 0)

    s = result.settings
    assert rig.console.get("TA_PULSE_WIDTH") == pytest.approx(s.pulse_width_us, abs=0.32)
    assert all(rig.left.gain_code(p) == s.gain_codes[(0, p)] for p in range(4))
    assert {pos for pos, _, _ in rig.left.writes} == {0, 1, 2, 3}


def test_reapply_does_not_need_a_sensor_the_scan_left_out():
    rig = Rig(SPREAD)
    result, wf, _ = _run(rig)
    fake_apply_laser_power(rig.console)
    rig.left.connected = False
    rig.left.writes.clear()

    wf.reapply(result, 0, 0xFF)     # the run never saw the right side

    s = result.settings
    assert rig.console.get("TA_PULSE_WIDTH") == pytest.approx(s.pulse_width_us, abs=0.32)
    assert rig.left.writes == [] and rig.right.writes == []


def test_reapply_refuses_a_disconnected_sensor_the_scan_uses():
    rig = Rig(SPREAD)
    result, wf, _ = _run(rig)
    rig.left.connected = False
    with pytest.raises(RuntimeError, match="left sensor not connected"):
        wf.reapply(result, 0x0F, 0)


def test_settings_from_a_failed_run_are_never_applied():
    result, wf, _ = _run(Rig(SPREAD), max_iterations=1)
    with pytest.raises(ValueError):
        wf.scan_overrides(result)
    with pytest.raises(ValueError):
        wf.reapply(result, 0xFF, 0)


def test_restore_defaults_restores_laser_and_cameras():
    rig = Rig(SPREAD)
    result, wf, iface = _run(rig)
    assert wf.restore_defaults(0xFF, 0) == []
    assert rig.console.restores == 1
    assert [rig.left.gain_code(p) for p in range(8)] == list(FW_DEFAULT_GAIN_CODES)
    assert rig.console.trigger_configs == [iface.resolve_trigger_config()]


def test_restore_defaults_reports_a_rejected_trigger_reset():
    rig = Rig(SPREAD)
    _, wf, _ = _run(rig)
    rig.console.reject_trigger = True
    problems = wf.restore_defaults(0xFF, 0)
    assert problems == ["trigger config reset failed: the console rejected the stock "
                        "trigger config"]


def test_motion_interface_exposes_one_workflow():
    from omotion.MotionInterface import MotionInterface

    iface = MotionInterface(demo_mode=True)
    assert iface.optimize_laser_workflow is iface.optimize_laser_workflow
    assert isinstance(iface.optimize_laser_workflow, OptimizeLaserWorkflow)


# ---- the frame sink ---------------------------------------------------------


def _batch(rows):
    """rows: (side, cam, frame_type, histogram-mean or None for empty)."""
    n = len(rows)
    hist = np.zeros((n, 2, 8, 1024), dtype=np.uint32)
    for i, (side, cam, _, mean) in enumerate(rows):
        if mean is not None:
            hist[i, side, cam, int(mean)] = 100
    return FrameBatch(
        cam_ids=np.array([r[1] for r in rows], dtype=np.int8),
        frame_ids=np.zeros(n, dtype=np.uint8), raw_histograms=hist,
        temperature_c=np.zeros((n, 2, 8), dtype=np.float32), timestamp_s=np.zeros(n),
        pdc=None, tcm=None, tcl=None,
        side_ids=np.array([r[0] for r in rows], dtype=np.int8),
        frame_type=np.array([r[2] for r in rows]))


def test_sink_keeps_the_first_light_frames_after_a_dark():
    sink = _OptimizeLaserSink([(0, 0), (1, 3)], frames_needed=2)
    sink.on_scan_start(None)
    sink.consume("raw", _batch([
        (0, 0, "stale", 900), (0, 0, "warmup", 900),
        (0, 0, "light", 500),                       # before any dark: skipped
        (0, 0, "dark", 64), (0, 0, "dark", 66),
        (0, 0, "light", 300), (0, 0, "light", None),  # empty histogram: skipped
        (0, 0, "light", 310), (0, 0, "light", 64),    # past frames_needed: skipped
        (1, 3, "dark", 70), (1, 3, "light", 200),
        (0, 5, "dark", 64), (0, 5, "light", 250),     # not an optimized camera
    ]))
    sink.consume("live", _batch([(0, 0, "light", 999)]))   # wrong channel

    obs = sink.observations()
    assert set(obs) == {(0, 0), (1, 3)}
    assert obs[(0, 0)].dark_dn == pytest.approx(65.0)
    assert [int(np.argmax(h)) for h in obs[(0, 0)].histograms] == [300, 310]
    assert len(obs[(1, 3)].histograms) == 1
    assert sink.light_frame_counts() == {(0, 0): 2, (1, 3): 1}


def test_sink_without_a_dark_has_no_observation():
    sink = _OptimizeLaserSink([(0, 0)], frames_needed=1)
    sink.consume("raw", _batch([(0, 0, "light", 300)]))
    assert sink.observations() == {}


# ---- active settings and the scan preparer ----------------------------------


def test_a_passed_run_becomes_active_and_a_failed_one_does_not():
    changes = []
    wf, _ = _workflow(Rig(SPREAD))
    wf.on_active_changed = changes.append
    result = wf.run(OptimizeLaserSettings(), left_camera_mask=0xFF, right_camera_mask=0)
    assert result.passed and wf.active is result
    assert changes == [None]

    result = wf.run(OptimizeLaserSettings(), left_camera_mask=0xFF, right_camera_mask=0,
                    max_iterations=1)
    assert not result.passed and wf.active is None      # the old settings went with the run
    assert changes == [None, None]


def test_the_workflow_registers_its_preparer_with_the_scan_workflow():
    class _Scans:
        def set_scan_preparer(self, preparer):
            self.preparer = preparer

    rig = Rig(SPREAD)
    iface = FakeInterface(rig)
    iface.scan_workflow = _Scans()
    wf = OptimizeLaserWorkflow(iface, actuator_factory=_actuator_factory)
    assert iface.scan_workflow.preparer == wf.prepare_scan


def test_prepare_scan_is_a_no_op_without_active_settings():
    rig = Rig(SPREAD)
    wf, _ = _workflow(rig)
    writes = len(rig.console.writes)
    assert wf.prepare_scan(0xFF, 0) is None
    assert len(rig.console.writes) == writes


def test_prepare_scan_reapplies_and_returns_the_request_overrides():
    import json

    rig = Rig(SPREAD)
    result, wf, _ = _run(rig)
    fake_apply_laser_power(rig.console)         # what a console reconnect does

    prepared = wf.prepare_scan(0xFF, 0)

    s = result.settings
    assert rig.console.get("TA_PULSE_WIDTH") == pytest.approx(s.pulse_width_us, abs=0.32)
    assert rig.console.get("EE_RATE_LL") == pytest.approx(22400.0)
    assert prepared["trigger_config"]["LaserPulseSkipDelayUsec"] == DARK_SKIP_US
    assert prepared["calibration_override"] == {"i_max": 2 * TARGET}
    record = prepared["extra_session_meta"]["optimize_laser"]
    assert record["status"] == "on_target" and record["iterations"] == result.iterations
    assert record["pulse_width_us"] == s.pulse_width_us
    assert record["gains"] == {f"L{p + 1}": s.gains[(0, p)] for p in range(8)}
    assert record["dark_skip_us"] == DARK_SKIP_US
    json.dumps(record)                          # plain JSON types only
    ScanRequest(subject_id="s", duration_sec=1, left_camera_mask=0xFF,
                right_camera_mask=0, **prepared)


def test_prepare_scan_refuses_the_scan_when_the_reapply_fails():
    rig = Rig(SPREAD)
    result, wf, _ = _run(rig)
    rig.left.connected = False
    with pytest.raises(RuntimeError, match="could not be re-applied"):
        wf.prepare_scan(0xFF, 0)
    assert wf.active is result                  # still active: the operator decides


def test_a_gate_reason_restores_the_defaults_and_the_scan_runs_stock():
    rig = Rig(SPREAD)
    changes = []
    _, wf, _ = _run(rig)
    wf.on_active_changed = changes.append
    wf.gate = lambda: "Optimize Laser needs engineering mode."

    assert wf.prepare_scan(0xFF, 0) is None
    assert wf.active is None
    assert rig.console.restores == 1
    assert [rig.left.gain_code(p) for p in range(8)] == list(FW_DEFAULT_GAIN_CODES)
    assert changes == ["Optimize Laser needs engineering mode."]


def test_a_gate_drop_whose_laser_restore_fails_refuses_the_scan():
    rig = Rig(SPREAD)
    _, wf, _ = _run(rig)
    rig.console.reject_trigger = True
    wf.gate = lambda: "Turn off Force laser fail first."
    with pytest.raises(RuntimeError, match="Could not restore the laser defaults"):
        wf.prepare_scan(0xFF, 0)
    assert wf.active is None


def test_a_gate_that_raises_drops_the_settings():
    rig = Rig(SPREAD)
    _, wf, _ = _run(rig)

    def gate():
        raise ValueError("boom")
    wf.gate = gate
    assert wf.prepare_scan(0xFF, 0) is None
    assert wf.active is None and rig.console.restores == 1


def test_restore_defaults_without_masks_uses_the_active_cameras_and_drops_them():
    rig = Rig(SPREAD)
    _, wf, _ = _run(rig)
    assert wf.restore_defaults() == []
    assert wf.active is None
    assert [rig.left.gain_code(p) for p in range(8)] == list(FW_DEFAULT_GAIN_CODES)


def test_force_fault_reaches_the_restore(monkeypatch):
    import omotion.laser as laser
    calls = []

    def fake(console, force_fault=False, lock=None):
        calls.append(force_fault)
        return True
    monkeypatch.setattr(laser, "apply_laser_power", fake)
    rig = Rig(SPREAD)
    wf = OptimizeLaserWorkflow(FakeInterface(rig), force_fault=lambda: True)
    wf.restore_defaults(0xFF, 0)
    assert calls == [True]
