"""Hardware-in-the-loop disconnect/reconnect recovery (combined-link cuts).

Bench: YKUSH YK27987 (port 1 carries the whole system's USB) + Shelly mains
outlet. These tests cut BOTH the console UART and the sensor USB-bulk links at
once (the current cabling cannot isolate them) at several app-lifecycle stages
and assert correct handling.

What these tests guarantee (the operational contract):
  - After a USB cut+restore the system returns to all-connected. When the
    console firmware wedges on an abrupt cut (an intermittent firmware bug —
    failure mode B, ~1 in 6 single cuts, recoverable only by a mains cycle;
    tracked in openmotion-console-fw issue #26) a mains cycle restores it. So
    the contract is "USB-recovers, else mains recovers", and each test records
    which path was needed.
  - The SDK never wedges its own threads: reconnect is bounded and the monitor
    keeps servicing, so the sensors recover independently of the console. That
    SDK guarantee is locked in by the software-only test_monitor_nonblocking.py.

Markers: all require `console` + `sensor` + `slow`. The one test that arms the
trigger is additionally `laser`; run the safe subset with `-m "not laser"`.
"""
import os
import sys
import time

import pytest

from omotion.connection_state import ConnectionState
from omotion.ScanWorkflow import ScanRequest

ykush = pytest.importorskip("ykush")  # tests/ is on sys.path under pytest

RECOVER_TIMEOUT = 10.0   # USB re-enumeration + bounded connect
MAINS_TIMEOUT = 25.0     # cold boot after a mains cycle
SYS_PORT = 1             # YKUSH port carrying console + both sensors on this bench
SHELLY_HOST = os.environ.get("SHELLY_IP_ADDRESS", "192.168.1.81")


def _hub():
    try:
        return ykush.default_hub()
    except Exception as e:
        pytest.skip(f"YKUSH hub not available: {e}")


def _shelly():
    """Return a Shelly outlet client, or skip if unreachable. The driver lives
    in the bloodflow-app repo on this bench."""
    sys.path.insert(0, r"C:\Users\ethan\Projects\openmotion-bloodflow-app\tests")
    try:
        import shelly
        outlet = shelly.ShellyOutlet(SHELLY_HOST)
        outlet.is_on()  # one round-trip to confirm reachable
        return outlet
    except Exception as e:
        pytest.skip(f"Shelly outlet not available: {e}")


def _all_connected(motion):
    return (motion.console.is_connected()
            and motion.left.is_connected()
            and motion.right.is_connected())


def _state_str(motion):
    return (f"console={motion.console.state.name} "
            f"left={motion.left.state.name} "
            f"right={motion.right.state.name}")


def _wait_all(motion, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _all_connected(motion):
            return True
        time.sleep(0.1)
    return False


def _assert_recovers(motion, hub, *, usb_timeout=RECOVER_TIMEOUT):
    """Assert the system recovers to all-connected. Prefer USB re-enumeration;
    if the console firmware wedged on the abrupt cut, fall back to a mains
    cycle (which always recovers it). Returns "usb" or "mains" for logging."""
    if _wait_all(motion, usb_timeout):
        return "usb"
    # Console firmware wedge (failure mode B) — mains cycle is the escape hatch.
    outlet = _shelly()
    print(f"  USB-alone did not recover ({_state_str(motion)}); mains cycling")
    outlet.power_cycle(off_time=6.0, settle_time=1.0)
    assert _wait_all(motion, MAINS_TIMEOUT), (
        f"system did not recover even after a mains cycle ({_state_str(motion)})"
    )
    return "mains"


@pytest.fixture
def hub():
    h = _hub()
    h.all_on()
    time.sleep(2.0)
    yield h
    h.all_on()


@pytest.mark.timeout(180)
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_system_cut_restore_recovers(motion, hub):
    """L2 CONNECTED-idle: cut both links, restore, recover (USB or mains), x3."""
    assert _wait_all(motion, MAINS_TIMEOUT), (
        f"preconditions: all three must start connected ({_state_str(motion)})"
    )
    paths = []
    for i in range(3):
        hub.off(SYS_PORT)
        time.sleep(4.0)
        assert not _all_connected(motion), f"iter {i}: handles should have dropped"
        hub.on(SYS_PORT)
        paths.append(_assert_recovers(motion, hub))
    print(f"recovery paths over 3 cuts: {paths}")


@pytest.mark.timeout(120)
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_cut_during_connecting_recovers(motion, hub):
    """L1 mid-CONNECTING: restore then cut again ~0.8 s later (interrupt the
    connect), then final restore. Must still recover (USB or mains)."""
    assert _wait_all(motion, MAINS_TIMEOUT)
    hub.off(SYS_PORT)
    time.sleep(4.0)
    hub.on(SYS_PORT)
    time.sleep(0.8)          # interrupt mid-connect
    hub.off(SYS_PORT)
    time.sleep(3.0)
    hub.on(SYS_PORT)         # final restore
    _assert_recovers(motion, hub, usb_timeout=RECOVER_TIMEOUT + 4)


@pytest.mark.timeout(180)
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_repeated_rapid_cuts_recoverable(motion, hub):
    """L7 stress: several rapid cut/restore cycles, then the system must be
    recoverable (USB or mains). Rapid abrupt cuts make the firmware wedge more
    likely, so the mains fallback is the expected escape hatch here."""
    assert _wait_all(motion, MAINS_TIMEOUT)
    for _ in range(5):
        hub.off(SYS_PORT)
        time.sleep(2.5)
        hub.on(SYS_PORT)
        time.sleep(2.5)
    path = _assert_recovers(motion, hub, usb_timeout=RECOVER_TIMEOUT + 6)
    print(f"recovery after rapid cuts via: {path}")


@pytest.mark.timeout(90)
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_mains_cycle_recovers(motion):
    """A full mains power cycle (clean cold boot) always reconnects all three."""
    outlet = _shelly()
    assert _wait_all(motion, MAINS_TIMEOUT)
    outlet.power_cycle(off_time=6.0, settle_time=1.0)
    assert _wait_all(motion, MAINS_TIMEOUT), (
        f"system did not reconnect after mains cycle ({_state_str(motion)})"
    )


@pytest.mark.timeout(120)
@pytest.mark.laser
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_force_safe_trigger_off_after_reconnect(motion, hub):
    """L4-ish (FIRES THE LASER): arm the trigger, cut+restore, assert
    TriggerStatus == 1 (OFF) once the console is reachable again — proves the
    SDK force-safes the trigger on every (re)connect."""
    assert _wait_all(motion, MAINS_TIMEOUT)
    cfg = motion.resolve_trigger_config()
    motion.console.set_trigger_json(data=cfg)
    motion.console.start_trigger()
    try:
        assert motion.console.get_trigger_json()["TriggerStatus"] == 2
        hub.off(SYS_PORT)
        time.sleep(4.0)
        hub.on(SYS_PORT)
        # Recover the console (USB or mains) before checking the trigger state.
        _assert_recovers(motion, hub)
        time.sleep(0.5)
        assert motion.console.get_trigger_json()["TriggerStatus"] == 1, (
            "console did not force-safe the trigger on reconnect"
        )
    finally:
        try:
            motion.console.stop_trigger(timeout=2)
        except Exception:
            pass


# ──────────────────────────────────────────────────────────────────────────
# Full-scan lifecycle disconnects (L3 bring-up, L4 mid-stream, L5 teardown,
# L6 calibration). These run a real scan through ScanWorkflow and cut the
# whole-system USB at a chosen phase. All FIRE THE LASER. The whole-system cut
# is the only physical disconnect (the hub is internal to the console), so a
# mid-scan cut drops the console too — the host cannot stop_trigger until it
# reconnects, at which point the SDK force-safes the trigger. Each test ends
# with the laser confirmed OFF (via USB-recover-or-mains) and the system back
# to all-connected so a fresh scan could run.
# ──────────────────────────────────────────────────────────────────────────


def _short_scan_request(subject, duration_sec):
    """One camera on the left, right sensor disabled, no storage — the
    minimal real scan for a HIL disconnect."""
    return ScanRequest(
        subject_id=subject,
        duration_sec=duration_sec,
        left_camera_mask=0x01,
        right_camera_mask=0x00,
        skip_default_storage=True,
    )


def _scan_lifecycle_cut(motion, hub, *, subject, cut_after_s, scan_duration_s,
                        expect_canceled):
    """Start a scan, cut whole-system USB after `cut_after_s`, wait for the
    worker to unwind, restore + recover, and assert the laser is OFF. Returns
    the recovery path ("usb"/"mains")."""
    assert _wait_all(motion, MAINS_TIMEOUT), (
        f"preconditions: all connected ({_state_str(motion)})"
    )
    sw = motion.scan_workflow
    started = motion.start_scan(_short_scan_request(subject, scan_duration_s))
    if not started:
        pytest.skip(f"scan refused to start: {sw.last_scan_error}")

    time.sleep(cut_after_s)
    hub.off(SYS_PORT)
    # The scan worker must unwind without hanging (bounded teardown).
    sw.await_complete(timeout_sec=30.0)
    assert not sw.running, "scan worker did not finish after mid-scan disconnect"
    if expect_canceled:
        assert sw.last_scan_canceled or sw.last_scan_error, (
            f"scan should have aborted on disconnect "
            f"(canceled={sw.last_scan_canceled} err={sw.last_scan_error})"
        )

    hub.on(SYS_PORT)
    path = _assert_recovers(motion, hub)
    time.sleep(0.5)
    assert motion.console.get_trigger_json()["TriggerStatus"] == 1, (
        "laser was not OFF after recovering from a mid-scan disconnect"
    )
    return path


@pytest.mark.timeout(180)
@pytest.mark.laser
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_disconnect_during_scan_bringup(motion, hub):
    """L3: cut during bring-up (enable_camera / flush window, first ~0.5 s)."""
    try:
        _scan_lifecycle_cut(
            motion, hub, subject="hil_l3_bringup",
            cut_after_s=0.15, scan_duration_s=8, expect_canceled=True,
        )
    finally:
        try:
            motion.console.stop_trigger(timeout=2)
        except Exception:
            pass


@pytest.mark.timeout(180)
@pytest.mark.laser
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_disconnect_during_scan_streaming(motion, hub):
    """L4: cut mid-stream (laser firing, histograms flowing)."""
    try:
        _scan_lifecycle_cut(
            motion, hub, subject="hil_l4_stream",
            cut_after_s=1.5, scan_duration_s=8, expect_canceled=True,
        )
    finally:
        try:
            motion.console.stop_trigger(timeout=2)
        except Exception:
            pass


@pytest.mark.timeout(180)
@pytest.mark.laser
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_disconnect_during_scan_teardown(motion, hub):
    """L5: short scan, cut as it enters teardown (stop_trigger→disable→close).
    The scan may report complete or canceled depending on the exact instant;
    what matters is no hang, full recovery, and laser OFF."""
    try:
        _scan_lifecycle_cut(
            motion, hub, subject="hil_l5_teardown",
            cut_after_s=1.6, scan_duration_s=1, expect_canceled=False,
        )
    finally:
        try:
            motion.console.stop_trigger(timeout=2)
        except Exception:
            pass


@pytest.mark.timeout(180)
@pytest.mark.laser
@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_disconnect_during_calibration(motion, hub, tmp_path):
    """L6: cut mid-calibration. Calibration drives sub-scans through the same
    ScanWorkflow abort path, so the contract is identical: the workflow stops,
    the system recovers, and the laser ends OFF."""
    from omotion.CalibrationWorkflow import CalibrationRequest, CalibrationThresholds

    assert _wait_all(motion, MAINS_TIMEOUT)
    z = [0.0] * 8
    req = CalibrationRequest(
        operator_id="hil",
        output_dir=str(tmp_path),
        left_camera_mask=0x01,
        right_camera_mask=0x00,
        thresholds=CalibrationThresholds(
            min_mean_per_camera=z, min_contrast_per_camera=z,
            min_bfi_per_camera=z, min_bvi_per_camera=z,
        ),
        duration_sec=3,
    )
    cw = motion.calibration_workflow
    try:
        if not motion.start_calibration(req):
            pytest.skip("calibration refused to start")
        # Let it get into a sub-scan, then cut.
        time.sleep(1.5)
        hub.off(SYS_PORT)
        # Wait for the workflow to stop running (bounded).
        deadline = time.monotonic() + 30
        while cw.running and time.monotonic() < deadline:
            time.sleep(0.2)
        assert not cw.running, "calibration did not stop after mid-scan disconnect"
        hub.on(SYS_PORT)
        _assert_recovers(motion, hub)
        time.sleep(0.5)
        assert motion.console.get_trigger_json()["TriggerStatus"] == 1, (
            "laser not OFF after recovering from a mid-calibration disconnect"
        )
    finally:
        try:
            motion.cancel_calibration()
        except Exception:
            pass
        try:
            motion.console.stop_trigger(timeout=2)
        except Exception:
            pass

