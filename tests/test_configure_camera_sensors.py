"""start_configure_camera_sensors — status-driven skip of already-ready cameras.

The firmware tracks per-camera state (status bit 0 = peripheral READY,
bit 1 = FPGA programmed, bit 2 = registers configured) and clears bits 1+2
whenever a camera loses power, so the configure worker trusts those bits to
skip the program/configure round-trips and the fixed sleeps that dominated
warm scan starts (issue #272). These tests pin the skip/no-skip decisions
and the settle-sleep gating with fake sensors.
"""

import threading
import time

import pytest

from omotion.ScanWorkflow import ConfigureRequest, ScanWorkflow


READY = 0x01
PROGRAMMED = 0x02
CONFIGURED = 0x04


class _FakeSensor:
    def __init__(self, *, status=None, powered=None, connected=True):
        # status: dict pos -> status byte (default: READY only)
        self._status = dict(status or {})
        self._powered = list(powered if powered is not None else [False] * 8)
        self._connected = connected
        self.calls = []

    def is_connected(self):
        return self._connected

    def get_camera_power_status(self):
        self.calls.append(("power_status",))
        return list(self._powered)

    def enable_camera_power(self, mask):
        self.calls.append(("enable_power", mask))
        for i in range(8):
            if mask & (1 << i):
                self._powered[i] = True
        return True

    def disable_camera_power(self, mask):
        self.calls.append(("disable_power", mask))
        for i in range(8):
            if mask & (1 << i):
                self._powered[i] = False
        return True

    def get_camera_status(self, mask):
        self.calls.append(("status", mask))
        if self._status is None:
            return None
        return {
            i: self._status.get(i, READY)
            for i in range(8)
            if mask & (1 << i)
        }

    def program_fpga(self, camera_position, manual_process):
        self.calls.append(("program", camera_position))
        return True

    def camera_configure_registers(self, camera_position):
        self.calls.append(("configure", camera_position))
        return True

    def named(self, name):
        return [c for c in self.calls if c[0] == name]


class _FakeInterface:
    def __init__(self, left, right):
        self.left = left
        self.right = right


class _Disconnected:
    def is_connected(self):
        return False


@pytest.fixture
def sleeps(monkeypatch):
    """Record time.sleep durations inside the ScanWorkflow module."""
    recorded = []
    import omotion.ScanWorkflow as sw_mod

    monkeypatch.setattr(sw_mod.time, "sleep", recorded.append)
    return recorded


def _run_configure(workflow, request, timeout=5.0):
    """Run the async configure worker to completion; return (result, logs)."""
    done = threading.Event()
    result = []
    logs = []
    started = workflow.start_configure_camera_sensors(
        request,
        on_log_fn=logs.append,
        on_complete_fn=lambda r: (result.append(r), done.set()),
    )
    assert started is True
    assert done.wait(timeout), "configure worker did not complete"
    # on_complete fires just before the worker clears its running flag;
    # wait for the flag so back-to-back runs in one test can't collide.
    deadline = time.monotonic() + timeout
    while workflow._config_running and time.monotonic() < deadline:
        time.sleep(0.01)
    return result[0], logs


def test_warm_cameras_skip_everything(sleeps):
    left = _FakeSensor(
        status={i: READY | PROGRAMMED | CONFIGURED for i in range(8)},
        powered=[True] * 8,
    )
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, logs = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0xFF, right_camera_mask=0x00)
    )
    assert result.ok, result.error
    assert left.named("program") == []
    assert left.named("configure") == []
    # One bulk status read for the whole side, not one per camera.
    assert left.named("status") == [("status", 0xFF)]
    # Power is still asserted (idempotent), but nothing settled or paused.
    assert left.named("enable_power") == [("enable_power", 0xFF)]
    assert sleeps == []
    assert any("already programmed" in line for line in logs)


def test_cold_cameras_program_and_configure_each(sleeps):
    left = _FakeSensor(status={i: READY for i in range(2)})
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0x03, right_camera_mask=0x00)
    )
    assert result.ok, result.error
    assert left.named("program") == [("program", 0x01), ("program", 0x02)]
    assert left.named("configure") == [("configure", 0x01), ("configure", 0x02)]
    # Off->on transition settles once; each real program pauses 0.1 s
    # before configuring through the freshly booted FPGA.
    assert sleeps.count(0.5) == 1
    assert sleeps.count(0.1) == 2


def test_mixed_side_only_touches_unready_camera(sleeps):
    left = _FakeSensor(
        status={0: READY | PROGRAMMED | CONFIGURED, 1: READY},
        powered=[True] * 8,
    )
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0x03, right_camera_mask=0x00)
    )
    assert result.ok, result.error
    assert left.named("program") == [("program", 0x02)]
    assert left.named("configure") == [("configure", 0x02)]
    assert sleeps == [0.1]


def test_programmed_but_unconfigured_reconfigures_without_pause(sleeps):
    left = _FakeSensor(
        status={0: READY | PROGRAMMED}, powered=[True] * 8
    )
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0x01, right_camera_mask=0x00)
    )
    assert result.ok, result.error
    assert left.named("program") == []
    assert left.named("configure") == [("configure", 0x01)]
    assert sleeps == []


def test_settle_shared_when_only_one_side_powers_on(sleeps):
    left = _FakeSensor(
        status={i: READY | PROGRAMMED | CONFIGURED for i in range(8)},
        powered=[True] * 8,
    )
    right = _FakeSensor(status={i: READY for i in range(8)})
    wf = ScanWorkflow(_FakeInterface(left, right))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0xFF, right_camera_mask=0xFF)
    )
    assert result.ok, result.error
    assert left.named("program") == []
    assert len(right.named("program")) == 8
    assert sleeps.count(0.5) == 1


def test_not_ready_camera_fails_the_side():
    left = _FakeSensor(status={0: 0x00}, powered=[True] * 8)
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0x01, right_camera_mask=0x00)
    )
    assert not result.ok
    assert "not READY" in result.error


def test_status_read_failure_fails_the_side():
    left = _FakeSensor(powered=[True] * 8)
    left._status = None
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0x01, right_camera_mask=0x00)
    )
    assert not result.ok
    assert "camera status" in result.error


def test_power_status_failure_falls_back_to_settle(sleeps):
    left = _FakeSensor(
        status={i: READY | PROGRAMMED | CONFIGURED for i in range(8)},
        powered=[True] * 8,
    )
    left.get_camera_power_status = lambda: []
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf, ConfigureRequest(left_camera_mask=0xFF, right_camera_mask=0x00)
    )
    assert result.ok, result.error
    # Unknown power state must be treated as off: settle, but still skip
    # the (status-confirmed) programmed cameras.
    assert sleeps.count(0.5) == 1
    assert left.named("program") == []


def test_power_off_unused_cameras_still_powers_down_others(sleeps):
    left = _FakeSensor(
        status={0: READY | PROGRAMMED | CONFIGURED},
        powered=[True] * 8,
    )
    wf = ScanWorkflow(_FakeInterface(left, _Disconnected()))
    result, _ = _run_configure(
        wf,
        ConfigureRequest(
            left_camera_mask=0x01,
            right_camera_mask=0x00,
            power_off_unused_cameras=True,
        ),
    )
    assert result.ok, result.error
    assert left.named("disable_power") == [("disable_power", 0xFE)]
    assert 0.05 in sleeps
    assert 0.5 not in sleeps
