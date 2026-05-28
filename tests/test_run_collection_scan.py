"""run_collection_scan — the shared short-scan engine behind the contact-quality
check and the calibration/test sub-scans."""

import threading

import pytest

from omotion.ScanWorkflow import run_collection_scan


class _FakeScanWorkflow:
    """Drives run_collection_scan via the public surface it relies on."""

    def __init__(self, *, refuse=False, error=None, canceled=False, running=False):
        self._refuse = refuse
        self.last_scan_error = error
        self.last_scan_canceled = canceled
        self.running = running
        self.requests = []
        self.await_timeouts = []
        self.cancel_calls = 0

    def start_scan(self, request):
        self.requests.append(request)
        return not self._refuse

    def await_complete(self, timeout_sec=None):
        self.await_timeouts.append(timeout_sec)

    def cancel_scan(self, **kwargs):
        self.cancel_calls += 1
        self.running = False


def test_builds_collector_only_request_and_awaits():
    sw = _FakeScanWorkflow()
    collector = object()
    ok = run_collection_scan(
        sw, collector, subject_id="s", duration_sec=0.5,
        left_camera_mask=0x0F, right_camera_mask=0x00,
    )
    assert ok is True
    req = sw.requests[0]
    assert req.sinks == [collector]
    assert req.skip_default_storage is True
    assert req.duration_sec == 1            # ceil(0.5)
    assert req.left_camera_mask == 0x0F and req.right_camera_mask == 0x00
    assert req.disable_laser is False and req.reduced_mode is False
    assert sw.await_timeouts == [0.5 + 2.0]  # single await, no poll
    assert sw.cancel_calls == 0


def test_raises_on_refused_start_when_raise_on_error():
    sw = _FakeScanWorkflow(refuse=True)
    with pytest.raises(RuntimeError, match="refused"):
        run_collection_scan(sw, object(), subject_id="s", duration_sec=1,
                            left_camera_mask=1, right_camera_mask=0, raise_on_error=True)


def test_raises_on_scan_error_when_raise_on_error():
    sw = _FakeScanWorkflow(error="boom")
    with pytest.raises(RuntimeError, match="boom"):
        run_collection_scan(sw, object(), subject_id="s", duration_sec=1,
                            left_camera_mask=1, right_camera_mask=0, raise_on_error=True)


def test_returns_false_when_last_scan_canceled():
    sw = _FakeScanWorkflow(canceled=True)
    assert run_collection_scan(sw, object(), subject_id="s", duration_sec=1,
                               left_camera_mask=1, right_camera_mask=0) is False


def test_stop_evt_cancels_mid_scan():
    sw = _FakeScanWorkflow(running=True)
    evt = threading.Event()
    evt.set()
    ok = run_collection_scan(
        sw, object(), subject_id="s", duration_sec=1,
        left_camera_mask=1, right_camera_mask=0, stop_evt=evt,
    )
    assert ok is False
    assert sw.cancel_calls == 1


def test_does_not_raise_on_refused_start_without_raise_on_error():
    # CQ path: ignores a falsy start_scan return (its mock returns None).
    sw = _FakeScanWorkflow(refuse=True)
    run_collection_scan(sw, object(), subject_id="s", duration_sec=1,
                        left_camera_mask=1, right_camera_mask=0)  # no raise
    assert sw.await_timeouts  # still awaited
