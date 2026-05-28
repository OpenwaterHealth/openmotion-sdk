"""Tests for ScanWorkflow and ScanRequest."""

import dataclasses
import threading
import time
from unittest import mock

from omotion.ScanWorkflow import ScanRequest


def test_scan_request_carries_sinks_field():
    """ScanRequest should carry sinks, skip_default_storage, raw_save_max_duration_s fields."""
    field_names = {f.name for f in dataclasses.fields(ScanRequest)}
    assert "sinks" in field_names
    assert "skip_default_storage" in field_names
    assert "raw_save_max_duration_s" in field_names


def test_scan_request_drops_legacy_fields():
    """ScanRequest should not carry legacy callback/csv/operator fields."""
    field_names = {f.name for f in dataclasses.fields(ScanRequest)}
    assert "on_uncorrected_fn" not in field_names
    assert "on_corrected_batch_fn" not in field_names
    assert "on_dark_frame_fn" not in field_names
    assert "on_rolling_avg_fn" not in field_names
    assert "on_realtime_corrected_fn" not in field_names
    assert "on_raw_frame_fn" not in field_names
    assert "write_raw_csv" not in field_names
    assert "raw_csv_duration_sec" not in field_names
    assert "operator" not in field_names


def test_scan_request_sinks_default_empty_list():
    """ScanRequest should default to empty sinks list and allow construction without these fields."""
    req = ScanRequest(
        subject_id="x", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF,
        disable_laser=False,
    )
    assert req.sinks == []
    assert req.skip_default_storage is False
    assert req.raw_save_max_duration_s is None
    assert req.trigger_config is None


# ---------------------------------------------------------------------------
# Phase E tests — new pipeline-based start_scan
# ---------------------------------------------------------------------------

def _build_motion_with_data_dir(data_dir, scan_db_path=None):
    """Helper: construct a demo-mode MotionInterface with the given output paths."""
    from omotion.MotionInterface import MotionInterface
    return MotionInterface(
        demo_mode=True,
        data_dir=data_dir,
        scan_db_path=scan_db_path,
        operator_id="test",
    )


def test_start_scan_uses_new_runner():
    """After Phase E, ScanWorkflow.start_scan attaches a ScanRunner instance
    to self._runner instead of a SciencePipeline."""
    from omotion.pipeline.runner import ScanRunner
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    assert isinstance(motion.scan_workflow._runner, ScanRunner)


def test_start_scan_auto_injects_csv_sink_when_data_dir_set(tmp_path):
    from omotion.pipeline.sinks import CsvSink
    motion = _build_motion_with_data_dir(str(tmp_path))
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    csv_sinks = [s for s in motion.scan_workflow._runner.sinks if isinstance(s, CsvSink)]
    assert len(csv_sinks) == 1


def test_start_scan_skips_csv_sink_when_data_dir_none():
    from omotion.pipeline.sinks import CsvSink
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    csv_sinks = [s for s in motion.scan_workflow._runner.sinks if isinstance(s, CsvSink)]
    assert csv_sinks == []


def test_start_scan_skips_default_storage_when_request_opts_out(tmp_path):
    from omotion.pipeline.sinks import CsvSink
    motion = _build_motion_with_data_dir(str(tmp_path))
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        skip_default_storage=True,
    )
    motion.scan_workflow.start_scan(request)
    assert all(not isinstance(s, CsvSink) for s in motion.scan_workflow._runner.sinks)


def test_start_scan_does_not_auto_wire_pipeline_telemetry_source():
    class _TelemetryChannelSink:
        channels = {"telemetry"}
        def on_scan_start(self, meta): pass
        def consume(self, channel, payload): pass
        def on_complete(self): pass

    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        sinks=[_TelemetryChannelSink()],
    )
    motion.scan_workflow.start_scan(request)
    assert not hasattr(motion.scan_workflow._runner, "telemetry_source")


def test_start_scan_runner_has_no_telemetry_source_attribute():
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
    )
    motion.scan_workflow.start_scan(request)
    assert not hasattr(motion.scan_workflow._runner, "telemetry_source")


# ---------------------------------------------------------------------------
# Duration-guard cancel race (item #2 in 2026-05-22 IMPORTANT review batch)
# ---------------------------------------------------------------------------
#
# When cancel_scan() runs mid-scan it sets _stop_evt and calls source.close().
# The duration guard's poll loop then exits its sleep early and (previously)
# unconditionally re-ran stop_trigger + sleep(0.35) + source.close(). On real
# hardware the second close ran against torn-down USB endpoints; in tests the
# guard would also block on a full _batch_queue sentinel slot.
# The fix: the guard checks whether the source is already closing (via
# source._stop.is_set()) and exits without duplicate work.

class _MockSource:
    """Hand-rolled stand-in for LiveUsbSource — exposes _stop / close /
    counts close() calls so tests can assert no double-close."""

    def __init__(self, *, metadata, **_kwargs):
        self.metadata = metadata
        self._stop = threading.Event()
        self.close_calls = 0
        # The runner iterates over this; it blocks until cancel sets _stop.
        self._iter_done = threading.Event()
        # Mirror LiveUsbSource public attrs touched elsewhere.
        self._packet_queues = {}

    def __iter__(self):
        # Block in iter so the runner stays in its for-loop until cancel
        # comes in and closes the source. Yield nothing — just wait.
        while not self._stop.is_set():
            time.sleep(0.02)
        # signal that iteration ended
        self._iter_done.set()
        return iter(())

    def close(self):
        self.close_calls += 1
        self._stop.set()


def test_duration_guard_skips_redundant_stop_trigger_and_close_on_cancel(tmp_path):
    """After cancel_scan(), the duration guard must not re-run stop_trigger
    or call close() again on the already-closed source. Before the fix the
    guard would unconditionally fire stop_trigger + sleep + source.close()
    even though the cancel path had already done that work."""
    motion = _build_motion_with_data_dir(None)

    stop_trigger_calls = [0]
    original_stop_trigger = motion.console.stop_trigger

    def _counting_stop_trigger(*a, **kw):
        stop_trigger_calls[0] += 1
        return original_stop_trigger(*a, **kw)

    # Long duration so the guard's poll loop is still inside the sleep
    # when we cancel.
    request = ScanRequest(
        subject_id="x", duration_sec=30,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        skip_default_storage=True,
    )

    captured_source = {}

    def _factory(*, console, left, right, batch_size_frames, metadata):
        src = _MockSource(metadata=metadata)
        captured_source["src"] = src
        return src

    with mock.patch("omotion.pipeline.sources.LiveUsbSource", _factory), \
         mock.patch.object(motion.console, "stop_trigger",
                            side_effect=_counting_stop_trigger):
        assert motion.scan_workflow.start_scan(request)

        # Wait until the worker has registered the source on the runner.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and "src" not in captured_source:
            time.sleep(0.01)
        assert "src" in captured_source, "mock source was not constructed"
        src = captured_source["src"]

        # Give the guard thread a moment to enter its poll loop.
        time.sleep(0.1)

        # Cancel mid-scan. This sets _stop_evt and calls source.close()
        # synchronously. The guard should then wake up, see the source
        # is already closing, and exit without calling stop_trigger or
        # close again.
        motion.scan_workflow.cancel_scan(join_timeout=5.0)

        # cancel_scan calls stop_trigger itself, exactly once. The
        # worker's finally also calls stop_trigger once. The guard
        # must NOT add a third call (which would happen pre-fix
        # because the guard ran its unconditional stop_trigger path).
        # So we expect <= 2.
        assert stop_trigger_calls[0] <= 2, (
            f"stop_trigger called {stop_trigger_calls[0]} times; the "
            "duration guard added a redundant call after cancel"
        )

        # close() should have been called exactly once (by cancel_scan).
        # The guard's branch must not re-close the source.
        assert src.close_calls == 1, (
            f"source.close() called {src.close_calls} times; cancel_scan "
            "called it once and the guard must not call it again"
        )


def test_start_scan_passes_raw_save_max_duration_s_to_pipeline():
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=1,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        raw_save_max_duration_s=60.0,
    )
    motion.scan_workflow.start_scan(request)
    raw_tee = next(s for s in motion.scan_workflow._runner.pipeline.stages
                   if s.name == "tee:raw")
    assert raw_tee.max_duration_s == 60.0


def _run_scan_capturing_trigger(motion, request):
    """Drive start_scan through the active-sensor block (which demo mode would
    otherwise skip) and return the ordered list of ("set", payload) /
    ("start", None) console calls. Forces one active side and stubs the
    per-side hardware bring-up so the worker reaches the trigger send."""
    calls = []

    def _factory(*, console, left, right, batch_size_frames, metadata):
        return _MockSource(metadata=metadata)

    fake_side = ("left", request.left_camera_mask or 0xFF, mock.MagicMock())

    with mock.patch("omotion.pipeline.sources.LiveUsbSource", _factory), \
         mock.patch.object(motion.scan_workflow, "_resolve_active_sides",
                           return_value=[fake_side]), \
         mock.patch.object(motion.scan_workflow, "_scan_subscribe_state",
                           lambda handles: None), \
         mock.patch.object(motion, "run_on_sensors", return_value=True), \
         mock.patch.object(motion.console, "set_trigger_json",
                           side_effect=lambda **kw: calls.append(("set", kw.get("data")))), \
         mock.patch.object(motion.console, "start_trigger",
                           side_effect=lambda *a, **k: calls.append(("start", None))):
        assert motion.scan_workflow.start_scan(request)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and not any(c[0] == "start" for c in calls):
            time.sleep(0.01)
        motion.scan_workflow.cancel_scan(join_timeout=5.0)
    return calls


def test_start_scan_sends_trigger_config_before_start_trigger():
    """start_scan must (re)send the resolved trigger config to reset the
    firmware fsync/dark schedule BEFORE starting the trigger, so the firmware's
    laser-skip (dark) frames align with the pipeline's dark-frame classification.
    Asserts both the ordering and that the sent payload is the resolved default."""
    motion = _build_motion_with_data_dir(None)
    expected_cfg = motion.resolve_trigger_config(None)
    request = ScanRequest(
        subject_id="x", duration_sec=30,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        skip_default_storage=True,
    )
    calls = _run_scan_capturing_trigger(motion, request)

    kinds = [c[0] for c in calls]
    assert "set" in kinds, "start_scan never sent the trigger config"
    assert "start" in kinds, "start_scan never started the trigger"
    assert kinds.index("set") < kinds.index("start"), \
        "set_trigger_json must be sent before start_trigger"
    sent = next(payload for kind, payload in calls if kind == "set")
    assert sent == expected_cfg


def test_start_scan_trigger_config_override_is_merged():
    """A ScanRequest.trigger_config override is shallow-merged on top of the
    interface default before being sent to the console."""
    motion = _build_motion_with_data_dir(None)
    request = ScanRequest(
        subject_id="x", duration_sec=30,
        left_camera_mask=0xFF, right_camera_mask=0, reduced_mode=False,
        skip_default_storage=True,
        trigger_config={"TriggerFrequencyHz": 20},
    )
    calls = _run_scan_capturing_trigger(motion, request)

    sent = next((payload for kind, payload in calls if kind == "set"), None)
    assert sent is not None, "trigger config was never sent"
    assert sent["TriggerFrequencyHz"] == 20
    # untouched keys fall through to the resolved default
    assert sent["LaserPulseSkipInterval"] == \
        motion.resolve_trigger_config(None)["LaserPulseSkipInterval"]
