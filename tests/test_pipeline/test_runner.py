"""ScanRunner — Source → Pipeline → Sinks. The only I/O orchestrator."""

import numpy as np
import pytest
from omotion.pipeline.batch import FrameBatch, LiveEmit, IntervalClosed
from omotion.pipeline.pipeline import Pipeline
from omotion.pipeline.runner import ScanRunner, CriticalSinkError
from omotion.pipeline.sinks import ScanMetadata


class _FakeSource:
    def __init__(self, batches, metadata):
        self._batches = batches
        self.metadata = metadata
    def __iter__(self):
        yield from self._batches
    def close(self):
        pass


class _RecordingSink:
    def __init__(self, channels):
        self.channels = set(channels)
        self.consumed = []
        self.on_start_calls = 0
        self.on_complete_calls = 0
    def on_scan_start(self, meta):
        self.on_start_calls += 1
    def consume(self, channel, payload):
        self.consumed.append((channel, payload))
    def on_complete(self):
        self.on_complete_calls += 1


class _EmitTagsStage:
    name = "emit_tags"
    def __init__(self, channels):
        self.channels = channels
    def process(self, batch):
        for ch in self.channels:
            batch.events.append(LiveEmit(channel=ch, payload=batch))
        return batch
    def reset(self):
        pass


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-05-22T00:00:00Z", duration_sec=60,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )


def _empty_batch():
    return FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


def test_runner_lifecycle_calls_on_start_and_on_complete():
    sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[sink],
    )
    runner.run()
    assert sink.on_start_calls == 1
    assert sink.on_complete_calls == 1


def test_runner_routes_live_events_to_subscribed_sinks_only():
    live_sink = _RecordingSink(channels={"live"})
    raw_sink  = _RecordingSink(channels={"raw"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live", "raw"])]),
        sinks=[live_sink, raw_sink],
    )
    runner.run()
    assert [c for c, _ in live_sink.consumed] == ["live"]
    assert [c for c, _ in raw_sink.consumed]  == ["raw"]


def test_runner_routes_interval_closed_to_final_sinks():
    final_sink = _RecordingSink(channels={"final"})

    class _IntervalStage:
        name = "interval"
        def process(self, batch):
            batch.events.append(IntervalClosed(corrected_batch="payload_x"))
            return batch
        def reset(self): pass

    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_IntervalStage()]),
        sinks=[final_sink],
    )
    runner.run()
    assert final_sink.consumed == [("final", "payload_x")]


def test_runner_isolates_sink_exceptions():
    class _CrashingSink:
        channels = {"live"}
        def on_scan_start(self, m): pass
        def consume(self, ch, p): raise RuntimeError("boom")
        def on_complete(self): pass

    good_sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[_CrashingSink(), good_sink],
    )
    runner.run()
    assert len(good_sink.consumed) == 1


def test_runner_no_longer_accepts_telemetry_source():
    with pytest.raises(TypeError):
        ScanRunner(
            source=_FakeSource([_empty_batch()], _meta()),
            pipeline=Pipeline([_EmitTagsStage([])]),
            sinks=[],
            telemetry_source=object(),
        )


def test_runner_skips_sink_whose_on_scan_start_raised():
    """If a sink's on_scan_start raises, it must be skipped for the rest of
    the scan — no consume() calls, no on_complete() call. Otherwise the
    runner ends up driving methods against a partially-initialized sink
    (e.g. _meta=None, no open file handle) and a sink crash at startup
    cascades into spurious consume-time errors."""

    class _BadOnStartSink:
        channels = {"live"}
        def __init__(self):
            self.on_start_calls = 0
            self.consume_calls = 0
            self.on_complete_calls = 0
        def on_scan_start(self, meta):
            self.on_start_calls += 1
            raise RuntimeError("boom in on_scan_start")
        def consume(self, channel, payload):
            self.consume_calls += 1
        def on_complete(self):
            self.on_complete_calls += 1

    bad_sink  = _BadOnStartSink()
    good_sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[bad_sink, good_sink],
    )
    runner.run()

    assert bad_sink.on_start_calls == 1
    assert bad_sink.consume_calls == 0, \
        "consume() must not be invoked against a sink whose on_scan_start raised"
    assert bad_sink.on_complete_calls == 0, \
        "on_complete() must not be invoked against a sink whose on_scan_start raised"
    # Other sinks are not affected.
    assert len(good_sink.consumed) == 1
    assert good_sink.on_complete_calls == 1


def test_runner_failed_sink_does_not_receive_diagnostic_or_final_events():
    """The skip applies to all channels, not just 'live' — diagnostics, final, etc."""

    class _BadOnStartSink:
        channels = {"diagnostics", "final", "live"}
        def __init__(self):
            self.consume_calls = []
        def on_scan_start(self, meta):
            raise RuntimeError("boom")
        def consume(self, channel, payload):
            self.consume_calls.append(channel)
        def on_complete(self):
            pass

    class _MultiEmitStage:
        name = "multi"
        def process(self, batch):
            batch.events.append(LiveEmit(channel="live", payload=batch))
            batch.events.append(IntervalClosed(corrected_batch="interval_payload"))
            return batch
        def reset(self): pass

    bad_sink = _BadOnStartSink()
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_MultiEmitStage()]),
        sinks=[bad_sink],
    )
    runner.run()

    assert bad_sink.consume_calls == [], \
        f"failed sink should see no consume calls; got {bad_sink.consume_calls}"


def test_runner_calls_on_scan_stop_and_dispatches_flush_events():
    """Stage.on_scan_stop() must be called after source exhausts; any events
    it appends (e.g. IntervalClosed from terminal dark flush) must be
    dispatched to the appropriate sinks."""

    class _OnScanStopStage:
        name = "stop_stage"
        def process(self, batch):
            return batch
        def reset(self):
            pass
        def on_scan_stop(self, batch):
            batch.events.append(IntervalClosed(corrected_batch="terminal_payload"))

    final_sink = _RecordingSink(channels={"final"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_OnScanStopStage()]),
        sinks=[final_sink],
    )
    runner.run()
    assert ("final", "terminal_payload") in final_sink.consumed


def test_critical_sink_failure_aborts_scan():
    """A sink marked critical that raises in on_scan_start must abort the
    whole scan (raise CriticalSinkError) — not silently disable itself and
    run on, which would leave no durable record of the data."""

    class _CriticalBadSink:
        channels = {"live"}
        critical = True
        def on_scan_start(self, meta):
            raise RuntimeError("db connection refused")
        def consume(self, channel, payload):
            raise AssertionError("consume must not run on an aborted scan")
        def on_complete(self):
            raise AssertionError("on_complete must not run on a failed critical sink")

    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[_CriticalBadSink()],
    )
    with pytest.raises(CriticalSinkError, match="db connection refused"):
        runner.run()


def test_critical_sink_failure_rolls_back_already_started_sinks():
    """When a critical sink fails, sinks that already started successfully
    get on_complete() so their handles/connections are released before the
    abort propagates."""

    good_sink = _RecordingSink(channels={"live"})  # starts first, succeeds

    class _CriticalBadSink:
        channels = {"live"}
        critical = True
        def on_scan_start(self, meta):
            raise RuntimeError("boom")
        def consume(self, channel, payload):
            pass
        def on_complete(self):
            pass

    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[good_sink, _CriticalBadSink()],
    )
    with pytest.raises(CriticalSinkError):
        runner.run()

    assert good_sink.on_start_calls == 1
    assert good_sink.on_complete_calls == 1, \
        "an already-started sink must be rolled back (on_complete) on abort"
    assert good_sink.consumed == [], "no frames should be consumed on an aborted scan"


def test_noncritical_sink_failure_still_disables_and_continues():
    """The default (critical=False) behavior is unchanged: a failing sink is
    disabled and the scan runs on."""

    class _BadSink:
        channels = {"live"}
        # critical defaults to False (attribute absent)
        def on_scan_start(self, meta):
            raise RuntimeError("boom")
        def consume(self, channel, payload):
            pass
        def on_complete(self):
            pass

    good_sink = _RecordingSink(channels={"live"})
    runner = ScanRunner(
        source=_FakeSource([_empty_batch()], _meta()),
        pipeline=Pipeline([_EmitTagsStage(["live"])]),
        sinks=[_BadSink(), good_sink],
    )
    runner.run()  # must NOT raise
    assert len(good_sink.consumed) == 1
