"""Tests for AsyncSink — the threaded wrapper that decouples a slow disk/DB
sink from the ScanRunner consume loop (and thus the live USB drain).

Regression context: in the 2026-06 long-scan soak the runner thread that
drains the live USB batch queue also ran every sink.consume(); a SQLite
commit / CSV flush stalled that drain, backpressured the parser and
dev.read, and overflowed the sensor firmware's 4-deep histo queue. AsyncSink
moves consume() onto a worker thread so a transient sink stall is absorbed by
a buffer instead of the USB path.
"""

from __future__ import annotations

import threading
import time

import pytest

from omotion.pipeline.async_sink import AsyncSink


class _RecordingSink:
    """Minimal Sink that records the order of lifecycle calls."""

    channels = {"final", "raw"}

    def __init__(self, *, consume_delay: float = 0.0) -> None:
        self._consume_delay = consume_delay
        self.events: list = []
        self._lock = threading.Lock()

    def on_scan_start(self, meta) -> None:
        with self._lock:
            self.events.append(("start", meta))

    def consume(self, channel, payload) -> None:
        if self._consume_delay:
            time.sleep(self._consume_delay)
        with self._lock:
            self.events.append(("consume", channel, payload))

    def on_complete(self) -> None:
        with self._lock:
            self.events.append(("complete",))


def test_consume_is_processed_in_fifo_order():
    inner = _RecordingSink()
    sink = AsyncSink(inner)

    sink.on_scan_start(meta="M")
    for i in range(5):
        sink.consume("final", i)
    sink.on_complete()

    assert inner.events == [
        ("start", "M"),
        ("consume", "final", 0),
        ("consume", "final", 1),
        ("consume", "final", 2),
        ("consume", "final", 3),
        ("consume", "final", 4),
        ("complete",),
    ]


def test_on_complete_drains_all_buffered_consumes_before_finalizing():
    # A slow sink: if on_complete did not wait for the worker to drain, the
    # ("complete",) marker would land before some ("consume", ...) entries.
    inner = _RecordingSink(consume_delay=0.01)
    sink = AsyncSink(inner)

    sink.on_scan_start(meta=None)
    for i in range(10):
        sink.consume("final", i)
    sink.on_complete()

    consumes = [e for e in inner.events if e[0] == "consume"]
    assert len(consumes) == 10
    assert inner.events[-1] == ("complete",)


def test_on_scan_start_propagates_exception_so_critical_abort_still_works():
    class _BadStart:
        channels = {"final"}
        critical = True

        def on_scan_start(self, meta):
            raise RuntimeError("db open failed")

        def consume(self, channel, payload):
            pass

        def on_complete(self):
            pass

    sink = AsyncSink(_BadStart())
    with pytest.raises(RuntimeError, match="db open failed"):
        sink.on_scan_start(meta=None)


def test_worker_isolates_a_consume_exception_and_keeps_processing():
    class _FlakySink:
        channels = {"final"}

        def __init__(self):
            self.seen = []

        def on_scan_start(self, meta):
            pass

        def consume(self, channel, payload):
            if payload == "boom":
                raise ValueError("transient sink error")
            self.seen.append(payload)

        def on_complete(self):
            pass

    inner = _FlakySink()
    sink = AsyncSink(inner)
    sink.on_scan_start(meta=None)
    sink.consume("final", "a")
    sink.consume("final", "boom")  # raises inside the worker
    sink.consume("final", "b")     # must still be processed
    sink.on_complete()

    assert inner.seen == ["a", "b"]


def test_exposes_channels_and_critical_from_wrapped_sink():
    class _CritSink:
        channels = {"final", "diagnostics"}
        critical = True

        def on_scan_start(self, meta):
            pass

        def consume(self, channel, payload):
            pass

        def on_complete(self):
            pass

    sink = AsyncSink(_CritSink())
    assert sink.channels == {"final", "diagnostics"}
    assert sink.critical is True

    # Default critical is False when the wrapped sink omits it.
    assert AsyncSink(_RecordingSink()).critical is False


def test_wrapped_exposes_the_inner_sink_for_introspection():
    inner = _RecordingSink()
    assert AsyncSink(inner).wrapped is inner


def test_async_sink_satisfies_the_sink_protocol():
    from omotion.pipeline import Sink
    assert isinstance(AsyncSink(_RecordingSink()), Sink)


def test_async_sink_is_transparent_through_scan_runner():
    """Wrapped in AsyncSink and driven by the real ScanRunner, the inner sink
    still sees on_scan_start, the dispatched event, and on_complete (last)."""
    import numpy as np
    from omotion.pipeline import Pipeline, ScanMetadata, ScanRunner
    from omotion.pipeline.batch import FrameBatch, LiveEmit

    inner = _RecordingSink()  # subscribes {"final", "raw"}

    class _EmitFinalStage:
        name = "emit_final"

        def process(self, batch):
            batch.events.append(LiveEmit(channel="final", payload="P"))
            return batch

        def reset(self):
            pass

    meta = ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-06-22T00:00:00Z", duration_sec=1,
        left_camera_mask=0, right_camera_mask=0, reduced_mode=False,
    )
    batch = FrameBatch(
        cam_ids=np.zeros(1, dtype=np.int8),
        frame_ids=np.zeros(1, dtype=np.uint8),
        raw_histograms=np.zeros((1, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((1, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(1, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )

    class _Src:
        metadata = meta

        def __iter__(self):
            yield batch

        def close(self):
            pass

    ScanRunner(source=_Src(), pipeline=Pipeline([_EmitFinalStage()]),
               sinks=[AsyncSink(inner)]).run()

    assert ("start", meta) in inner.events
    assert ("consume", "final", "P") in inner.events
    assert inner.events[-1] == ("complete",)
