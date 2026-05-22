"""ScanRunner — pulls batches from a Source, runs them through the Pipeline,
dispatches batch events to the right Sinks via channel subscriptions."""

from __future__ import annotations

import logging
import threading

import numpy as np

from .batch import FrameBatch, LiveEmit, IntervalClosed, BatchEvent
from .pipeline import Pipeline
from .sinks import Sink
from .sources import Source


logger = logging.getLogger("omotion.pipeline.runner")


def _empty_batch_for_flush() -> FrameBatch:
    """Build an empty FrameBatch for the terminal on_scan_stop flush."""
    return FrameBatch(
        cam_ids=np.zeros(0, dtype=np.int8),
        frame_ids=np.zeros(0, dtype=np.uint8),
        raw_histograms=np.zeros((0, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((0, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(0, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


class ScanRunner:
    def __init__(self, *, source: Source, pipeline: Pipeline, sinks: list[Sink],
                 telemetry_source=None):
        self.source = source
        self.pipeline = pipeline
        self.sinks = list(sinks)
        self.telemetry_source = telemetry_source
        self._telemetry_thread = None

    def _sinks_for(self, channel: str) -> list[Sink]:
        return [s for s in self.sinks if channel in getattr(s, "channels", set())]

    def _safe_consume(self, sink: Sink, channel: str, payload) -> None:
        try:
            sink.consume(channel, payload)
        except Exception:
            logger.exception("sink %r raised on channel %s; continuing",
                             type(sink).__name__, channel)

    def run(self) -> None:
        for sink in self.sinks:
            try:
                sink.on_scan_start(self.source.metadata)
            except Exception:
                logger.exception("sink %r raised in on_scan_start", type(sink).__name__)

        if self.telemetry_source is not None:
            self._telemetry_thread = threading.Thread(
                target=self._telemetry_loop, daemon=True,
                name="ScanRunner-telemetry",
            )
            self._telemetry_thread.start()

        try:
            for batch in self.source:
                try:
                    result = self.pipeline.process(batch)
                except Exception:
                    logger.exception("pipeline.process raised — resetting and continuing")
                    self.pipeline.reset()
                    continue
                self._dispatch(result)

            flush_batch = _empty_batch_for_flush()
            self.pipeline.on_scan_stop(flush_batch)
            self._dispatch(flush_batch)
        finally:
            if self.telemetry_source is not None:
                self.telemetry_source.close()
                if self._telemetry_thread is not None:
                    self._telemetry_thread.join(timeout=2.0)
            for sink in self.sinks:
                try:
                    sink.on_complete()
                except Exception:
                    logger.exception("sink %r raised in on_complete", type(sink).__name__)

    def _telemetry_loop(self) -> None:
        for event in self.telemetry_source:
            if self.pipeline.telemetry_aggregator is not None:
                self.pipeline.telemetry_aggregator.update(event)
            for sink in self._sinks_for("telemetry"):
                self._safe_consume(sink, "telemetry", event)

    def _dispatch(self, batch: FrameBatch) -> None:
        for event in batch.events:
            if isinstance(event, LiveEmit):
                for sink in self._sinks_for(event.channel):
                    self._safe_consume(sink, event.channel, event.payload)
            elif isinstance(event, IntervalClosed):
                for sink in self._sinks_for("final"):
                    self._safe_consume(sink, "final", event.corrected_batch)
            elif isinstance(event, BatchEvent):
                for sink in self._sinks_for("diagnostics"):
                    self._safe_consume(sink, "diagnostics", event)
