"""ScanRunner — pulls batches from a Source, runs them through the Pipeline,
dispatches batch events to the right Sinks via channel subscriptions."""

from __future__ import annotations

import logging

from .batch import FrameBatch, LiveEmit, IntervalClosed, BatchEvent
from .pipeline import Pipeline
from .sinks import Sink
from .sources import Source


logger = logging.getLogger("omotion.pipeline.runner")


class ScanRunner:
    def __init__(self, *, source: Source, pipeline: Pipeline, sinks: list[Sink]):
        self.source = source
        self.pipeline = pipeline
        self.sinks = list(sinks)

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

        try:
            for batch in self.source:
                try:
                    result = self.pipeline.process(batch)
                except Exception:
                    logger.exception("pipeline.process raised — resetting and continuing")
                    self.pipeline.reset()
                    continue
                self._dispatch(result)
        finally:
            for sink in self.sinks:
                try:
                    sink.on_complete()
                except Exception:
                    logger.exception("sink %r raised in on_complete", type(sink).__name__)

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
