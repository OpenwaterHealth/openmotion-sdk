"""ScanRunner — pulls batches from a Source, runs them through the Pipeline,
dispatches batch events to the right Sinks via channel subscriptions."""

from __future__ import annotations

import logging

import numpy as np

from .batch import FrameBatch, LiveEmit, IntervalClosed, BatchEvent, PipelineError
from .pipeline import Pipeline
from .sinks import Sink
from .sources import Source


logger = logging.getLogger("openmotion.sdk.pipeline.runner")


class CriticalSinkError(RuntimeError):
    """A sink marked ``critical`` failed to initialize, so the scan was
    aborted rather than run with no durable record of the data.

    Sinks opt in by setting ``critical = True`` (default is False). The
    canonical critical sink is ScanDBSink: if the scan database can't be
    opened there is nowhere to persist the corrected (final-branch) record,
    and (when the corrected CSV is opt-in and off) the scan would otherwise
    complete with no data at all."""


def _empty_batch_for_flush() -> FrameBatch:
    """Build an empty FrameBatch for the terminal on_scan_stop flush."""
    return FrameBatch(
        cam_ids=np.zeros(0, dtype=np.int8),
        frame_ids=np.zeros(0, dtype=np.uint8),
        side_ids=np.zeros(0, dtype=np.int8),
        raw_histograms=np.zeros((0, 2, 8, 1024), dtype=np.uint32),
        temperature_c=np.zeros((0, 2, 8), dtype=np.float32),
        timestamp_s=np.zeros(0, dtype=np.float64),
        pdc=None, tcm=None, tcl=None,
    )


class ScanRunner:
    def __init__(self, *, source: Source, pipeline: Pipeline, sinks: list[Sink]):
        self.source = source
        self.pipeline = pipeline
        self.sinks = list(sinks)
        # Sinks whose on_scan_start raised. They're left in self.sinks for
        # introspection but skipped by _sinks_for / on_complete dispatch so
        # consume() never runs against a partially-initialized sink
        # (e.g. _meta=None, file handle not open).
        self._failed_sinks: set = set()

    def _sinks_for(self, channel: str) -> list[Sink]:
        return [s for s in self.sinks
                if s not in self._failed_sinks
                and channel in getattr(s, "channels", set())]

    def _safe_consume(self, sink: Sink, channel: str, payload) -> None:
        try:
            sink.consume(channel, payload)
        except Exception:
            logger.exception("sink %r raised on channel %s; continuing",
                             type(sink).__name__, channel)

    def run(self) -> None:
        started: list[Sink] = []
        for sink in self.sinks:
            try:
                sink.on_scan_start(self.source.metadata)
                started.append(sink)
            except Exception as exc:
                if getattr(sink, "critical", False):
                    logger.exception(
                        "critical sink %r raised in on_scan_start — aborting scan",
                        type(sink).__name__,
                    )
                    # Roll back sinks that already started so their file
                    # handles / DB connections don't leak on the aborted scan.
                    for s in reversed(started):
                        try:
                            s.on_complete()
                        except Exception:
                            logger.exception(
                                "sink %r raised during abort rollback",
                                type(s).__name__,
                            )
                    raise CriticalSinkError(
                        f"{type(sink).__name__} failed to initialize: {exc}"
                    ) from exc
                logger.exception(
                    "sink %r raised in on_scan_start — disabling for this scan",
                    type(sink).__name__,
                )
                self._failed_sinks.add(sink)

        try:
            for batch in self.source:
                try:
                    result = self.pipeline.process(batch)
                except Exception as exc:
                    # Drop the batch but PRESERVE stage state. Resetting here
                    # would clear the frame unwrappers — re-tripping the
                    # stale-first guard (~6.4 s of dropped frames per camera)
                    # and permanently misaligning the positional dark schedule
                    # for the rest of the scan. A dropped batch is the same
                    # gap shape as USB packet loss, which stages already
                    # tolerate.
                    n = int(batch.frame_ids.shape[0])
                    logger.exception(
                        "pipeline.process raised — dropping batch (%d frames), "
                        "stage state preserved", n,
                    )
                    self.dispatch_event(PipelineError(
                        error=repr(exc),
                        n_frames=n,
                        first_timestamp_s=(
                            float(batch.timestamp_s[0])
                            if batch.timestamp_s is not None and batch.timestamp_s.size > 0
                            else None
                        ),
                    ))
                    continue
                self._dispatch(result)

            flush_batch = _empty_batch_for_flush()
            self.pipeline.on_scan_stop(flush_batch)
            self._dispatch(flush_batch)
        finally:
            for sink in self.sinks:
                if sink in self._failed_sinks:
                    # on_scan_start raised — sink is partially initialized;
                    # calling on_complete could touch unset attrs (_meta,
                    # file handles). Skip.
                    continue
                try:
                    sink.on_complete()
                except Exception:
                    logger.exception("sink %r raised in on_complete", type(sink).__name__)

    def _dispatch(self, batch: FrameBatch) -> None:
        for event in batch.events:
            self.dispatch_event(event)

    def dispatch_event(self, event: BatchEvent) -> None:
        """Route one event to the sinks subscribed to its channel.

        LiveEmit routes to its declared channel; IntervalClosed routes to
        "final"; everything else (DarkIntegrityWarning, PipelineError,
        TriggerStateEvent, …) routes to "diagnostics". Also the entry point
        for out-of-band events not tied to a FrameBatch (e.g. ScanWorkflow's
        TriggerStateEvent transitions).
        """
        if isinstance(event, LiveEmit):
            channel, payload = event.channel, event.payload
        elif isinstance(event, IntervalClosed):
            channel, payload = "final", event.corrected_batch
        else:
            channel, payload = "diagnostics", event
        for sink in self._sinks_for(channel):
            self._safe_consume(sink, channel, payload)
