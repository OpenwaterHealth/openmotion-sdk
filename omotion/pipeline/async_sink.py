"""AsyncSink — run a slow sink's consume() on its own worker thread.

The ScanRunner consume loop runs on the same thread that drains the live
USB batch queue (see ``LiveUsbSource`` / ``StreamInterface``). A sink that
blocks there — an fsync on a SQLite commit, a CSV buffer flushing to disk —
stalls that drain, which backpressures the histogram parser and then
``dev.read``, overflowing the sensor firmware's shallow 4-deep histo queue
and dropping frames. (Root-caused on the 2026-06 long-scan soak: the
firmware "HISTO enqueue fail: queue full" drops were brief host-side drain
stalls, not a USB fault.)

``AsyncSink`` wraps a sink and moves ``consume()`` onto a dedicated worker
thread fed by a bounded FIFO queue, so a *transient* sink stall is absorbed
by the buffer instead of the USB path. Sustained overload (a sink that can
never keep up) still backpressures once the buffer fills — that is correct;
you cannot outrun the disk forever and dropping persisted data is worse.

Lifecycle:
  * ``on_scan_start`` runs the wrapped sink synchronously and propagates any
    exception, so a ``critical`` sink whose start fails still aborts the scan
    (ScanRunner reads ``.critical``) and the sink is fully initialized before
    any ``consume`` is enqueued.
  * ``consume`` enqueues ``(channel, payload)`` (cheap) for the worker.
  * ``on_complete`` enqueues a sentinel, waits for the worker to drain every
    buffered item, then calls the wrapped sink's ``on_complete`` — so no
    buffered data is lost.
"""

from __future__ import annotations

import logging
import queue
import threading
from typing import Any

logger = logging.getLogger("openmotion.sdk.pipeline.async_sink")

_SENTINEL = object()


class AsyncSink:
    """Threaded buffer in front of a Sink. Duck-types the Sink protocol."""

    def __init__(self, sink: Any, *, max_queue: int = 64,
                 join_timeout_s: float = 60.0) -> None:
        self._sink = sink
        self._queue: queue.Queue = queue.Queue(maxsize=max(1, int(max_queue)))
        self._join_timeout_s = float(join_timeout_s)
        self._worker: threading.Thread | None = None
        self._started = False
        self._closed = False
        self._name = type(sink).__name__
        # Mirror the attributes the runner reads off a sink so AsyncSink is a
        # drop-in stand-in (channels for routing, critical for abort policy).
        self.channels = getattr(sink, "channels", set())
        self.critical = getattr(sink, "critical", False)

    @property
    def wrapped(self):
        """The underlying sink, for callers that need to introspect past the
        async wrapper (e.g. ``isinstance`` checks on the concrete sink type)."""
        return self._sink

    def on_scan_start(self, meta) -> None:
        # Synchronous on purpose: a critical sink that fails here must still
        # raise so ScanRunner aborts before the laser fires, and the sink must
        # be ready before the first enqueued consume runs on the worker.
        self._sink.on_scan_start(meta)
        self._closed = False
        self._worker = threading.Thread(
            target=self._run, name=f"AsyncSink-{self._name}", daemon=True,
        )
        self._started = True
        self._worker.start()

    def consume(self, channel: str, payload: Any) -> None:
        if not self._started or self._closed:
            # No worker to service the item (start failed/skipped or already
            # finalized). Dropping is correct: there is nowhere to deliver it.
            return
        # Blocking put: absorbs transient stalls up to max_queue, and applies
        # honest backpressure only on sustained overload.
        self._queue.put((channel, payload))

    def on_complete(self) -> None:
        if not self._started:
            # on_scan_start never ran (or raised) — no worker was created;
            # finalize the wrapped sink directly so it still tears down.
            self._sink.on_complete()
            return
        if self._closed:
            return
        self._closed = True
        self._queue.put(_SENTINEL)
        self._worker.join(timeout=self._join_timeout_s)
        if self._worker.is_alive():
            logger.error(
                "AsyncSink(%s): worker did not drain within %.0fs; "
                "%d buffered item(s) may be lost",
                self._name, self._join_timeout_s, self._queue.qsize(),
            )
        self._sink.on_complete()

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is _SENTINEL:
                    return
                channel, payload = item
                try:
                    self._sink.consume(channel, payload)
                except Exception:
                    logger.exception(
                        "AsyncSink(%s): consume raised on channel %s; "
                        "continuing", self._name, channel,
                    )
            finally:
                self._queue.task_done()
