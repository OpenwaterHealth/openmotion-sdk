"""Tee — a stage that emits a LiveEmit event for a named channel.

Tee stages are positional markers in the pipeline. The runner reads
LiveEmit events from batch.events and dispatches the payload to sinks
subscribed to the named channel.

A Tee may carry:
- an optional `filter` predicate over frame_type. If supplied and no
  frame in the batch passes the filter, no LiveEmit is appended.
- an optional `max_duration_s` cap. If supplied and the batch's first
  frame timestamp exceeds it, no LiveEmit is appended. Used to cap
  raw-CSV writing at a configurable duration.
- an optional `snapshot` flag. When True, the LiveEmit payload is a
  deep copy of the batch (via FrameBatch.snapshot()) rather than a live
  reference. Required for any tee that runs *upstream* of in-place
  mutating stages, because the runner dispatches the event only after
  the whole pipeline finishes — by then a referenced batch has already
  been mutated. The "raw" tee needs this; the "live" tee (last stage)
  does not.
"""

from __future__ import annotations

from typing import Callable, Optional

from .batch import FrameBatch, LiveEmit


class Tee:
    """Emits one LiveEmit per batch (or zero, if filter excludes all frames).

    The default payload is the FrameBatch itself; sinks slice out the rows
    they care about based on the channel and their own logic.
    """

    def __init__(self, channel: str, *,
                 filter: Optional[Callable[[str], bool]] = None,
                 max_duration_s: Optional[float] = None,
                 snapshot: bool = False):
        self.name = f"tee:{channel}"
        self.channel = channel
        self.filter = filter
        self.max_duration_s = max_duration_s
        self.snapshot = snapshot

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self.max_duration_s is not None and batch.timestamp_s.size > 0:
            if float(batch.timestamp_s[0]) > self.max_duration_s:
                return batch

        if self.filter is not None:
            if batch.frame_type is None:
                return batch
            if not any(self.filter(ft) for ft in batch.frame_type):
                return batch
        payload = batch.snapshot() if self.snapshot else batch
        batch.events.append(LiveEmit(channel=self.channel, payload=payload))
        return batch

    def reset(self) -> None:
        pass
