"""Tee — a stage that emits a LiveEmit event for a named channel.

Tee stages are positional markers in the pipeline. The runner reads
LiveEmit events from batch.events and dispatches the payload to sinks
subscribed to the named channel.

A Tee may carry an optional `filter` predicate over frame_type. If supplied
and no frame in the batch passes the filter, no LiveEmit is appended for
that batch. This is how the "live" tee excludes warmup and stale frames
while the "raw" tee includes them.
"""

from __future__ import annotations

from typing import Callable, Optional

from .batch import FrameBatch, LiveEmit


class Tee:
    """Emits one LiveEmit per batch (or zero, if filter excludes all frames).

    The default payload is the FrameBatch itself; sinks slice out the rows
    they care about based on the channel and their own logic.
    """

    def __init__(self, channel: str, *, filter: Optional[Callable[[str], bool]] = None):
        self.name = f"tee:{channel}"
        self.channel = channel
        self.filter = filter

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self.filter is not None:
            if batch.frame_type is None:
                return batch
            if not any(self.filter(ft) for ft in batch.frame_type):
                return batch
        batch.events.append(LiveEmit(channel=self.channel, payload=batch))
        return batch

    def reset(self) -> None:
        pass
