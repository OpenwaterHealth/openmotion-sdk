"""Tee — a stage that emits a LiveEmit event for a named channel.

Tee stages are positional markers in the pipeline. The runner reads
LiveEmit events from batch.events and dispatches the payload to sinks
subscribed to the named channel.

A Tee may carry:
- an optional `emit_if_any` predicate over frame_type. This is a
  BATCH-LEVEL gate, not a row filter: if ANY frame in the batch passes,
  the WHOLE batch (non-passing rows included) is emitted. Sinks must do
  their own per-row filtering — use FrameBatch.iter_rows(exclude=...),
  the canonical helper, instead of hand-rolling the loop.
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
    """Emits one LiveEmit per batch (or zero, if no frame passes the gate).

    The payload is the whole FrameBatch (zero-copy by default — slicing
    rows here would force a per-batch copy on the hot live channel); sinks
    slice out the rows they care about via FrameBatch.iter_rows.
    """

    def __init__(self, channel: str, *,
                 emit_if_any: Optional[Callable[[str], bool]] = None,
                 max_duration_s: Optional[float] = None,
                 snapshot: bool = False):
        self.name = f"tee:{channel}"
        self.channel = channel
        self.emit_if_any = emit_if_any
        self.max_duration_s = max_duration_s
        self.snapshot = snapshot

    def process(self, batch: FrameBatch) -> FrameBatch:
        if self.max_duration_s is not None and batch.timestamp_s.size > 0:
            if float(batch.timestamp_s[0]) > self.max_duration_s:
                return batch

        if self.emit_if_any is not None:
            if batch.frame_type is None:
                return batch
            if not any(self.emit_if_any(ft) for ft in batch.frame_type):
                return batch
        payload = batch.snapshot() if self.snapshot else batch
        batch.events.append(LiveEmit(channel=self.channel, payload=payload))
        return batch

    def reset(self) -> None:
        pass
