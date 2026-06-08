"""Pipeline class and Stage protocol.

This module is pure plumbing — no science. Stages declare their behavior
through the Stage protocol; Pipeline drives them in order.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .batch import FrameBatch


@runtime_checkable
class Stage(Protocol):
    """A single pipeline transformation.

    Stages mutate the FrameBatch in place and return it (for chainability).
    Stages may maintain cross-batch state (history, ring buffers, frame
    counters); reset() clears that state for a new scan or replay.
    """
    name: str

    def process(self, batch: FrameBatch) -> FrameBatch: ...

    def reset(self) -> None: ...


class Pipeline:
    """Ordered list of stages. Pure transformation, no I/O.

    The runner (omotion.pipeline.runner.ScanRunner) feeds FrameBatches in,
    Pipeline routes them through each stage in order, returns the result.
    """

    def __init__(self, stages: list[Stage]):
        self.stages = list(stages)

    def process(self, batch: FrameBatch) -> FrameBatch:
        for stage in self.stages:
            batch = stage.process(batch)
        return batch

    def reset(self) -> None:
        """Call reset() on every stage. Done at scan start and after any
        stage exception during a scan."""
        for stage in self.stages:
            stage.reset()

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Lifecycle hook for stages that need to do end-of-scan cleanup
        (e.g. DarkCorrectionStage's terminal dark flush)."""
        for stage in self.stages:
            if hasattr(stage, "on_scan_stop"):
                stage.on_scan_stop(batch)
