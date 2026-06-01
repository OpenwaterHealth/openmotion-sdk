"""omotion.pipeline — composable, numpy-vectorized histogram -> BFI/BVI pipeline.

See docs/SciencePipeline.md for the algorithm.
See docs/superpowers/specs/2026-05-22-data-pipeline-rearchitecture-design.md
for the design that this package implements.

Typical use::

    from omotion.pipeline import (
        default_pipeline, ScanRunner,
        CsvReplaySource, CsvSink, ScanMetadata, SensorPedestals,
    )

    meta = ScanMetadata(...)
    pipeline = default_pipeline(metadata=meta, calibration=cal,
                                pedestals=SensorPedestals.from_sensors(left, right))
    source = CsvReplaySource(raw_csv_left=..., raw_csv_right=..., metadata=meta)
    sinks  = [CsvSink(output_dir="./out")]
    ScanRunner(source=source, pipeline=pipeline, sinks=sinks).run()
"""

from .batch import (
    FrameBatch,
    BatchEvent,
    IntervalClosed,
    LiveEmit,
    DarkIntegrityWarning,
    StencilFallback,
    TerminalDarkResult,
)
from .pipeline import Pipeline, Stage
from .tee import Tee
from .runner import ScanRunner, CriticalSinkError
from .factory import default_pipeline
from .pedestal import SensorPedestals
from .sinks import Sink, ScanMetadata, CsvSink, ScanDBSink
from .sources import Source, LiveUsbSource, CsvReplaySource, DbReplaySource


__all__ = [
    "FrameBatch", "BatchEvent", "IntervalClosed", "LiveEmit",
    "DarkIntegrityWarning", "StencilFallback", "TerminalDarkResult",
    "Pipeline", "Stage", "Tee",
    "ScanRunner", "CriticalSinkError",
    "default_pipeline",
    "SensorPedestals",
    "Sink", "ScanMetadata",
    "CsvSink", "ScanDBSink",
    "Source", "LiveUsbSource", "CsvReplaySource", "DbReplaySource",
]
