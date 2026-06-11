"""Public API surface — symbols documented as importable from omotion.pipeline."""

from __future__ import annotations


def test_public_api_symbols_importable() -> None:
    """All symbols in __all__ must be importable from the top-level package."""
    from omotion.pipeline import (
        Pipeline, Stage,
        FrameBatch, BatchEvent, IntervalClosed, LiveEmit,
        DarkIntegrityWarning, StencilFallback, TerminalDarkResult,
        PipelineError, TimestampMisalignmentWindow,
        ScanRunner, CriticalSinkError,
        Source, LiveUsbSource, CsvReplaySource,
        Sink, ScanMetadata,
        CsvSink, ScanDBSink, DiagnosticsLogSink,
        Tee, default_pipeline,
        SensorPedestals,
    )
    for sym in (
        Pipeline, Stage, FrameBatch, BatchEvent, IntervalClosed, LiveEmit,
        DarkIntegrityWarning, StencilFallback, TerminalDarkResult,
        PipelineError, TimestampMisalignmentWindow,
        ScanRunner, CriticalSinkError,
        Source, LiveUsbSource, CsvReplaySource,
        Sink, ScanMetadata, CsvSink, ScanDBSink, DiagnosticsLogSink,
        Tee, default_pipeline, SensorPedestals,
    ):
        assert sym is not None


def test_public_api_all_list_complete() -> None:
    """__all__ must enumerate every publicly exported symbol."""
    import omotion.pipeline as pkg

    expected = {
        "FrameBatch", "BatchEvent", "IntervalClosed", "LiveEmit",
        "DarkIntegrityWarning", "StencilFallback", "TerminalDarkResult",
        "PipelineError", "TimestampMisalignmentWindow",
        "Pipeline", "Stage", "Tee",
        "ScanRunner", "CriticalSinkError",
        "default_pipeline",
        "SensorPedestals",
        "Sink", "ScanMetadata",
        "CsvSink", "ScanDBSink", "DiagnosticsLogSink",
        "Source", "LiveUsbSource", "CsvReplaySource",
    }

    assert set(pkg.__all__) == expected
