"""
Sink — common interface for scan-data persistence endpoints.

The corrected-pipeline scan workflow is sink-agnostic: it computes
``Sample`` / ``CorrectedBatch`` events and fans them out to a list of
``Sink`` implementations. Each implementation decides what to do with
the events — write a CSV row, insert a DB row, push a websocket frame,
fan out to a streaming consumer, etc.

Today's concrete implementations:

* ``omotion.CsvSink`` — writes the raw-histogram, corrected, and
  telemetry CSVs the SDK has always produced.
* ``omotion.ScanDBSink`` — writes the SQLite ``sessions`` / ``session_data``
  / ``session_raw`` tables added in issue #92.

``MotionInterface`` composes the active sink list at construction time
(``csv_enabled=True`` adds a ``CsvSink``; ``db_path=...`` adds a
``ScanDBSink``; both, either, or neither are valid). ``ScanWorkflow``
itself never knows or cares which sinks are present — it just calls
the four hooks below in order.

All four methods default to no-ops so subclasses opt into only the
hooks they care about. None of the hooks may raise into the worker
thread; sinks are expected to swallow their own exceptions and surface
them via ``insert_errors`` / logging.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from omotion.MotionProcessing import CorrectedBatch
    from omotion.ScanWorkflow import ScanRequest, ScanResult


class Sink:
    """Base class for scan-data sinks. Override the hooks you implement."""

    def on_scan_start(
        self,
        *,
        ts: str,
        session_start_ts: float,
        request: "ScanRequest",
        meta: dict,
    ) -> None:
        """Fired once at the top of the scan worker, immediately after the
        canonical ``YYYYMMDD_HHMMSS`` timestamp is computed.

        Parameters
        ----------
        ts
            Canonical timestamp string for this scan (also the prefix of
            CSV filenames).
        session_start_ts
            Wall-clock seconds (``time.time()``) at scan start.
        request
            The full ``ScanRequest`` driving this scan. Sinks can read
            ``subject_id``, ``data_dir``, ``duration_sec``, the various
            ``write_*`` flags, etc. as needed.
        meta
            Provenance dict assembled by ``MotionInterface`` —
            subject_id, masks, active cams, sdk/fw versions, hw ids,
            sdk flags. JSON-serializable.
        """
        pass

    def on_raw_frame(
        self,
        side: str,
        cam_id: int,
        frame_id: int,
        timestamp_s: float,
        hist: bytes,
        temp: float,
        sum_counts: int,
        tcm: float,
        tcl: float,
        pdc: float,
    ) -> None:
        """Fired for every raw histogram frame produced by the per-side
        USB stream parsers (~40 Hz × active cameras × 2 sides).

        ``timestamp_s`` is already normalized to per-scan t=0 by
        ``parse_histogram_stream``'s ``t0_normalizer`` — see
        ``docs/ScanDatabase.md`` for the rationale.
        """
        pass

    def on_corrected_batch(self, batch: "CorrectedBatch") -> None:
        """Fired once per dark-frame interval with all per-camera
        ``Sample`` rows the science pipeline produced for that interval.
        """
        pass

    def on_complete(self, result: "ScanResult") -> None:
        """Fired at scan completion (success, cancellation, or error).
        Sinks should flush any buffered state and close file handles
        / DB connections here. Must be idempotent — may be called more
        than once during teardown.
        """
        pass
