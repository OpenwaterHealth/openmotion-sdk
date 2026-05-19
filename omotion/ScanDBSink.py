"""
ScanDBSink — adapts ScanWorkflow callbacks to ScanDatabase inserts.

Owns one database connection and one open session row for the duration
of a scan.  Constructed and wired up by ``MotionInterface.start_scan``
when the interface was built with a ``db_path``; ``ScanWorkflow``
itself stays unaware of the database.

See ``docs/superpowers/specs/2026-04-14-scan-db-sink-design.md`` for
the rationale and ``docs/superpowers/plans/2026-04-14-scan-db-sink.md``
for the task breakdown.
"""

from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING, Any, Optional

from omotion import _log_root
from omotion.ScanDatabase import ScanDatabase

if TYPE_CHECKING:
    from omotion.MotionProcessing import CorrectedBatch

logger = logging.getLogger(
    f"{_log_root}.ScanDBSink" if _log_root else "ScanDBSink"
)


class ScanDBSink:
    """Single-scan adapter: ScanWorkflow callbacks → ScanDatabase rows."""

    def __init__(
        self,
        db_path: str,
        *,
        write_raw: bool = False,
        compress_raw_hist: bool = True,
        raw_batch_size: int = 200,
    ) -> None:
        self._db_path = db_path
        self._write_raw = write_raw
        self._compress_raw_hist = compress_raw_hist
        self._raw_batch_size = max(1, int(raw_batch_size))

        self._db: Optional[ScanDatabase] = None
        self._session_id: Optional[int] = None
        self._closed: bool = False
        self._lock = threading.Lock()
        self._raw_buffer: list[dict[str, Any]] = []
        self._insert_errors: int = 0

    @property
    def insert_errors(self) -> int:
        return self._insert_errors

    @property
    def session_id(self) -> Optional[int]:
        return self._session_id

    def open(
        self,
        *,
        label: str,
        start_ts: float,
        notes: str,
        meta: dict,
    ) -> int:
        with self._lock:
            if self._db is not None:
                raise RuntimeError("ScanDBSink.open called twice")
            self._db = ScanDatabase(
                db_path=self._db_path,
                compress_raw_hist=self._compress_raw_hist,
            )
            self._session_id = self._db.create_session(
                session_label=label,
                session_start=start_ts,
                session_notes=notes,
                session_meta=meta,
            )
            return self._session_id

    def close(self, end_ts: float) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._flush_raw_locked()
                if self._db is not None and self._session_id is not None:
                    self._db.close_session(self._session_id, end_ts)
            except Exception:
                logger.exception("ScanDBSink.close failed while finalising session")
            finally:
                if self._db is not None:
                    try:
                        self._db.close()
                    except Exception:
                        logger.exception("ScanDBSink: error closing ScanDatabase")
                self._db = None

    def on_corrected_batch(self, batch: "CorrectedBatch") -> None:
        self._require_open()
        # Real implementation lands in Task 5.

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
        self._require_open()
        # Real implementation lands in Task 4.

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_open(self) -> None:
        if self._db is None or self._session_id is None:
            raise RuntimeError(
                "ScanDBSink callback invoked before open() — "
                "call open() to start the session first."
            )

    def _flush_raw_locked(self) -> None:
        """Flush buffered raw frames.  Caller holds self._lock."""
        # Real implementation lands in Task 4.
        self._raw_buffer.clear()
