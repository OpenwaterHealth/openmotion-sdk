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
import time
from typing import TYPE_CHECKING, Any, Optional

from omotion import _log_root
from omotion.ScanDatabase import ScanDatabase
from omotion.Sink import Sink

if TYPE_CHECKING:
    from omotion.MotionProcessing import CorrectedBatch
    from omotion.ScanWorkflow import ScanRequest, ScanResult

logger = logging.getLogger(
    f"{_log_root}.ScanDBSink" if _log_root else "ScanDBSink"
)


class ScanDBSink(Sink):
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

    def on_scan_start(
        self,
        *,
        ts: str,
        session_start_ts: float,
        request: "ScanRequest",
        meta: dict,
    ) -> int:
        """Open the session row. Returns the assigned session_id."""
        label = f"{ts}_{request.subject_id}"
        notes = getattr(request, "notes", "") or ""
        with self._lock:
            if self._db is not None:
                raise RuntimeError("ScanDBSink.on_scan_start called twice")
            self._db = ScanDatabase(
                db_path=self._db_path,
                compress_raw_hist=self._compress_raw_hist,
            )
            self._session_id = self._db.create_session(
                session_label=label,
                session_start=float(session_start_ts),
                session_notes=notes,
                session_meta=meta,
            )
            return self._session_id

    def on_complete(self, result: "ScanResult" = None) -> None:
        """Flush any buffered raw frames, write session_end (wall-clock now),
        and close the DB connection. Idempotent — second call is a no-op."""
        end_ts = time.time()
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._flush_raw_locked()
                if self._db is not None and self._session_id is not None:
                    self._db.close_session(self._session_id, end_ts)
            except Exception:
                logger.exception("ScanDBSink.on_complete failed while finalising session")
            finally:
                if self._db is not None:
                    try:
                        self._db.close()
                    except Exception:
                        logger.exception("ScanDBSink: error closing ScanDatabase")
                self._db = None

    def on_corrected_batch(self, batch: "CorrectedBatch") -> None:
        self._require_open()
        with self._lock:
            if self._db is None or self._session_id is None:
                # Lost the race with close(); drop silently.
                return
            # Flush any buffered raw frames first so raw and corrected
            # writes land in the DB in roughly the order they were produced.
            self._flush_raw_locked()

            if not batch.samples:
                return

            rows: list[dict[str, Any]] = []
            for s in batch.samples:
                side_int = (
                    0 if s.side == "left"
                    else 1 if s.side == "right"
                    else None
                )
                if side_int is None:
                    logger.warning(
                        "ScanDBSink: unknown side %r, skipping sample", s.side
                    )
                    self._insert_errors += 1
                    continue
                rows.append(
                    {
                        "session_id": self._session_id,
                        "session_raw_id": None,
                        "cam_id": int(s.cam_id),
                        "side": side_int,
                        # 6-decimal rounding matches the corrected CSV
                        # writer exactly — Task 9 relies on this for a
                        # clean cell-for-cell equivalence comparison.
                        "timestamp_s": round(float(s.timestamp_s), 6),
                        "bfi": round(float(s.bfi), 6),
                        "bvi": round(float(s.bvi), 6),
                        "contrast": round(float(s.contrast), 6),
                        "mean": round(float(s.mean), 6),
                    }
                )

            if not rows:
                return
            try:
                self._db.insert_session_data_rows(rows)
            except Exception:
                logger.exception(
                    "ScanDBSink: failed to insert %d session_data rows",
                    len(rows),
                )
                self._insert_errors += len(rows)

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
        # Pre-open check runs first so misuse surfaces regardless of
        # write_raw (the early no-op below would otherwise swallow it).
        self._require_open()
        if not self._write_raw:
            return
        with self._lock:
            if self._db is None or self._session_id is None:
                # Lost the race with close() — drop silently rather than
                # raise from a worker thread post-shutdown.
                return
            self._raw_buffer.append(
                {
                    "session_id": self._session_id,
                    "side": side,
                    "cam_id": int(cam_id),
                    "frame_id": int(frame_id),
                    # Project-wide convention: store floats to 6 decimals
                    # (matches the corrected CSV writer). Anything beyond
                    # 6 is noise we don't need to keep.
                    "timestamp_s": round(float(timestamp_s), 6),
                    "hist": bytes(hist),
                    "temp": round(float(temp), 6) if temp is not None else None,
                    "sum_counts": int(sum_counts) if sum_counts is not None else None,
                    "tcm": round(float(tcm), 6),
                    "tcl": round(float(tcl), 6),
                    "pdc": round(float(pdc), 6),
                }
            )
            if len(self._raw_buffer) >= self._raw_batch_size:
                self._flush_raw_locked()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_open(self) -> None:
        if self._db is None or self._session_id is None:
            raise RuntimeError(
                "ScanDBSink callback invoked before on_scan_start() — "
                "call on_scan_start() to start the session first."
            )

    def _flush_raw_locked(self) -> None:
        """Flush buffered raw frames.  Caller holds self._lock."""
        if not self._raw_buffer or self._db is None:
            self._raw_buffer.clear()
            return
        try:
            self._db.insert_raw_frames(self._raw_buffer)
        except Exception:
            logger.exception(
                "ScanDBSink: failed to flush %d buffered raw frames",
                len(self._raw_buffer),
            )
            self._insert_errors += len(self._raw_buffer)
        finally:
            self._raw_buffer.clear()
