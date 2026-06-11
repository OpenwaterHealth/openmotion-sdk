"""
Playback utilities — read a finished session out of ``scans.db`` and
rebuild the on-disk corrected CSV.

Issue #92 (Step D): when ``csvEnabled=false`` runs a scan, only the
DB sink writes. The bloodflow-app's "Visualize BFI/BVI" button expects
a corrected CSV next to the session, so this module gives callers a
one-call way to materialize that CSV on demand from ``session_data``.

``session_data`` holds the final-branch (interval-corrected) record:
per-camera rows in normal mode, cam_id=-1 side-average rows in reduced
mode. Sessions whose ``session_meta`` lacks the ``data_semantics``
marker were written by older SDKs and hold realtime (live-branch)
values instead — playback still works, but the values are the
pre-refinement ones.

The output matches what ``CsvSink`` writes during the scan **for the
columns ``session_data`` carries** (bfi, bvi, contrast, mean). Columns
the science pipeline produces but the DB doesn't store — ``temp_*``,
``std_*`` — are emitted as empty cells. The visualizer
(``plot_corrected_scan.py``) ignores those columns, so the resulting
plot matches the live-scan output.

Reduced-mode column layout is recovered from
``session_meta.sdk_flags.reduced_mode`` (stamped by ScanDBSink).
Legacy reduced sessions without that key are mis-detected as
non-reduced and produce an empty-celled CSV — callers should treat
sessions without ``sdk_flags`` as not playback-capable in reduced mode.

Requires the post-#92-Step-F schema (``session_data.frame_id``
present). For sessions older than that migration, ``frame_id`` is
``-1`` for every row and merging collapses to a single row — callers
should detect that case and skip playback.
"""

from __future__ import annotations

import csv
import json
import logging
from typing import Optional

from omotion import _log_root
from omotion.ScanDatabase import ScanDatabase

logger = logging.getLogger(
    f"{_log_root}.SessionPlayback" if _log_root else "SessionPlayback"
)


def _corrected_columns(reduced_mode: bool, include_quality: bool = False) -> list[str]:
    """Match CsvSink._corrected_columns(reduced_mode) exactly."""
    if reduced_mode:
        return ["bfi_left", "bfi_right", "bvi_left", "bvi_right"]
    cols = (
        [f"bfi_l{i}" for i in range(1, 9)]
        + [f"bfi_r{i}" for i in range(1, 9)]
        + [f"bvi_l{i}" for i in range(1, 9)]
        + [f"bvi_r{i}" for i in range(1, 9)]
        + [f"mean_l{i}" for i in range(1, 9)]
        + [f"mean_r{i}" for i in range(1, 9)]
        + [f"contrast_l{i}" for i in range(1, 9)]
        + [f"contrast_r{i}" for i in range(1, 9)]
        + [f"temp_l{i}" for i in range(1, 9)]
        + [f"temp_r{i}" for i in range(1, 9)]
    )
    if include_quality:
        cols += [f"quality_l{i}" for i in range(1, 9)]
        cols += [f"quality_r{i}" for i in range(1, 9)]
    return cols


def materialize_corrected_csv(
    db_path: str,
    session_id: int,
    output_path: str,
    *,
    include_quality: bool = False,
) -> str:
    """
    Read ``session_data`` for ``session_id`` and write a corrected-format
    CSV to ``output_path``. Returns ``output_path`` on success.

    Reads ``session_meta`` from the ``sessions`` row to recover
    ``reduced_mode`` (column layout) — defaults to non-reduced if the
    meta is missing or malformed. Active-camera detection is implicit:
    cells without rows in ``session_data`` end up empty in the CSV
    (same behavior as the live writer when a (side, cam) pair was
    masked off).

    When ``include_quality`` is True (non-reduced mode only), per-camera
    ``quality_l1`` … ``quality_r8`` columns are appended.

    Raises ``ValueError`` if the session doesn't exist, or
    ``RuntimeError`` if the session was recorded before #92 Step F
    (``frame_id`` is the sentinel -1 for every row).
    """
    db = ScanDatabase(db_path=db_path)
    try:
        row = db._connection().execute(
            "SELECT session_label, session_meta FROM sessions WHERE id = ?",
            (session_id,),
        ).fetchone()
        if row is None:
            raise ValueError(
                f"materialize_corrected_csv: session_id={session_id} not found"
            )
        meta_json = row[1]
        meta: dict = {}
        if meta_json:
            try:
                meta = json.loads(meta_json)
            except json.JSONDecodeError:
                logger.warning(
                    "materialize_corrected_csv: session_meta for sid=%d "
                    "is not valid JSON; assuming non-reduced layout",
                    session_id,
                )
        reduced_mode = bool(meta.get("sdk_flags", {}).get("reduced_mode", False))
        emit_quality = include_quality and not reduced_mode

        # Pull every per-(side, cam, frame) cell for this session. Order
        # by frame_id so we can stream-merge into per-frame rows.
        select_cols = "frame_id, timestamp_s, side, cam_id, bfi, bvi, contrast, mean"
        if emit_quality:
            select_cols += ", quality"
        cur = db._connection().execute(
            f"""
            SELECT {select_cols}
            FROM session_data
            WHERE session_id = ?
            ORDER BY frame_id ASC, side ASC, cam_id ASC
            """,
            (session_id,),
        )

        cols = _corrected_columns(reduced_mode, include_quality=emit_quality)
        rows_written = 0
        first_frame_id: Optional[int] = None

        def _emit(out_writer, frame_id, ts, values: dict) -> None:
            nonlocal rows_written
            out = [frame_id, ts]
            out.extend(values.get(c, "") for c in cols)
            out_writer.writerow(out)
            rows_written += 1

        # Streaming merge: when frame_id changes, flush the buffered row.
        # Per-frame timestamp_s is the min of all contributing samples
        # — matches the CsvSink merge behavior.
        with open(output_path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["frame_id", "timestamp_s", *cols])

            buf_fid: Optional[int] = None
            buf_ts: Optional[float] = None
            buf_vals: dict = {}
            # Reduced-mode accumulator: sum + count per side per frame.
            buf_red: dict = {}

            for db_row in cur:
                fid       = int(db_row[0])
                ts        = float(db_row[1])
                side_int  = int(db_row[2])
                cam_id    = int(db_row[3])
                bfi       = db_row[4]
                bvi       = db_row[5]
                contrast  = db_row[6]
                mean      = db_row[7]
                quality   = db_row[8] if emit_quality else None

                if first_frame_id is None:
                    first_frame_id = fid

                if fid != buf_fid and buf_fid is not None:
                    # Flush previous frame.
                    if reduced_mode:
                        vals: dict = {}
                        for sd, acc in buf_red.items():
                            n = max(1, acc["count"])
                            vals[f"bfi_{sd}"] = round(acc["bfi"] / n, 6)
                            vals[f"bvi_{sd}"] = round(acc["bvi"] / n, 6)
                        _emit(w, buf_fid, buf_ts, vals)
                    else:
                        _emit(w, buf_fid, buf_ts, buf_vals)
                    buf_vals = {}
                    buf_red = {}
                    buf_ts = None

                buf_fid = fid
                if buf_ts is None or ts < buf_ts:
                    buf_ts = ts

                if reduced_mode:
                    sd_name = "left" if side_int == 0 else "right"
                    acc = buf_red.get(sd_name)
                    if acc is None:
                        acc = {"bfi": 0.0, "bvi": 0.0, "count": 0}
                        buf_red[sd_name] = acc
                    if bfi is not None:
                        acc["bfi"] += float(bfi)
                    if bvi is not None:
                        acc["bvi"] += float(bvi)
                    acc["count"] += 1
                else:
                    suffix = f"{'l' if side_int == 0 else 'r'}{cam_id + 1}"
                    if bfi      is not None: buf_vals[f"bfi_{suffix}"]      = float(bfi)
                    if bvi      is not None: buf_vals[f"bvi_{suffix}"]      = float(bvi)
                    if contrast is not None: buf_vals[f"contrast_{suffix}"] = float(contrast)
                    if mean     is not None: buf_vals[f"mean_{suffix}"]     = float(mean)
                    if quality  is not None: buf_vals[f"quality_{suffix}"]  = quality

            # Flush the final frame.
            if buf_fid is not None:
                if reduced_mode:
                    vals = {}
                    for sd, acc in buf_red.items():
                        n = max(1, acc["count"])
                        vals[f"bfi_{sd}"] = round(acc["bfi"] / n, 6)
                        vals[f"bvi_{sd}"] = round(acc["bvi"] / n, 6)
                    _emit(w, buf_fid, buf_ts, vals)
                else:
                    _emit(w, buf_fid, buf_ts, buf_vals)

        if first_frame_id == -1:
            raise RuntimeError(
                f"materialize_corrected_csv: session_id={session_id} was "
                "recorded before #92 Step F (no per-row frame_id); "
                "cannot reconstruct the corrected CSV from this DB"
            )

        logger.info(
            "materialize_corrected_csv: session_id=%d → %s (%d rows)",
            session_id, output_path, rows_written,
        )
        return output_path
    finally:
        db.close()
