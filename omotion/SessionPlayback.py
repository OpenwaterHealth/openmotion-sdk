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

The output matches what ``CsvSink`` writes during the scan
(bfi, bvi, contrast, mean, temp). ``temp`` cells are empty only for
rows recorded before ``session_data.temp`` existed (schema v2, issue
#221). Dark rows are covered too: the stencilled row's temp is
fabricated by the same neighbour stencil as its other metrics.

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

from omotion import _log_root, correction_status
from omotion.ScanDatabase import ScanDatabase
from omotion.pipeline.sinks import _NORMAL_HEADERS, _REDUCED_HEADERS

logger = logging.getLogger(
    f"{_log_root}.SessionPlayback" if _log_root else "SessionPlayback"
)


_CAM_TAGS = [f"{s}{i}" for s in ("l", "r") for i in range(1, 9)]


def _corrected_columns(reduced_mode: bool, include_status: bool = False) -> list[str]:
    """CsvSink's corrected-CSV columns (sans the frame_id / timestamp_s
    prefix), taken from the live writer's own header lists so the two
    layouts cannot drift; plus the export-only status columns
    (bloodflow-app#589): one camera-tagged ``correction_status`` list and a
    per-camera ``cq_l1`` … ``cq_r8`` contact-quality verdict."""
    cols = list((_REDUCED_HEADERS if reduced_mode else _NORMAL_HEADERS)[2:])
    if include_status:
        cols.append("correction_status")
        cols += [f"cq_{tag}" for tag in _CAM_TAGS]
    return cols


def materialize_corrected_csv(
    db_path: str,
    session_id: int,
    output_path: str,
    *,
    include_status: bool = False,
    include_quality: Optional[bool] = None,
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

    When ``include_status`` is True, a ``correction_status`` column (the
    comma-separated, camera-tagged list of pipeline corrections at that
    frame, e.g. ``l3:ts_corrected,r5:nan_filled``; empty = clean) and
    per-camera ``cq_l1`` … ``cq_r8`` columns (the live contact-quality
    verdict latched at that frame: ``ok`` / ``poor_contact`` /
    ``ambient_light`` / ``poor_contact,ambient_light``) are appended. A
    ``cq_`` cell is empty for a camera outside the scan mask and for scans
    recorded without a live monitor or before schema v3. Reduced-mode
    sessions get the same columns, decoded from the side-average rows'
    camera-tagged lists. ``include_quality`` is the pre-#589 name for the
    same switch.

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
        sdk_flags = meta.get("sdk_flags", {}) or {}
        reduced_mode = bool(sdk_flags.get("reduced_mode", False))
        emit_status = bool(include_status or include_quality)
        side_masks = (
            int(sdk_flags.get("left_camera_mask", 0xFF) or 0),
            int(sdk_flags.get("right_camera_mask", 0xFF) or 0),
        )

        # Pull every per-(side, cam, frame) cell for this session. Order
        # by frame_id so we can stream-merge into per-frame rows.
        select_cols = (
            "frame_id, timestamp_s, side, cam_id, bfi, bvi, contrast, mean, temp"
        )
        if emit_status:
            select_cols += ", correction_status, contact_quality"
        cur = db._connection().execute(
            f"""
            SELECT {select_cols}
            FROM session_data
            WHERE session_id = ?
            ORDER BY frame_id ASC, side ASC, cam_id ASC
            """,
            (session_id,),
        )

        cols = _corrected_columns(reduced_mode, include_status=emit_status)
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
            # Status accumulators for the frame being buffered.
            buf_status: list = []
            buf_cq: dict = {}

            def _status_vals() -> dict:
                if not emit_status:
                    return {}
                vals = {"correction_status": correction_status.join(buf_status)}
                for tag, verdict in buf_cq.items():
                    vals[f"cq_{tag}"] = verdict
                return vals

            for db_row in cur:
                fid       = int(db_row[0])
                ts        = float(db_row[1])
                side_int  = int(db_row[2])
                cam_id    = int(db_row[3])
                bfi       = db_row[4]
                bvi       = db_row[5]
                contrast  = db_row[6]
                mean      = db_row[7]
                temp      = db_row[8]
                status    = db_row[9] if emit_status else None
                cq        = db_row[10] if emit_status else None

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
                        _emit(w, buf_fid, buf_ts, {**vals, **_status_vals()})
                    else:
                        _emit(w, buf_fid, buf_ts, {**buf_vals, **_status_vals()})
                    buf_vals = {}
                    buf_red = {}
                    buf_status = []
                    buf_cq = {}
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
                    if emit_status:
                        # Side-average rows already carry camera-tagged lists.
                        buf_status.append(status or "")
                        if cq is not None:
                            by_cam = correction_status.split_tagged(cq)
                            prefix = "l" if side_int == 0 else "r"
                            for cam in range(8):
                                if side_masks[side_int] & (1 << cam):
                                    tag = f"{prefix}{cam + 1}"
                                    buf_cq[tag] = ",".join(by_cam.get(tag, [])) or "ok"
                else:
                    suffix = f"{'l' if side_int == 0 else 'r'}{cam_id + 1}"
                    if bfi      is not None: buf_vals[f"bfi_{suffix}"]      = float(bfi)
                    if bvi      is not None: buf_vals[f"bvi_{suffix}"]      = float(bvi)
                    if contrast is not None: buf_vals[f"contrast_{suffix}"] = float(contrast)
                    if mean     is not None: buf_vals[f"mean_{suffix}"]     = float(mean)
                    if temp     is not None: buf_vals[f"temp_{suffix}"]     = float(temp)
                    if emit_status:
                        buf_status.extend(correction_status.tagged(suffix, status))
                        if cq is not None:
                            buf_cq[suffix] = cq

            # Flush the final frame.
            if buf_fid is not None:
                if reduced_mode:
                    vals = {}
                    for sd, acc in buf_red.items():
                        n = max(1, acc["count"])
                        vals[f"bfi_{sd}"] = round(acc["bfi"] / n, 6)
                        vals[f"bvi_{sd}"] = round(acc["bvi"] / n, 6)
                    _emit(w, buf_fid, buf_ts, {**vals, **_status_vals()})
                else:
                    _emit(w, buf_fid, buf_ts, {**buf_vals, **_status_vals()})

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
