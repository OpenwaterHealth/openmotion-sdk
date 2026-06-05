"""TimestampRepairStage -- EMI timestamp correction + NaN-fill.

Detects EMI-induced timestamp misalignment via two conditions:
  1. Timestamp deviation: |actual_dt - expected_dt| > tolerance
  2. In-packet frame_id disagreement: cameras at the same timestamp
     report different frame_ids

Repairs bad runs by linear interpolation re-anchored between the last
and next good device timestamps. Inserts synthetic NaN-fill rows for
missing abs_frame_id gaps. Logs one WARNING per misalignment window
plus an end-of-scan summary.

See docs/superpowers/specs/2026-06-05-eft-timestamp-repair-design.md.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from dataclasses import dataclass
from itertools import groupby
from operator import attrgetter
from typing import Optional

import numpy as np

from ..batch import FrameBatch


logger = logging.getLogger("openmotion.sdk.pipeline.stages.timestamp_repair")

_INITIAL_NOMINAL_PERIOD_S = 0.025  # 25 ms (40 Hz)
_EMA_ALPHA = 0.01  # slow convergence for nominal period


@dataclass
class _BufferedFrame:
    """One frame held in the look-ahead buffer during a bad run.

    Stores all per-frame data from the batch so the buffer is independent
    of any batch's arrays.  When flushed, a new row is constructed from
    the stored data — no stale-index problem across batch or side
    boundaries.

    ``raw_histograms`` and ``temperature_c`` start as views into the
    source batch for zero-copy within-batch flushes.  ``_detach_arrays``
    is called at the top of the *next* ``process()`` to deep-copy any
    surviving views before the old batch's memory is recycled.
    """
    cam_id: int
    side_idx: int
    abs_frame_id: int
    frame_id: int
    original_ts: float
    frame_type: str
    raw_histograms: np.ndarray   # (2, 8, 1024) — view or owned copy
    temperature_c: np.ndarray    # (2, 8) — view or owned copy
    _owns_arrays: bool = False

    def detach(self) -> None:
        """Deep-copy array views so the buffer survives batch turnover."""
        if not self._owns_arrays:
            self.raw_histograms = self.raw_histograms.copy()
            self.temperature_c = self.temperature_c.copy()
            self._owns_arrays = True


@dataclass
class _WindowStats:
    """Tracks one misalignment window for coalesced logging."""
    onset_fid: int
    onset_t: float
    end_fid: int = 0
    end_t: float = 0.0
    n_corrected: int = 0
    n_nan: int = 0


class TimestampRepairStage:
    """Detect and correct EMI-induced timestamp corruption.

    Placed after FrameClassificationStage, before the science chain.
    Rewrites batch.timestamp_s in place for corrected frames. Sets
    batch.quality for every frame. May expand the batch by inserting
    synthetic NaN-fill rows for missing abs_frame_id gaps.
    """

    name = "timestamp_repair"

    def __init__(self, *, tolerance_s: float = 0.002,
                 max_buffer_frames: int = 16):
        self._tolerance = float(tolerance_s)
        self._max_buffer = int(max_buffer_frames)
        self._reset_state()

    def _reset_state(self) -> None:
        self._nominal_period = _INITIAL_NOMINAL_PERIOD_S
        # Per (side, cam): last known good (abs_frame_id, timestamp_s)
        self._last_good: dict[tuple[int, int], tuple[int, float]] = {}
        self._buffer: list[_BufferedFrame] = []
        self._in_bad_run = False
        self._window_onset: Optional[_WindowStats] = None
        # Scan-wide summary
        self._scan_windows: list[_WindowStats] = []
        self._total_frames_seen = 0

    def process(self, batch: FrameBatch) -> FrameBatch:
        if batch.abs_frame_ids is None or batch.frame_type is None:
            return batch

        n = len(batch.cam_ids)
        if n == 0:
            batch.quality = np.empty(0, dtype="<U14")
            return batch

        # Detach any surviving buffer frames so they own their arrays
        # before the previous batch's memory is recycled.
        for bf in self._buffer:
            bf.detach()

        self._total_frames_seen += n

        # --- Condition 2: in-packet frame_id disagreement ---
        bad_by_cond2 = self._detect_frame_id_disagreement(batch)

        # --- Per-row processing: condition 1 + routing ---
        quality = np.full(n, "ok", dtype="<U14")
        output_indices: list[int] = []
        # Synthetic rows to insert: (position_in_output, list_of_dicts).
        # Dicts come from _make_nan_fills, _reanchor_flush, _force_flush.
        synthetic_inserts: list[tuple[int, list[dict]]] = []

        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype in ("warmup", "stale"):
                output_indices.append(i)
                continue

            cam_id = int(batch.cam_ids[i])
            side_idx = int(batch.side_ids[i])
            abs_fid = int(batch.abs_frame_ids[i])
            ts = float(batch.timestamp_s[i])
            key = (side_idx, cam_id)

            is_bad = i in bad_by_cond2

            # Condition 1: timestamp deviation
            if not is_bad and key in self._last_good:
                prev_fid, prev_ts = self._last_good[key]
                fid_gap = abs_fid - prev_fid
                if fid_gap > 0:
                    expected_dt = fid_gap * self._nominal_period
                    actual_dt = ts - prev_ts
                    if abs(actual_dt - expected_dt) > self._tolerance:
                        is_bad = True

            if is_bad:
                if not self._in_bad_run:
                    self._in_bad_run = True
                    self._window_onset = _WindowStats(
                        onset_fid=abs_fid, onset_t=ts,
                    )
                self._buffer.append(_BufferedFrame(
                    cam_id=cam_id, side_idx=side_idx,
                    abs_frame_id=abs_fid, frame_id=int(batch.frame_ids[i]),
                    original_ts=ts, frame_type=ftype,
                    raw_histograms=batch.raw_histograms[i],
                    temperature_c=batch.temperature_c[i],
                ))
                if len(self._buffer) >= self._max_buffer:
                    flushed = self._force_flush_buffer()
                    if flushed:
                        synthetic_inserts.append(
                            (len(output_indices), flushed))
            else:
                if self._in_bad_run:
                    # Re-anchor: this good frame closes the bad run
                    flushed = self._reanchor_flush(
                        anchor_fid=abs_fid, anchor_ts=ts,
                        anchor_side=side_idx, anchor_cam=cam_id,
                    )
                    if flushed:
                        synthetic_inserts.append(
                            (len(output_indices), flushed))

                # Check for NaN-fill gaps
                if key in self._last_good:
                    prev_fid, prev_ts = self._last_good[key]
                    gap = abs_fid - prev_fid
                    if gap > 1:
                        fills = self._make_nan_fills(
                            side_idx=side_idx, cam_id=cam_id,
                            start_fid=prev_fid + 1, end_fid=abs_fid,
                            start_ts=prev_ts, end_ts=ts,
                            batch=batch,
                        )
                        synthetic_inserts.append(
                            (len(output_indices), fills))

                # Update nominal period from good single-step frames
                if key in self._last_good:
                    prev_fid, prev_ts = self._last_good[key]
                    if abs_fid - prev_fid == 1:
                        measured_dt = ts - prev_ts
                        if measured_dt > 0:
                            self._nominal_period = (
                                (1 - _EMA_ALPHA) * self._nominal_period
                                + _EMA_ALPHA * measured_dt
                            )

                self._last_good[key] = (abs_fid, ts)
                quality[i] = "ok"
                output_indices.append(i)

        batch.quality = quality

        # Rebuild the batch when:
        # - there are synthetic rows to insert (flushed buffer or NaN fills)
        # - some rows were buffered and excluded from output_indices
        needs_rebuild = synthetic_inserts or len(output_indices) != n
        if needs_rebuild:
            batch = self._rebuild_batch_with_fills(
                batch, output_indices, synthetic_inserts,
            )

        return batch

    def _detect_frame_id_disagreement(self, batch: FrameBatch) -> set[int]:
        """Condition 2: find rows where cameras at the same timestamp
        disagree on frame_id."""
        bad_indices: set[int] = set()
        ts_groups: dict[float, list[int]] = defaultdict(list)
        n = len(batch.cam_ids)
        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype in ("warmup", "stale"):
                continue
            ts_groups[float(batch.timestamp_s[i])].append(i)

        for indices in ts_groups.values():
            if len(indices) < 2:
                continue
            fids = set()
            for idx in indices:
                fids.add(int(batch.frame_ids[idx]))
            if len(fids) > 1:
                bad_indices.update(indices)

        return bad_indices

    def _reanchor_flush(self, *, anchor_fid: int, anchor_ts: float,
                        anchor_side: int, anchor_cam: int) -> list[dict]:
        """Interpolate buffered frames between last good and the re-anchor.

        Only frames matching the anchor's (side, cam) are re-anchored.
        Frames from other (side, cam) pairs are passed through with
        original timestamps and quality="ok" (they were buffered only
        because they coincided with a bad run on a different stream —
        e.g. FM-6 scan-end glitch on one side while the other side is
        clean).

        Returns a list of frame dicts (same schema as _make_nan_fills)
        suitable for _rebuild_batch_with_fills.
        """
        flushed: list[dict] = []
        if not self._buffer:
            self._in_bad_run = False
            return flushed

        # Partition: same-side vs cross-side.
        # Same-side frames (possibly different cameras) are re-anchored.
        # Cross-side frames are dropped — they were false-positives from
        # a side boundary (e.g. FM-6 scan-end frames from the left side
        # that got buffered and then flushed by the right side's first
        # frame).  Inserting them into the wrong-side batch would cause
        # non-monotonic timestamps downstream.
        same_side = [bf for bf in self._buffer if bf.side_idx == anchor_side]
        cross_side = [bf for bf in self._buffer if bf.side_idx != anchor_side]

        # --- Re-anchor same-side frames via interpolation ---
        # Group by (side, cam) for per-camera left-anchor lookup
        same_side_sorted = sorted(same_side, key=attrgetter("cam_id"))
        for cam_id, cam_group in groupby(same_side_sorted, key=attrgetter("cam_id")):
            frames = list(cam_group)
            key = (anchor_side, cam_id)
            if key in self._last_good:
                left_fid, left_ts = self._last_good[key]
            else:
                left_fid = frames[0].abs_frame_id - 1
                left_ts = frames[0].original_ts - self._nominal_period

            fid_span = anchor_fid - left_fid
            ts_span = anchor_ts - left_ts

            for bf in frames:
                if fid_span > 0:
                    corrected_ts = (
                        left_ts
                        + (bf.abs_frame_id - left_fid) / fid_span * ts_span
                    )
                else:
                    corrected_ts = bf.original_ts
                flushed.append(self._bf_to_dict(bf, corrected_ts, "ts_corrected"))

        # Log the window
        if self._window_onset is not None:
            self._window_onset.end_fid = self._buffer[-1].abs_frame_id
            self._window_onset.end_t = self._buffer[-1].original_ts
            self._window_onset.n_corrected = len(same_side)
            self._scan_windows.append(self._window_onset)
            logger.warning(
                "Misalignment window: frames %d–%d (t=%.2f–%.2fs), "
                "%d frames re-timestamped, %d frames NaN-filled",
                self._window_onset.onset_fid, self._window_onset.end_fid,
                self._window_onset.onset_t, self._window_onset.end_t,
                self._window_onset.n_corrected, self._window_onset.n_nan,
            )
        if cross_side:
            logger.warning(
                "Dropped %d cross-side frame(s) from buffer "
                "(side boundary / FM-6)",
                len(cross_side),
            )

        # Update last_good for re-anchored frames
        for bf in same_side:
            bk = (bf.side_idx, bf.cam_id)
            # Find the corrected ts from the flushed dicts
            for fd in flushed:
                if (fd["side_idx"] == bf.side_idx
                        and fd["cam_id"] == bf.cam_id
                        and fd["abs_frame_id"] == bf.abs_frame_id):
                    self._last_good[bk] = (bf.abs_frame_id, fd["timestamp_s"])
                    break

        self._buffer.clear()
        self._in_bad_run = False
        self._window_onset = None
        return flushed

    def _force_flush_buffer(self) -> list[dict]:
        """Buffer full with no re-anchor: interpolate using nominal period.

        Returns frame dicts suitable for _rebuild_batch_with_fills.
        """
        flushed: list[dict] = []
        if not self._buffer:
            return flushed

        first = self._buffer[0]
        key = (first.side_idx, first.cam_id)
        if key in self._last_good:
            left_fid, left_ts = self._last_good[key]
        else:
            left_fid = first.abs_frame_id - 1
            left_ts = first.original_ts - self._nominal_period

        for bf in self._buffer:
            fid_delta = bf.abs_frame_id - left_fid
            corrected_ts = left_ts + fid_delta * self._nominal_period
            flushed.append(self._bf_to_dict(bf, corrected_ts, "ts_corrected"))

        if self._window_onset is not None:
            self._window_onset.end_fid = self._buffer[-1].abs_frame_id
            self._window_onset.end_t = self._buffer[-1].original_ts
            self._window_onset.n_corrected = len(self._buffer)
            self._scan_windows.append(self._window_onset)
            logger.warning(
                "Misalignment window (forced flush): frames %d–%d "
                "(t=%.2f–%.2fs), %d frames re-timestamped, "
                "%d frames NaN-filled",
                self._window_onset.onset_fid, self._window_onset.end_fid,
                self._window_onset.onset_t, self._window_onset.end_t,
                self._window_onset.n_corrected, self._window_onset.n_nan,
            )

        for bf in self._buffer:
            bk = (bf.side_idx, bf.cam_id)
            # Find the corrected ts from the flushed dicts
            for fd in flushed:
                if (fd["side_idx"] == bf.side_idx
                        and fd["cam_id"] == bf.cam_id
                        and fd["abs_frame_id"] == bf.abs_frame_id):
                    self._last_good[bk] = (bf.abs_frame_id, fd["timestamp_s"])
                    break

        self._buffer.clear()
        self._in_bad_run = False
        self._window_onset = None
        return flushed

    @staticmethod
    def _bf_to_dict(bf: _BufferedFrame, timestamp_s: float,
                    quality: str) -> dict:
        """Convert a buffered frame to a dict for _rebuild_batch_with_fills."""
        return {
            "cam_id": bf.cam_id,
            "frame_id": bf.frame_id,
            "side_idx": bf.side_idx,
            "abs_frame_id": bf.abs_frame_id,
            "timestamp_s": timestamp_s,
            "frame_type": bf.frame_type,
            "quality": quality,
            "raw_histograms": bf.raw_histograms,
            "temperature_c": bf.temperature_c,
        }

    def _make_nan_fills(self, *, side_idx: int, cam_id: int,
                        start_fid: int, end_fid: int,
                        start_ts: float, end_ts: float,
                        batch: FrameBatch) -> list[dict]:
        """Create synthetic NaN-fill frame descriptors for missing
        abs_frame_ids."""
        fills = []
        fid_span = end_fid - (start_fid - 1)
        ts_span = end_ts - start_ts
        for fid in range(start_fid, end_fid):
            frac = (fid - (start_fid - 1)) / fid_span if fid_span > 0 else 0
            interp_ts = start_ts + frac * ts_span
            fills.append({
                "cam_id": cam_id,
                "frame_id": fid & 0xFF,
                "side_idx": side_idx,
                "abs_frame_id": fid,
                "timestamp_s": interp_ts,
                "frame_type": "light",
                "quality": "nan_filled",
            })
        if self._window_onset is not None:
            self._window_onset.n_nan += len(fills)
        return fills

    def _rebuild_batch_with_fills(
        self,
        batch: FrameBatch,
        output_indices: list[int],
        synthetic_inserts: list[tuple[int, list[dict]]],
    ) -> FrameBatch:
        """Rebuild the batch, inserting synthetic rows at the right positions.

        Synthetic rows come from three sources:
          - _make_nan_fills (NaN-fill for missing frames — zero histograms)
          - _reanchor_flush (buffered frames with stored data)
          - _force_flush_buffer (buffered frames with stored data)

        Dicts from flushed buffers carry ``raw_histograms`` and
        ``temperature_c`` numpy arrays; NaN-fill dicts do not (zeros
        are used).
        """
        inserts_by_pos = sorted(synthetic_inserts, key=lambda x: x[0])

        total_fills = sum(len(fills) for _, fills in inserts_by_pos)
        n_orig = len(output_indices)
        n_new = n_orig + total_fills

        # Nothing to do: all original rows kept, no synthetics
        if total_fills == 0 and n_orig == len(batch.cam_ids):
            return batch

        # Create new arrays
        new_cam_ids = np.zeros(n_new, dtype=np.int8)
        new_frame_ids = np.zeros(n_new, dtype=np.uint8)
        new_side_ids = np.zeros(n_new, dtype=np.int8)
        new_abs_frame_ids = np.zeros(n_new, dtype=np.int64)
        new_timestamp_s = np.zeros(n_new, dtype=np.float64)
        new_frame_type = np.empty(n_new, dtype="<U14")
        new_quality = np.empty(n_new, dtype="<U14")
        new_raw_histograms = np.zeros(
            (n_new, 2, 8, 1024), dtype=np.uint32,
        )
        new_temperature_c = np.zeros(
            (n_new, 2, 8), dtype=np.float32,
        )

        # Build an ordered sequence of (source_type, data) entries
        entries: list[tuple[str, object]] = []
        fill_iter = iter(inserts_by_pos)
        next_fill = next(fill_iter, None)

        for pos, orig_idx in enumerate(output_indices):
            while next_fill is not None and next_fill[0] == pos:
                for fd in next_fill[1]:
                    entries.append(("fill", fd))
                next_fill = next(fill_iter, None)
            entries.append(("orig", orig_idx))

        # Drain remaining fills
        while next_fill is not None:
            for fd in next_fill[1]:
                entries.append(("fill", fd))
            next_fill = next(fill_iter, None)

        for out_i, (src_type, data) in enumerate(entries):
            if src_type == "orig":
                orig_i = data
                new_cam_ids[out_i] = batch.cam_ids[orig_i]
                new_frame_ids[out_i] = batch.frame_ids[orig_i]
                new_side_ids[out_i] = batch.side_ids[orig_i]
                new_abs_frame_ids[out_i] = batch.abs_frame_ids[orig_i]
                new_timestamp_s[out_i] = batch.timestamp_s[orig_i]
                new_frame_type[out_i] = str(batch.frame_type[orig_i])
                new_quality[out_i] = str(batch.quality[orig_i])
                new_raw_histograms[out_i] = batch.raw_histograms[orig_i]
                new_temperature_c[out_i] = batch.temperature_c[orig_i]
            else:
                fd = data
                new_cam_ids[out_i] = fd["cam_id"]
                new_frame_ids[out_i] = fd["frame_id"]
                new_side_ids[out_i] = fd["side_idx"]
                new_abs_frame_ids[out_i] = fd["abs_frame_id"]
                new_timestamp_s[out_i] = fd["timestamp_s"]
                new_frame_type[out_i] = fd["frame_type"]
                new_quality[out_i] = fd["quality"]
                # Flushed buffer frames carry their own data arrays;
                # NaN-fill dicts do not (zeros are correct for those).
                if "raw_histograms" in fd:
                    new_raw_histograms[out_i] = fd["raw_histograms"]
                if "temperature_c" in fd:
                    new_temperature_c[out_i] = fd["temperature_c"]

        new_batch = FrameBatch(
            cam_ids=new_cam_ids,
            frame_ids=new_frame_ids,
            side_ids=new_side_ids,
            raw_histograms=new_raw_histograms,
            temperature_c=new_temperature_c,
            timestamp_s=new_timestamp_s,
            pdc=None, tcm=None, tcl=None,
        )
        new_batch.abs_frame_ids = new_abs_frame_ids
        new_batch.frame_type = new_frame_type
        new_batch.quality = new_quality
        new_batch.events = batch.events
        return new_batch

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Close any open misalignment window + emit end-of-scan summary.

        If frames remain in the buffer at scan end, they were already
        returned (with original timestamps) in the last process() call.
        We record the window stats for the summary log but cannot
        rewrite the already-returned batch.  In practice, the bounded
        buffer (force-flush at max capacity) limits how many frames
        could reach this path -- typically zero.
        """
        if self._buffer and self._window_onset is not None:
            # Close the open window for stats
            self._window_onset.end_fid = self._buffer[-1].abs_frame_id
            self._window_onset.end_t = self._buffer[-1].original_ts
            self._window_onset.n_corrected = len(self._buffer)
            self._scan_windows.append(self._window_onset)
            logger.warning(
                "Misalignment window (scan end): frames %d–%d "
                "(t=%.2f–%.2fs), %d frames un-corrected at scan end, "
                "%d frames NaN-filled",
                self._window_onset.onset_fid, self._window_onset.end_fid,
                self._window_onset.onset_t, self._window_onset.end_t,
                self._window_onset.n_corrected, self._window_onset.n_nan,
            )
            self._buffer.clear()
            self._in_bad_run = False
            self._window_onset = None

        if self._scan_windows:
            total_corrected = sum(
                w.n_corrected for w in self._scan_windows
            )
            total_nan = sum(w.n_nan for w in self._scan_windows)
            pct = (
                (total_corrected + total_nan)
                / max(1, self._total_frames_seen) * 100
            )
            logger.warning(
                "Scan summary: %d misalignment window(s), %d frames "
                "re-timestamped, %d frames NaN-filled "
                "(%.1f%% of scan affected)",
                len(self._scan_windows), total_corrected, total_nan, pct,
            )

    def reset(self) -> None:
        self._reset_state()
