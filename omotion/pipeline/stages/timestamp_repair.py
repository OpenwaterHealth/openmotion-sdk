"""TimestampRepairStage — EMI timestamp correction + NaN-fill.

Detects EMI-induced timestamp misalignment via two conditions:
  1. Timestamp deviation: |actual_Δt - expected_Δt| > tolerance
  2. In-packet frame_id disagreement: cameras at the same timestamp
     report different frame_ids

Bad frames get their timestamps corrected in-place. Within the batch,
the stage looks ahead for a re-anchor (the next good frame from the
same camera). If no re-anchor exists in the batch, it falls back to
nominal-period interpolation. Missing abs_frame_id gaps get synthetic
NaN-fill rows inserted (the only case that rebuilds the batch).

Misalignment windows are tracked PER SIDE, log one coalesced WARNING
each, and emit a TimestampMisalignmentWindow diagnostics event so the
scan DB's session_meta summary records them. The firmware's terminal
stop frame — the laser-off frame fired ~150 ms off the 25 ms grid at
every scan stop — is recognised at on_scan_stop and reclassified as an
expected artifact (INFO, excluded from the misalignment record).

See docs/superpowers/specs/2026-06-05-eft-timestamp-repair-design.md.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from ..batch import FrameBatch, TimestampMisalignmentWindow

logger = logging.getLogger("openmotion.sdk.pipeline.stages.timestamp_repair")

_INITIAL_NOMINAL_PERIOD_S = 0.025
_DEFAULT_TOLERANCE_S = 0.008
_TOLERANCE_EPS_S = 1e-9
_EMA_ALPHA = 0.01


@dataclass
class _WindowStats:
    """Running stats for one contiguous per-side misalignment window, used
    for coalesced logging: the frame-id/time span it spans, how many frames
    within it were re-timestamped vs NaN-filled, and which (side, cam)
    frames it flagged (for the terminal stop-frame check)."""
    side: int
    onset_fid: int
    onset_t: float
    end_fid: int = 0
    end_t: float = 0.0
    n_corrected: int = 0
    n_nan: int = 0
    # (key, abs_fid) of each flagged frame — consulted at scan stop to
    # recognise the firmware's terminal stop-frame artifact.
    frames: list = field(default_factory=list)


class TimestampRepairStage:
    """Pipeline stage that repairs EMI-corrupted capture timestamps.

    See the module docstring for the algorithm. One instance per scan;
    cross-batch state (nominal-period estimate, per-camera last-good
    anchors, the open misalignment window) persists across process() calls
    and is cleared by reset()."""

    name = "timestamp_repair"

    def __init__(self, *, tolerance_s: float = _DEFAULT_TOLERANCE_S,
                 max_buffer_frames: int = 16):
        """Configure the stage.

        Args:
            tolerance_s: max |actual Δt − expected Δt| (seconds) before a
                frame's timestamp is treated as EMI-corrupted (condition 1).
            max_buffer_frames: reserved bound on the look-ahead used when
                re-anchoring a divergent run.
        """
        self._tolerance = float(tolerance_s)
        self._max_buffer = int(max_buffer_frames)
        self._reset_state()

    def _reset_state(self) -> None:
        """Clear all per-scan state — nominal-period estimate, per-camera
        last-good anchors, the open per-side windows and window list, and the
        running re-timestamped / NaN-filled totals. Called from __init__ and
        reset()."""
        self._nominal_period = _INITIAL_NOMINAL_PERIOD_S
        self._last_good: dict[tuple[int, int], tuple[int, float]] = {}
        self._open_window: dict[int, Optional[_WindowStats]] = {0: None, 1: None}
        self._scan_windows: list[_WindowStats] = []
        self._total_frames_seen = 0
        self._nan_last_seen: dict[tuple[int, int], tuple[int, float]] = {}
        self._total_corrected = 0
        self._total_nan = 0
        self._terminal_frames = 0

    # ── Main entry ──────────────────────────────────────────────────────

    def process(self, batch: FrameBatch) -> FrameBatch:
        """Detect and repair EMI timestamp corruption for one batch.

        Flags each non-warmup/stale frame via condition 2 (in-packet
        frame_id disagreement) or condition 1 (timestamp deviation vs the
        frame_id cadence), rewrites bad timestamps in place by re-anchoring
        interpolation, and inserts synthetic NaN-fill rows for missing
        abs_frame_id gaps. Sets ``batch.quality`` and returns the batch
        (a new, larger batch when NaN-fills were inserted)."""
        if batch.abs_frame_ids is None or batch.frame_type is None:
            return batch

        n = len(batch.cam_ids)
        if n == 0:
            batch.quality = np.empty(0, dtype="<U14")
            return batch

        self._total_frames_seen += n
        quality = np.full(n, "ok", dtype="<U14")

        # Condition 2 (batch-wide): in-packet frame_id disagreement
        bad_cond2 = self._detect_frame_id_disagreement(batch)

        # Pre-build good-frame lookahead for re-anchoring.
        # A frame is "provisionally good" if it passes condition 2.
        # Condition 1 is checked inline below (needs _last_good context).
        good_ahead = self._build_good_lookahead(batch, bad_cond2)

        # Single pass: detect condition 1 inline, correct, track windows
        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype in ("warmup", "stale"):
                continue

            cam_id = int(batch.cam_ids[i])
            side_idx = int(batch.side_ids[i])
            abs_fid = int(batch.abs_frame_ids[i])
            ts = float(batch.timestamp_s[i])
            key = (side_idx, cam_id)

            # Condition 1: timestamp deviation (checked inline with _last_good)
            is_bad = i in bad_cond2
            if not is_bad and key in self._last_good:
                prev_fid, prev_ts = self._last_good[key]
                fid_gap = abs_fid - prev_fid
                if fid_gap > 0:
                    expected_dt = fid_gap * self._nominal_period
                    actual_dt = ts - prev_ts
                    if abs(actual_dt - expected_dt) > self._tolerance + _TOLERANCE_EPS_S:
                        is_bad = True

            if is_bad:
                corrected_ts = self._interpolate(key, abs_fid, good_ahead.get(key))
                batch.timestamp_s[i] = corrected_ts
                quality[i] = "ts_corrected"
                self._total_corrected += 1
                self._track_window_open(side_idx, key, abs_fid, ts)
                self._last_good[key] = (abs_fid, corrected_ts)
            else:
                self._track_window_close(side_idx, batch.events)
                self._update_nominal_period(key, abs_fid, ts)
                self._last_good[key] = (abs_fid, ts)

        batch.quality = quality

        # Insert NaN-fill rows for missing abs_frame_id gaps
        nan_fills = self._collect_nan_fills(batch)
        if nan_fills:
            batch = self._insert_nan_fills(batch, nan_fills)

        return batch

    # ── Detection ───────────────────────────────────────────────────────

    def _detect_frame_id_disagreement(self, batch: FrameBatch) -> set[int]:
        """Condition 2: cameras at the same timestamp with different frame_ids."""
        bad: set[int] = set()
        groups: dict[tuple[int, float], list[int]] = defaultdict(list)
        for i in range(len(batch.cam_ids)):
            ft = str(batch.frame_type[i])
            if ft in ("warmup", "stale"):
                continue
            groups[(int(batch.side_ids[i]), float(batch.timestamp_s[i]))].append(i)
        for indices in groups.values():
            if len(indices) < 2:
                continue
            if len({int(batch.frame_ids[j]) for j in indices}) > 1:
                bad.update(indices)
        return bad

    # ── Look-ahead for re-anchoring ─────────────────────────────────────

    def _build_good_lookahead(self, batch: FrameBatch,
                              bad_set: set[int]) -> dict[tuple[int, int], list]:
        """Collect, per (side, cam), the (abs_fid, ts) of frames not flagged
        by condition 2 and not warmup/stale — the candidate right-anchors
        that ``_interpolate`` searches when re-anchoring a bad frame."""
        ahead: dict[tuple[int, int], list] = defaultdict(list)
        for i in range(len(batch.cam_ids)):
            if i in bad_set:
                continue
            ft = str(batch.frame_type[i])
            if ft in ("warmup", "stale"):
                continue
            key = (int(batch.side_ids[i]), int(batch.cam_ids[i]))
            ahead[key].append((int(batch.abs_frame_ids[i]), float(batch.timestamp_s[i])))
        return ahead

    def _interpolate(self, key: tuple[int, int], abs_fid: int,
                     good_frames: list | None) -> float:
        """Re-anchor a bad frame's timestamp by interpolation.

        Interpolates between the last good timestamp for this (side, cam)
        and the next good one in ``good_frames`` (the within-batch
        look-ahead), distributed by frame_id count. Falls back to the last
        good anchor plus the nominal period when there's no right anchor,
        and to abs_fid × nominal period when there's no anchor at all
        (start of scan)."""
        if key in self._last_good:
            left_fid, left_ts = self._last_good[key]
        else:
            return abs_fid * self._nominal_period

        # Try to find a right anchor from the look-ahead
        if good_frames:
            for right_fid, right_ts in good_frames:
                if right_fid > abs_fid:
                    fid_span = right_fid - left_fid
                    if fid_span > 0:
                        return left_ts + (abs_fid - left_fid) / fid_span * (right_ts - left_ts)

        # Fallback: nominal period from left anchor
        return left_ts + (abs_fid - left_fid) * self._nominal_period

    # ── Nominal period tracking ─────────────────────────────────────────

    def _update_nominal_period(self, key: tuple[int, int],
                               abs_fid: int, ts: float) -> None:
        """Refine the true frame period via an EMA over clean single-step
        intervals (only when this frame is exactly one frame_id past the
        last good frame for ``key``). Tracks the real ~25.02 ms cadence so
        the expected-Δt check in process() doesn't drift off nominal."""
        if key not in self._last_good:
            return
        prev_fid, prev_ts = self._last_good[key]
        if abs_fid - prev_fid == 1:
            dt = ts - prev_ts
            if dt > 0:
                self._nominal_period = (
                    (1 - _EMA_ALPHA) * self._nominal_period + _EMA_ALPHA * dt
                )

    # ── Window tracking (coalesced logging) ─────────────────────────────

    def _track_window_open(self, side: int, key: tuple[int, int],
                           abs_fid: int, ts: float) -> None:
        """Open this side's misalignment window (or extend it) and count the
        re-timestamped frame, for coalesced per-window logging. ``ts`` is the
        original pre-correction device timestamp."""
        w = self._open_window[side]
        if w is None:
            w = _WindowStats(side=side, onset_fid=abs_fid, onset_t=ts)
            self._open_window[side] = w
        w.end_fid = abs_fid
        w.end_t = ts
        w.n_corrected += 1
        w.frames.append((key, abs_fid))

    def _track_window_close(self, side: int, events: list) -> None:
        """Close this side's open misalignment window, if any: record it,
        emit one coalesced WARNING for the whole window (per spec R3 — never
        one line per frame), and append a TimestampMisalignmentWindow event
        so the diagnostics channel / scan-DB summary record it. Called on the
        first good same-side frame after a divergent run."""
        w = self._open_window[side]
        if w is None:
            return
        self._open_window[side] = None
        self._scan_windows.append(w)
        logger.warning(
            "Misalignment window: side=%d frames %d-%d (t=%.2f-%.2fs), "
            "%d frames re-timestamped, %d frames NaN-filled",
            w.side, w.onset_fid, w.end_fid, w.onset_t, w.end_t,
            w.n_corrected, w.n_nan,
        )
        events.append(TimestampMisalignmentWindow(
            side=w.side, onset_fid=w.onset_fid, end_fid=w.end_fid,
            onset_t=w.onset_t, end_t=w.end_t,
            n_corrected=w.n_corrected, n_nan=w.n_nan,
        ))

    def _is_terminal_artifact(self, w: _WindowStats) -> bool:
        """True when a window still open at scan stop is the firmware's
        terminal stop frame: every flagged frame is the LAST frame its
        (side, cam) ever produced, and each camera was flagged exactly once.
        (The laser-off stop frame fires ~150 ms off the 25 ms grid by
        protocol — its timestamp is truthful, just not on the capture
        schedule.) A genuine end-of-scan EMI burst flags multiple frames per
        camera and stays a real window."""
        if not w.frames:
            return False
        seen_keys: set = set()
        for key, fid in w.frames:
            if key in seen_keys:
                return False  # >1 flagged frame for this camera — real run
            seen_keys.add(key)
            last = self._nan_last_seen.get(key)
            if last is None or fid != last[0]:
                return False  # frames followed it — not the terminal frame
        return True

    # ── NaN-fill for missing frames ─────────────────────────────────────

    def _collect_nan_fills(self, batch: FrameBatch) -> list[tuple[int, dict]]:
        """Find gaps in abs_frame_id per (side, cam) and build fill descriptors.

        Returns [(insert_after_idx, fill_dict), ...] sorted by insert position.
        """
        fills = []
        for i in range(len(batch.cam_ids)):
            ft = str(batch.frame_type[i])
            if ft in ("warmup", "stale"):
                continue
            key = (int(batch.side_ids[i]), int(batch.cam_ids[i]))
            abs_fid = int(batch.abs_frame_ids[i])
            ts = float(batch.timestamp_s[i])

            if key in self._nan_last_seen:
                prev_fid, prev_ts = self._nan_last_seen[key]
                gap = abs_fid - prev_fid
                if gap > 1:
                    for fid in range(prev_fid + 1, abs_fid):
                        frac = (fid - prev_fid) / gap
                        fills.append((i, {
                            "cam_id": key[1],
                            "frame_id": fid & 0xFF,
                            "side_idx": key[0],
                            "abs_frame_id": fid,
                            "timestamp_s": prev_ts + frac * (ts - prev_ts),
                            "frame_type": "light",
                            "quality": "nan_filled",
                        }))
                        self._total_nan += 1
                        w = self._open_window[key[0]]
                        if w is not None:
                            w.n_nan += 1

            self._nan_last_seen[key] = (abs_fid, ts)

        fills.sort(key=lambda x: x[0])
        return fills

    def _insert_nan_fills(self, batch: FrameBatch,
                          fills: list[tuple[int, dict]]) -> FrameBatch:
        """Rebuild the batch with NaN-fill rows inserted at the right positions."""
        n_orig = len(batch.cam_ids)
        n_fills = len(fills)
        n_new = n_orig + n_fills

        new_cam = np.zeros(n_new, dtype=np.int8)
        new_fid = np.zeros(n_new, dtype=np.uint8)
        new_sid = np.zeros(n_new, dtype=np.int8)
        new_abs = np.zeros(n_new, dtype=np.int64)
        new_ts = np.zeros(n_new, dtype=np.float64)
        new_ft = np.empty(n_new, dtype="<U14")
        new_q = np.empty(n_new, dtype="<U14")
        new_hist = np.zeros((n_new, 2, 8, 1024), dtype=np.uint32)
        new_temp = np.zeros((n_new, 2, 8), dtype=np.float32)

        # Build insertion map: for each original index, which fills precede it
        fill_before: dict[int, list[dict]] = defaultdict(list)
        for insert_at, fd in fills:
            fill_before[insert_at].append(fd)

        out = 0
        for i in range(n_orig):
            for fd in fill_before.get(i, []):
                new_cam[out] = fd["cam_id"]
                new_fid[out] = fd["frame_id"]
                new_sid[out] = fd["side_idx"]
                new_abs[out] = fd["abs_frame_id"]
                new_ts[out] = fd["timestamp_s"]
                new_ft[out] = fd["frame_type"]
                new_q[out] = fd["quality"]
                out += 1
            new_cam[out] = batch.cam_ids[i]
            new_fid[out] = batch.frame_ids[i]
            new_sid[out] = batch.side_ids[i]
            new_abs[out] = batch.abs_frame_ids[i]
            new_ts[out] = batch.timestamp_s[i]
            new_ft[out] = str(batch.frame_type[i])
            new_q[out] = str(batch.quality[i])
            new_hist[out] = batch.raw_histograms[i]
            new_temp[out] = batch.temperature_c[i]
            out += 1

        new_batch = FrameBatch(
            cam_ids=new_cam, frame_ids=new_fid, side_ids=new_sid,
            raw_histograms=new_hist, temperature_c=new_temp,
            timestamp_s=new_ts, pdc=None, tcm=None, tcl=None,
        )
        new_batch.abs_frame_ids = new_abs
        new_batch.frame_type = new_ft
        new_batch.quality = new_q
        new_batch.events = batch.events
        return new_batch

    # ── Lifecycle ───────────────────────────────────────────────────────

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """End-of-scan lifecycle hook.

        Windows still open at the final frame are either reclassified as the
        expected terminal stop-frame artifact (INFO, excluded from the
        misalignment record — the firmware's laser-off frame fires ~150 ms
        off-grid on every scan) or closed normally with their WARNING +
        diagnostics event. Then the one-line scan summary (real window count,
        frames re-timestamped, NaN-filled, % of scan affected) is emitted
        whenever any genuine correction or fill occurred."""
        for side in (0, 1):
            w = self._open_window[side]
            if w is None:
                continue
            if self._is_terminal_artifact(w):
                self._open_window[side] = None
                self._total_corrected -= w.n_corrected
                self._terminal_frames += w.n_corrected
                logger.info(
                    "Terminal stop frame re-timestamped on side=%d "
                    "(%d camera(s)) — expected firmware stop artifact "
                    "(laser-off frame fires ~150 ms off-grid); not counted "
                    "as misalignment", side, w.n_corrected,
                )
            else:
                self._track_window_close(side, batch.events)

        if self._total_corrected or self._total_nan:
            pct = (self._total_corrected + self._total_nan) / max(1, self._total_frames_seen) * 100
            logger.warning(
                "Scan summary: %d misalignment window(s), %d frames "
                "re-timestamped, %d frames NaN-filled (%.1f%% of scan affected)",
                len(self._scan_windows), self._total_corrected, self._total_nan, pct,
            )

    def reset(self) -> None:
        """Pipeline lifecycle hook — drop all per-scan state so the stage is
        ready for a fresh scan or replay."""
        self._reset_state()
