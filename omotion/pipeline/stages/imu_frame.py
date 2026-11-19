"""ImuFrameStage — per-frame IMU motion averages (bloodflow-app#783).

Joins two streams per side:

- IMU samples, pushed from the IMU consumer thread by
  ``omotion.imu_csv.ImuStreamLogger`` (``add_sample``) once each sample's
  scan time is known (same per-side ``timestamp_s`` axis as the histograms);
- histogram frames, seen here on the runner thread as each batch passes
  (``abs_frame_ids`` + repaired ``timestamp_s``, the keys session_data uses).

For each frame of a side it averages the IMU samples with
``prev_frame_t < t <= frame_t`` (the side's previous emitted frame; one
nominal frame period before the first) and appends an ``ImuFrameMotion``
event to the batch, which the runner routes to the ``"imu_frame"`` channel.
Per sample: the accelerometer's ``|a - a_mean|`` against that side's 2 s
running mean (``imu_csv.AccelBaseline``: bias- and orientation-free, ~0 at
rest) and the gyroscope's ``|omega|`` (``imu_csv.gyro_magnitude``); only
these two magnitudes are kept, not the axes.

A frame is emitted once that side's IMU has delivered a sample at or past
the frame's time, or once the frame is more than ``lag_s`` older than the
side's newest frame (IMU late or stopped), or at scan stop. A side that
never delivers an IMU sample (no IMU stream, old firmware) emits nothing.
Warmup and stale frames are skipped, like the live tee.

Stamping the corrected record: every ``IntervalClosed`` event in the batch
(the dark-corrected frames bound for the "final" channel, ~15 s after
capture, and the reduced-mode side-average intervals) gets each frame's
``imu_n_samples`` / ``imu_accel_dev_g`` / ``imu_gyro_dps`` set from the same
per-frame result, keyed by (side, abs_frame_id). ScanDBSink writes them into
session_data, so the IMU sits on the scan rows themselves. Frames of the
interval still waiting for the IMU are averaged then with what has arrived;
frames of a side without IMU stay None. Nothing is delayed for the IMU.

Placed last in the pipeline, so ``timestamp_s`` is post-repair and every
stage's IntervalClosed events are already in ``batch.events``. A failure
here is logged and never drops the batch.
"""

from __future__ import annotations

import logging
import threading
from collections import deque
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from omotion import _log_root
from omotion.imu_csv import (
    ACCEL_LSB_PER_G, GYRO_LSB_PER_DPS, AccelBaseline, gyro_magnitude,
)

from ..batch import FrameBatch, ImuFrameMotion, IntervalClosed

logger = logging.getLogger(
    f"{_log_root}.ImuFrameStage" if _log_root else "ImuFrameStage")

_SIDE_NAMES = {0: "left", 1: "right"}
# Per-frame results kept for stamping: frames this far (in frames) behind the
# side's newest stamped frame are dropped. Intervals close ~15 s (600
# frames) after capture; 2400 leaves ample margin.
_RESULT_KEEP_FRAMES = 2400
_SKIP_FRAME_TYPES = {"warmup", "stale"}

# One IMU sample: (t, accel_dev_g, gyro_dps)
_Sample = Tuple[float, float, float]


def average_imu_samples(side: str, frame_id: int, timestamp_s: float,
                        samples: Sequence[_Sample]) -> ImuFrameMotion:
    """Average already-selected scaled samples into one ImuFrameMotion
    (all values None when there are none)."""
    n = len(samples)
    if n == 0:
        return ImuFrameMotion(side, int(frame_id), float(timestamp_s), 0,
                              None, None)
    return ImuFrameMotion(side, int(frame_id), float(timestamp_s), n,
                          sum(s[1] for s in samples) / n,
                          sum(s[2] for s in samples) / n)


class ImuFrameStage:
    name = "imu_frame"

    def __init__(self, *, lag_s: float = 1.0,
                 nominal_period_s: float = 0.025) -> None:
        self._lag_s = float(lag_s)
        self._period_s = float(nominal_period_s)
        self._lock = threading.Lock()
        self.reset()

    # ── IMU side (any thread) ──────────────────────────────────────────

    def add_sample(self, side: str, timestamp_s: float, sample) -> None:
        """Add one resolved IMU sample (``imu_csv.ImuSample``, raw counts).
        Signature matches ``ImuStreamLogger(sample_fn=...)``."""
        a = [v / ACCEL_LSB_PER_G for v in sample.accel]
        g = [v / GYRO_LSB_PER_DPS for v in sample.gyro]
        with self._lock:
            base = self._baselines.get(side)
            if base is None:
                base = self._baselines[side] = AccelBaseline()
        # Per side, called in time order from that side's IMU thread.
        acc = base.update(float(timestamp_s), a)
        row: _Sample = (float(timestamp_s), acc, gyro_magnitude(g))
        with self._lock:
            self._samples.setdefault(side, deque()).append(row)
            wm = self._watermark.get(side)
            if wm is None or timestamp_s > wm:
                self._watermark[side] = float(timestamp_s)

    # ── Stage protocol (runner thread) ─────────────────────────────────

    def reset(self) -> None:
        with self._lock:
            self._samples: Dict[str, Deque[_Sample]] = {}
            self._watermark: Dict[str, float] = {}
            self._baselines: Dict[str, AccelBaseline] = {}
        self._pending: Dict[str, Dict[int, float]] = {}
        self._last: Dict[str, Tuple[int, float]] = {}
        # (side, abs_frame_id) -> (n_samples, accel_dev_g, gyro_dps)
        self._results: Dict[Tuple[str, int],
                            Tuple[int, Optional[float], Optional[float]]] = {}
        self.emitted = 0
        self.stamped = 0

    def process(self, batch: FrameBatch) -> FrameBatch:
        try:
            self._ingest(batch)
            out = self._emit_ready(final=False)
            out += self._stamp_intervals(batch.events)
            batch.events.extend(out)
        except Exception:
            logger.exception("ImuFrameStage: failed on a batch; skipped")
        return batch

    def on_scan_stop(self, batch: FrameBatch) -> None:
        try:
            out = self._emit_ready(final=True)
            self._stamp_intervals(batch.events)
            batch.events.extend(out)
        except Exception:
            logger.exception("ImuFrameStage: final flush failed")

    # ── internals ──────────────────────────────────────────────────────

    def _ingest(self, batch: FrameBatch) -> None:
        ids = batch.abs_frame_ids
        sides = batch.side_ids
        ts = batch.timestamp_s
        if ids is None or sides is None or ts is None or len(ids) == 0:
            return
        types = batch.frame_type
        for i in range(len(ids)):
            if types is not None and types[i] in _SKIP_FRAME_TYPES:
                continue
            side = _SIDE_NAMES.get(int(sides[i]))
            t = float(ts[i])
            if side is None or not np.isfinite(t):
                continue
            fid = int(ids[i])
            last = self._last.get(side)
            if last is not None and fid <= last[0]:
                continue                       # already emitted (late row)
            self._pending.setdefault(side, {}).setdefault(fid, t)

    def _stamp_intervals(self, events) -> List[ImuFrameMotion]:
        """Set the IMU fields on every frame of the IntervalClosed events in
        ``events``. Returns the ImuFrameMotion events of frames that had to
        be averaged early (still waiting for the IMU) to stamp them."""
        intervals = [getattr(e.corrected_batch, "frames", None)
                     for e in events if isinstance(e, IntervalClosed)]
        intervals = [fr for fr in intervals if fr]
        if not intervals:
            return []
        upto: Dict[str, float] = {}
        for frames in intervals:
            for f in frames:
                side, t = getattr(f, "side", None), getattr(f, "t", None)
                if side in self._pending and t is not None:
                    upto[side] = max(upto.get(side, t), float(t))
        out = self._emit_ready(final=False, force_upto=upto)
        newest: Dict[str, int] = {}
        for frames in intervals:
            for f in frames:
                key = (getattr(f, "side", None), getattr(f, "abs_frame_id", None))
                r = self._results.get(key)
                if r is None:
                    continue
                f.imu_n_samples, f.imu_accel_dev_g, f.imu_gyro_dps = r
                self.stamped += 1
                newest[key[0]] = max(newest.get(key[0], key[1]), key[1])
        if newest:
            self._results = {
                k: v for k, v in self._results.items()
                if k[1] >= newest.get(k[0], k[1]) - _RESULT_KEEP_FRAMES
            }
        return out

    def _emit_ready(self, final: bool,
                    force_upto: Optional[Dict[str, float]] = None,
                    ) -> List[ImuFrameMotion]:
        out: List[ImuFrameMotion] = []
        for side, frames in self._pending.items():
            if not frames:
                continue
            with self._lock:
                wm = self._watermark.get(side)
            order = sorted(frames)
            newest_t = max(frames.values())
            force_t = (force_upto or {}).get(side)
            for fid in order:
                t = frames[fid]
                ready = (final or (wm is not None and wm >= t)
                         or newest_t - t > self._lag_s
                         or (force_t is not None and t <= force_t))
                if not ready:
                    break
                del frames[fid]
                if wm is None:
                    continue                   # this side has no IMU: drop
                out.append(self._average(side, fid, t))
        self.emitted += len(out)
        return out

    def _average(self, side: str, fid: int, t: float) -> ImuFrameMotion:
        last = self._last.get(side)
        lo = last[1] if last is not None and last[1] < t else t - self._period_s
        picked: List[_Sample] = []
        with self._lock:
            q = self._samples.get(side)
            while q and q[0][0] <= t:
                s = q.popleft()
                if s[0] > lo:
                    picked.append(s)
        self._last[side] = (fid, t)
        ev = average_imu_samples(side, fid, t, picked)
        self._results[(side, fid)] = (ev.n_samples, ev.accel_dev_g, ev.gyro_dps)
        return ev
