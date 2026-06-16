"""Dark correction subsystem.

Built up across 5 tasks:
    Task 14: DarkHistory          — ring buffer of recent dark observations
    Task 15: DarkIntegrityGuard   — u1 > pedestal+threshold check
    Task 16: HybridRealtimePredictor — avg-of-3 u1 + linear-extrap std + ZOH
    Task 17: PendingInterval, LinearInterpolation, DarkFrameQuadraticStencil
    Task 18: DarkCorrectionStage  — orchestrator (NOT in this batch)
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import logging
from typing import Any, Deque, Optional

import numpy as np

from ..batch import DarkIntegrityWarning, FrameBatch, IntervalClosed, TerminalDarkResult
from ..pedestal import SensorPedestals


logger = logging.getLogger("openmotion.sdk.pipeline.stages.dark")

# Maps the console's fsync pulse count (OW_CTRL_GET_FSYNC, read by the
# workflow after stop_trigger) onto the camera-side abs_frame_id of the
# terminal laser-off frame: expected_abs = count + offset. Confirmed exact
# on HIL 2026-06-11: every live camera across four scans (counts 47 to
# 20851) matched abs_id == count. A camera whose last buffered frame is
# older than the count stopped delivering before scan end (dropout); the
# flush logs that and falls back to content-based detection per camera.
_TERMINAL_FSYNC_ABS_OFFSET = 0


@dataclass(frozen=True)
class DarkObservation:
    """One dark-frame measurement for one camera."""
    t:   float
    u1:  float
    std: float


class DarkHistory:
    """Per-(side, cam) ring buffer of recent DarkObservations."""

    def __init__(self, max_darks: int = 4):
        if max_darks < 1:
            raise ValueError(f"max_darks must be >= 1, got {max_darks}")
        self._max = int(max_darks)
        self._rings: dict[tuple[str, int], Deque[DarkObservation]] = {}

    def append(self, side: str, cam_id: int, *, t: float, u1: float, std: float) -> None:
        key = (side, int(cam_id))
        ring = self._rings.get(key)
        if ring is None:
            ring = deque(maxlen=self._max)
            self._rings[key] = ring
        ring.append(DarkObservation(t=float(t), u1=float(u1), std=float(std)))

    def recent(self, side: str, cam_id: int, n: int) -> list[DarkObservation]:
        """Return up to the most recent n entries in chronological order."""
        ring = self._rings.get((side, int(cam_id)))
        if ring is None:
            return []
        if n >= len(ring):
            return list(ring)
        return list(ring)[-n:]

    def size(self, side: str, cam_id: int) -> int:
        ring = self._rings.get((side, int(cam_id)))
        return 0 if ring is None else len(ring)

    def is_empty(self, side: str, cam_id: int) -> bool:
        return self.size(side, cam_id) == 0

    def clear(self) -> None:
        self._rings.clear()


class DarkIntegrityGuard:
    """Flag dark frames whose u1 looks suspiciously bright.

    A genuine dark frame should have u1 within ~5 DN of the sensor pedestal.
    Higher u1 means the dark reference is contaminated — either the laser
    wasn't actually off (firmware off-by-one / fsync misalignment) or ambient
    light is leaking onto the sensor. The guard appends a diagnostic event and
    logs a WARNING, but does not drop the frame.

    See docs/SciencePipeline.md §11 (input validation rails).
    """

    def __init__(self, max_above_pedestal: float = 5.0):
        self.max_above_pedestal = float(max_above_pedestal)

    def check(self, *, side: str, cam_id: int, abs_frame_id: int,
              u1: float, pedestal: float, events: list) -> bool:
        """Return True if the dark frame passes; False if it failed (warning emitted)."""
        threshold = pedestal + self.max_above_pedestal
        if u1 > threshold:
            events.append(DarkIntegrityWarning(
                side=side, cam_id=int(cam_id), abs_frame_id=int(abs_frame_id),
                u1=float(u1), pedestal=float(pedestal),
                threshold=float(threshold),
            ))
            logger.warning(
                "dark frame brighter than expected: side=%s cam=%d abs_id=%d "
                "u1=%.1f exceeds pedestal+%.1f=%.1f — dark reference contaminated "
                "(laser on for this frame from trigger/fsync misalignment, or "
                "ambient light leaking onto the sensor); dark correction will be skewed.",
                side, int(cam_id), int(abs_frame_id),
                float(u1), self.max_above_pedestal, float(threshold),
            )
            return False
        return True


class HybridRealtimePredictor:
    """Realtime dark-baseline predictor.

    Algorithm (see docs/SciencePipeline.md §7.4.1):
        u1   ← average of last 3 dark observations (truncated; ZOH with 1)
        std  ← linear extrapolation through last 2 darks; ZOH with 1 or
                when both darks share a timestamp

    Returns None when no darks have been observed yet — caller skips
    realtime emission for that frame (warmup window).
    """

    def predict(self, side: str, cam_id: int, *, history: DarkHistory,
                target_t: float) -> Optional[tuple[float, float]]:
        recent = history.recent(side, cam_id, n=3)
        if not recent:
            return None

        u1_pred = sum(o.u1 for o in recent) / len(recent)

        if len(recent) < 2 or history.size(side, cam_id) < 2:
            std_pred = recent[-1].std
        else:
            last2 = history.recent(side, cam_id, n=2)
            a, b = last2
            dt = b.t - a.t
            if dt <= 0:
                std_pred = b.std
            else:
                slope = (b.std - a.std) / dt
                std_pred = b.std + slope * (target_t - b.t)

        return (float(u1_pred), float(std_pred))


@dataclass
class _LightSample:
    abs_frame_id: int
    t: float
    u1: float
    u2: float
    quality: str = "ok"


@dataclass
class _DarkBoundary:
    obs: DarkObservation
    abs_frame_id: int


@dataclass
class Interval:
    """A closed dark-bounded interval, ready for batch correction."""
    left:         _DarkBoundary
    right:        _DarkBoundary
    light_frames: list[_LightSample]

    @property
    def left_abs(self) -> int:
        return self.left.abs_frame_id

    @property
    def right_abs(self) -> int:
        return self.right.abs_frame_id


@dataclass
class CorrectedFrame:
    """One corrected sample, output of batch correction.

    After DarkCorrectionStage emits this, downstream stages mutate it:
      ShotNoiseCorrectionStage: overwrites std, sets contrast
      BfiBviStage: reads contrast/mean to produce EnrichedCorrectedFrame
    """
    abs_frame_id: int
    t:             float
    side:          str       # "left" or "right"
    cam_id:        int       # 0..7
    mean:          float     # dark-subtracted u1 (no shot-noise yet)
    std:           float     # dark-subtracted std (no shot-noise yet)
    raw_u1:        float     # original raw mean (for shot-noise use by downstream)
    raw_var:       float     # u2 - u1^2 (raw variance before dark sub)
    dark_var:      float     # interpolated dark baseline variance
    contrast:      Optional[float] = None  # set by ShotNoiseCorrectionStage
    quality:       str = "ok"


@dataclass
class CorrectedInterval:
    """Output of batch correction."""
    left_abs:  int
    right_abs: int
    frames:    list[CorrectedFrame]
    left_t:    float = 0.0   # timestamp of the left dark boundary (for stencil)


@dataclass
class EnrichedCorrectedFrame:
    """CorrectedFrame after shot-noise correction + BFI/BVI calibration."""
    abs_frame_id: int
    t:        float
    side:     str
    cam_id:   int
    mean:     float
    std:      float     # post-shot-noise std
    contrast: float
    bfi:      float
    bvi:      float
    quality:  str = "ok"


@dataclass
class EnrichedCorrectedInterval:
    """Output of the enrichment pass (parallel to CorrectedInterval)."""
    left_abs:  int
    right_abs: int
    frames:    list[EnrichedCorrectedFrame]
    left_t:    float = 0.0   # timestamp of the left dark boundary (for stencil)


class PendingInterval:
    """Buffers non-dark frames between two bounding darks."""

    def __init__(self):
        self._left:  Optional[_DarkBoundary] = None
        self._right: Optional[_DarkBoundary] = None
        self._light: list[_LightSample] = []

    def set_left_dark(self, obs: DarkObservation, *, abs_frame_id: int) -> None:
        self._left = _DarkBoundary(obs=obs, abs_frame_id=int(abs_frame_id))
        self._light = []
        self._right = None

    def add_light(self, *, abs_frame_id: int, t: float, u1: float, u2: float,
                  quality: str = "ok") -> None:
        self._light.append(_LightSample(
            abs_frame_id=int(abs_frame_id), t=float(t), u1=float(u1), u2=float(u2),
            quality=str(quality),
        ))

    def set_right_dark(self, obs: DarkObservation, *, abs_frame_id: int) -> None:
        self._right = _DarkBoundary(obs=obs, abs_frame_id=int(abs_frame_id))

    def is_closed(self) -> bool:
        return self._left is not None and self._right is not None

    def flush(self) -> Interval:
        """Return the closed interval; reset for next interval."""
        assert self.is_closed(), "flush() called on non-closed interval"
        interval = Interval(
            left=self._left,
            right=self._right,
            light_frames=list(self._light),
        )
        self._left = self._right
        self._right = None
        self._light = []
        return interval


class LinearInterpolation:
    """Compute corrected values for a closed dark-bounded interval.

    See docs/SciencePipeline.md §8.1–§8.3.
    """

    def correct_interval(self, interval: Interval, *,
                         side: str, cam_id: int) -> CorrectedInterval:
        d_prev = interval.left.obs
        d_next = interval.right.obs
        d_abs = interval.right_abs - interval.left_abs

        corrected_frames: list[CorrectedFrame] = []
        for lf in interval.light_frames:
            t_frac = ((lf.abs_frame_id - interval.left_abs) / d_abs
                      if d_abs > 0 else 0.0)
            baseline_u1  = d_prev.u1  + t_frac * (d_next.u1  - d_prev.u1)
            baseline_var = d_prev.std ** 2 + t_frac * (d_next.std ** 2 - d_prev.std ** 2)

            mean = lf.u1 - baseline_u1
            raw_var = lf.u2 - lf.u1 ** 2
            if raw_var < 0:
                logger.debug(
                    "batch raw variance clamped negative (float rounding): "
                    "side=%s cam=%d abs_id=%d raw_var=%.6f",
                    side, cam_id, lf.abs_frame_id, raw_var,
                )
                raw_var = 0.0
            corrected_var = raw_var - baseline_var
            if corrected_var < 0:
                logger.debug(
                    "batch dark sub clamped negative variance: "
                    "side=%s cam=%d abs_id=%d raw_var=%.3f "
                    "baseline_var=%.3f deficit=%.3f",
                    side, cam_id, lf.abs_frame_id, raw_var,
                    baseline_var, -corrected_var,
                )
                corrected_var = 0.0
            std = float(corrected_var ** 0.5)

            corrected_frames.append(CorrectedFrame(
                abs_frame_id=lf.abs_frame_id, t=lf.t,
                side=side, cam_id=cam_id,
                mean=mean, std=std,
                raw_u1=lf.u1, raw_var=raw_var, dark_var=baseline_var,
                quality=lf.quality,
            ))

        return CorrectedInterval(
            left_abs=interval.left_abs, right_abs=interval.right_abs,
            frames=corrected_frames,
        )


class DarkFrameQuadraticStencil:
    """4-point quadratic interpolation for the dark frame's own corrected value.

    Stencil:
        v(D) = (-1/6) v(D-2) + (2/3) v(D-1) + (2/3) v(D+1) + (-1/6) v(D+2)

    Fallback chain (see SciencePipeline.md §8.4):
        full        — all four neighbours present
        right_only  — left missing, right ≥2 → (v(+1) + v(+2)) / 2
        simple_avg  — only v(-1) and v(+1) → (v(-1) + v(+1)) / 2
        repeat_right — only v(+1) → v(+1)
    """

    def interpolate_dark_value(self, *,
                               v_minus_2: Optional[float],
                               v_minus_1: Optional[float],
                               v_plus_1: Optional[float],
                               v_plus_2: Optional[float]) -> float:
        if all(v is not None for v in (v_minus_2, v_minus_1, v_plus_1, v_plus_2)):
            return (-1/6) * v_minus_2 + (2/3) * v_minus_1 \
                + (2/3) * v_plus_1 + (-1/6) * v_plus_2

        if v_minus_1 is None and v_plus_1 is not None and v_plus_2 is not None:
            return (v_plus_1 + v_plus_2) / 2

        if v_minus_1 is not None and v_plus_1 is not None:
            return (v_minus_1 + v_plus_1) / 2

        if v_plus_1 is not None:
            return v_plus_1

        raise ValueError("DarkFrameQuadraticStencil needs at least v_plus_1")


class DarkCorrectionStage:
    """Orchestrates the dual-output dark correction.

    Per non-dark frame: populates batch.dark_baseline_rt, batch.mean_dc_rt,
    batch.std_dc_rt using HybridRealtimePredictor (NaN where no prediction
    is available — the warmup window).

    Per dark frame: runs the integrity guard, appends to DarkHistory + the
    appropriate PendingInterval, and emits an IntervalClosed event when an
    interval bookends.

    The emitted IntervalClosed carries a raw CorrectedInterval — downstream
    stages (ShotNoiseCorrectionStage, BfiBviStage, DarkFrameHoldStage)
    handle enrichment and the dark-frame quadratic stencil.

    on_scan_stop(batch) performs the terminal-dark flush — per §8.6, the
    last buffered light frame is the firmware-guaranteed terminal dark frame;
    it is promoted to a dark boundary, removed from the light list, and the
    remaining lights (if any) are emitted.

    See docs/SciencePipeline.md §7.4 (realtime) and §8 (batched).
    """
    name = "dark_correction"

    SIDE_NAMES = ("left", "right")

    def __init__(self, *,
                 realtime_estimator: HybridRealtimePredictor,
                 batch_estimator: LinearInterpolation,
                 pedestals: Optional[SensorPedestals] = None,
                 realtime_history_size: int = 4,
                 integrity_max_above_pedestal: float = 5.0):
        self._realtime = realtime_estimator
        self._batch = batch_estimator
        self._pedestals = pedestals or SensorPedestals(left=64.0, right=64.0)
        self._history = DarkHistory(max_darks=realtime_history_size)
        self._pending: dict[tuple[str, int], PendingInterval] = {}
        self._guard = DarkIntegrityGuard(
            max_above_pedestal=integrity_max_above_pedestal
        )
        self._last_realtime: dict[tuple[str, int], tuple[float, float, float]] = {}
        self._terminal_fsync_count: Optional[int] = None

    def set_terminal_fsync_count(self, count: int) -> None:
        """Ground-truth index of the final FSYNC pulse, from the console
        firmware (OW_CTRL_GET_FSYNC). Trigger_Stop hard-disables the laser
        timer before the deferred final pulse fires, so the frame captured
        on that pulse is laser-off by construction — this count identifies
        the terminal dark positively; content is demoted to verification.

        Called from the workflow teardown thread after stop_trigger; read
        in on_scan_stop, which runs strictly after source close, so plain
        assignment is safely ordered."""
        self._terminal_fsync_count = int(count)

    def _emit_interval(
        self,
        key: "tuple[str, int]",
        interval: "Interval",
        events: list,
    ) -> None:
        """Correct interval and emit IntervalClosed with raw CorrectedInterval.

        Downstream stages handle shot-noise, BFI/BVI, and the dark-frame
        quadratic stencil.
        """
        side, cam_id = key
        corrected = self._batch.correct_interval(interval, side=side, cam_id=cam_id)
        corrected.left_t = interval.left.obs.t
        events.append(IntervalClosed(corrected_batch=corrected))

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.frame_ids.shape[0]
        baseline_rt = np.full((n, 2, 8), np.nan, dtype=np.float32)
        mean_dc_rt  = np.full((n, 2, 8), np.nan, dtype=np.float32)
        std_dc_rt   = np.full((n, 2, 8), np.nan, dtype=np.float32)

        for i in range(n):
            ftype = str(batch.frame_type[i])
            if ftype not in ("light", "dark"):
                continue
            cam_id  = int(batch.cam_ids[i])
            abs_id  = int(batch.abs_frame_ids[i])
            t       = float(batch.timestamp_s[i])
            # Read side from the per-row side_ids set by the source.
            # See FrameBatch.side_ids docstring for why inference from
            # raw_histograms is unsafe (zero-filled rows misroute to side 0).
            side_idx = int(batch.side_ids[i])
            side = self.SIDE_NAMES[side_idx]

            u1 = float(batch.mean_raw[i, side_idx, cam_id])
            std = float(batch.std_raw[i, side_idx, cam_id])

            if ftype == "dark":
                last_rt = self._last_realtime.get((side, cam_id))
                if last_rt is not None:
                    u1_hat, mean_rt, std_rt = last_rt
                    baseline_rt[i, side_idx, cam_id] = np.float32(u1_hat)
                    mean_dc_rt[i, side_idx, cam_id] = np.float32(mean_rt)
                    std_dc_rt[i, side_idx, cam_id] = np.float32(std_rt)

                pedestal = (self._pedestals.left if side == "left"
                            else self._pedestals.right)
                self._guard.check(
                    side=side, cam_id=cam_id, abs_frame_id=abs_id,
                    u1=u1, pedestal=pedestal, events=batch.events,
                )
                self._history.append(side, cam_id, t=t, u1=u1, std=std)

                pi = self._pending.get((side, cam_id))
                if pi is None:
                    pi = PendingInterval()
                    self._pending[(side, cam_id)] = pi
                    pi.set_left_dark(DarkObservation(t=t, u1=u1, std=std),
                                     abs_frame_id=abs_id)
                else:
                    pi.set_right_dark(DarkObservation(t=t, u1=u1, std=std),
                                      abs_frame_id=abs_id)
                    if pi.is_closed():
                        interval = pi.flush()
                        # After flush, pi's left has rolled to the just-flushed right.
                        # _emit_interval applies the stencil for D_prev and appends
                        # the IntervalClosed event (§8.4).
                        self._emit_interval((side, cam_id), interval, batch.events)

            else:  # light
                # Dark-like "light" frame — the laser was actually off; most
                # commonly the firmware's terminal laser-off frame at scan
                # stop, which never falls on a scheduled dark position so the
                # classifier types it "light". Realtime dark subtraction
                # would emit a near-zero mean and a garbage contrast that
                # rails the live BFI/BVI display, so suppress realtime
                # emission (row stays NaN) and keep _last_realtime on the
                # last genuine light. Same threshold as the on_scan_stop tail
                # detection, which still needs the frame buffered below for
                # the batch path.
                pedestal = (self._pedestals.left if side == "left"
                            else self._pedestals.right)
                dark_like = u1 <= pedestal + self._guard.max_above_pedestal

                pred = None
                if not dark_like:
                    pred = self._realtime.predict(
                        side, cam_id, history=self._history, target_t=t,
                    )
                if pred is not None:
                    u1_hat, std_hat = pred
                    baseline_rt[i, side_idx, cam_id] = np.float32(u1_hat)
                    mean_dc_rt[i, side_idx, cam_id]  = np.float32(u1 - u1_hat)
                    raw_var = std ** 2
                    corr_var = raw_var - std_hat ** 2
                    if corr_var < 0:
                        logger.debug(
                            "realtime dark sub clamped negative variance: "
                            "side=%s cam=%d abs_id=%d raw_var=%.3f "
                            "dark_var=%.3f deficit=%.3f",
                            side, cam_id, abs_id, raw_var,
                            std_hat ** 2, -corr_var,
                        )
                        corr_var = 0.0
                    std_dc_rt[i, side_idx, cam_id] = np.float32(corr_var ** 0.5)
                    self._last_realtime[(side, cam_id)] = (
                        float(u1_hat),
                        float(mean_dc_rt[i, side_idx, cam_id]),
                        float(std_dc_rt[i, side_idx, cam_id]),
                    )

                # Buffer for the batch path even when realtime emission was
                # suppressed (dark-like frame) — the terminal flush finds the
                # dark-like tail in the pending light list.
                pi = self._pending.get((side, cam_id))
                if pi is not None:
                    u2 = std ** 2 + u1 ** 2
                    q = str(batch.quality[i]) if batch.quality is not None else "ok"
                    pi.add_light(abs_frame_id=abs_id, t=t, u1=u1, u2=u2,
                                 quality=q)

        batch.dark_baseline_rt = baseline_rt
        batch.mean_dc_rt = mean_dc_rt
        batch.std_dc_rt = std_dc_rt
        return batch

    def reset(self) -> None:
        self._history.clear()
        self._pending.clear()
        self._last_realtime.clear()
        self._terminal_fsync_count = None

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Terminal dark flush — see SciencePipeline.md §8.6.

        The firmware guarantees the end of every scan contains a dark (laser-
        off) frame.  That frame may not fall on a scheduled dark position, so
        the pipeline receives it as a buffered light in pi._light.  Host-side
        trigger-stop drain can deliver a short dark-like tail; all contiguous
        dark-like frames at the end are removed from the light list.

        Following the legacy SciencePipeline._flush_terminal_dark logic:
          0. When the workflow reported the firmware's final fsync pulse
             index (set_terminal_fsync_count), identify the terminal frame
             by abs_frame_id — ground truth from the chip that fired the
             pulse — and use content only to VERIFY it is dark-like. A
             count that matches no buffered frame falls back to step 1.
          1. Find the trailing dark-like tail in pi._light (content-based,
             used when no fsync count is available or it didn't match).
          2. Use the last tail frame's actual data (not the last scheduled dark's)
             as the right boundary of the synthetic interval.
          3. Remove the whole tail from the light list so it is not double-counted.
          4. Call _emit_interval with the remaining lights (which may be empty).
             The stencil for D_prev is applied as normal; if there are no lights,
             D_prev cannot be stencilled (no right neighbours) and is skipped.
        """
        expected_abs = (
            None if self._terminal_fsync_count is None
            else self._terminal_fsync_count + _TERMINAL_FSYNC_ABS_OFFSET
        )

        for (side, cam_id), pi in self._pending.items():
            if not pi._light:
                continue
            if self._history.size(side, cam_id) < 1:
                continue

            pedestal = self._pedestals.left if side == "left" else self._pedestals.right
            threshold = pedestal + self._guard.max_above_pedestal

            def _is_dark_like(light_frame) -> bool:
                return light_frame.u1 <= threshold

            # ── Step 0: positive identification by fsync count ────────────
            identified_by = "content"
            if expected_abs is not None:
                match_idx = next(
                    (j for j in range(len(pi._light) - 1, -1, -1)
                     if pi._light[j].abs_frame_id == expected_abs),
                    None,
                )
                if match_idx is None:
                    logger.warning(
                        "terminal fsync count %d names abs_id %d but side=%s "
                        "cam_id=%d last delivered abs_id=%d — camera stopped "
                        "delivering %d frames before scan end (dropout); "
                        "falling back to content-based detection",
                        self._terminal_fsync_count, expected_abs, side, cam_id,
                        pi._light[-1].abs_frame_id,
                        expected_abs - pi._light[-1].abs_frame_id,
                    )
                else:
                    identified_by = "fsync"
                    if match_idx != len(pi._light) - 1:
                        # No pulse can follow the final pulse — anything after
                        # it is drain garbage; drop it with the tail.
                        logger.warning(
                            "%d buffered frames follow the firmware-identified "
                            "terminal frame abs_id=%d for side=%s cam_id=%d — "
                            "dropping them",
                            len(pi._light) - 1 - match_idx, expected_abs,
                            side, cam_id,
                        )
                        pi._light = pi._light[:match_idx + 1]

            # The terminal candidate must be dark-like. If not, leave the
            # interval open and log loudly: with an fsync identification the
            # laser was demonstrably ON during the final pulse (trigger/laser
            # fault); without one, the firmware did not produce the expected
            # laser-off frame at all.
            terminal_light = pi._light[-1]
            if not _is_dark_like(terminal_light):
                if identified_by == "fsync":
                    logger.error(
                        "TERMINAL DARK CONTAMINATED: side=%s cam_id=%d — "
                        "firmware reports abs_id=%d was the final (laser-off) "
                        "pulse but u1=%.3f exceeds dark threshold %.3f; the "
                        "laser appears to have been on during the terminal "
                        "pulse. Interval left open; corrected data for this "
                        "interval is lost.",
                        side, cam_id, terminal_light.abs_frame_id,
                        terminal_light.u1, threshold,
                    )
                else:
                    logger.error(
                        "TERMINAL DARK MISSING: side=%s cam_id=%d — "
                        "last frame abs_id=%d u1=%.3f exceeds dark threshold %.3f. "
                        "Firmware did not produce a laser-off frame at scan stop. "
                        "Interval left open; corrected data for this interval is lost.",
                        side, cam_id, terminal_light.abs_frame_id,
                        terminal_light.u1, threshold,
                    )
                batch.events.append(TerminalDarkResult(
                    side=side, cam_id=cam_id,
                    abs_frame_id=terminal_light.abs_frame_id,
                    u1=terminal_light.u1, threshold=threshold,
                    found=False, identified_by=identified_by,
                ))
                continue

            logger.info(
                "terminal dark confirmed (%s): side=%s cam_id=%d "
                "abs_id=%d u1=%.3f (threshold=%.3f)",
                identified_by, side, cam_id, terminal_light.abs_frame_id,
                terminal_light.u1, threshold,
            )
            batch.events.append(TerminalDarkResult(
                side=side, cam_id=cam_id,
                abs_frame_id=terminal_light.abs_frame_id,
                u1=terminal_light.u1, threshold=threshold,
                found=True, identified_by=identified_by,
            ))

            tail_start = len(pi._light) - 1
            while tail_start > 0 and _is_dark_like(pi._light[tail_start - 1]):
                tail_start -= 1

            terminal_var = max(0.0, terminal_light.u2 - terminal_light.u1 ** 2)
            terminal_obs = DarkObservation(
                t=terminal_light.t,
                u1=terminal_light.u1,
                std=terminal_var ** 0.5,
            )

            # Remove the terminal dark-like tail from pi._light so those frames
            # are not emitted as corrected light frames.
            tail_len = len(pi._light) - tail_start
            if tail_len > 1:
                logger.info(
                    "terminal dark flush removed %d trailing dark-like frames "
                    "for side=%s cam_id=%d",
                    tail_len, side, cam_id,
                )
            pi._light = pi._light[:tail_start]

            # Close the synthetic interval [D_prev, terminal_dark].
            pi.set_right_dark(terminal_obs, abs_frame_id=terminal_light.abs_frame_id)
            interval = pi.flush()

            # _emit_interval applies the stencil for D_prev and emits the event.
            self._emit_interval((side, cam_id), interval, batch.events)
