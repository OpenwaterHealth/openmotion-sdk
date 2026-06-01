"""DarkFrameHoldStage — hold last light values + dark-frame quadratic stencil.

Two responsibilities:

Realtime path: the laser is off during dark frames (the periodic dark-baseline
intervals and the laser-off frame the firmware emits at scan stop), so the
BFI/BVI computed for them are meaningless and would spike the live trace. This
stage replaces each dark frame's per-camera BFI/BVI with the most recent LIGHT
frame's values for that camera, so the live trace holds steady through the dark
interval instead of spiking.

Batch path: for IntervalClosed events carrying EnrichedCorrectedInterval, the
stage applies the 4-point quadratic stencil (§8.4) to compute corrected values
for the leading dark frame D_prev and prepends it to the interval. This gives
the "final" sinks a complete, gapless time series.

Placed AFTER BfiBviStage (so bfi_live/bvi_live and enriched events exist) and
BEFORE SideAverageStage (so the per-side averages reflect the held values).
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..batch import FrameBatch, IntervalClosed
from .dark import (
    DarkFrameQuadraticStencil,
    EnrichedCorrectedFrame, EnrichedCorrectedInterval,
)

# Display metrics held across dark frames. Deliberately excludes
# mean_dc_rt / contrast_sn_rt — those feed downstream correction logic.
_HELD_FIELDS = ("bfi_live", "bvi_live")


class DarkFrameHoldStage:
    name = "dark_frame_hold"

    def __init__(self) -> None:
        # ── Realtime hold state ──
        # (side_idx, cam_id) -> {field_name: last finite light value}
        self._last_light: dict[tuple[int, int], dict[str, float]] = {}

        # ── Batch stencil state ──
        self._stencil = DarkFrameQuadraticStencil()
        # Ring buffer (≤2 entries) of the last two EnrichedCorrectedFrames from
        # the previous interval, keyed by (side, cam_id). Used as the left
        # neighbours v(D-1) and v(D-2) for the quadratic stencil (§8.4).
        self._prev_interval_tail: dict[tuple[str, int], list[EnrichedCorrectedFrame]] = {}

    def process(self, batch: FrameBatch) -> FrameBatch:
        self._process_realtime(batch)
        self._process_events(batch)
        return batch

    # ── Realtime hold ────────────────────────────────────────────────────

    def _process_realtime(self, batch: FrameBatch) -> None:
        ft = batch.frame_type
        if ft is None or batch.bfi_live is None:
            return
        side_ids = batch.side_ids
        cam_ids = batch.cam_ids
        if side_ids is None or cam_ids is None:
            return

        arrays = [(name, getattr(batch, name)) for name in _HELD_FIELDS
                  if getattr(batch, name, None) is not None]

        n = batch.bfi_live.shape[0]
        for i in range(n):
            ftype = str(ft[i])
            s = int(side_ids[i])
            c = int(cam_ids[i])
            if s < 0 or s > 1 or c < 0 or c >= 8:
                continue
            key = (s, c)

            if ftype == "light":
                # Snapshot this camera's finite light values for the hold.
                snap = {}
                for name, arr in arrays:
                    v = float(arr[i, s, c])
                    if np.isfinite(v):
                        snap[name] = v
                if snap:
                    self._last_light[key] = snap
            elif ftype == "dark":
                held = self._last_light.get(key)
                if held is None:
                    continue  # no prior light value to hold yet
                for name, arr in arrays:
                    if name in held:
                        arr[i, s, c] = held[name]

    # ── Batch stencil ────────────────────────────────────────────────────

    def _process_events(self, batch: FrameBatch) -> None:
        """Apply the quadratic stencil for D_prev on EnrichedCorrectedInterval events."""
        for event in batch.events:
            if not isinstance(event, IntervalClosed):
                continue
            eci = event.corrected_batch
            if not isinstance(eci, EnrichedCorrectedInterval):
                continue
            if not eci.frames:
                continue

            # All frames in an interval share (side, cam_id).
            first = eci.frames[0]
            key = (first.side, first.cam_id)

            dark_ef = self._apply_stencil(key, eci.left_abs, eci.left_t, eci.frames)

            # Prepend the dark-frame corrected row (chronological order).
            if dark_ef is not None:
                eci.frames.insert(0, dark_ef)

            # Update the tail for the NEXT interval's stencil (last two light
            # frames, excluding the prepended dark row itself).
            light_frames = [f for f in eci.frames if f.abs_frame_id != eci.left_abs]
            tail = light_frames[-2:] if len(light_frames) >= 2 else light_frames[-1:]
            self._prev_interval_tail[key] = list(tail)

    def _apply_stencil(
        self,
        key: tuple[str, int],
        d_prev_abs: int,
        d_prev_t: float,
        enriched_frames: list[EnrichedCorrectedFrame],
    ) -> Optional[EnrichedCorrectedFrame]:
        """Compute the stencil-interpolated corrected value for the dark frame D_prev.

        Uses the quadratic 4-point stencil (§8.4):
            v(D) = (-1/6)*v(D-2) + (2/3)*v(D-1) + (2/3)*v(D+1) + (-1/6)*v(D+2)

        Left neighbours  v(D-1), v(D-2) come from self._prev_interval_tail[key].
        Right neighbours v(D+1), v(D+2) come from the first two frames of enriched_frames.

        Falls back gracefully when fewer neighbours are available (see
        DarkFrameQuadraticStencil.interpolate_dark_value for the fallback chain).

        Returns None if there is no v(D+1) (interval has no corrected light frames).
        """
        if not enriched_frames:
            return None

        side, cam_id = key
        right1 = enriched_frames[0]
        right2 = enriched_frames[1] if len(enriched_frames) >= 2 else None
        prev_tail = self._prev_interval_tail.get(key, [])
        left1 = prev_tail[-1] if len(prev_tail) >= 1 else None
        left2 = prev_tail[-2] if len(prev_tail) >= 2 else None

        def _interp(attr: str) -> float:
            r1 = getattr(right1, attr)
            r2 = getattr(right2, attr) if right2 is not None else None
            l1 = getattr(left1,  attr) if left1  is not None else None
            l2 = getattr(left2,  attr) if left2  is not None else None
            return self._stencil.interpolate_dark_value(
                v_minus_2=l2, v_minus_1=l1,
                v_plus_1=r1, v_plus_2=r2,
            )

        return EnrichedCorrectedFrame(
            abs_frame_id=d_prev_abs,
            t=d_prev_t,
            side=side,
            cam_id=cam_id,
            mean=_interp("mean"),
            std=_interp("std"),
            contrast=_interp("contrast"),
            bfi=_interp("bfi"),
            bvi=_interp("bvi"),
        )

    # ── Lifecycle ────────────────────────────────────────────────────────

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Process events from the terminal flush."""
        self._process_events(batch)

    def reset(self) -> None:
        self._last_light.clear()
        self._prev_interval_tail.clear()
