"""CorrectedSideAverageStage — dark-corrected per-side average for the DB.

Final-path stage. DarkCorrectionStage emits one IntervalClosed per (side, cam)
when that camera's dark-bounded interval closes, carrying an
EnrichedCorrectedInterval of that camera's corrected frames (BFI/BVI from the
interpolated dark baseline + shot-noise + calibration). This stage gathers
those per-camera intervals across the active cameras of a side, groups them by
frame_id, spatially averages the selected cameras (spatial_side_average), and
emits one LiveEmit(channel="final_side", SideAverageSample) per capture — the
accurate side-average record the scan DB persists at cam_id=-1.

Purely spatial (group-by-frame_id across cameras); the cross-camera / -event
gathering is plumbing. Gated on reduced mode. A side's pending interval window
finalizes when the next window for that side begins; any remaining windows flush
at on_scan_stop. Live display uses the realtime average (LiveSideAverageStage);
this corrected average is the persisted record — the two differ by design.

See docs/superpowers/specs/2026-05-28-reduced-mode-side-average-design.md.
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from ..batch import FrameBatch, IntervalClosed, LiveEmit, SideAverageSample
from .side_avg import _mask_to_cam_indices, spatial_side_average


_SIDE_STR_TO_INT = {"left": 0, "right": 1}


class CorrectedSideAverageStage:
    name = "corrected_side_average"

    def __init__(self, *, enabled: bool, left_camera_mask: int, right_camera_mask: int):
        self.enabled = bool(enabled)
        self._cams = (_mask_to_cam_indices(left_camera_mask),
                      _mask_to_cam_indices(right_camera_mask))
        self._reset_state()

    def _reset_state(self) -> None:
        # Per side: the bounds (left_abs, right_abs) of the interval window
        # currently accumulating (None = none open), and that window's frames
        # keyed by frame_id -> {"t", "bfi"(8), "bvi"(8), "mean"(8), "contrast"(8)}.
        self._window: list[Optional[tuple]] = [None, None]
        self._frames: list[dict] = [dict(), dict()]

    def process(self, batch: FrameBatch) -> FrameBatch:
        if not self.enabled:
            return batch
        for event in batch.events:
            if not isinstance(event, IntervalClosed):
                continue
            ci = event.corrected_batch
            frames = getattr(ci, "frames", None)
            if not frames:
                continue
            bounds = (getattr(ci, "left_abs", None), getattr(ci, "right_abs", None))
            for f in frames:
                side = _SIDE_STR_TO_INT.get(getattr(f, "side", None))
                if side is None:
                    continue
                cam = int(getattr(f, "cam_id", -1))
                if cam < 0 or cam >= 8:
                    continue
                # A new interval window for this side → finalize the previous.
                if self._window[side] is not None and bounds != self._window[side]:
                    self._emit_window(side, batch)
                if self._window[side] != bounds:
                    self._window[side] = bounds
                    self._frames[side] = {}
                self._ingest(side, cam, f)
        return batch

    def _ingest(self, side: int, cam: int, f) -> None:
        fid = int(getattr(f, "abs_frame_id"))
        rec = self._frames[side].get(fid)
        if rec is None:
            rec = {
                "t": float(getattr(f, "t", 0.0)),
                "bfi": np.full(8, np.nan), "bvi": np.full(8, np.nan),
                "mean": np.full(8, np.nan), "contrast": np.full(8, np.nan),
            }
            self._frames[side][fid] = rec
        rec["bfi"][cam] = float(getattr(f, "bfi", np.nan))
        rec["bvi"][cam] = float(getattr(f, "bvi", np.nan))
        rec["mean"][cam] = float(getattr(f, "mean", np.nan))
        rec["contrast"][cam] = float(getattr(f, "contrast", np.nan))

    def _emit_window(self, side: int, batch: FrameBatch) -> None:
        cams = self._cams[side]
        for fid in sorted(self._frames[side]):
            rec = self._frames[side][fid]
            batch.events.append(LiveEmit(
                channel="final_side",
                payload=SideAverageSample(
                    t=rec["t"], frame_id=int(fid), side=side,
                    bfi=spatial_side_average(rec["bfi"], cams),
                    bvi=spatial_side_average(rec["bvi"], cams),
                    mean=spatial_side_average(rec["mean"], cams),
                    contrast=spatial_side_average(rec["contrast"], cams),
                ),
            ))
        self._window[side] = None
        self._frames[side] = {}

    def on_scan_stop(self, batch: FrameBatch) -> None:
        for side in (0, 1):
            if self._window[side] is not None:
                self._emit_window(side, batch)

    def reset(self) -> None:
        self._reset_state()
