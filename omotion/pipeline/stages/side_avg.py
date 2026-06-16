"""SideAverageStage — per-side spatial average for reduced-mode display and DB.

Combines realtime and corrected side averaging into a single stage:

  Realtime path: averages bfi_live/bvi_live per capture as they stream in,
  emits LiveEmit("live_side", SideAverageSample) for the live UI trace.

  Corrected path: reads IntervalClosed events from DarkCorrectionStage,
  groups the per-camera corrected frames by frame_id, spatially averages
  the selected cameras, and emits a synthetic IntervalClosed whose
  EnrichedCorrectedFrames carry ``cam_id=-1`` — the side-average
  convention. These ride the ordinary "final" channel; ScanDBSink
  persists them as the reduced-mode record.

Both paths use spatial_side_average — a purely spatial operation (across
cameras at one instant, no temporal element). Gated on ``enabled``
(reduced mode).

See docs/SciencePipeline.md §16 and
docs/superpowers/specs/2026-05-28-reduced-mode-side-average-design.md.
"""

from __future__ import annotations

import warnings
from typing import Optional

import numpy as np

from ..batch import FrameBatch, IntervalClosed, LiveEmit, SideAverageSample
from .dark import EnrichedCorrectedFrame, EnrichedCorrectedInterval


_SIDE_STR_TO_INT = {"left": 0, "right": 1}
_SIDE_INT_TO_STR = ("left", "right")

# Higher rank = worse quality; the side average inherits the worst quality
# of any camera that contributed to it. Mirrors sinks._QUALITY_RANK.
_QUALITY_RANK = {"ok": 0, "ts_corrected": 1, "nan_filled": 2}


def _mask_to_cam_indices(mask: int) -> np.ndarray:
    return np.array([i for i in range(8) if mask & (1 << i)], dtype=np.int8)


def spatial_side_average(values_by_cam: np.ndarray, cam_indices: np.ndarray) -> float:
    """Nan-aware mean of the SELECTED cameras' values at one capture instant.

    This is the single definition of the reduced-mode side average — a purely
    SPATIAL operation (across cameras at one instant), with no temporal element.
    `values_by_cam` is a per-camera 1-D array (length 8); `cam_indices` selects
    the active cameras. Returns NaN when the selection is empty or every selected
    camera is non-finite."""
    if len(cam_indices) == 0:
        return float("nan")
    selected = np.asarray(values_by_cam)[cam_indices]
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", r"Mean of empty slice",
                                category=RuntimeWarning)
        with np.errstate(invalid="ignore"):
            return float(np.nanmean(selected))


class SideAverageStage:
    """Combined realtime + corrected per-side average for reduced mode.

    Realtime: the live USB path delivers ONE camera per frame row, so a
    capture's cameras (sharing a frame_id) arrive as consecutive rows. A
    per-side accumulator gathers a capture's per-camera BFI/BVI; when the
    next capture begins, it averages the selected cameras and emits a
    LiveEmit(channel="live_side", SideAverageSample). The final open
    capture is flushed at on_scan_stop.

    Corrected: DarkCorrectionStage emits one IntervalClosed per (side, cam)
    when that camera's dark-bounded interval closes. This stage gathers
    those per-camera intervals, groups by frame_id, spatially averages the
    selected cameras, and emits one synthetic IntervalClosed per side whose
    frames are EnrichedCorrectedFrames with cam_id=-1 — routed to "final"
    sinks like any other interval.
    """

    name = "side_average"

    def __init__(self, *, enabled: bool, left_camera_mask: int, right_camera_mask: int):
        self.enabled = bool(enabled)
        self._cams = (_mask_to_cam_indices(left_camera_mask),
                      _mask_to_cam_indices(right_camera_mask))
        self._reset_state()

    def _reset_state(self) -> None:
        # ── Realtime accumulator state ──
        # Per side: the open capture's frame_id (None = none open), its
        # accumulated per-camera BFI/BVI (length-8, NaN = camera not seen this
        # capture), and the capture's timestamp.
        self._open_fid: list[Optional[int]] = [None, None]
        self._acc_bfi = [np.full(8, np.nan), np.full(8, np.nan)]
        self._acc_bvi = [np.full(8, np.nan), np.full(8, np.nan)]
        self._acc_t = [0.0, 0.0]

        # ── Corrected accumulator state ──
        # Per side: accumulated frames keyed by frame_id ->
        # {"t", "bfi"(8), "bvi"(8), "mean"(8), "contrast"(8), "quality"},
        # plus per-camera progress: cam_id -> right_abs of the last interval
        # that camera closed. A capture is flushed once EVERY mask-enabled
        # camera's progress has moved past it (the cross-camera watermark),
        # so each frame_id is emitted exactly once even when one camera's
        # interval spans a missed dark or closes a batch later than its
        # siblings.
        self._frames: list[dict] = [dict(), dict()]
        self._progress: list[dict] = [dict(), dict()]

    def process(self, batch: FrameBatch) -> FrameBatch:
        if not self.enabled:
            return batch
        self._process_realtime(batch)
        self._process_corrected(batch)
        return batch

    # ── Realtime path ────────────────────────────────────────────────────

    def _process_realtime(self, batch: FrameBatch) -> None:
        if batch.bfi_live is None or batch.side_ids is None or batch.cam_ids is None:
            return
        fids = batch.abs_frame_ids if batch.abs_frame_ids is not None else batch.frame_ids
        n = batch.bfi_live.shape[0]
        for i in range(n):
            side = int(batch.side_ids[i])
            cam = int(batch.cam_ids[i])
            if side < 0 or side > 1 or cam < 0 or cam >= 8:
                continue
            fid = int(fids[i])
            if self._open_fid[side] is not None and fid != self._open_fid[side]:
                self._emit_realtime(side, batch)      # previous capture complete
            if self._open_fid[side] != fid:            # start a fresh capture
                self._open_fid[side] = fid
                self._acc_bfi[side][:] = np.nan
                self._acc_bvi[side][:] = np.nan
            self._acc_bfi[side][cam] = float(batch.bfi_live[i, side, cam])
            self._acc_bvi[side][cam] = float(batch.bvi_live[i, side, cam])
            self._acc_t[side] = float(batch.timestamp_s[i])

    def _emit_realtime(self, side: int, batch: FrameBatch) -> None:
        fid = self._open_fid[side]
        if fid is None:
            return
        cams = self._cams[side]
        batch.events.append(LiveEmit(
            channel="live_side",
            payload=SideAverageSample(
                t=self._acc_t[side],
                frame_id=int(fid),
                side=side,
                bfi=spatial_side_average(self._acc_bfi[side], cams),
                bvi=spatial_side_average(self._acc_bvi[side], cams),
            ),
        ))
        self._open_fid[side] = None

    # ── Corrected path ───────────────────────────────────────────────────

    def _process_corrected(self, batch: FrameBatch) -> None:
        # Snapshot: _flush_ready appends our own synthetic IntervalClosed
        # events to batch.events; iterating the live list would re-scan them.
        # (Their cam_id=-1 frames are skipped by the cam-range guard anyway,
        # but a snapshot keeps the traversal well-defined.)
        changed = [False, False]
        for event in list(batch.events):
            if not isinstance(event, IntervalClosed):
                continue
            ci = event.corrected_batch
            frames = getattr(ci, "frames", None)
            if not frames:
                continue
            right_abs = getattr(ci, "right_abs", None)
            for f in frames:
                side = _SIDE_STR_TO_INT.get(getattr(f, "side", None))
                if side is None:
                    continue
                cam = int(getattr(f, "cam_id", -1))
                if cam < 0 or cam >= 8:
                    continue
                self._ingest_corrected(side, cam, f)
                changed[side] = True
                # Track how far this camera has progressed: everything below
                # its interval's right boundary has been delivered.
                if right_abs is not None:
                    prev = self._progress[side].get(cam, -1)
                    if int(right_abs) > prev:
                        self._progress[side][cam] = int(right_abs)

        # Flush AFTER ingesting the whole batch, so sibling cameras whose
        # intervals close in the same batch all contribute before emission.
        for side in (0, 1):
            if changed[side]:
                self._flush_ready(side, batch)

    def _watermark(self, side: int) -> Optional[int]:
        """Highest frame_id (exclusive) that every enabled camera has
        delivered. None until each mask-enabled camera has closed at least
        one interval — flushing earlier would emit partial averages and
        re-emit (duplicate) the frame when the late camera caught up.
        A mask-enabled camera that never streams stalls the corrected side
        average until on_scan_stop, which flushes everything regardless."""
        cams = self._cams[side]
        if len(cams) == 0:
            return None
        progress = self._progress[side]
        wm = None
        for cam in cams:
            p = progress.get(int(cam))
            if p is None:
                return None
            wm = p if wm is None else min(wm, p)
        return wm

    def _flush_ready(self, side: int, batch: FrameBatch) -> None:
        wm = self._watermark(side)
        if wm is None:
            return
        ready = sorted(fid for fid in self._frames[side] if fid < wm)
        if ready:
            self._emit_frames(side, ready, batch, right_abs=wm)

    def _ingest_corrected(self, side: int, cam: int, f) -> None:
        fid = int(getattr(f, "abs_frame_id"))
        rec = self._frames[side].get(fid)
        if rec is None:
            rec = {
                "t": float(getattr(f, "t", 0.0)),
                "bfi": np.full(8, np.nan), "bvi": np.full(8, np.nan),
                "mean": np.full(8, np.nan), "contrast": np.full(8, np.nan),
                "quality": "ok",
            }
            self._frames[side][fid] = rec
        rec["bfi"][cam] = float(getattr(f, "bfi", np.nan))
        rec["bvi"][cam] = float(getattr(f, "bvi", np.nan))
        rec["mean"][cam] = float(getattr(f, "mean", np.nan))
        rec["contrast"][cam] = float(getattr(f, "contrast", np.nan))
        fq = str(getattr(f, "quality", "ok") or "ok")
        if _QUALITY_RANK.get(fq, 0) > _QUALITY_RANK.get(rec["quality"], 0):
            rec["quality"] = fq

    def _emit_frames(self, side: int, fids: list, batch: FrameBatch,
                     *, right_abs: int) -> None:
        """Average and emit the given frame_ids as one synthetic interval of
        cam_id=-1 frames, removing them from the accumulator."""
        cams = self._cams[side]
        avg_frames: list[EnrichedCorrectedFrame] = []
        for fid in fids:
            rec = self._frames[side].pop(fid)
            avg_frames.append(EnrichedCorrectedFrame(
                abs_frame_id=int(fid),
                t=rec["t"],
                side=_SIDE_INT_TO_STR[side],
                cam_id=-1,
                mean=spatial_side_average(rec["mean"], cams),
                std=float("nan"),  # std of a spatial average is undefined here
                contrast=spatial_side_average(rec["contrast"], cams),
                bfi=spatial_side_average(rec["bfi"], cams),
                bvi=spatial_side_average(rec["bvi"], cams),
                quality=rec["quality"],
            ))
        if avg_frames:
            batch.events.append(IntervalClosed(
                corrected_batch=EnrichedCorrectedInterval(
                    left_abs=int(fids[0]),
                    right_abs=int(right_abs),
                    frames=avg_frames,
                )
            ))

    # ── Lifecycle ────────────────────────────────────────────────────────

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Flush the final open realtime captures and every corrected capture
        still below the watermark (including the terminal-flush intervals
        DarkCorrectionStage appends during its own on_scan_stop, which runs
        before this stage's)."""
        if not self.enabled:
            return
        self._process_corrected(batch)
        for side in (0, 1):
            self._emit_realtime(side, batch)
            remaining = sorted(self._frames[side])
            if remaining:
                self._emit_frames(side, remaining, batch,
                                  right_abs=remaining[-1] + 1)

    def reset(self) -> None:
        self._reset_state()


# ── Backward-compatible aliases ──────────────────────────────────────────────
# These allow existing imports / tests to keep working during the transition.
LiveSideAverageStage = SideAverageStage
CorrectedSideAverageStage = SideAverageStage
