"""BfiBviStage — affine calibration map (contrast, mean) → (BFI, BVI).

Operates on two paths:
  Realtime: batch fields contrast_sn_rt / mean_dc_rt → bfi_live, bvi_live
  Batch:    IntervalClosed events carrying CorrectedInterval (after shot-noise)
            → replaces with EnrichedCorrectedInterval carrying per-frame BFI/BVI.

See docs/SciencePipeline.md §9:
    BFI = (1 - (K - C_min) / (C_max - C_min)) * 10
    BVI = (1 - (mean - I_min) / (I_max - I_min)) * 10

Fallback: identity scaling (K * 10, mean * 10) when calibration span is zero.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..batch import FrameBatch, IntervalClosed
from .dark import (
    CorrectedInterval, EnrichedCorrectedFrame, EnrichedCorrectedInterval,
)


class BfiBviStage:
    name = "bfi_bvi"

    def __init__(self, calibration: Any):
        """`calibration` must expose c_min, c_max, i_min, i_max as (2, 8) ndarrays."""
        self._c_min = np.asarray(calibration.c_min, dtype=np.float32).reshape(1, 2, 8)
        self._c_max = np.asarray(calibration.c_max, dtype=np.float32).reshape(1, 2, 8)
        self._i_min = np.asarray(calibration.i_min, dtype=np.float32).reshape(1, 2, 8)
        self._i_max = np.asarray(calibration.i_max, dtype=np.float32).reshape(1, 2, 8)
        # Flat (2, 8) views for the scalar event path.
        self._c_min_flat = self._c_min.reshape(2, 8)
        self._c_max_flat = self._c_max.reshape(2, 8)
        self._i_min_flat = self._i_min.reshape(2, 8)
        self._i_max_flat = self._i_max.reshape(2, 8)

    def process(self, batch: FrameBatch) -> FrameBatch:
        self._process_realtime(batch)
        self._process_events(batch)
        return batch

    def _process_realtime(self, batch: FrameBatch) -> None:
        K = batch.contrast_sn_rt
        m = batch.mean_dc_rt
        if K is None or m is None:
            return

        with np.errstate(divide='ignore', invalid='ignore'):
            c_span = self._c_max - self._c_min
            i_span = self._i_max - self._i_min
            bfi = (1.0 - (K - self._c_min) / np.where(c_span > 0, c_span, 1)) * 10.0
            bvi = (1.0 - (m - self._i_min) / np.where(i_span > 0, i_span, 1)) * 10.0

        c_span_broadcast = np.broadcast_to(c_span, bfi.shape)
        i_span_broadcast = np.broadcast_to(i_span, bvi.shape)
        bfi = np.where(c_span_broadcast > 0, bfi, K * 10.0)
        bvi = np.where(i_span_broadcast > 0, bvi, m * 10.0)

        batch.bfi_live = bfi.astype(np.float32)
        batch.bvi_live = bvi.astype(np.float32)

    def _process_events(self, batch: FrameBatch) -> None:
        """Convert CorrectedInterval → EnrichedCorrectedInterval with BFI/BVI."""
        for event in batch.events:
            if not isinstance(event, IntervalClosed):
                continue
            ci = event.corrected_batch
            if not isinstance(ci, CorrectedInterval):
                continue
            enriched_frames = []
            for f in ci.frames:
                side_idx = 0 if f.side == "left" else 1
                cam_pos = int(f.cam_id) % 8

                c_min = float(self._c_min_flat[side_idx, cam_pos])
                c_max = float(self._c_max_flat[side_idx, cam_pos])
                i_min = float(self._i_min_flat[side_idx, cam_pos])
                i_max = float(self._i_max_flat[side_idx, cam_pos])
                c_span = c_max - c_min
                i_span = i_max - i_min

                contrast = f.contrast if f.contrast is not None else 0.0

                if c_span > 0:
                    bfi = (1.0 - (contrast - c_min) / c_span) * 10.0
                else:
                    bfi = contrast * 10.0
                if i_span > 0:
                    bvi = (1.0 - (f.mean - i_min) / i_span) * 10.0
                else:
                    bvi = f.mean * 10.0

                enriched_frames.append(EnrichedCorrectedFrame(
                    abs_frame_id=f.abs_frame_id, t=f.t,
                    side=f.side, cam_id=f.cam_id,
                    mean=float(f.mean), std=float(f.std),
                    contrast=float(contrast),
                    bfi=float(bfi), bvi=float(bvi),
                    quality=f.quality,
                ))
            event.corrected_batch = EnrichedCorrectedInterval(
                left_abs=ci.left_abs, right_abs=ci.right_abs,
                left_t=ci.left_t,
                frames=enriched_frames,
            )

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Process events from the terminal flush."""
        self._process_events(batch)

    def reset(self) -> None:
        pass
