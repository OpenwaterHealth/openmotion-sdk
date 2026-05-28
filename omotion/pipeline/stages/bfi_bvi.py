"""BfiBviStage — affine calibration map (contrast, mean) → (BFI, BVI).

See docs/SciencePipeline.md §9:
    BFI = (1 - (K - C_min) / (C_max - C_min)) * 10
    BVI = (1 - (mean - I_min) / (I_max - I_min)) * 10

Fallback: identity scaling (K * 10, mean * 10) when calibration span is zero.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..batch import FrameBatch


# Sanity bounds for finite BFI/BVI output. Values outside (or AT) the
# formula's calibrated extremes [0, 10] indicate degenerate input —
# either near-zero mean at scan stop when the laser is turning off
# (making K = std/mean blow up) or K << C_min driving the formula past
# its calibrated extreme. Exact-10.0 only happens when K == C_min as
# float-bitwise-identical, which never arises from real continuous
# measurements (it's a degenerate-sentinel pattern). NaN propagates
# naturally through every downstream consumer (LivePlotSink,
# ScanDBSink, SideAveragingStage's nanmean) so out-of-range values
# become display gaps rather than spikes.
_BFI_BVI_SANITY_LO = -2.0   # exclusive: legit "high blood flow" tops at ~3-5
_BFI_BVI_SANITY_HI = 10.0   # exclusive: exact 10.0 == formula extreme == junk


class BfiBviStage:
    name = "bfi_bvi"

    def __init__(self, calibration: Any):
        """`calibration` must expose c_min, c_max, i_min, i_max as (2, 8) ndarrays."""
        self._c_min = np.asarray(calibration.c_min, dtype=np.float32).reshape(1, 2, 8)
        self._c_max = np.asarray(calibration.c_max, dtype=np.float32).reshape(1, 2, 8)
        self._i_min = np.asarray(calibration.i_min, dtype=np.float32).reshape(1, 2, 8)
        self._i_max = np.asarray(calibration.i_max, dtype=np.float32).reshape(1, 2, 8)

    def process(self, batch: FrameBatch) -> FrameBatch:
        K  = batch.contrast_sn_rt
        m  = batch.mean_dc_rt

        with np.errstate(divide='ignore', invalid='ignore'):
            c_span = self._c_max - self._c_min
            i_span = self._i_max - self._i_min
            bfi = (1.0 - (K - self._c_min) / np.where(c_span > 0, c_span, 1)) * 10.0
            bvi = (1.0 - (m - self._i_min) / np.where(i_span > 0, i_span, 1)) * 10.0

        c_span_broadcast = np.broadcast_to(c_span, bfi.shape)
        i_span_broadcast = np.broadcast_to(i_span, bvi.shape)
        bfi = np.where(c_span_broadcast > 0, bfi, K * 10.0)
        bvi = np.where(i_span_broadcast > 0, bvi, m * 10.0)

        # Sanity filter — see module-level constants for rationale.
        # Lower bound is inclusive (0.0 is a legitimate occlusion-test
        # reading); upper bound is exclusive (exact 10.0 is the formula's
        # degenerate-input marker from laser turn-off, never produced
        # by real continuous measurements).
        with np.errstate(invalid='ignore'):
            bfi = np.where(
                (bfi >= _BFI_BVI_SANITY_LO) & (bfi < _BFI_BVI_SANITY_HI),
                bfi, np.nan,
            )
            bvi = np.where(
                (bvi >= _BFI_BVI_SANITY_LO) & (bvi < _BFI_BVI_SANITY_HI),
                bvi, np.nan,
            )

        batch.bfi_live = bfi.astype(np.float32)
        batch.bvi_live = bvi.astype(np.float32)
        return batch

    def reset(self) -> None:
        pass
