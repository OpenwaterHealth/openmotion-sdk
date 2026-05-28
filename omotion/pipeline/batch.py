"""FrameBatch — the typed data carrier that flows through all pipeline stages.

Each stage's docstring states which fields it owns. Stages mutate the batch
in place for performance (no per-batch allocation churn). Tests assert field
ownership: only the owning stage writes a given field.

All per-frame fields are numpy arrays of shape (N, ...). N is the batch
size — typically 10-100 frames per batch from LiveUsbSource.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


class BatchEvent:
    """Base type for events that don't fit cleanly into per-frame arrays."""


@dataclass
class IntervalClosed(BatchEvent):
    """Fired by DarkCorrectionStage when a dark interval closes.

    Carries the accurately-corrected interval (linear interpolation between
    bounding darks + 4-point quadratic stencil for the dark frame itself).
    The runner routes this to "final" channel sinks (storage).
    """
    corrected_batch: object


@dataclass
class LiveEmit(BatchEvent):
    """Fired by Tee stages — signals "emit this batch's snapshot to this channel."

    The runner reads channel name and routes the payload to subscribed sinks.
    """
    channel: str
    payload: object


@dataclass
class DarkIntegrityWarning(BatchEvent):
    """A dark frame's u1 exceeded pedestal+threshold. Frame is still processed;
    this is a diagnostic event, not a drop signal."""
    side: str
    cam_id: int
    abs_frame_id: int
    u1: float
    pedestal: float
    threshold: float


@dataclass
class StencilFallback(BatchEvent):
    """The 4-point dark-frame quadratic stencil fell back to a simpler scheme
    because some neighbours were unavailable. Diagnostic, not an error."""
    side: str
    cam_id: int
    abs_frame_id: int
    fallback_used: str


@dataclass
class TriggerStateEvent(BatchEvent):
    """Emitted whenever the laser trigger transitions ON or OFF.

    Dispatched on the "diagnostics" channel. Sinks that care (e.g. the
    bloodflow-app's scan-notes builder) use these to compute the actual
    trigger-ON time, distinct from the wall-clock scan duration which
    includes pre-scan setup and post-scan USB drain.

    ``timestamp_s`` is scan-relative, matching the other event types.
    """
    state:        str    # "ON" or "OFF"
    timestamp_s:  float


@dataclass
class SideAverageSample:
    """One reduced-mode per-side average for a single capture instant.

    Carried as the payload of a ``LiveEmit`` — channel ``"live_side"`` for the
    realtime average (LiveSideAverageStage) and ``"final_side"`` for the
    dark-corrected average (CorrectedSideAverageStage). One sample per capture
    (``frame_id``) per side. ``mean`` / ``contrast`` are populated only on the
    corrected path; the live path leaves them ``None``."""
    t:         float
    frame_id:  int
    side:      int            # 0 = left, 1 = right
    bfi:       float
    bvi:       float
    mean:      Optional[float] = None
    contrast:  Optional[float] = None


@dataclass
class FrameBatch:
    """N frames worth of data, two sides, 8 cameras each.

    Field ownership (which stage populates which field):
      Parse:           cam_ids, frame_ids, side_ids, raw_histograms,
                       temperature_c, timestamp_s, pdc, tcm, tcl
      Classify:        abs_frame_ids, frame_type
      NoiseFloor:      (mutates raw_histograms in place — no new field)
      Moments:         mean_raw, std_raw, contrast_raw
      PedestalSubtraction: display_mean
      DarkCorrection:  dark_baseline_rt, mean_dc_rt, std_dc_rt
                       (also appends IntervalClosed to events when interval closes)
      ShotNoise:       std_sn_rt, contrast_sn_rt
      BfiBvi:          bfi_live, bvi_live
      LiveSideAverage: appends LiveEmit(channel="live_side", SideAverageSample)
                       per capture (reduced mode only)
      Tee:             appends LiveEmit to events
    """

    cam_ids:        np.ndarray
    frame_ids:      np.ndarray
    raw_histograms: np.ndarray
    temperature_c:  np.ndarray
    timestamp_s:    np.ndarray
    pdc:            Optional[np.ndarray]
    tcm:            Optional[np.ndarray]
    tcl:            Optional[np.ndarray]

    # side_ids[i] ∈ {0, 1}: which sensor module produced row i (0=left, 1=right).
    # Set by the source (the only place that authoritatively knows the side).
    # Downstream stages read this directly — never infer side from the
    # raw_histograms (a dropped/zero-filled frame would silently misroute
    # to side 0). Optional only so existing sources can be migrated
    # incrementally and so a missing-source-side defaults to None.
    side_ids:       Optional[np.ndarray] = None

    abs_frame_ids:  Optional[np.ndarray] = None
    frame_type:     Optional[np.ndarray] = None

    mean_raw:       Optional[np.ndarray] = None
    std_raw:        Optional[np.ndarray] = None
    contrast_raw:   Optional[np.ndarray] = None

    display_mean:   Optional[np.ndarray] = None

    dark_baseline_rt: Optional[np.ndarray] = None
    mean_dc_rt:       Optional[np.ndarray] = None
    std_dc_rt:        Optional[np.ndarray] = None

    std_sn_rt:        Optional[np.ndarray] = None
    contrast_sn_rt:   Optional[np.ndarray] = None

    bfi_live:       Optional[np.ndarray] = None
    bvi_live:       Optional[np.ndarray] = None

    events:         list[BatchEvent] = field(default_factory=list)
