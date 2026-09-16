"""FrameBatch — the typed data carrier that flows through all pipeline stages.

Each stage's docstring states which fields it owns. Stages mutate the batch
in place for performance (no per-batch allocation churn). Tests assert field
ownership: only the owning stage writes a given field.

All per-frame fields are numpy arrays of shape (N, ...). N is the batch
size — typically 10-100 frames per batch from LiveUsbSource.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields as dataclass_fields
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
class TerminalDarkResult(BatchEvent):
    """Emitted during on_scan_stop for each (side, cam) pending interval.

    ``found=True``: the terminal frame was dark-like (u1 ≤ threshold), as
    expected from firmware. The interval was closed normally.

    ``found=False``: the terminal frame was NOT dark-like — the laser
    appears to have been on for the final frame. This is a firmware issue:
    the trigger-stop did not produce the expected laser-off frame. The
    interval is left open (data for this interval is lost).

    ``identified_by``: "fsync" when the terminal frame was identified by
    the console's reported final fsync pulse index (ground truth, content
    used only as verification); "content" when identified by the legacy
    dark-like content scan (no fsync count available, or it didn't match
    a buffered frame).
    """
    side:           str
    cam_id:         int
    abs_frame_id:   int
    u1:             float
    threshold:      float
    found:          bool
    identified_by:  str = "content"


@dataclass
class TimestampMisalignmentWindow(BatchEvent):
    """A contiguous run of frames on one side whose timestamps deviated from
    the frame-id cadence and were re-timestamped / NaN-filled by
    TimestampRepairStage. Routed to "diagnostics" so the scan DB's
    session_meta summary records it.

    The expected terminal stop-frame artifact — the firmware's laser-off
    frame fires ~150 ms off the 25 ms grid at scan stop, on every scan —
    is deliberately NOT reported as one of these.
    """
    side:        int
    onset_fid:   int
    end_fid:     int
    onset_t:     float
    end_t:       float
    n_corrected: int
    n_nan:       int


@dataclass
class FrameIdConsensusCorrection(BatchEvent):
    """One camera's wire frame_id disagreed with its packet-mates and was
    repaired to the packet majority before unwrapping, preserving the
    frame's histogram data (sdk#220 / sensor-fw#123 fid_single evidence).

    The wire record (raw CSV) keeps the original value — the correction
    applies to the unwrapped abs_frame_id and everything downstream.
    Routed to "diagnostics"."""
    side:               int
    cam_id:             int
    timestamp_s:        float
    wire_frame_id:      int    # the corrupted 8-bit value as received
    corrected_frame_id: int    # the packet-majority 8-bit value used instead
    abs_frame_id:       int    # unwrapped id after correction
    packet_id:          Optional[int] = None


@dataclass
class FrameIdPacketAnomaly(BatchEvent):
    """Cameras in one packet disagreed on frame_id with no strict majority
    to adjudicate (sensor-fw#123 fid_multi evidence). No consensus repair
    is possible; the per-camera counter-vs-clock check quarantines the
    inconsistent frames instead. Routed to "diagnostics"."""
    side:        int
    timestamp_s: float
    cam_ids:     list   # cameras in the packet, wire order
    frame_ids:   list   # their wire frame_ids, same order
    packet_id:   Optional[int] = None
    reason:      str = "no_single_outlier_consensus"


@dataclass
class FrameQuarantined(BatchEvent):
    """One wire row was excluded before the science path.

    The event preserves both counter/clock witnesses and the decision reason
    for post-scan diagnosis. The raw CSV retains the row; science stages skip
    it through the legacy ``frame_type='stale'`` label.
    """
    side:                 int
    cam_id:               int
    packet_id:            Optional[int]
    timestamp_s:          float
    wire_frame_id:        int
    previous_abs_frame_id: Optional[int]
    clock_anchor_abs_frame_id: Optional[int]
    clock_anchor_timestamp_s: Optional[float]
    step:                 Optional[int]
    elapsed_s:            Optional[float]
    reason:               str


@dataclass
class CameraStreamGap(BatchEvent):
    """An expected camera disappeared from source packets for >8 captures.

    One ``state="missing"`` event is emitted when the threshold is crossed,
    and one ``state="resumed"`` event when that camera next appears.  The
    prolonged-loss watchdog remains responsible for stopping the scan; this
    event supplies the early warning and durable packet evidence needed to
    diagnose an intermittent camera after the scan.
    """
    side:                       int
    cam_id:                     int
    state:                      str  # "missing" or "resumed"
    missing_frames:             int
    packet_id:                  Optional[int]
    timestamp_s:                float
    first_missing_packet_id:    Optional[int]
    first_missing_timestamp_s:  float
    reason:                      str = "camera_missing_from_source_packets"


@dataclass
class TimestampRepairInputAnomaly(BatchEvent):
    """Consecutive captures arrived carrying one reused packet timestamp
    (frozen clock) while the frame counter advanced — the sensor-fw#123
    timestamp_freeze signature. The affected frames are re-timestamped by
    TimestampRepairStage; one event is emitted per (side, frozen value).
    Routed to "diagnostics"."""
    side:        int
    timestamp_s: float  # the reused timestamp value
    n_frames:    int    # frames observed carrying it beyond the first capture


@dataclass
class FrameGapFillAnomaly(BatchEvent):
    """A gap in one camera's abs_frame_id sequence (lost or quarantined
    frames) was back-filled with synthetic nan_filled placeholder rows —
    the sensor-fw#123 packet_drop evidence, emitted when the next frame
    arrives and the gap becomes visible. Routed to "diagnostics"."""
    side:          int
    cam_id:        int
    gap_start_fid: int    # first missing abs_frame_id
    gap_end_fid:   int    # last missing abs_frame_id
    n_filled:      int
    timestamp_s:   float  # timestamp of the gap-closing frame


@dataclass
class PipelineError(BatchEvent):
    """A stage raised during pipeline.process(); the batch was dropped.

    Stage state is deliberately preserved (no reset): frame-id alignment and
    dark history survive, and the gap left by the dropped batch is equivalent
    to USB packet loss for those frames. Resetting instead would re-trip the
    stale-first guard and permanently misalign the positional dark schedule.
    Routed to the "diagnostics" channel.
    """
    error:             str              # repr of the exception
    n_frames:          int              # rows in the dropped batch
    first_timestamp_s: Optional[float]  # batch's first timestamp, if any


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
class TerminalFsyncCount(BatchEvent):
    """The console's fsync pulse count read after stop_trigger.

    Trigger_Stop hard-disables the laser timer, then the FSYNC timer fires
    one final deferred pulse — so the frame captured on the last pulse is
    laser-off by construction, and this count is the ground-truth identity
    of the terminal dark frame. ScanWorkflow reads it via OW_CTRL_GET_FSYNC
    during teardown, hands it to DarkCorrectionStage (positive terminal-dark
    identification), and dispatches this event on the "diagnostics" channel
    for observability.

    ``timestamp_s`` is scan-relative, matching the other event types.
    """
    count:        int
    timestamp_s:  float


@dataclass
class SideAverageSample:
    """One reduced-mode realtime per-side average for a single capture instant.

    Carried as the payload of a ``LiveEmit`` on the ``"live_side"`` channel
    (SideAverageStage realtime path), one sample per capture (``frame_id``)
    per side. The corrected side average does NOT use this type — it rides
    the ``"final"`` channel as cam_id=-1 EnrichedCorrectedFrames."""
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
      Parse:           cam_ids, frame_ids, side_ids, packet_ids, raw_histograms,
                       temperature_c, timestamp_s, pdc, tcm, tcl
      Classify:        abs_frame_ids, frame_type
      NoiseFloor:      (mutates raw_histograms in place — no new field)
      Moments:         mean_raw, std_raw, contrast_raw
      PedestalSubtraction: subtracted_mean
      DarkCorrection:  dark_baseline_rt, mean_dc_rt, std_dc_rt, low_light_rt
                       (also appends IntervalClosed to events when interval closes)
      ShotNoise:       std_sn_rt, contrast_sn_rt
      BfiBvi:          bfi_live, bvi_live
      SideAverage:     appends LiveEmit(channel="live_side", SideAverageSample)
                       per capture, plus synthetic IntervalClosed events whose
                       frames carry cam_id=-1 (corrected side averages, routed
                       to "final") — reduced mode only
      Tee:             appends LiveEmit to events
    """

    # ── Source fields (set by the Source that produces the batch) ────────

    # (N,) int8 — camera index 0..7 within the sensor module for each row.
    cam_ids:        np.ndarray

    # (N,) uint8 — raw 8-bit rolling frame counter from the camera firmware.
    # Wraps at 256; FrameClassificationStage unwraps into abs_frame_ids.
    frame_ids:      np.ndarray

    # (N, 2, 8, 1024) uint32 — raw histogram bins per frame, per side,
    # per camera. 1024 bins from the 10-bit ADC. This is the primary input
    # to the science pipeline. NoiseFloorStage mutates this in place.
    raw_histograms: np.ndarray

    # (N, 2, 8) float32 — on-chip temperature sensor reading in °C per
    # camera. Used for diagnostics/telemetry, not in the science path.
    temperature_c:  np.ndarray

    # (N,) float64 — capture timestamp in seconds, normalized to t=0 at
    # scan start by the source. Used for dark-interval interpolation and
    # telemetry CSV output.
    timestamp_s:    np.ndarray

    # (N,) optional float — per-frame photodiode current (PDC) from the
    # console telemetry poller. Used for dark-correction diagnostics.
    # None when telemetry is unavailable.
    pdc:            Optional[np.ndarray]

    # (N,) optional int64 — MCU (lsync) trigger count from console
    # telemetry. Diagnostic only.
    tcm:            Optional[np.ndarray]

    # (N,) optional int64 — laser trigger count from console telemetry.
    # Diagnostic only.
    tcl:            Optional[np.ndarray]

    # ── Source: side assignment ───────────────────────────────────────────

    # (N,) int8 — which sensor module produced row i: 0 = left, 1 = right.
    # Set by the source (the only place that authoritatively knows the
    # side). Downstream stages read this directly — never infer side from
    # raw_histograms (a dropped/zero-filled frame would silently misroute
    # to side 0). Optional only so existing sources can be migrated
    # incrementally; a missing-source-side defaults to None.
    side_ids:       Optional[np.ndarray] = None

    # (N,) int64 — source packet ordinal, unique within a side for the scan.
    # All rows parsed from one USB packet share the same id. Never substitute
    # timestamp_s for packet identity: frozen clocks intentionally repeat it.
    packet_ids:     Optional[np.ndarray] = None

    # ── FrameClassificationStage outputs ─────────────────────────────────

    # (N,) int64 — monotonic absolute frame ID, unwrapped from the 8-bit
    # rolling frame_ids. Used by all downstream stages to identify frames
    # across the scan.
    abs_frame_ids:  Optional[np.ndarray] = None

    # (N,) str — classification label for each frame. One of:
    #   "stale"  — leftover from a prior scan in the USB buffer (discarded)
    #   "warmup" — first discard_count frames while camera stabilizes
    #   "dark"   — laser off, scheduled by firmware at dark_interval
    #   "light"  — laser on, real measurement
    frame_type:     Optional[np.ndarray] = None

    # ── MomentsStage outputs ─────────────────────────────────────────────

    # (N, 2, 8) float32 — first moment of the histogram (mean pixel value
    # in DN). Includes the sensor pedestal; use subtracted_mean or
    # mean_dc_rt for pedestal/dark-corrected values.
    mean_raw:       Optional[np.ndarray] = None

    # (N, 2, 8) float32 — standard deviation of the histogram (sqrt of
    # variance = u2 - u1²). Raw, before any dark or shot-noise correction.
    std_raw:        Optional[np.ndarray] = None

    # (N, 2, 8) float32 — raw contrast. Intentionally left None by
    # MomentsStage because contrast requires pedestal subtraction
    # (K = std / (mean - pedestal)). Computed downstream by ShotNoiseCorrectionStage.
    contrast_raw:   Optional[np.ndarray] = None

    # ── PedestalSubtractionStage output ──────────────────────────────────

    # (N, 2, 8) float32 — mean_raw minus the per-side sensor pedestal.
    # Can be negative (valid noise below the pedestal average). Used only
    # for dark-frame diagnostics (ambient-light gate in CalibrationWorkflow,
    # dark-max check in ContactQualityWorkflow) — NOT used in the BFI/BVI
    # science path, which uses mean_dc_rt.
    subtracted_mean:   Optional[np.ndarray] = None

    # ── DarkCorrectionStage outputs (realtime path) ──────────────────────

    # (N, 2, 8) float32 — predicted dark baseline u1 from
    # HybridRealtimePredictor. NaN during warmup (no dark observed yet).
    dark_baseline_rt: Optional[np.ndarray] = None

    # (N, 2, 8) float32 — dark-corrected mean: mean_raw minus the
    # predicted dark baseline. This is the realtime estimate of optical
    # signal strength. NaN during warmup.
    mean_dc_rt:       Optional[np.ndarray] = None

    # (N, 2, 8) float32 — dark-corrected std: sqrt(max(0, raw_var -
    # predicted_dark_var)). Realtime estimate. NaN during warmup.
    std_dc_rt:        Optional[np.ndarray] = None

    # (N, 2, 8) bool — True where a LIGHT-typed frame's raw mean was at or
    # below pedestal + max_above_pedestal, i.e. the camera received no
    # meaningful light (sensor covered / off target / laser-off artifact).
    # For these slots DarkCorrectionStage suppresses realtime emission
    # (mean_dc_rt stays NaN), so this flag is the only way consumers can
    # distinguish "frame arrived but unlit" from "no prediction yet"
    # (warmup). Always False for scheduled dark frames — those are expected
    # to be unlit. None until DarkCorrectionStage runs.
    low_light_rt:     Optional[np.ndarray] = None

    # ── ShotNoiseCorrectionStage outputs ─────────────────────────────────

    # (N, 2, 8) float32 — std after Poisson shot-noise variance
    # subtraction: sqrt(max(0, std_dc_rt² - adc_gain × mean × cam_gain)).
    # The speckle-only standard deviation.
    std_sn_rt:        Optional[np.ndarray] = None

    # (N, 2, 8) float32 — speckle contrast: std_sn_rt / mean_dc_rt.
    # 0 when mean ≤ 0, NaN when mean is NaN (warmup). This is the input
    # to the BFI calibration.
    contrast_sn_rt:   Optional[np.ndarray] = None

    # ── BfiBviStage outputs ──────────────────────────────────────────────

    # (N, 2, 8) float32 — Blood Flow Index from affine calibration:
    # BFI = (1 - (contrast - c_min) / (c_max - c_min)) × 10.
    # Nominally 0–10 scale. Used for live display; during dark frames,
    # DarkFrameHoldStage overwrites with the last light frame's value.
    bfi_live:       Optional[np.ndarray] = None

    # (N, 2, 8) float32 — Blood Volume Index from affine calibration:
    # BVI = (1 - (mean - i_min) / (i_max - i_min)) × 10.
    # Nominally 0–10 scale. Same hold behavior as bfi_live.
    bvi_live:       Optional[np.ndarray] = None

    # ── TimestampRepairStage output ──────────────────────────────────────

    # (N,) str — per-frame quality flag. Set by TimestampRepairStage.
    # "ok" = device timestamp passed through unchanged
    # "ts_corrected" = timestamp replaced by re-anchoring interpolation
    # "nan_filled" = synthetic row for a missing frame (zero histogram)
    quality:        Optional[np.ndarray] = None

    # ── Event queue ──────────────────────────────────────────────────────

    # Accumulated events from stages during process(). The runner reads
    # these after each batch and dispatches by type:
    #   LiveEmit        → routed to sinks by channel name
    #   IntervalClosed  → routed to "final" channel sinks
    #   everything else → routed to "diagnostics" channel sinks
    # Cleared implicitly by creating a new FrameBatch per source iteration.
    events:         list[BatchEvent] = field(default_factory=list)

    def snapshot(self) -> "FrameBatch":
        """Return a copy with every numpy array deep-copied.

        Used by ``Tee(snapshot=True)`` to freeze the batch's data at tee
        time. The runner dispatches a Tee's LiveEmit only *after* the whole
        pipeline has run, so a by-reference payload would expose later
        in-place mutations (NoiseFloorStage zeroes ``raw_histograms``;
        TimestampRepairStage rewrites ``timestamp_s``) to a sink that is
        meant to record the faithful, pre-processing capture.

        ``events`` is deliberately not carried over — a snapshot is a
        passive data payload, not a live batch, so it starts with a fresh
        empty event queue (avoiding a payload that references itself).
        """
        kwargs = {}
        for f in dataclass_fields(self):
            if f.name == "events":
                continue
            value = getattr(self, f.name)
            kwargs[f.name] = value.copy() if isinstance(value, np.ndarray) else value
        return FrameBatch(**kwargs)

    def iter_rows(self, *, exclude: "set[str] | frozenset[str]" = frozenset()):
        """Yield ``(i, side_idx, cam_id, frame_type)`` per row, skipping rows
        whose ``frame_type`` is in ``exclude``.

        The canonical per-row iterator for sinks. Callers state their own
        exclusion policy explicitly: science/live consumers skip stale rows,
        while the raw wire record intentionally retains them.

        ``side_idx`` is -1 when the batch carries no ``side_ids`` (legacy
        replay batches); ``frame_type`` is "" before classification.
        """
        ft = self.frame_type
        side_ids = self.side_ids
        n = self.cam_ids.shape[0]
        for i in range(n):
            ftype = str(ft[i]) if ft is not None else ""
            if ftype in exclude:
                continue
            side_idx = int(side_ids[i]) if side_ids is not None else -1
            yield i, side_idx, int(self.cam_ids[i]), ftype

    def packet_groups(self) -> list[list[int]]:
        """Return source-packet row indices in wire order.

        Live/replay sources provide ``packet_ids``. Hand-built legacy batches
        fall back to the wire invariant that one packet contains at most one
        row per camera: a timestamp change or repeated camera starts a new
        packet. The fallback therefore remains correct for timestamp freezes.
        """
        n = self.cam_ids.shape[0]
        if n == 0:
            return []

        sides = self.side_ids
        if self.packet_ids is not None:
            groups: dict[tuple[int, int], list[int]] = {}
            for i in range(n):
                side = int(sides[i]) if sides is not None else -1
                key = (side, int(self.packet_ids[i]))
                groups.setdefault(key, []).append(i)
            return list(groups.values())

        completed: list[list[int]] = []
        active: dict[int, tuple[float, set[int], list[int]]] = {}
        for i in range(n):
            side = int(sides[i]) if sides is not None else -1
            cam = int(self.cam_ids[i])
            ts = float(self.timestamp_s[i])
            current = active.get(side)
            if current is None:
                active[side] = (ts, {cam}, [i])
                continue
            packet_ts, seen_cams, indices = current
            if cam in seen_cams or abs(ts - packet_ts) > 1e-9:
                completed.append(indices)
                active[side] = (ts, {cam}, [i])
            else:
                seen_cams.add(cam)
                indices.append(i)

        completed.extend(indices for _, _, indices in active.values())
        completed.sort(key=lambda indices: indices[0])
        return completed
