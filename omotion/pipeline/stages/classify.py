"""FrameClassificationStage — packet consensus + abs_frame_id unwrap +
frame_type labeling.

A packet-mate consensus pass repairs wire frame_ids that disagree with a
strict in-packet majority (FrameIdConsensusCorrection; no-majority packets
emit FrameIdPacketAnomaly instead) before unwrapping — see sdk#220 and
the sensor-fw#123 fault-injection contract. Then, per (side, cam_id)
pair, the stage maintains a FrameUnwrapper (8-bit → monotonic absolute
index, counter cross-checked against the capture clock) and a "first
frame seen" guard. Each row is labeled with one of: "warmup", "dark",
"light", "stale".

Dark frames are determined strictly by position — matching the firmware's
LaserPulseSkipInterval schedule. Content-based detection is not used;
the terminal dark frame (firmware laser-off at scan stop) is handled
separately by DarkCorrectionStage.on_scan_stop.

See docs/SciencePipeline.md §5.1 (unwrap, quarantine, classification).
"""

from __future__ import annotations

import logging
from collections import Counter
from dataclasses import dataclass

import numpy as np

from ..batch import (
    CameraStreamGap, FrameBatch, FrameIdConsensusCorrection,
    FrameIdPacketAnomaly, FrameQuarantined,
)


logger = logging.getLogger("openmotion.sdk.pipeline.stages.frame_classification")

_FRAME_ID_MODULUS = 256
_NOMINAL_PERIOD_S = 0.025
_CLOCK_RESIDUAL_ABS_S = 0.012
_CLOCK_RESIDUAL_FRAC = 0.10
# A larger jump needs more evidence than one counter/clock pair before it can
# re-anchor the stream.  The first returning frame is quarantined; a second
# consecutive counter/clock pair confirms a coherent resumed stream below.
_MAX_FORWARD_GAP_FRAMES = 8
_CAMERA_GAP_ALERT_FRAMES = 8


@dataclass(frozen=True)
class _UnwrapResult:
    abs_frame_id: int
    accepted: bool
    reason: str | None = None
    detail: str | None = None
    step: int | None = None
    elapsed_s: float | None = None
    counter_anchor_abs_frame_id: int | None = None
    clock_anchor_abs_frame_id: int | None = None
    clock_anchor_timestamp_s: float | None = None


@dataclass
class _CameraGapState:
    missing_frames: int
    first_packet_id: int | None
    first_timestamp_s: float
    alerted: bool = False


class _FrameUnwrapper:
    """8-bit rolling → monotonic. One instance per (side, cam_id).

    Counter and clock are independent witnesses. Counter state advances on
    credible forward steps; the clock anchor advances only when both agree.
    A single-step counter with a bad timestamp is kept for downstream repair
    without adopting that timestamp as future classifier state.

    Acceptance rules, in order:

    1. **Backward or duplicate step (<= 0)** — rejected as stale. Covers
       leftover buffer contents at scan start (raw 1, 255, 173, 4, 5 …),
       mid-scan counter blips, and corrupted frame_ids that happen to read
       backward. State is untouched so the next genuine frame resumes the
       sequence cleanly.
    2. **Single forward step** — accepted. A clock disagreement makes the
       timestamp untrusted but cannot poison the counter sequence.
    3. **Multi-step gap** — accepted only when clock elapsed time supports it
       and the gap is within the live-stream safety bound.
    """

    __slots__ = (
        "epoch", "last_raw", "last_abs", "clock_abs", "clock_ts",
        "resync_candidate",
    )

    def __init__(self):
        self.epoch = 0
        self.last_raw = -1
        self.last_abs: int | None = None
        # The clock anchor advances only when counter and timestamp agree.
        # A timestamp outlier can therefore be repaired downstream without
        # making every following honest frame look backward in time.
        self.clock_abs: int | None = None
        self.clock_ts: float | None = None
        # A >8-frame jump is not adopted from a single counter/clock pair.
        # Retain it separately so one more consecutive pair can re-anchor a
        # genuinely resumed camera without poisoning the accepted state.
        self.resync_candidate: tuple[int, int, float] | None = None

    def unwrap(
        self,
        raw_frame_id: int,
        timestamp_s: float,
        *,
        allow_large_resync: bool = False,
    ) -> _UnwrapResult:
        """Adjudicate a counter/clock pair without adopting a bad witness."""
        ts = float(timestamp_s)
        if self.last_abs is None:
            if raw_frame_id != 1:
                return _UnwrapResult(
                    abs_frame_id=raw_frame_id,
                    accepted=False,
                    reason="leading_frame",
                    detail="leading frame id is not the scan-start value 1",
                )
            self.last_raw = raw_frame_id
            self.last_abs = raw_frame_id
            self.clock_abs = raw_frame_id
            self.clock_ts = ts
            return _UnwrapResult(raw_frame_id, True)

        counter_anchor_abs = self.last_abs
        clock_anchor_abs = self.clock_abs
        clock_anchor_ts = self.clock_ts

        pending = self.resync_candidate if allow_large_resync else None
        if not allow_large_resync:
            self.resync_candidate = None
        if pending is not None:
            pending_raw, pending_abs, pending_ts = pending
            pending_step = ((raw_frame_id - pending_raw + 128) & 0xFF) - 128
            pending_elapsed = ts - pending_ts
            pending_slack = max(
                _CLOCK_RESIDUAL_ABS_S,
                _CLOCK_RESIDUAL_FRAC * _NOMINAL_PERIOD_S,
            )
            self.resync_candidate = None
            if (pending_step == 1
                    and abs(pending_elapsed - _NOMINAL_PERIOD_S)
                    <= pending_slack):
                abs_id = pending_abs + 1
                self.epoch = abs_id // _FRAME_ID_MODULUS
                self.last_raw = raw_frame_id
                self.last_abs = abs_id
                self.clock_abs = abs_id
                self.clock_ts = ts
                return _UnwrapResult(
                    abs_frame_id=abs_id,
                    accepted=True,
                    reason="resynchronized_after_large_gap",
                    detail=("two consecutive counter/clock pairs confirmed "
                            "the resumed stream"),
                    step=abs_id - counter_anchor_abs,
                    elapsed_s=(ts - clock_anchor_ts
                               if clock_anchor_ts is not None else None),
                    counter_anchor_abs_frame_id=counter_anchor_abs,
                    clock_anchor_abs_frame_id=clock_anchor_abs,
                    clock_anchor_timestamp_s=clock_anchor_ts,
                )

        # Signed step in [-128, 127]: positive = forward, <= 0 = backward
        # (stale leftover) or duplicate.
        step = ((raw_frame_id - self.last_raw + 128) & 0xFF) - 128
        if step <= 0:
            return _UnwrapResult(
                abs_frame_id=self.epoch * _FRAME_ID_MODULUS + raw_frame_id,
                accepted=False,
                reason="non_monotonic",
                detail="non-monotonic frame id (backward/duplicate)",
                step=step,
                elapsed_s=(ts - clock_anchor_ts
                           if clock_anchor_ts is not None else None),
                counter_anchor_abs_frame_id=counter_anchor_abs,
                clock_anchor_abs_frame_id=clock_anchor_abs,
                clock_anchor_timestamp_s=clock_anchor_ts,
            )

        candidate_epoch = self.epoch + int(raw_frame_id <= self.last_raw)
        abs_id = candidate_epoch * _FRAME_ID_MODULUS + raw_frame_id
        elapsed_s = (ts - clock_anchor_ts
                     if clock_anchor_ts is not None else None)

        clock_residual_s: float | None = None
        clock_slack_s: float | None = None
        if self.clock_abs is not None and self.clock_ts is not None:
            clock_gap = abs_id - self.clock_abs
            claimed_s = clock_gap * _NOMINAL_PERIOD_S
            elapsed_s = ts - self.clock_ts
            clock_residual_s = elapsed_s - claimed_s
            clock_slack_s = max(
                _CLOCK_RESIDUAL_ABS_S,
                _CLOCK_RESIDUAL_FRAC * claimed_s,
            )
            if step > _MAX_FORWARD_GAP_FRAMES:
                if (allow_large_resync
                        and abs(clock_residual_s) <= clock_slack_s):
                    self.resync_candidate = (raw_frame_id, abs_id, ts)
                next_step = (
                    "waiting for a second resumed frame"
                    if allow_large_resync
                    else "no independent camera-outage evidence permits re-anchoring"
                )
                return _UnwrapResult(
                    abs_frame_id=abs_id,
                    accepted=False,
                    reason="gap_too_large",
                    detail=(f"forward frame-id gap +{step} exceeds direct "
                            f"acceptance limit {_MAX_FORWARD_GAP_FRAMES}; "
                            f"{next_step}"),
                    step=step,
                    elapsed_s=elapsed_s,
                    counter_anchor_abs_frame_id=counter_anchor_abs,
                    clock_anchor_abs_frame_id=clock_anchor_abs,
                    clock_anchor_timestamp_s=clock_anchor_ts,
                )
            # A single-step counter is the least ambiguous witness: accept it
            # and let TimestampRepairStage repair the clock. For a multi-step
            # counter claim, disagreement is a frame-id fault and fails closed.
            if step > 1 and abs(clock_residual_s) > clock_slack_s:
                return _UnwrapResult(
                    abs_frame_id=abs_id,
                    accepted=False,
                    reason="counter_clock_mismatch",
                    detail=(f"frame id claims +{step} frames but clock "
                            f"residual is {clock_residual_s * 1e3:.0f} ms"),
                    step=step,
                    elapsed_s=elapsed_s,
                    counter_anchor_abs_frame_id=counter_anchor_abs,
                    clock_anchor_abs_frame_id=clock_anchor_abs,
                    clock_anchor_timestamp_s=clock_anchor_ts,
                )

        self.epoch = candidate_epoch
        self.last_raw = raw_frame_id
        self.last_abs = abs_id
        if (clock_residual_s is None or clock_slack_s is None
                or abs(clock_residual_s) <= clock_slack_s):
            self.clock_abs = abs_id
            self.clock_ts = ts
        return _UnwrapResult(
            abs_frame_id=abs_id,
            accepted=True,
            step=step,
            elapsed_s=elapsed_s,
            counter_anchor_abs_frame_id=counter_anchor_abs,
            clock_anchor_abs_frame_id=clock_anchor_abs,
            clock_anchor_timestamp_s=clock_anchor_ts,
        )


class FrameClassificationStage:
    name = "frame_classification"

    def __init__(
        self,
        discard_count: int = 9,
        dark_interval: int = 600,
        expected_camera_masks: tuple[int, int] | None = None,
    ):
        self.discard_count = int(discard_count)
        self.dark_interval = int(dark_interval)
        self._unwrappers: dict[tuple[int, int], _FrameUnwrapper] = {}
        masks = expected_camera_masks or (0, 0)
        if len(masks) != 2:
            raise ValueError("expected_camera_masks must contain left and right masks")
        self._expected_cameras = {
            side: {cam for cam in range(8) if int(mask) & (1 << cam)}
            for side, mask in enumerate(masks)
        }
        self._camera_gaps: dict[tuple[int, int], _CameraGapState] = {}
        # Long-gap re-anchoring is permitted only after source-packet evidence
        # proved that this expected camera really disappeared and returned.
        self._resync_allowed: set[tuple[int, int]] = set()
        # A non-zero quarantine count is a hardware-health signal. Log one
        # example per reason live, then report reason totals at scan stop.
        self._quarantine_counts: dict[str, int] = {}
        self._quarantine_logged: set[str] = set()

    def _packet_consensus(self, batch: FrameBatch) -> dict[int, int]:
        """Packet-mate consensus on wire frame_ids (sdk#220 / sensor-fw#123).

        Source packet ids define each group; timestamps do not. Cameras in
        one packet must agree on frame_id. When one disagrees and a strict
        majority exists, dissenting rows are repaired to that value. The
        histogram data is perfectly good; only its label byte was corrupted
        in flight, so repairing beats quarantining (zero data loss). The
        repair applies to the value fed into the unwrapper; the wire record
        (batch.frame_ids, and therefore the raw CSV) is never mutated.

        Returns {row_index: corrected_raw_id} for the repaired rows.
        Disagreeing packets with NO strict majority (two cameras, or a tie)
        emit a FrameIdPacketAnomaly and are left to the per-camera
        counter-vs-clock check, which quarantines the inconsistent frames.
        """
        corrections: dict[int, int] = {}
        for indices in batch.packet_groups():
            if len(indices) < 2:
                continue
            first = indices[0]
            side_idx = int(batch.side_ids[first])
            ts = float(batch.timestamp_s[first])
            packet_id = (int(batch.packet_ids[first])
                         if batch.packet_ids is not None else None)
            fids = [int(batch.frame_ids[j]) for j in indices]
            if len(set(fids)) <= 1:
                continue
            majority_fid, majority_n = Counter(fids).most_common(1)[0]
            if len(indices) >= 3 and majority_n * 2 > len(indices):
                for j, f in zip(indices, fids):
                    if f != majority_fid:
                        corrections[j] = majority_fid
            else:
                batch.events.append(FrameIdPacketAnomaly(
                    side=side_idx, timestamp_s=ts,
                    cam_ids=[int(batch.cam_ids[j]) for j in indices],
                    frame_ids=fids,
                    packet_id=packet_id,
                ))
        return corrections

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.frame_ids.shape[0]
        abs_ids = np.zeros(n, dtype=np.int64)
        types = np.empty(n, dtype="<U8")

        self._track_camera_delivery(batch)
        consensus = self._packet_consensus(batch)

        for i in range(n):
            cam_id = int(batch.cam_ids[i])
            wire_id = int(batch.frame_ids[i])
            raw_id = consensus.get(i, wire_id)
            # Side is authoritatively set by the source (see FrameBatch.side_ids
            # docstring). Inferring from raw_histograms would misclassify any
            # zero-filled row — e.g. a firmware-dropped frame — as side 0.
            side_idx = int(batch.side_ids[i])

            key = (side_idx, cam_id)
            unwrapper = self._unwrappers.get(key)
            if unwrapper is None:
                unwrapper = _FrameUnwrapper()
                self._unwrappers[key] = unwrapper

            result = unwrapper.unwrap(
                raw_id,
                float(batch.timestamp_s[i]),
                allow_large_resync=key in self._resync_allowed,
            )
            if result.accepted:
                self._resync_allowed.discard(key)
            abs_id = result.abs_frame_id
            abs_ids[i] = abs_id

            packet_id = (int(batch.packet_ids[i])
                         if batch.packet_ids is not None else None)
            if result.accepted and i in consensus:
                # The repaired id fits this camera's sequence — record the
                # correction. (If the packet "majority" was itself corrupt,
                # the counter-vs-clock check rejects it above and the row is
                # quarantined instead, so a correction is never reported for
                # a frame that ends up discarded.)
                batch.events.append(FrameIdConsensusCorrection(
                    side=side_idx, cam_id=cam_id,
                    timestamp_s=float(batch.timestamp_s[i]),
                    wire_frame_id=wire_id, corrected_frame_id=raw_id,
                    abs_frame_id=abs_id,
                    packet_id=packet_id,
                ))

            if not result.accepted:
                # Stale leftover frame (unflushed histogram buffer at scan
                # start, a mid-scan counter blip) or a quarantined corrupt
                # frame_id (sdk#220). Excluded downstream either way so it
                # can't poison the dark/timestamp alignment.
                types[i] = "stale"
                event = FrameQuarantined(
                    side=side_idx,
                    cam_id=cam_id,
                    packet_id=packet_id,
                    timestamp_s=float(batch.timestamp_s[i]),
                    wire_frame_id=wire_id,
                    previous_abs_frame_id=result.counter_anchor_abs_frame_id,
                    clock_anchor_abs_frame_id=result.clock_anchor_abs_frame_id,
                    clock_anchor_timestamp_s=result.clock_anchor_timestamp_s,
                    step=result.step,
                    elapsed_s=result.elapsed_s,
                    reason=result.reason or "unknown",
                )
                batch.events.append(event)
                self._note_quarantine(event, result.detail or "unknown")
            elif abs_id <= self.discard_count:
                types[i] = "warmup"
            elif self._is_dark(abs_id):
                types[i] = "dark"
            else:
                types[i] = "light"

        batch.abs_frame_ids = abs_ids
        batch.frame_type = types
        return batch

    def _track_camera_delivery(self, batch: FrameBatch) -> None:
        """Alert once when an enabled camera is absent for >8 packets."""
        for indices in batch.packet_groups():
            first = indices[0]
            side = int(batch.side_ids[first])
            expected = self._expected_cameras.get(side, set())
            if not expected:
                continue
            present = {int(batch.cam_ids[i]) for i in indices}
            packet_id = (int(batch.packet_ids[first])
                         if batch.packet_ids is not None else None)
            timestamp_s = float(batch.timestamp_s[first])

            for cam_id in expected:
                key = (side, cam_id)
                if cam_id in present:
                    self._note_camera_present(
                        batch, key, packet_id, timestamp_s
                    )
                else:
                    self._note_camera_missing(
                        batch, key, packet_id, timestamp_s
                    )

    def _note_camera_present(
        self,
        batch: FrameBatch,
        key: tuple[int, int],
        packet_id: int | None,
        timestamp_s: float,
    ) -> None:
        gap = self._camera_gaps.pop(key, None)
        if gap is None or not gap.alerted:
            return
        side, cam_id = key
        self._resync_allowed.add(key)
        batch.events.append(CameraStreamGap(
            side=side,
            cam_id=cam_id,
            state="resumed",
            missing_frames=gap.missing_frames,
            packet_id=packet_id,
            timestamp_s=timestamp_s,
            first_missing_packet_id=gap.first_packet_id,
            first_missing_timestamp_s=gap.first_timestamp_s,
        ))
        logger.warning(
            "camera stream resumed: side=%s(%d) cam_id=%d after %d missing "
            "frame(s); first_missing_packet=%s resume_packet=%s "
            "resume_timestamp=%.6f",
            self._side_name(side), side, cam_id, gap.missing_frames,
            gap.first_packet_id, packet_id, timestamp_s,
        )

    def _note_camera_missing(
        self,
        batch: FrameBatch,
        key: tuple[int, int],
        packet_id: int | None,
        timestamp_s: float,
    ) -> None:
        gap = self._camera_gaps.get(key)
        if gap is None:
            gap = _CameraGapState(1, packet_id, timestamp_s)
            self._camera_gaps[key] = gap
        else:
            gap.missing_frames += 1

        if gap.missing_frames != _CAMERA_GAP_ALERT_FRAMES + 1:
            return
        gap.alerted = True
        side, cam_id = key
        batch.events.append(CameraStreamGap(
            side=side,
            cam_id=cam_id,
            state="missing",
            missing_frames=gap.missing_frames,
            packet_id=packet_id,
            timestamp_s=timestamp_s,
            first_missing_packet_id=gap.first_packet_id,
            first_missing_timestamp_s=gap.first_timestamp_s,
        ))
        logger.error(
            "CAMERA STREAM ALERT: side=%s(%d) cam_id=%d missing for %d "
            "consecutive frame(s); first_missing_packet=%s "
            "current_packet=%s first_timestamp=%.6f "
            "current_timestamp=%.6f; prolonged-loss handling remains active",
            self._side_name(side), side, cam_id, gap.missing_frames,
            gap.first_packet_id, packet_id, gap.first_timestamp_s, timestamp_s,
        )

    @staticmethod
    def _side_name(side: int) -> str:
        return {0: "left", 1: "right"}.get(side, "unknown")

    def _note_quarantine(self, event: FrameQuarantined, detail: str) -> None:
        """Count quarantines and log the first example of each reason."""
        reason = event.reason
        self._quarantine_counts[reason] = (
            self._quarantine_counts.get(reason, 0) + 1
        )
        if reason not in self._quarantine_logged:
            self._quarantine_logged.add(reason)
            logger.warning(
                "quarantining frame: side=%d cam=%d packet=%s raw_frame_id=%d "
                "timestamp=%.6f reason=%s (%s); further examples coalesced",
                event.side, event.cam_id, event.packet_id,
                event.wire_frame_id, event.timestamp_s, reason, detail,
            )

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Summarize every frame excluded by counter/clock adjudication."""
        total = sum(self._quarantine_counts.values())
        if total:
            logger.warning(
                "Scan summary: quarantined %d frame(s) by reason: %s",
                total, dict(sorted(self._quarantine_counts.items())),
            )

    def _is_dark(self, abs_id: int) -> bool:
        """Per SciencePipeline.md §5.1 FrameClassificationStage (frame-type labelling):
            n == discard_count + 1 OR (n > discard_count + 1 AND (n-1) mod dark_interval == 0)
        """
        if abs_id == self.discard_count + 1:
            return True
        if abs_id <= self.discard_count + 1:
            return False
        return (abs_id - 1) % self.dark_interval == 0

    def reset(self) -> None:
        self._unwrappers.clear()
        self._quarantine_counts.clear()
        self._quarantine_logged.clear()
        self._camera_gaps.clear()
        self._resync_allowed.clear()
