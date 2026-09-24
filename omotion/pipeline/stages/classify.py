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
# Forward steps up to this size are accepted directly when the clock agrees.
# Anything larger re-anchors only through a clock-derived candidate that a
# second consecutive counter/clock pair confirms (#286).
_MAX_FORWARD_GAP_FRAMES = 8
_CAMERA_GAP_ALERT_FRAMES = 8
# The sensor MCU and console crystals may disagree by up to ~100 ppm. Over a
# long gap that is how far the device clock can drift from the frame cadence.
_CLOCK_DRIFT_FRAC = 1e-4


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

    Acceptance rules:

    1. **Single forward step** — accepted. A clock disagreement makes the
       timestamp untrusted but cannot poison the counter sequence, unless the
       clock says whole 8-bit wraps went by (a gap of 256*n + 1 frames reads
       as +1 on the wire); that case goes through rule 4.
    2. **Forward step 2–8** — accepted when the clock supports the full step.
    3. **Everything else is quarantined**: backward or duplicate steps (stale
       leftovers at scan start, counter blips, corrupted ids that read
       backward), steps > 8, and multi-step claims the clock contradicts.
       Accepted state is untouched, so the next genuine frame resumes.
    4. **Re-anchoring after a real gap.** When a quarantined frame's device
       clock places it at an absolute id whose low byte is its wire id, that
       id is held as a candidate. The stream re-anchors there only if the next
       frame continues it by exactly one frame and ~25 ms (#286). A corrupted
       frame id fails the first test (its low byte won't match the clock); a
       single capture with matched counter+timestamp corruption fails the
       second (the next clean frame doesn't continue it). A real gap of any
       length, one camera or the whole module, passes both, and because the
       epoch comes from the clock, absolute ids stay correct past 8-bit wraps.
       Only the first resumed frame is lost.
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
        # (raw id, clock-implied abs id, timestamp) of a quarantined frame the
        # next frame may confirm as the start of a resumed stream (rule 4).
        self.resync_candidate: tuple[int, int, float] | None = None

    def _adopt(self, abs_id: int, raw_frame_id: int, ts: float, *,
               clock: bool) -> None:
        self.epoch = abs_id // _FRAME_ID_MODULUS
        self.last_raw = raw_frame_id
        self.last_abs = abs_id
        if clock:
            self.clock_abs = abs_id
            self.clock_ts = ts

    def _clock_candidate(self, raw_frame_id: int, ts: float) -> int | None:
        """The absolute id the device clock implies for this frame, if its low
        byte is the wire id. None when the clock can't vouch for the frame."""
        if self.clock_abs is None or self.clock_ts is None:
            return None
        elapsed = ts - self.clock_ts
        if elapsed <= 0:
            return None
        nearest = self.clock_abs + round(elapsed / _NOMINAL_PERIOD_S)
        slack = _CLOCK_RESIDUAL_ABS_S + _CLOCK_DRIFT_FRAC * elapsed
        for abs_id in (nearest - 1, nearest, nearest + 1):
            if abs_id <= self.last_abs:
                continue
            if (abs_id - raw_frame_id) % _FRAME_ID_MODULUS:
                continue
            claimed = (abs_id - self.clock_abs) * _NOMINAL_PERIOD_S
            if abs(elapsed - claimed) <= slack:
                return abs_id
        return None

    def unwrap(self, raw_frame_id: int, timestamp_s: float) -> _UnwrapResult:
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
            self._adopt(raw_frame_id, raw_frame_id, ts, clock=True)
            return _UnwrapResult(raw_frame_id, True)

        anchors = dict(
            counter_anchor_abs_frame_id=self.last_abs,
            clock_anchor_abs_frame_id=self.clock_abs,
            clock_anchor_timestamp_s=self.clock_ts,
        )
        elapsed_s = (ts - self.clock_ts) if self.clock_ts is not None else None

        pending, self.resync_candidate = self.resync_candidate, None
        if pending is not None:
            pending_raw, pending_abs, pending_ts = pending
            pending_slack = max(
                _CLOCK_RESIDUAL_ABS_S,
                _CLOCK_RESIDUAL_FRAC * _NOMINAL_PERIOD_S,
            )
            if (raw_frame_id == (pending_raw + 1) % _FRAME_ID_MODULUS
                    and abs((ts - pending_ts) - _NOMINAL_PERIOD_S)
                    <= pending_slack):
                abs_id = pending_abs + 1
                step = abs_id - self.last_abs
                self._adopt(abs_id, raw_frame_id, ts, clock=True)
                return _UnwrapResult(
                    abs_frame_id=abs_id,
                    accepted=True,
                    reason="resynchronized_after_large_gap",
                    detail=("the device clock placed the resumed stream and "
                            "the next frame confirmed it"),
                    step=step,
                    elapsed_s=elapsed_s,
                    **anchors,
                )

        # Signed step in [-128, 127]: positive = forward, <= 0 = backward
        # (stale leftover) or duplicate.
        step = ((raw_frame_id - self.last_raw + 128) & 0xFF) - 128
        clock_residual_s: float | None = None
        clock_ok = True
        if step > 0:
            abs_id = self.last_abs + step
            if self.clock_abs is not None and self.clock_ts is not None:
                claimed_s = (abs_id - self.clock_abs) * _NOMINAL_PERIOD_S
                clock_residual_s = (ts - self.clock_ts) - claimed_s
                clock_ok = abs(clock_residual_s) <= max(
                    _CLOCK_RESIDUAL_ABS_S, _CLOCK_RESIDUAL_FRAC * claimed_s,
                )
            if step <= _MAX_FORWARD_GAP_FRAMES and clock_ok:
                self._adopt(abs_id, raw_frame_id, ts, clock=True)
                return _UnwrapResult(
                    abs_frame_id=abs_id, accepted=True, step=step,
                    elapsed_s=elapsed_s, **anchors,
                )
        else:
            abs_id = self.epoch * _FRAME_ID_MODULUS + raw_frame_id

        candidate = self._clock_candidate(raw_frame_id, ts)
        if step == 1 and candidate in (None, abs_id):
            # A single-step counter is the least ambiguous witness: accept it.
            # When the clock disagrees, TimestampRepairStage repairs the
            # timestamp and the clock anchor stays where it was.
            self._adopt(abs_id, raw_frame_id, ts, clock=candidate == abs_id)
            return _UnwrapResult(
                abs_frame_id=abs_id, accepted=True, step=step,
                elapsed_s=elapsed_s, **anchors,
            )
        if 1 < step <= _MAX_FORWARD_GAP_FRAMES and candidate == abs_id:
            # Within the drift allowance for a long-stale clock anchor.
            self._adopt(abs_id, raw_frame_id, ts, clock=True)
            return _UnwrapResult(
                abs_frame_id=abs_id, accepted=True, step=step,
                elapsed_s=elapsed_s, **anchors,
            )

        if candidate is not None:
            self.resync_candidate = (raw_frame_id, candidate, ts)
            return _UnwrapResult(
                abs_frame_id=candidate,
                accepted=False,
                reason="gap_too_large",
                detail=(f"device clock places this frame "
                        f"{candidate - self.last_abs} frames after the last "
                        f"accepted one; held until the next frame confirms "
                        f"the resumed stream"),
                step=step,
                elapsed_s=elapsed_s,
                **anchors,
            )
        if step <= 0:
            return _UnwrapResult(
                abs_frame_id=abs_id,
                accepted=False,
                reason="non_monotonic",
                detail="non-monotonic frame id (backward/duplicate)",
                step=step,
                elapsed_s=elapsed_s,
                **anchors,
            )
        if step > _MAX_FORWARD_GAP_FRAMES:
            return _UnwrapResult(
                abs_frame_id=abs_id,
                accepted=False,
                reason="gap_too_large",
                detail=(f"forward frame-id gap +{step} exceeds direct "
                        f"acceptance limit {_MAX_FORWARD_GAP_FRAMES} and the "
                        f"device clock does not place the frame there"),
                step=step,
                elapsed_s=elapsed_s,
                **anchors,
            )
        # A multi-step counter claim the clock contradicts is a frame-id
        # fault and fails closed.
        return _UnwrapResult(
            abs_frame_id=abs_id,
            accepted=False,
            reason="counter_clock_mismatch",
            detail=(f"frame id claims +{step} frames but clock "
                    f"residual is {(clock_residual_s or 0.0) * 1e3:.0f} ms"),
            step=step,
            elapsed_s=elapsed_s,
            **anchors,
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

            result = unwrapper.unwrap(raw_id, float(batch.timestamp_s[i]))
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
        """Per SciencePipeline.md §4.2:
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
