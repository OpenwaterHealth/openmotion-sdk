"""FrameClassificationStage — abs_frame_id unwrap + frame_type labeling.

Per (side, cam_id) pair, the stage maintains a FrameUnwrapper (8-bit →
monotonic absolute index) and a "first frame seen" guard. Each row is
labeled with one of: "warmup", "dark", "light", "stale".

Dark frames are determined strictly by position — matching the firmware's
LaserPulseSkipInterval schedule. Content-based detection is not used;
the terminal dark frame (firmware laser-off at scan stop) is handled
separately by DarkCorrectionStage.on_scan_stop.

See docs/SciencePipeline.md §3 (unwrapping) and §4 (classification).
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch


_FRAME_ID_MODULUS = 256
_FRAME_ROLLOVER_THRESHOLD = 128


class _FrameUnwrapper:
    """8-bit rolling → monotonic. One instance per (side, cam_id).

    Robust against a non-monotonic frame stream. The sensor occasionally
    emits stale/garbage frames — leftover buffer contents at scan start
    (e.g. raw 1, 255, 173, 4, 5 …) or a mid-scan counter blip — when its
    histogram DMA buffer isn't flushed. The old unwrapper trusted every
    frame after the first, so a backward-looking value (255 after 1) was
    read as a huge forward jump and the next real frame (4) tripped the
    rollover test, injecting a permanent +256 epoch offset that shifted the
    whole positional dark/warmup schedule for the rest of the scan.

    Now each frame is gated by its *signed* 8-bit step from the last
    accepted frame: only forward steps (1..127) advance state; a backward
    or duplicate step (<= 0) is rejected as stale and does NOT advance the
    counter, so isolated garbage frames can't corrupt the epoch. The 8-bit
    counter can't disambiguate a genuine forward gap > 127 frames (a >3.2 s
    intra-scan dropout) from a backward step; such ambiguous frames are
    rejected — that data is already lost in a gap that large.
    """

    __slots__ = ("epoch", "last_raw", "seen_first", "first_was_stale")

    def __init__(self):
        self.epoch = 0
        self.last_raw = -1
        self.seen_first = False
        self.first_was_stale = False

    def unwrap(self, raw_frame_id: int) -> tuple[int, bool]:
        """Return (abs_frame_id, accepted).

        accepted=False marks a stale/non-monotonic frame: the abs_id is
        advisory only and the unwrapper state is left untouched so the
        next genuine frame resumes the sequence cleanly.
        """
        if not self.seen_first:
            self.seen_first = True
            self.first_was_stale = (raw_frame_id != 1)
            self.last_raw = raw_frame_id
            return raw_frame_id, True

        # Signed step in [-128, 127]: positive = forward, <= 0 = backward
        # (stale leftover) or duplicate.
        step = ((raw_frame_id - self.last_raw + 128) & 0xFF) - 128
        if step <= 0:
            return self.epoch * _FRAME_ID_MODULUS + raw_frame_id, False

        # Forward step (1..127). A wrap shows up as raw <= last_raw.
        if raw_frame_id <= self.last_raw:
            self.epoch += 1
        self.last_raw = raw_frame_id
        return self.epoch * _FRAME_ID_MODULUS + raw_frame_id, True


class FrameClassificationStage:
    name = "frame_classification"

    def __init__(self, discard_count: int = 9, dark_interval: int = 600):
        self.discard_count = int(discard_count)
        self.dark_interval = int(dark_interval)
        self._unwrappers: dict[tuple[int, int], _FrameUnwrapper] = {}

    def process(self, batch: FrameBatch) -> FrameBatch:
        n = batch.frame_ids.shape[0]
        abs_ids = np.zeros(n, dtype=np.int64)
        types = np.empty(n, dtype="<U8")

        for i in range(n):
            cam_id = int(batch.cam_ids[i])
            raw_id = int(batch.frame_ids[i])
            # Side is authoritatively set by the source (see FrameBatch.side_ids
            # docstring). Inferring from raw_histograms would misclassify any
            # zero-filled row — e.g. a firmware-dropped frame — as side 0.
            side_idx = int(batch.side_ids[i])

            key = (side_idx, cam_id)
            unwrapper = self._unwrappers.get(key)
            if unwrapper is None:
                unwrapper = _FrameUnwrapper()
                self._unwrappers[key] = unwrapper

            abs_id, accepted = unwrapper.unwrap(raw_id)
            abs_ids[i] = abs_id

            if not accepted:
                # Non-monotonic / stale leftover frame (e.g. unflushed
                # histogram buffer at scan start or a mid-scan counter blip).
                # Excluded downstream so it can't poison dark/timestamp align.
                types[i] = "stale"
            elif unwrapper.first_was_stale and abs_id == raw_id:
                types[i] = "stale"
            elif abs_id <= self.discard_count:
                types[i] = "warmup"
            elif self._is_dark(abs_id):
                types[i] = "dark"
            else:
                types[i] = "light"

        batch.abs_frame_ids = abs_ids
        batch.frame_type = types
        return batch

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
