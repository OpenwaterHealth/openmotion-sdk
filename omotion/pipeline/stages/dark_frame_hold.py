"""DarkFrameHoldStage — repeat the last light frame in place of dark frames.

The laser is off during dark frames (the periodic dark-baseline
intervals and the laser-off frame the firmware emits at scan stop), so
the BFI/BVI computed for them are meaningless and would spike the live
trace. This stage replaces each dark frame's per-camera BFI/BVI with
the most recent LIGHT frame's values for that camera, so the live trace
holds steady through the dark interval instead of spiking.

Placed AFTER BfiBviStage (so bfi_live/bvi_live exist) and BEFORE
LiveSideAverageStage (so the per-side averages reflect the held values).
Frames stay labeled "dark" — only their display values are held; the
upstream dark-correction baseline machinery is untouched. Only the
display metrics (bfi_live, bvi_live) are held; mean/contrast feed the
correction two-pass refinement and are left as computed.
"""

from __future__ import annotations

import numpy as np

from ..batch import FrameBatch

# Display metrics held across dark frames. Deliberately excludes
# mean_dc_rt / contrast_sn_rt — those feed downstream correction logic.
_HELD_FIELDS = ("bfi_live", "bvi_live")


class DarkFrameHoldStage:
    name = "dark_frame_hold"

    def __init__(self) -> None:
        # (side_idx, cam_id) -> {field_name: last finite light value}
        self._last_light: dict[tuple[int, int], dict[str, float]] = {}

    def process(self, batch: FrameBatch) -> FrameBatch:
        ft = batch.frame_type
        if ft is None or batch.bfi_live is None:
            return batch
        side_ids = batch.side_ids
        cam_ids = batch.cam_ids
        if side_ids is None or cam_ids is None:
            return batch

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

        return batch

    def reset(self) -> None:
        self._last_light.clear()
