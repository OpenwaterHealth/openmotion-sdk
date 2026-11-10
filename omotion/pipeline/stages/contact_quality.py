"""ContactQualityStage — live contact-quality verdict, in the corrected record.

Runs :class:`~omotion.contact_quality.ContactQualityMonitor` over every batch
(the realtime ``mean_dc_rt`` / ``subtracted_mean`` / ``low_light_rt`` values)
and then:

* publishes each raise or clear edge as a ``LiveEmit`` on the
  ``"contact_quality"`` channel, carrying a ``ContactQualityTransition``.
  ``ContactQualityTransitionSink`` turns those into the caller's callback:
  the live warning the operator sees;
* adds the verdict latched for each camera at each frame to the ``quality``
  flags of the per-camera corrected frames that close in this batch
  (``poor_contact`` / ``ambient_light``, bloodflow-app#589).

Placed AFTER SideAverageStage and BEFORE Tee("live"):

* the realtime values are final there; the monitor used to read the very
  same batch off the "live" channel;
* a corrected interval that closes in this batch holds frames that arrived in
  this batch or an earlier one, and the stage judges the batch before it
  stamps, so every frame is stamped with a final verdict. (Done in a sink,
  the "final" interval arrives before the same batch's "live" rows, so the
  rows had to be held back until the verdict caught up.);
* the reduced-mode side-average frames (``cam_id = -1``) already exist and
  are skipped, so the verdict never folds into a side row.
"""

from __future__ import annotations

from typing import Any

from omotion.contact_quality import (
    CONTACT_QUALITY_CHANNEL, CQSettings, ContactQualityMonitor,
)

from ..batch import FrameBatch, IntervalClosed, LiveEmit
from ..quality_flags import join as join_quality


class ContactQualityStage:
    name = "contact_quality"

    def __init__(self, settings: CQSettings, *, metadata: Any) -> None:
        self._settings = settings
        self._metadata = metadata
        self._monitor = self._new_monitor()

    def _new_monitor(self) -> ContactQualityMonitor:
        s = self._settings
        monitor = ContactQualityMonitor(
            thresholds=s.thresholds,
            rolling_window=s.rolling_window,
            light_activate_debounce=s.light_activate_debounce,
            light_clear_debounce=s.light_clear_debounce,
            dark_debounce=s.dark_debounce,
        )
        monitor.on_scan_start(self._metadata)
        return monitor

    @property
    def monitor(self) -> ContactQualityMonitor:
        return self._monitor

    def process(self, batch: FrameBatch) -> FrameBatch:
        self._monitor.consume("live", batch)
        for transition in self._monitor.take_transitions():
            batch.events.append(
                LiveEmit(channel=CONTACT_QUALITY_CHANNEL, payload=transition))
        self._stamp(batch)
        return batch

    def on_scan_stop(self, batch: FrameBatch) -> None:
        """Stamp the intervals of the terminal dark flush."""
        self._stamp(batch)
        self._monitor.on_complete()

    def reset(self) -> None:
        self._monitor = self._new_monitor()

    def _stamp(self, batch: FrameBatch) -> None:
        for event in batch.events:
            if not isinstance(event, IntervalClosed):
                continue
            for f in getattr(event.corrected_batch, "frames", None) or ():
                cam_id = int(f.cam_id)
                if cam_id < 0:
                    continue   # side average: no verdict (Clinical rule)
                verdict = self._monitor.verdict_at(f.side, cam_id, int(f.abs_frame_id))
                if verdict is not None:
                    f.quality = join_quality(f.quality, verdict)
