"""Motion hardware adapter for the seedless C_min measurement.

Implements :class:`omotion.calibration.seedless_cmin.CminBench` over a
started :class:`omotion.MotionInterface` whose cameras are already powered
and configured, and whose laser operating point is applied.

Each capture is one short scan through the normal science pipeline
(``ScanWorkflow.start_scan``) with a collector sink on the corrected stream,
so the frames are dark-corrected exactly like a clinical scan's. A scan
starts with the console's dark preamble (frames 1..10: warmup + the
boundary dark), so the light frames are abs ids 11 onward; the scan runs
just long enough for the requested light frames plus a small pad, and the
first ``light_frames`` per camera are returned. Seedless captures add
:data:`omotion.seedless.SEEDLESS_TRIGGER_OVERRIDES` so the dark frames stay
dark under the 2295 us exposure.
"""

from __future__ import annotations

import logging
from typing import Optional

from omotion.ScanWorkflow import ScanRequest
from omotion.pipeline.quality_flags import NAN_FILLED, has as has_quality
from omotion.seedless import (
    SEEDLESS_TRIGGER_OVERRIDES,
    SeedlessController,
    seedless_unsupported_reason,
)

from .seedless_cmin import FrameRecord

logger = logging.getLogger("openmotion.sdk.calibration.seedless_cmin")

FRAME_RATE_HZ = 40.0
#: First light frame of a scan (frames 1..9 warmup, 10 the boundary dark).
FIRST_LIGHT_ABS_ID = 11
#: Extra frames of scan time: start-up jitter, the duration guard's 0.2 s
#: poll, and the terminal dark the pipeline strips at scan stop.
PAD_FRAMES = 6
SCAN_TIMEOUT_PAD_S = 30.0


class _LightFrameCollector:
    """Collects corrected light frames from the ``final`` channel."""

    channels = frozenset({"final"})

    def __init__(self) -> None:
        self.frames: list[FrameRecord] = []

    def on_scan_start(self, meta) -> None:  # noqa: D401
        self.frames.clear()

    def consume(self, channel: str, payload) -> None:
        if channel != "final":
            return
        for f in payload.frames:
            # Gap-fill placeholders are not measurements (#270).
            if has_quality(getattr(f, "quality", None), NAN_FILLED):
                continue
            self.frames.append(FrameRecord(
                side=f.side, cam_id=int(f.cam_id),
                abs_frame_id=int(f.abs_frame_id), mean=float(f.mean),
                std=float(f.std), contrast=float(f.contrast)))

    def on_complete(self) -> None:
        pass


def first_light_frames(frames: list[FrameRecord], light_frames: int) -> list[FrameRecord]:
    """The first ``light_frames`` light frames of each camera, by frame id."""
    out: list[FrameRecord] = []
    by_cam: dict[tuple[str, int], list[FrameRecord]] = {}
    for f in frames:
        if f.abs_frame_id >= FIRST_LIGHT_ABS_ID:
            by_cam.setdefault((f.side, f.cam_id), []).append(f)
    for key in sorted(by_cam):
        out.extend(sorted(by_cam[key], key=lambda f: f.abs_frame_id)[:light_frames])
    return out


class MotionCminBench:
    """Args:
        interface: started MotionInterface, cameras configured, laser applied.
        camera_masks: {"left": mask, "right": mask} (0 = side unused).
        trigger_config: the standard trigger payload for every capture.
        settle_s: SeedlessController settle delay (tests pass 0).
    """

    def __init__(self, interface, camera_masks: dict, trigger_config: dict,
                 *, settle_s: float = 0.05,
                 controller_factory=SeedlessController):
        self._iface = interface
        self._masks = {"left": int(camera_masks.get("left", 0)),
                       "right": int(camera_masks.get("right", 0))}
        self._trigger = dict(trigger_config)
        self._settle_s = settle_s
        self._controller_factory = controller_factory
        self._ctrl: Optional[SeedlessController] = None
        self.scans_run = 0

    @property
    def controller(self) -> Optional[SeedlessController]:
        return self._ctrl

    def capture(self, light_frames: int, *, seedless: bool) -> list[FrameRecord]:
        if seedless != bool(self._ctrl is not None and self._ctrl.applied):
            raise RuntimeError("capture(seedless=%s) does not match the "
                               "register state" % seedless)
        trigger = dict(self._trigger)
        if seedless:
            trigger.update(SEEDLESS_TRIGGER_OVERRIDES)
        n_frames = FIRST_LIGHT_ABS_ID - 1 + int(light_frames) + PAD_FRAMES
        duration_s = n_frames / FRAME_RATE_HZ
        collector = _LightFrameCollector()
        request = ScanRequest(
            subject_id="SEEDLESS_CMIN" if seedless else "NOMINAL_CMIN",
            duration_sec=duration_s,   # float: the duration guard takes it as is
            left_camera_mask=self._masks["left"],
            right_camera_mask=self._masks["right"],
            disable_laser=False,
            sinks=[collector],
            skip_default_storage=True,
            raw_save_max_duration_s=0,
            trigger_config=trigger,
        )
        workflow = self._iface.scan_workflow
        if not workflow.start_scan(request):
            raise RuntimeError("the scan did not start: "
                               f"{workflow.last_scan_error or 'refused'}")
        self.scans_run += 1
        workflow.await_complete(timeout_sec=duration_s + SCAN_TIMEOUT_PAD_S)
        if workflow.running:
            try:
                workflow.cancel_scan()
            finally:
                raise RuntimeError("the scan did not finish in time")
        if workflow.last_scan_error:
            raise RuntimeError(f"the scan failed: {workflow.last_scan_error}")
        return first_light_frames(collector.frames, light_frames)

    def _sensors(self) -> list:
        out = []
        for side in ("left", "right"):
            if self._masks[side]:
                out.append((side, getattr(self._iface, side), self._masks[side]))
        return out

    def enter_seedless(self, ta_pulse_width_us: float) -> None:
        console = self._iface.console
        reason = seedless_unsupported_reason(console)
        if reason:
            raise RuntimeError(reason)
        self._ctrl = self._controller_factory(
            console=console, sensors=self._sensors(),
            ta_pulse_width_us=ta_pulse_width_us, settle_s=self._settle_s)
        if not self._ctrl.apply():
            raise RuntimeError("could not apply the seedless settings "
                               "(see the log); they are being rolled back")

    def set_ta_pulse_width_us(self, ta_pulse_width_us: float) -> None:
        if self._ctrl is None or not self._ctrl.set_ta_pulse_width_us(
                ta_pulse_width_us):
            raise RuntimeError(f"could not set the TA pulse width to "
                               f"{ta_pulse_width_us:.1f} us")

    def exit_seedless(self) -> Optional[str]:
        ctrl = self._ctrl
        if ctrl is None:
            return None
        # The trigger is stopped between scans, but make sure: the rate
        # lower limits are re-armed here and must not see seedless pulses.
        try:
            self._iface.console.stop_trigger()
        except Exception:
            logger.warning("stop_trigger before seedless restore raised",
                           exc_info=True)
        if not ctrl.restore():
            return ("restore incomplete: power-cycle the console (its laser "
                    "registers are volatile) before any other use")
        mismatches = ctrl.verify_restored()
        if mismatches:
            return "registers read back wrong after restore: " + ", ".join(
                f"{k}={v[0]} (expected {v[1]})" for k, v in mismatches.items())
        return None
