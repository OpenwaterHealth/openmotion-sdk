"""ContactQualityWorkflow — SDK-owned contact-quality check procedure.

Runs a short scan, monitors per-camera BFI signal levels against
caller-supplied dark/light thresholds, and returns a pass/fail verdict
with per-camera diagnostics.  Symmetric with CalibrationWorkflow: both
use the sink-based ScanRequest API (sinks list, skip_default_storage=True)
so no production CSV or DB output is written for these diagnostic scans.

See spec section 3.8 for the contact-quality procedure definition.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from omotion.ScanWorkflow import ScanRequest


@dataclass
class CamCQResult:
    """Per-camera contact-quality verdict."""

    side:    str    # "left" or "right"
    cam_id:  int    # 0-based camera index within module
    passed:  bool
    avg_bfi: float  # mean of collected rolling-BFI samples (NaN when no data)
    reason:  str    # "ok" | "below_dark" | "above_light" | "no_signal"


@dataclass
class ContactQualityResult:
    """Overall contact-quality verdict returned by :meth:`ContactQualityWorkflow.check`."""

    passed:       bool
    per_camera:   dict   # (side: str, cam_id: int) -> CamCQResult
    duration_sec: float


class _ContactQualitySink:
    """Internal: collects rolling-averaged BFI values per camera during a
    short scan and evaluates them against dark/light thresholds.

    Subscribes to the "rolling" pipeline channel (per-frame rolling-mean
    FrameBatch emitted by the RollingAverage stage).
    """

    channels: set = frozenset({"rolling"})

    def __init__(
        self,
        dark_thresholds: list[float],
        light_thresholds: list[float],
    ) -> None:
        self._dark = list(dark_thresholds)
        self._light = list(light_thresholds)
        # (side: str, cam_id: int) -> list[float]
        self._accum: dict = {}

    def on_scan_start(self, meta) -> None:
        self._accum.clear()

    def consume(self, channel: str, batch) -> None:
        if channel != "rolling":
            return
        if batch.bfi_rolling is None:
            return
        n = batch.bfi_rolling.shape[0]
        for i in range(n):
            # Skip non-data frame types if frame_type is available.
            if batch.frame_type is not None:
                ft = str(batch.frame_type[i])
                if ft in ("warmup", "stale", "dark"):
                    continue
            for side_idx, side in enumerate(("left", "right")):
                for cam_id in range(8):
                    v = float(batch.bfi_rolling[i, side_idx, cam_id])
                    if not math.isfinite(v):
                        continue
                    key = (side, cam_id)
                    if key not in self._accum:
                        self._accum[key] = []
                    self._accum[key].append(v)

    def on_complete(self) -> None:
        pass

    def result(
        self,
        *,
        left_mask: int,
        right_mask: int,
        duration_sec: float,
    ) -> ContactQualityResult:
        per_cam: dict = {}
        for side_idx, (side, mask) in enumerate(
            (("left", left_mask), ("right", right_mask))
        ):
            for cam_id in range(8):
                if not (mask & (1 << cam_id)):
                    continue
                vals = self._accum.get((side, cam_id), [])
                avg = float(np.mean(vals)) if vals else float("nan")
                if not vals or not math.isfinite(avg):
                    reason, passed = "no_signal", False
                elif avg < self._dark[cam_id]:
                    reason, passed = "below_dark", False
                elif avg > self._light[cam_id]:
                    reason, passed = "above_light", False
                else:
                    reason, passed = "ok", True
                per_cam[(side, cam_id)] = CamCQResult(
                    side=side,
                    cam_id=cam_id,
                    passed=passed,
                    avg_bfi=avg,
                    reason=reason,
                )
        return ContactQualityResult(
            passed=all(r.passed for r in per_cam.values()),
            per_camera=per_cam,
            duration_sec=duration_sec,
        )


class ContactQualityWorkflow:
    """Run a short scan and evaluate per-camera signal levels against
    caller-supplied thresholds.

    Accepts a ``scan_workflow`` argument (a :class:`~omotion.ScanWorkflow.ScanWorkflow`
    or compatible mock) so it can be constructed independently of a full
    :class:`~omotion.MotionInterface.MotionInterface` instance for testing.
    """

    def __init__(self, scan_workflow) -> None:
        self._scan_workflow = scan_workflow

    def check(
        self,
        *,
        duration_sec: float = 1.0,
        rolling_window: int = 10,
        dark_threshold_per_camera: list[float],
        light_threshold_per_camera: list[float],
        left_camera_mask: int,
        right_camera_mask: int,
    ) -> ContactQualityResult:
        """Run a contact-quality check scan and return the verdict.

        Parameters
        ----------
        duration_sec:
            How many seconds to capture.  Rounded up to an integer for the
            ScanRequest duration field.
        rolling_window:
            Number of frames in the rolling-average window.
        dark_threshold_per_camera:
            Per-camera (length 8) lower bound on BFI.  An average below this
            value indicates no contact (dark).
        light_threshold_per_camera:
            Per-camera (length 8) upper bound on BFI.  An average above this
            value indicates ambient-light contamination.
        left_camera_mask / right_camera_mask:
            Bitmask of active cameras to evaluate.
        """
        sink = _ContactQualitySink(
            dark_thresholds=dark_threshold_per_camera,
            light_thresholds=light_threshold_per_camera,
        )
        request = ScanRequest(
            subject_id="_cq_check",
            duration_sec=int(math.ceil(duration_sec)),
            left_camera_mask=left_camera_mask,
            right_camera_mask=right_camera_mask,
            disable_laser=False,
            reduced_mode=False,
            rolling_avg_enabled=True,
            rolling_avg_window=rolling_window,
            sinks=[sink],
            skip_default_storage=True,
        )
        self._scan_workflow.start_scan(request)
        self._scan_workflow.await_complete(
            timeout_sec=duration_sec + 2.0
        )
        return sink.result(
            left_mask=left_camera_mask,
            right_mask=right_camera_mask,
            duration_sec=duration_sec,
        )
