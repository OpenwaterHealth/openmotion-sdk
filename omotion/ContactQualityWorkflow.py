"""ContactQualityWorkflow — SDK-owned contact-quality check procedure.

Runs a short scan, monitors per-camera **DN-scale** signal levels against
caller-supplied dark/light thresholds, and returns a pass/fail verdict
with per-camera diagnostics.  Symmetric with CalibrationWorkflow: both
use the sink-based ScanRequest API (sinks list, skip_default_storage=True)
so no production CSV or DB output is written for these diagnostic scans.

Thresholds are **background-subtracted DN** (i.e. raw_mean - pedestal),
matching the legacy ContactQuality module semantics.  Two failure modes:

* AMBIENT_LIGHT  — any dark frame's DN exceeds dark_threshold_per_camera
                   (ambient light leaking onto the sensor)
* POOR_CONTACT   — rolling average of light frame DN falls below
                   light_threshold_per_camera (laser not coupled)

See spec section 3.8 for the contact-quality procedure definition.
"""

from __future__ import annotations

import collections
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from omotion.ScanWorkflow import run_collection_scan


@dataclass
class CamCQResult:
    """Per-camera contact-quality verdict (DN-scale)."""

    side:        str    # "left" or "right"
    cam_id:      int    # 0-based camera index within module
    passed:      bool
    light_avg_dn: float # mean of rolling-window light-frame display_mean (NaN when no data)
    light_std_dn: float # mean of rolling-window light-frame std_raw (NaN when no data)
    dark_max_dn:  float # max of dark-frame display_mean (NaN when no data)
    dark_std_dn:  float # std_raw recorded with dark_max_dn (NaN when no data)
    reason:      str    # "ok" | "poor_contact" | "ambient_light" | "no_signal"


@dataclass
class ContactQualityResult:
    """Overall contact-quality verdict returned by :meth:`ContactQualityWorkflow.check`."""

    passed:       bool
    per_camera:   dict   # (side: str, cam_id: int) -> CamCQResult
    duration_sec: float


class _ContactQualitySink:
    """Internal: collects per-camera light/dark DN values during a short scan
    and evaluates them against background-subtracted DN thresholds.

    Subscribes to the "live" pipeline channel and reads two different
    signals depending on frame type:

      * Dark frames → ``display_mean`` (= ``max(0, mean_raw - pedestal)``
        from PedestalSubtractionStage). Measures ambient light leaking onto
        the sensor; AMBIENT_LIGHT threshold gates on this.

      * Light frames → ``mean_dc_rt`` (= ``mean_raw - predicted_dark_baseline``
        from DarkCorrectionStage). Measures the actual laser-driven signal
        above the just-measured dark, not signal + dark; POOR_CONTACT
        threshold gates on this.
    """

    channels: set = frozenset({"live"})

    def __init__(
        self,
        dark_thresholds: list[float],
        light_thresholds: list[float],
        rolling_window: int = 10,
    ) -> None:
        self._dark = list(dark_thresholds)
        self._light = list(light_thresholds)
        self._window_size = max(1, int(rolling_window))
        # (side, cam_id) -> deque[float]   (light-frame display_mean values)
        self._light_window: dict = {}
        # (side, cam_id) -> deque[float]   (light-frame std_raw values)
        self._light_std_window: dict = {}
        # (side, cam_id) -> float          (max dark-frame display_mean seen)
        self._dark_max: dict = {}
        # (side, cam_id) -> float          (std_raw paired with max dark frame)
        self._dark_std: dict = {}
        # (side, cam_id) -> int            (count of light-frame samples seen)
        self._light_count: dict = {}
        # (side, cam_id) -> float          (running sum of light display_mean)
        self._light_sum: dict = {}

    def on_scan_start(self, meta) -> None:
        self._light_window.clear()
        self._light_std_window.clear()
        self._dark_max.clear()
        self._dark_std.clear()
        self._light_count.clear()
        self._light_sum.clear()

    def consume(self, channel: str, batch) -> None:
        if channel != "live":
            return
        # Two different DN-scale signals — both pedestal-aware, but different
        # baselines:
        #   * Dark frames: use display_mean = max(0, mean_raw - pedestal).
        #     For a laser-off frame this measures ambient light leaking onto
        #     the sensor relative to the zero-light pedestal — exactly what
        #     the AMBIENT_LIGHT threshold gates on.
        #   * Light frames: use mean_dc_rt = mean_raw - predicted_dark_baseline
        #     from DarkCorrectionStage. This is the actual laser-driven signal
        #     above the just-measured dark frame, not signal + dark. The
        #     POOR_CONTACT threshold should compare against signal strength,
        #     not signal + dark.
        # display_mean is set by PedestalSubtractionStage; mean_dc_rt is set
        # by DarkCorrectionStage. Both arrays have shape (N, 2, 8) when
        # present.
        if batch.display_mean is None or getattr(batch, "mean_dc_rt", None) is None:
            return
        n = batch.display_mean.shape[0]
        for i in range(n):
            ft = None
            if batch.frame_type is not None:
                ft = str(batch.frame_type[i])
                if ft in ("warmup", "stale"):
                    continue
            for side_idx, side in enumerate(("left", "right")):
                for cam_id in range(8):
                    std_v = float("nan")
                    if getattr(batch, "std_raw", None) is not None:
                        std_v = float(batch.std_raw[i, side_idx, cam_id])
                    key = (side, cam_id)
                    if ft == "dark":
                        v = float(batch.display_mean[i, side_idx, cam_id])
                        if not math.isfinite(v):
                            continue
                        prev = self._dark_max.get(key, float("-inf"))
                        if v > prev:
                            self._dark_max[key] = v
                            self._dark_std[key] = std_v
                    else:
                        # Light or unclassified — treat as light for CQ purposes.
                        # mean_dc_rt is NaN for early light frames before any
                        # dark has been observed (predictor returns None →
                        # baseline_rt/mean_dc_rt stay at their NaN init).
                        # Skip those frames; the rolling window will fill up
                        # once the first dark lands.
                        v = float(batch.mean_dc_rt[i, side_idx, cam_id])
                        if not math.isfinite(v):
                            continue
                        w = self._light_window.get(key)
                        if w is None:
                            w = collections.deque(maxlen=self._window_size)
                            self._light_window[key] = w
                        sw = self._light_std_window.get(key)
                        if sw is None:
                            sw = collections.deque(maxlen=self._window_size)
                            self._light_std_window[key] = sw
                        w.append(v)
                        if math.isfinite(std_v):
                            sw.append(std_v)
                        self._light_sum[key]   = self._light_sum.get(key, 0.0) + v
                        self._light_count[key] = self._light_count.get(key, 0) + 1

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
        for side, mask in (("left", left_mask), ("right", right_mask)):
            for cam_id in range(8):
                if not (mask & (1 << cam_id)):
                    continue
                key = (side, cam_id)
                window = self._light_window.get(key)
                light_count = self._light_count.get(key, 0)

                if light_count > 0 and window is not None and len(window) > 0:
                    # Rolling window avg (matches legacy live-detection logic);
                    # cumulative sum/count remains available for diagnostics.
                    light_avg = float(sum(window) / len(window))
                else:
                    light_avg = float("nan")
                std_window = self._light_std_window.get(key)
                if std_window is not None and len(std_window) > 0:
                    light_std = float(sum(std_window) / len(std_window))
                else:
                    light_std = float("nan")

                dark_max = self._dark_max.get(key, float("nan"))
                dark_std = self._dark_std.get(key, float("nan"))

                dark_threshold = (
                    self._dark[cam_id] if cam_id < len(self._dark) else float("inf")
                )
                light_threshold = (
                    self._light[cam_id] if cam_id < len(self._light) else 0.0
                )

                if not math.isfinite(light_avg):
                    reason, passed = "no_signal", False
                elif math.isfinite(dark_max) and dark_max > dark_threshold:
                    reason, passed = "ambient_light", False
                elif light_avg < light_threshold:
                    reason, passed = "poor_contact", False
                else:
                    reason, passed = "ok", True
                per_cam[key] = CamCQResult(
                    side=side,
                    cam_id=cam_id,
                    passed=passed,
                    light_avg_dn=light_avg,
                    light_std_dn=light_std,
                    dark_max_dn=dark_max,
                    dark_std_dn=dark_std,
                    reason=reason,
                )
        return ContactQualityResult(
            passed=all(r.passed for r in per_cam.values()),
            per_camera=per_cam,
            duration_sec=duration_sec,
        )


class ContactQualityWorkflow:
    """Run a short scan and evaluate per-camera signal levels against
    caller-supplied **DN-scale** thresholds.

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
            Number of light frames in the rolling-average window.
        dark_threshold_per_camera:
            Per-camera (length 8) upper bound on background-subtracted DN
            for **dark** frames.  Any dark frame exceeding this value
            triggers AMBIENT_LIGHT (e.g. legacy default 3.0 DN).
        light_threshold_per_camera:
            Per-camera (length 8) lower bound on background-subtracted DN
            for **light** frames (rolling-window mean).  Falling below
            triggers POOR_CONTACT (e.g. legacy default 15.0 DN).
        left_camera_mask / right_camera_mask:
            Bitmask of active cameras to evaluate.
        """
        sink = _ContactQualitySink(
            dark_thresholds=dark_threshold_per_camera,
            light_thresholds=light_threshold_per_camera,
            rolling_window=rolling_window,
        )
        # Shared short-scan engine (see ScanWorkflow.run_collection_scan), the
        # same one the calibration/test sub-scans use. CQ runs it synchronously
        # (no stop_evt) and reads the verdict off its own sink.
        run_collection_scan(
            self._scan_workflow,
            sink,
            subject_id="_cq_check",
            duration_sec=duration_sec,
            left_camera_mask=left_camera_mask,
            right_camera_mask=right_camera_mask,
        )
        return sink.result(
            left_mask=left_camera_mask,
            right_mask=right_camera_mask,
            duration_sec=duration_sec,
        )
