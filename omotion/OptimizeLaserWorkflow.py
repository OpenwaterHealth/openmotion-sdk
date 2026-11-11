"""OptimizeLaserWorkflow — the Optimize Laser procedure on a connected device.

Steps the camera gains, the laser pulse width and the drive current until
every camera's dark-subtracted mean sits on the target (bloodflow-app
docs/auto_exposure). Each iteration writes the settings with the trigger off
(:class:`~omotion.exposure.actuator.LaserCameraActuator`), runs one short scan
through the shared collection-scan engine, and hands the raw light-frame
histograms and the dark level to the
:class:`~omotion.exposure.controller.ExposureController`.

Console firmware with the 1.8.2-rc.3 safety envelope is refused before the
first write. The run raises ``PULSE_WIDTH_UL``, lowers ``RATE_LL`` (for the
2400 us dark skip every scan uses at the 2196 us exposure) and, when needed,
raises ``DRIVE_CL``. These are volatile register writes: a console power-cycle
or :meth:`OptimizeLaserWorkflow.restore_defaults` puts them back.

A run that passes leaves the device at its final settings and makes them
:attr:`OptimizeLaserWorkflow.active`. From then on every scan the interface's
``ScanWorkflow`` starts (app scans, contact-quality checks, calibration,
scripts) goes through :meth:`OptimizeLaserWorkflow.prepare_scan` first: the
settings are re-applied, since a reconnect resets the laser driver and each
scan reconfigures its cameras, and the request takes the matching trigger
config, gains, BVI calibration and a ``session_meta.optimize_laser`` record.
A scan whose re-apply fails is refused. They stay active until
:meth:`OptimizeLaserWorkflow.restore_defaults`, the next run, or a
:attr:`OptimizeLaserWorkflow.gate` that says they may no longer be used. A run
that fails, is canceled or errors restores the defaults before returning.
"""

from __future__ import annotations

import csv
import datetime
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Callable, Mapping, Optional

import numpy as np

from omotion.config import CAMERA_GAIN_MAP, LONG_EXPOSURE_DARK_SKIP_US
from omotion.exposure.actuator import LaserBaseline, LaserCameraActuator
from omotion.exposure.controller import (
    DEFAULT_MAX_ITERATIONS,
    KNEE_GUESS_MA,
    THRESHOLD_GUESS_MA,
    Camera,
    CameraObservation,
    ExposureController,
    ExposureSettings,
    IterationRecord,
    OptimizeStatus,
)
from omotion.exposure.estimate import histogram_mean
from omotion.exposure.settings import GainLadder, HardwareLimits, OptimizeLaserSettings
from omotion.ScanWorkflow import run_collection_scan

logger = logging.getLogger("openmotion.sdk.exposure")

# Dark skip for every scan at the long exposure: the displaced dark pulse
# starts at 100 + 2400 = 2500 us, after a 2196 us exposure ends.
DARK_SKIP_US = LONG_EXPOSURE_DARK_SKIP_US
# restore() problems that mean the laser side may still hold optimized
# settings or the 2400 us skip. A camera write failure alone does not block
# a scan, because every scan reconfigures its cameras first.
LASER_RESTORE_PROBLEMS = ("laser power restore", "trigger config reset")
# The burst a restore fires at the stock settings before it tightens the
# safety limits (LaserCameraActuator._settle_at_stock): 20 pulses at 40 Hz,
# then time for the trigger stop to land before the limit writes.
RESTORE_BURST_S = 0.5
RESTORE_BURST_SETTLE_S = 0.3
FRAME_RATE_HZ = 40.0
# default_pipeline's discard_count: the first frames after the start are warmup.
WARMUP_FRAMES = 9
# Light frames past the ones needed, so a short scan never runs out.
SPARE_FRAMES = 10
# on_not_converged answers (anything else stops and restores).
CONTINUE = "continue"
ACCEPT = "accept"

FW_DEFAULT_GAINS = tuple(float(g) for g in CAMERA_GAIN_MAP)

CSV_COLUMNS = (
    "iteration", "t_s", "side", "camera", "gain", "gain_code", "exposure_us",
    "pulse_width_us", "emitted_pulse_width_us", "current_ma", "method", "mean_raw",
    "dark_dn", "signal", "estimate_kind", "sat_frac", "points", "points_in_range",
    "on_target", "desired_factor", "camera_status", "next_gain", "next_pulse_width_us",
    "next_current_ma", "done", "status",
)


def cameras_from_masks(left_camera_mask: int, right_camera_mask: int) -> list[Camera]:
    return [(side, pos)
            for side, mask in enumerate((left_camera_mask, right_camera_mask))
            for pos in range(8) if mask & (1 << pos)]


class _OptimizeLaserSink:
    """Collects each camera's dark level and first light-frame histograms
    from one short scan.

    Reads the ``raw`` channel: ``NoiseFloorStage`` zeroes low-count bins on
    ``live``, which would bias the fit's tails. A camera's light frames count
    only after its first dark frame, and only the first ``frames_needed`` of
    them, so the terminal laser-off frame of a scan never lands in the
    estimate. The dark level is the mean of the camera's dark frames (DN,
    pedestal included, like the light histograms).
    """

    channels: frozenset = frozenset({"raw"})

    def __init__(self, cameras, frames_needed: int) -> None:
        self._cameras = frozenset(cameras)
        self._needed = max(1, int(frames_needed))
        self._darks: dict = {}
        self._lights: dict = {}

    def on_scan_start(self, meta) -> None:
        self._darks.clear()
        self._lights.clear()

    def consume(self, channel: str, batch) -> None:
        if channel != "raw" or batch.raw_histograms is None:
            return
        for i, side_idx, cam_id, ft in batch.iter_rows(exclude={"warmup", "stale"}):
            key = (side_idx, cam_id)
            if key not in self._cameras:
                continue
            hist = batch.raw_histograms[i, side_idx, cam_id]
            if not hist.any():
                continue
            if ft == "dark":
                self._darks.setdefault(key, []).append(histogram_mean(hist))
            elif ft == "light" and key in self._darks:
                frames = self._lights.setdefault(key, [])
                if len(frames) < self._needed:
                    frames.append(np.array(hist, dtype=np.float64))

    def on_complete(self) -> None:
        pass

    def observations(self) -> dict[Camera, CameraObservation]:
        return {key: CameraObservation(tuple(frames), float(np.mean(self._darks[key])))
                for key, frames in self._lights.items() if frames}

    def light_frame_counts(self) -> dict[Camera, int]:
        return {key: len(self._lights.get(key, ())) for key in sorted(self._cameras)}


@dataclass(frozen=True)
class OptimizeResult:
    """How an Optimize Laser run ended.

    ``status`` is the controller's verdict, None when the run stopped early
    (``canceled`` or ``error``). ``settings`` is what the device holds now:
    the final settings after a pass, None after a restore. ``accepted``: the
    run did not converge and the caller chose to keep its last settings
    (``on_not_converged``), which then count as a pass.
    """

    config: OptimizeLaserSettings
    status: Optional[OptimizeStatus]
    settings: Optional[ExposureSettings]
    exposure_us: float
    cameras: tuple[Camera, ...]
    records: tuple[IterationRecord, ...] = ()
    csv_path: Optional[str] = None
    learned_threshold_ma: Optional[float] = None
    baseline: Optional[LaserBaseline] = None
    canceled: bool = False
    error: Optional[str] = None
    restored: bool = False
    restore_problems: tuple[str, ...] = ()
    accepted: bool = False

    @property
    def passed(self) -> bool:
        return (self.status is not None and (self.status.passed or self.accepted)
                and not self.canceled and self.error is None and self.settings is not None)

    @property
    def iterations(self) -> int:
        return len(self.records)


class _IterationCsv:
    """One row per iteration x camera, flushed every iteration so an aborted
    run still leaves its log. Settings and device identity go in ``#`` lines
    above the header row."""

    def __init__(self, path: str, header: Mapping[str, object]) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.path = path
        self._file = open(path, "w", newline="", encoding="utf-8")
        for key, value in header.items():
            self._file.write(f"# {key}: {value}\n")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_COLUMNS)
        self._file.flush()

    def write(self, record: IterationRecord, t_s: float, *, method: str, exposure_us: float,
              truncation_us: float) -> None:
        used, nxt = record.settings, record.next_settings
        for (side, pos), cam in sorted(record.cameras.items()):
            self._writer.writerow([
                record.iteration, f"{t_s:.2f}", ("left", "right")[side], pos + 1,
                cam.gain, cam.gain_code, exposure_us, used.pulse_width_us,
                round(used.pulse_width_us - truncation_us, 1), round(used.current_ma, 2), method,
                _num(cam.mean_raw), _num(cam.dark_dn), _num(cam.signal), cam.kind or "",
                _num(cam.sat_frac, 5), cam.points, cam.points_in_range, int(cam.on_target),
                _num(cam.desired_factor, 4), cam.status.value, nxt.gains[(side, pos)],
                nxt.pulse_width_us, round(nxt.current_ma, 2), int(record.done),
                record.status.value if record.status else "",
            ])
        self._file.flush()

    def close(self, footer: Mapping[str, object]) -> None:
        try:
            for key, value in footer.items():
                self._file.write(f"# {key}: {value}\n")
            self._file.close()
        except Exception:
            logger.exception("Optimize Laser CSV close failed")


def _num(value, digits: int = 2):
    if value is None or not math.isfinite(value):
        return ""
    return round(float(value), digits)


class OptimizeLaserWorkflow:
    """Runs Optimize Laser on ``interface`` (a :class:`~omotion.MotionInterface`
    or anything with ``console``, ``left``, ``right`` and ``scan_workflow``).

    ``run`` blocks; call it from a worker thread. It shares the scan lock
    with every other scan, so it can't overlap a scan or a CQ check.

    A caller may set three attributes, all optional:

    ``force_fault``: ``() -> bool``. When set, restores load the laser
    through ``apply_laser_power(force_fault=force_fault())``, like the app's
    other laser loads (its Force laser fail test flag).

    ``gate``: ``() -> str | None``. Asked before each scan while settings
    are active. A reason means they may no longer be used (the app: clinical
    mode, engineering mode off, Force laser fail armed): the defaults are
    restored and the scan runs at them.

    ``on_active_changed``: ``(reason: str | None) -> None``. Called after
    :attr:`active` changes, on whatever thread changed it. ``reason`` is the
    gate's text when the gate dropped the settings, else None.
    """

    def __init__(self, interface, *, lock=None, limits: HardwareLimits | None = None,
                 actuator_factory: Callable[..., LaserCameraActuator] | None = None,
                 force_fault: Callable[[], bool] | None = None):
        self._interface = interface
        self._lock = lock
        self.limits = limits or HardwareLimits()
        self._actuator_factory = actuator_factory or LaserCameraActuator
        self._run_lock = threading.Lock()
        self.force_fault = force_fault
        self.gate: Callable[[], Optional[str]] | None = None
        self.on_active_changed: Callable[[Optional[str]], None] | None = None
        self._active: OptimizeResult | None = None
        # The console's own laser values, from the preflight of a run that
        # started with nothing active (so the device was at stock).
        self._stock: LaserBaseline | None = None
        self._capture_stock = False
        # Every scan this interface starts asks prepare_scan first.
        register = getattr(getattr(interface, "scan_workflow", None),
                           "set_scan_preparer", None)
        if callable(register):
            register(self.prepare_scan)

    @property
    def running(self) -> bool:
        return self._run_lock.locked()

    @property
    def active(self) -> OptimizeResult | None:
        """The passed run whose settings scans use now, or None."""
        return self._active

    def _set_active(self, result: OptimizeResult | None, reason: str | None = None) -> None:
        if result is self._active:
            return
        self._active = result
        if self.on_active_changed is not None:
            try:
                self.on_active_changed(reason)
            except Exception:
                logger.exception("Optimize Laser on_active_changed callback raised")

    def actuator(self) -> LaserCameraActuator:
        iface = self._interface
        kwargs = {}
        if self.force_fault is not None:
            from omotion.laser import apply_laser_power

            def restore_laser(console, lock=None):
                return apply_laser_power(console, force_fault=bool(self.force_fault()),
                                         lock=lock)
            kwargs["restore_laser"] = restore_laser
        return self._actuator_factory(iface.console, {0: iface.left, 1: iface.right},
                                      lock=self._lock, **kwargs)

    # ---- the run ----------------------------------------------------------

    def run(
        self,
        settings: OptimizeLaserSettings,
        *,
        left_camera_mask: int,
        right_camera_mask: int,
        ladder: GainLadder | None = None,
        initial_gains: Mapping[Camera, float] | None = None,
        threshold_guess: float = THRESHOLD_GUESS_MA,
        knee_guess: float = KNEE_GUESS_MA,
        max_iterations: int = DEFAULT_MAX_ITERATIONS,
        log_path: str | None = None,
        on_iteration: Callable[[IterationRecord], None] | None = None,
        stop_evt: threading.Event | None = None,
        on_not_converged: Callable[[IterationRecord], str] | None = None,
    ) -> OptimizeResult:
        """Optimize the cameras in the masks. Returns how it ended; raises
        only ``ValueError`` for invalid settings or masks (nothing written).

        ``ladder``: measured gain multipliers (nominal code/16 until the
        table exists). ``initial_gains``: the cameras' gains now, for
        ``start_point="current"`` (default: the sensor firmware's).
        ``threshold_guess``: this laser unit's stored threshold, if any.
        ``log_path``: CSV log to write. ``on_iteration`` is called with each
        record on this thread; an exception in it is logged and ignored.
        ``on_not_converged`` is asked, on this thread and with the trigger
        off, when the run hits its iteration limit without settling (e.g. a
        passing range too narrow): it returns :data:`CONTINUE` (another
        ``max_iterations``), :data:`ACCEPT` (keep the last settings, which
        then count as a pass) or anything else to stop and restore. Without
        it, such a run restores.
        """
        settings.validate(self.limits)
        cams = tuple(cameras_from_masks(left_camera_mask, right_camera_mask))
        if not cams:
            raise ValueError("no cameras in the masks")
        if not self._run_lock.acquire(blocking=False):
            raise RuntimeError("Optimize Laser is already running")
        try:
            # The new run replaces any active settings: its short scans must
            # not be prepared with the old ones, and it either passes (its
            # own settings become active) or restores the defaults.
            self._capture_stock = self._active is None
            self._set_active(None)
            result = self._run(settings, cams, left_camera_mask, right_camera_mask, ladder,
                               initial_gains, threshold_guess, knee_guess, max_iterations,
                               log_path, on_iteration, stop_evt or threading.Event(),
                               on_not_converged)
            if result.passed:
                self._set_active(result)
            return result
        finally:
            self._run_lock.release()

    def _run(self, settings, cams, left_mask, right_mask, ladder, initial_gains,
             threshold_guess, knee_guess, max_iterations, log_path, on_iteration,
             stop_evt, on_not_converged=None) -> OptimizeResult:
        limits = self.limits
        actuator = self.actuator()
        base = dict(config=settings, exposure_us=limits.exposure_us, cameras=cams)
        try:
            self._check_ready(cams)
            baseline = actuator.preflight(settings, limits)
        except Exception as e:
            logger.error("Optimize Laser refused: %s", e)
            return OptimizeResult(status=None, settings=None, error=str(e), **base)
        if self._capture_stock:
            self._stock = baseline

        initial = ExposureSettings(
            gain_codes={}, gains=dict(self._initial_gains(cams, initial_gains)),
            pulse_width_us=baseline.ta_pulse_width_us, current_ma=baseline.ta_current_ma)
        controller = ExposureController(
            settings, cams, limits=limits, ladder=ladder, initial=initial,
            threshold_guess=threshold_guess, knee_guess=knee_guess,
            max_iterations=max_iterations)
        log = None
        if log_path:
            try:
                log = _IterationCsv(log_path, self._csv_header(settings, baseline, cams))
            except OSError as e:
                logger.error("Optimize Laser refused: cannot write its log: %s", e)
                return OptimizeResult(status=None, settings=None, baseline=baseline,
                                      error=f"cannot write the Optimize Laser log: {e}", **base)
        t0 = time.monotonic()
        canceled, error, accepted = False, None, False
        try:
            actuator.loosen_limits(baseline, settings.current_bounds.high)
            while not controller.done:
                if stop_evt.is_set():
                    canceled = True
                    break
                observations = self._iterate(actuator, controller, left_mask, right_mask,
                                             stop_evt)
                if observations is None:
                    canceled = True
                    break
                record = controller.update(observations)
                if log is not None:
                    log.write(record, time.monotonic() - t0, method=controller.config.method,
                              exposure_us=limits.exposure_us,
                              truncation_us=limits.pulse_truncation_us)
                if on_iteration is not None:
                    try:
                        on_iteration(record)
                    except Exception:
                        logger.exception("Optimize Laser on_iteration callback raised")
                if (controller.status is OptimizeStatus.NOT_CONVERGED
                        and on_not_converged is not None and not stop_evt.is_set()):
                    choice = self._ask(on_not_converged, record)
                    if stop_evt.is_set():
                        canceled = True
                    elif choice == CONTINUE:
                        controller.resume(max_iterations)
                    elif choice == ACCEPT:
                        accepted = True
                        logger.warning("Optimize Laser: did not converge; the caller accepted "
                                       "the last settings")
        except Exception as e:
            logger.exception("Optimize Laser aborted")
            error = str(e)

        status = controller.status if controller.done else None
        passed = (status is not None and (status.passed or accepted)
                  and not canceled and error is None)
        restore_problems: tuple[str, ...] = ()
        if not passed:
            restore_problems = (tuple(self._restore(actuator, cams))
                                + tuple(self._reset_trigger_config()))
        result = OptimizeResult(
            status=status, settings=controller.settings if passed else None,
            records=tuple(controller.records), csv_path=log.path if log else None,
            learned_threshold_ma=controller.learned_threshold(), baseline=baseline,
            canceled=canceled, error=error, restored=not passed,
            restore_problems=restore_problems, accepted=accepted and passed, **base)
        outcome = (status.value + (" (accepted)" if result.accepted else "") if status
                   else ("canceled" if canceled else "error"))
        if log is not None:
            log.close({"result": outcome, "error": error or "",
                       "iterations": result.iterations, "restored": result.restored,
                       "elapsed_s": round(time.monotonic() - t0, 2),
                       "learned_threshold_ma": _num(result.learned_threshold_ma, 1)})
        logger.info("Optimize Laser done: %s after %d iteration(s)%s", outcome,
                    result.iterations, ", defaults restored" if result.restored else "")
        return result

    @staticmethod
    def _ask(on_not_converged, record) -> str:
        try:
            return on_not_converged(record)
        except Exception:
            logger.exception("Optimize Laser on_not_converged callback raised; stopping")
            return "stop"

    def _iterate(self, actuator, controller, left_mask, right_mask,
                 stop_evt) -> dict[Camera, CameraObservation] | None:
        """Apply the controller's settings, run one short scan, and return
        the observations; None if the scan was canceled."""
        s = controller.settings
        actuator.apply(s.gain_codes, s.pulse_width_us, s.current_ma, self.limits.exposure_us)
        needed = controller.frames_needed
        sink = _OptimizeLaserSink(controller.cameras, needed)
        frames = WARMUP_FRAMES + 1 + needed + SPARE_FRAMES
        scan_workflow = self._interface.scan_workflow
        try:
            completed = run_collection_scan(
                scan_workflow, sink, subject_id="_optimize_laser",
                duration_sec=max(1.0, frames / FRAME_RATE_HZ),
                left_camera_mask=left_mask, right_camera_mask=right_mask,
                stop_evt=stop_evt, raise_on_error=True,
                trigger_config=self.trigger_config(s.pulse_width_us))
        except RuntimeError as e:
            reason = scan_workflow.last_scan_error
            if reason and reason not in str(e):
                raise RuntimeError(f"{e} ({reason})") from e
            raise
        if not completed or stop_evt.is_set():
            return None
        observations = sink.observations()
        if not observations:
            raise RuntimeError(
                "the scan delivered no usable frames (a dark frame then light frames) "
                f"from any camera: {sink.light_frame_counts()}")
        short = {cam: n for cam, n in sink.light_frame_counts().items() if n < needed}
        if short:
            logger.warning("Optimize Laser: fewer than %d light frames from %s", needed, short)
        return observations

    # ---- after a run ------------------------------------------------------

    def trigger_config(self, pulse_width_us: float) -> dict:
        """Per-scan trigger override for scans at the optimized settings.

        The 2400 us dark skip keeps the displaced dark pulse out of a 2196 us
        exposure (loosen_limits lowered RATE_LL for it). The pulse width is
        mirrored into the trigger config so scan records show it; the TA
        driver register is what times the emitted pulse.
        """
        return {"LaserPulseSkipDelayUsec": DARK_SKIP_US,
                "LaserPulseWidthUsec": int(round(pulse_width_us))}

    def scan_overrides(self, result: OptimizeResult) -> dict:
        """``ScanRequest`` fields for a scan at a passed run's settings:
        ``trigger_config``, ``camera_gains`` (true multipliers, (2, 8)) and
        ``calibration_override`` (BVI full scale at twice the target)."""
        if not result.passed:
            raise ValueError("Optimize Laser did not pass; its settings are not active")
        gains = np.tile(np.asarray(CAMERA_GAIN_MAP, dtype=np.float64), (2, 1))
        for (side, pos), gain in result.settings.gains.items():
            gains[side, pos] = gain
        return {
            "trigger_config": self.trigger_config(result.settings.pulse_width_us),
            "camera_gains": gains,
            "calibration_override": {"i_max": 2.0 * result.config.target_mean},
        }

    def reapply(self, result: OptimizeResult, left_camera_mask: int,
                right_camera_mask: int) -> None:
        """Put a passed run's settings back on the device before a scan or a
        CQ check (a console reconnect resets the laser driver to baseline).
        Same preflight and limit order as the run; raises on any problem.

        Only the run's cameras that are in these masks are written: a scan on
        fewer cameras powers the others off, and a write to one fails."""
        if not result.passed:
            raise ValueError("Optimize Laser did not pass; there are no settings to apply")
        scanned = set(cameras_from_masks(left_camera_mask, right_camera_mask))
        actuator = self.actuator()
        self._check_ready(scanned.intersection(result.cameras))
        baseline = actuator.preflight(result.config, self.limits)
        actuator.loosen_limits(baseline, result.config.current_bounds.high)
        s = result.settings
        gain_codes = {cam: code for cam, code in s.gain_codes.items() if cam in scanned}
        actuator.apply(gain_codes, s.pulse_width_us, s.current_ma, result.exposure_us)

    def restore_defaults(self, left_camera_mask: int | None = None,
                         right_camera_mask: int | None = None, *,
                         reason: str | None = None) -> list[str]:
        """Laser driver, safety limits and cameras back to their defaults
        (``apply_laser_power`` plus the sensor firmware's camera settings),
        and the stock trigger config back on the console. Returns what could
        not be restored; empty means all of it.

        Without masks, the cameras are the active run's, or every camera on a
        connected sensor when nothing is active. The active settings are
        dropped either way: re-applying them over a console whose restore
        half-failed would compound it."""
        if left_camera_mask is None or right_camera_mask is None:
            cams = (self._active.cameras if self._active is not None
                    else self._connected_cameras())
        else:
            cams = cameras_from_masks(left_camera_mask, right_camera_mask)
        problems = self._restore(self.actuator(), cams) + self._reset_trigger_config()
        self._set_active(None, reason)
        return problems

    def _restore(self, actuator, cams) -> list[str]:
        return actuator.restore(cams, stock=self._stock, burst=self._stock_burst)

    def _stock_burst(self) -> None:
        """Fire the laser briefly with the stock trigger config. The caller
        has just put the TA pulse width and current back to stock; the
        safety limits are still the widened ones. Raises if the console
        won't take the config or start."""
        iface = self._interface
        console = iface.console
        applied = console.set_trigger_json(data=iface.resolve_trigger_config())
        if applied is None and not getattr(getattr(console, "uart", None), "demo_mode", False):
            raise RuntimeError("the console rejected the stock trigger config")
        if not console.start_trigger():
            reason = getattr(console, "last_start_refusal_reason", None)
            raise RuntimeError("the trigger did not start" + (f" ({reason})" if reason else ""))
        try:
            time.sleep(RESTORE_BURST_S)
        finally:
            console.stop_trigger()
        time.sleep(RESTORE_BURST_SETTLE_S)

    def prepare_scan(self, left_camera_mask: int, right_camera_mask: int) -> dict | None:
        """What ``ScanWorkflow.start_scan`` calls before every scan (trigger
        off): None when no settings are active, else the settings are
        re-applied and the scan's request overrides returned
        (:meth:`scan_overrides` plus a ``session_meta.optimize_laser``
        record).

        Raises ``RuntimeError`` when the scan must not start: the re-apply
        failed (its 2400 us dark skip under an unwidened ``RATE_LL`` would
        latch the safety interlock, E-202), or the gate dropped the settings
        and the laser side could not be restored.
        """
        result = self._active
        if result is None or self.running:
            return None
        reason = self._gate_reason()
        if reason:
            logger.warning("Optimize Laser settings dropped before this scan: %s", reason)
            blocking = [p for p in self.restore_defaults(reason=reason)
                        if p.startswith(LASER_RESTORE_PROBLEMS)]
            if blocking:
                raise RuntimeError("Could not restore the laser defaults: "
                                   + "; ".join(blocking)
                                   + ". Power-cycle the console to be sure.")
            return None
        try:
            self.reapply(result, left_camera_mask, right_camera_mask)
        except Exception as e:
            logger.error("Optimize Laser settings could not be re-applied: %s", e)
            raise RuntimeError(
                f"The Optimize Laser settings could not be re-applied ({e}). "
                "Restore the default laser settings or run Optimize Laser again.") from e
        logger.warning("OPTIMIZE LASER SETTINGS ACTIVE for this scan: %s us / %.0f mA",
                       result.settings.pulse_width_us, result.settings.current_ma)
        return {**self.scan_overrides(result),
                "extra_session_meta": {"optimize_laser": self.session_record(result)}}

    def session_record(self, result: OptimizeResult) -> dict:
        """A passed run's settings as plain JSON types, for the
        ``session_meta.optimize_laser`` of every scan that runs at them."""
        if not result.passed:
            raise ValueError("Optimize Laser did not pass; its settings are not active")
        s, cfg = result.settings, result.config

        def label(cam):
            return f"{('L', 'R')[cam[0]]}{cam[1] + 1}"

        return {
            "status": result.status.value,
            "accepted": result.accepted,
            "iterations": result.iterations,
            "method": getattr(cfg.method, "value", cfg.method),
            "target_mean_dn": float(cfg.target_mean),
            "passing_range_dn": float(cfg.passing_range),
            "exposure_us": float(result.exposure_us),
            "pulse_width_us": float(s.pulse_width_us),
            "emitted_pulse_width_us": round(
                float(s.pulse_width_us) - self.limits.pulse_truncation_us, 1),
            "current_ma": round(float(s.current_ma), 2),
            "dark_skip_us": DARK_SKIP_US,
            "bvi_i_max_dn": 2.0 * float(cfg.target_mean),
            "gains": {label(c): float(g) for c, g in sorted(s.gains.items())},
            "gain_codes": {label(c): int(g) for c, g in sorted(s.gain_codes.items())},
            "log": os.path.basename(result.csv_path) if result.csv_path else "",
        }

    def _gate_reason(self) -> str | None:
        if self.gate is None:
            return None
        try:
            return self.gate() or None
        except Exception as e:
            # A gate that can't answer must not leave widened limits in use.
            logger.exception("Optimize Laser gate raised")
            return f"the Optimize Laser gate failed ({e})"

    def _connected_cameras(self) -> list[Camera]:
        iface = self._interface
        return [(side, pos)
                for side, sensor in enumerate((iface.left, iface.right))
                if sensor is not None and sensor.is_connected()
                for pos in range(8)]

    def _reset_trigger_config(self) -> list[str]:
        """Send the interface's stock trigger config (not started).

        The console keeps the last trigger config it was sent, which after an
        Optimize Laser scan carries the 2400 us dark skip. Every scan path
        re-sends its own config first, but a direct trigger start would run
        the held one under the restored RATE_LL and latch E-202 on the first
        dark frame (bench, 2026-10-28). Returns what failed (empty = done).
        """
        iface = self._interface
        console = iface.console
        try:
            applied = console.set_trigger_json(data=iface.resolve_trigger_config())
        except Exception as e:
            return [f"trigger config reset failed: {e}"]
        if applied is None and not getattr(getattr(console, "uart", None), "demo_mode", False):
            return ["trigger config reset failed: the console rejected the stock trigger config"]
        logger.info("Optimize Laser: stock trigger config restored")
        return []

    # ---- helpers ----------------------------------------------------------

    def _check_ready(self, cams) -> None:
        iface = self._interface
        scan_workflow = iface.scan_workflow
        if scan_workflow.running:
            raise RuntimeError("a scan is running")
        for side in sorted({c[0] for c in cams}):
            sensor = (iface.left, iface.right)[side]
            if sensor is None or not sensor.is_connected():
                raise RuntimeError(f"{('left', 'right')[side]} sensor not connected")

    @staticmethod
    def _initial_gains(cams, initial_gains) -> dict[Camera, float]:
        gains = {c: FW_DEFAULT_GAINS[c[1]] for c in cams}
        for cam, gain in (initial_gains or {}).items():
            if cam in gains:
                gains[cam] = float(gain)
        return gains

    def _csv_header(self, settings: OptimizeLaserSettings, baseline: LaserBaseline,
                    cams) -> dict:
        console = self._interface.console
        try:
            hwid = console.get_hardware_id()
        except Exception:
            hwid = ""
        bounds = ", ".join(f"row {i + 1} {b.low:g}-{b.high:g}"
                           for i, b in enumerate(settings.gain_bounds))
        started = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
        return {
            "Optimize Laser log": started,
            "console_firmware": baseline.firmware,
            "console_hardware_id": hwid or "",
            "cameras": " ".join(f"{('L', 'R')[s]}{p + 1}" for s, p in cams),
            "exposure_us": self.limits.exposure_us,
            "method": getattr(settings.method, "value", settings.method),
            "target_mean_dn": f"{settings.target_mean:g} +/- {settings.passing_range:g}",
            "start_point": getattr(settings.start_point, "value", settings.start_point),
            "order": ",".join(getattr(k, "value", k) for k in settings.order),
            "gain_mode": getattr(settings.gain_mode, "value", settings.gain_mode),
            "gain_priority": getattr(settings.gain_priority, "value", settings.gain_priority),
            "gain_bounds": bounds,
            "pulse_width_bounds_us": f"{settings.pulse_width_bounds.low:g}-"
                                     f"{settings.pulse_width_bounds.high:g}",
            "current_bounds_ma": f"{settings.current_bounds.low:g}-"
                                 f"{settings.current_bounds.high:g}",
            "frames_per_iteration": settings.frames_per_iteration,
            "in_range_points": settings.in_range_points,
            "dark_skip_us": DARK_SKIP_US,
            "baseline": ", ".join(
                [f"TA_PULSE_WIDTH {baseline.ta_pulse_width_us:.1f}",
                 f"TA_CURRENT_DRV {baseline.ta_current_ma:.1f}",
                 f"TA_CURRENT_LIMIT {baseline.ta_current_limit_ma:.0f}"]
                + [f"{n} {v:g}" for n, v in baseline.registers.items()]),
        }
