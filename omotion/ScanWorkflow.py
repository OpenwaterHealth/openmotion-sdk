import concurrent.futures
import datetime
import csv
import logging
import math
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional, TYPE_CHECKING

from omotion import _log_root
from omotion.connection_state import ConnectionState
from omotion.MotionProcessing import (
    CorrectedBatch,
    HISTO_SIZE_WORDS,
)
from omotion.Calibration import Calibration

if TYPE_CHECKING:
    from omotion.MotionInterface import MotionInterface

logger = logging.getLogger(f"{_log_root}.ScanWorkflow" if _log_root else "ScanWorkflow")

# ---------------------------------------------------------------------------
# Null CSV writer — used when a raw-stream writer thread must keep running
# (to feed the science pipeline) but file I/O has been disabled.
# ---------------------------------------------------------------------------



# ---------------------------------------------------------------------------
# ConsoleTelemetry CSV helpers
# ---------------------------------------------------------------------------

_TELEMETRY_HEADERS: list[str] = [
    "timestamp",
    "tcm", "tcl", "pdc",
    "tec_v_raw", "tec_set_raw", "tec_curr_raw", "tec_volt_raw", "tec_good",
    *[f"pdu_raw_{i}" for i in range(16)],
    *[f"pdu_volt_{i}" for i in range(16)],
    "safety_se", "safety_so", "safety_ok",
    "read_ok", "error",
]


def _snap_to_row(snap) -> list:
    """Convert a ConsoleTelemetry snapshot to a flat CSV row."""
    row: list = [
        snap.timestamp,
        snap.tcm, snap.tcl, snap.pdc,
        snap.tec_v_raw, snap.tec_set_raw, snap.tec_curr_raw, snap.tec_volt_raw,
        int(snap.tec_good),
    ]
    pdu_raws = snap.pdu_raws or []
    pdu_volts = snap.pdu_volts or []
    for i in range(16):
        row.append(pdu_raws[i] if i < len(pdu_raws) else "")
    for i in range(16):
        row.append(pdu_volts[i] if i < len(pdu_volts) else "")
    row.extend([
        snap.safety_se, snap.safety_so, int(snap.safety_ok),
        int(snap.read_ok), snap.error or "",
    ])
    return row


class _TelemetryCsvWriter:
    """Listener registered on ConsoleTelemetryPoller that writes each snapshot
    to ``{scan_id}_{subject_id}_telemetry.csv`` for the duration of one scan.

    Constructed and started by ``start_scan``; ``close()`` is called from the
    worker's ``finally`` block. The listener runs on the poller thread, so
    ``__call__`` must stay non-blocking.
    """

    def __init__(self, path: str, poller) -> None:
        self._poller = poller
        self._file = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(_TELEMETRY_HEADERS)
        self._file.flush()
        self.path = path
        poller.add_listener(self)

    def __call__(self, snap) -> None:
        try:
            self._writer.writerow(_snap_to_row(snap))
            self._file.flush()
        except Exception:
            logger.exception("telemetry CSV write failed")

    def close(self) -> None:
        try:
            self._poller.remove_listener(self)
        except Exception:
            pass
        try:
            self._file.close()
        except Exception:
            pass


@dataclass
class ScanRequest:
    subject_id: str
    duration_sec: int
    left_camera_mask: int
    right_camera_mask: int
    disable_laser: bool = False
    expected_size: int = 32837
    # CSV output flags — all enabled by default.  Flip to False once the
    # corresponding downstream consumer no longer needs the file, so the
    # pipeline avoids unnecessary disk I/O.
    write_corrected_csv: bool = True
    write_telemetry_csv: bool = True
    # When True, the pipeline averages all active cameras per side into
    # single left/right BFI/BVI values.  The corrected CSV contains only
    # bfi_left, bfi_right, bvi_left, bvi_right columns.  Uncorrected
    # samples emitted to the UI are also averaged per-side per-frame.
    reduced_mode: bool = False
    # Pipeline sinks list — will be injected by the runner at start_scan time.
    # Normally managed by the SDK at MotionInterface construction (data_dir, scan_db_path).
    sinks: list = field(default_factory=list)
    # Skip injecting default storage sinks (CSV, DB). Set to True when
    # a caller supplies all sinks via the sinks list.
    skip_default_storage: bool = False
    # Duration cap for raw histogram output. If set and > 0, includes Tee("raw")
    # with max_duration_s. If 0 or negative, omits raw output.
    raw_save_max_duration_s: float | None = None
    # Number of histogram frames to batch before pushing to the pipeline.
    batch_size_frames: int = 10
    # Trigger-config override sent to the console before the scan starts.
    # start_scan always (re)sends a resolved trigger config so the firmware
    # fsync/dark schedule is reset and aligned to the pipeline's dark-frame
    # classification — without this the boundary dark frame can land on a
    # laser-on frame and corrupt dark correction. None -> the interface's
    # resolved default (DEFAULT_TRIGGER_CONFIG ⊕ constructor override). A dict
    # here is shallow-merged on top (e.g. {"TriggerFrequencyHz": 20}).
    trigger_config: dict | None = None


@dataclass
class ScanResult:
    ok: bool
    error: str
    left_path: str
    right_path: str
    canceled: bool
    scan_timestamp: str
    corrected_path: str = ""
    telemetry_path: str = ""
    # Populated when the science pipeline detected schedule/measurement
    # disagreement on dark frames (firmware off-by-one, unwrapper
    # alignment quirk, or significant ambient light in a dark slot).
    # Empty on a clean scan. Diagnostic only — calibration no longer
    # aborts on this; the per-camera FT dark mean-max check gates that.
    dark_integrity_warnings: list[str] = field(default_factory=list)


@dataclass
class ConfigureRequest:
    left_camera_mask: int
    right_camera_mask: int
    power_off_unused_cameras: bool = False


@dataclass
class ConfigureResult:
    ok: bool
    error: str


def run_collection_scan(
    scan_workflow,
    collector,
    *,
    subject_id: str,
    duration_sec,
    left_camera_mask: int,
    right_camera_mask: int,
    disable_laser: bool = False,
    reduced_mode: bool = False,
    stop_evt: "threading.Event | None" = None,
    poll_interval_s: float = 0.1,
    timeout_pad_s: float = 2.0,
    raise_on_error: bool = False,
) -> bool:
    """Run one short scan whose only sink is ``collector``, blocking until it
    finishes. This is the shared "threshold scan" engine behind both the
    contact-quality check and the calibration/test sub-scans: build a
    ``ScanRequest`` with default storage skipped and just the collector
    attached, start it, and wait. The collector's own per-camera accumulation
    and verdict logic stays specialized to each caller.

    ``stop_evt`` (calibration): poll in ``poll_interval_s`` slices and
    ``cancel_scan()`` if the event is set, returning ``False`` (canceled).
    Without it (CQ): a single ``await_complete(duration_sec + timeout_pad_s)``.
    ``raise_on_error``: a refused start or a scan-level error raises
    ``RuntimeError``. Returns ``False`` when the scan was canceled, else ``True``.

    Operates only on ``scan_workflow``'s public surface (``start_scan`` /
    ``await_complete`` / ``running`` / ``cancel_scan`` / ``last_scan_error`` /
    ``last_scan_canceled``) so callers can pass a real ``ScanWorkflow`` or a
    compatible mock.
    """
    request = ScanRequest(
        subject_id=subject_id,
        duration_sec=int(math.ceil(duration_sec)),
        left_camera_mask=left_camera_mask,
        right_camera_mask=right_camera_mask,
        disable_laser=disable_laser,
        reduced_mode=reduced_mode,
        sinks=[collector],
        skip_default_storage=True,
    )
    started = scan_workflow.start_scan(request)
    if raise_on_error and not started:
        raise RuntimeError("ScanWorkflow refused start_scan.")

    if stop_evt is None:
        scan_workflow.await_complete(timeout_sec=duration_sec + timeout_pad_s)
    else:
        while scan_workflow.running:
            scan_workflow.await_complete(timeout_sec=poll_interval_s)
            if stop_evt.is_set() and scan_workflow.running:
                try:
                    scan_workflow.cancel_scan()
                except Exception:
                    pass
                scan_workflow.await_complete(timeout_sec=5.0)
                return False

    if raise_on_error and scan_workflow.last_scan_error:
        raise RuntimeError(f"sub-scan failed: {scan_workflow.last_scan_error}")
    return not scan_workflow.last_scan_canceled


class ScanWorkflow:
    def __init__(self, interface: "MotionInterface"):
        self._interface = interface
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()
        self._running = False
        self._lock = threading.Lock()
        self._config_thread: threading.Thread | None = None
        self._config_stop_evt = threading.Event()
        self._config_running = False

        self._calibration: Calibration = Calibration.default()

        # Phase E: new-pipeline runner + scan thread (set synchronously
        # in start_scan before the worker thread spawns so test assertions
        # on _runner are valid immediately after start_scan returns).
        self._runner = None
        self._scan_thread: threading.Thread | None = None

        # Per-scan state for the disconnect-abort subscription. Reset at
        # each scan start by _scan_subscribe_state.
        self._scan_subs: list[tuple] = []  # (signal, handler) pairs
        self._scan_active_handles: list = []
        self._scan_abort_reason: str | None = None

        # Per-scan active (side, mask, sensor) tuples, snapshotted by the
        # worker once it resolves the request. cancel_scan reads this to
        # tear down cameras BEFORE source.close() — without it, drain_final
        # fights the still-capturing firmware for several seconds.
        self._scan_active_sides: list = []

        # Set by the worker thread's finally block. Callers (notably
        # CalibrationWorkflow) check this after await_complete to decide
        # whether the scan succeeded.
        self._last_scan_error: str | None = None
        self._last_scan_canceled: bool = False
        # Label of the scan-DB session for the most recent start_scan, set
        # synchronously when the scan's metadata is built. Lets callers bind
        # to THIS scan's session row (e.g. the app's live plot DB tail) by
        # exact label instead of guessing the newest session.
        self._current_scan_label: str | None = None
        # Cancel signal flag. Distinct from _stop_evt (which is also pulsed
        # by the worker's inner finally to wake the duration guard on a
        # clean exit). Set ONLY by the user-cancel paths — cancel() and
        # cancel_scan() — so _last_scan_canceled isn't tripped by normal
        # scan completion.
        self._cancel_requested: bool = False

    @property
    def running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def config_running(self) -> bool:
        with self._lock:
            return self._config_running

    def set_realtime_calibration(
        self,
        bfi_c_min,
        bfi_c_max,
        bfi_i_min,
        bfi_i_max,
    ) -> None:
        """Override the cached calibration. Marks source as ``override``.

        Validates shapes — historically this method silently stored
        ``None``s if shapes were wrong; that is now a ``ValueError``.
        """
        import numpy as np
        arrs = []
        for name, val in (
            ("bfi_c_min", bfi_c_min), ("bfi_c_max", bfi_c_max),
            ("bfi_i_min", bfi_i_min), ("bfi_i_max", bfi_i_max),
        ):
            arr = np.asarray(val, dtype=float)
            if arr.shape != (2, 8):
                raise ValueError(
                    f"set_realtime_calibration: {name} has shape "
                    f"{arr.shape}; expected (2, 8)"
                )
            arrs.append(arr)
        self._install_calibration(
            Calibration(
                c_min=arrs[0],
                c_max=arrs[1],
                i_min=arrs[2],
                i_max=arrs[3],
                source="override",
            )
        )

    def _install_calibration(self, cal: Calibration) -> None:
        """Replace the cached calibration. Used by the connect-time loader
        and by ``set_realtime_calibration``.
        """
        self._calibration = cal
        logger.info("Calibration installed (source=%s).", cal.source)

    def get_single_histogram(
        self,
        side: str,
        camera_id: int,
        test_pattern_id: int = 4,
        auto_upload: bool = True,
    ):
        side_key = (side or "").strip().lower()
        if side_key not in ("left", "right"):
            logger.error("Invalid side for get_single_histogram: %s", side)
            return None
        sensor = getattr(self._interface, side_key, None)
        if sensor is None or not sensor.is_connected():
            logger.error("%s sensor not connected", side_key.capitalize())
            return None
        return sensor.get_camera_histogram(
            camera_id=int(camera_id),
            test_pattern_id=int(test_pattern_id),
            auto_upload=bool(auto_upload),
        )

    def start_scan(
        self,
        request: ScanRequest,
        **_legacy_kwargs,
    ) -> bool:
        """Drive a scan through the new pipeline.

        Constructs ScanMetadata, default_pipeline, LiveUsbSource,
        auto-injects default storage sinks (unless request.skip_default_storage),
        and spawns a worker thread that runs the ScanRunner.

        Sets ``self._runner`` synchronously before the thread starts so
        callers can inspect runner attributes immediately after this method
        returns True.

        ``_legacy_kwargs`` silently discards callback arguments from pre-Phase-E
        callers (on_complete_fn, on_corrected_batch_fn, etc.) so they don't
        hard-crash before Phase G migrates those callers to the sink API.
        """
        if _legacy_kwargs:
            logger.warning(
                "start_scan: ignoring legacy callback kwargs %s — "
                "migrate callers to the sink API (Phase G).",
                sorted(_legacy_kwargs.keys()),
            )
        from omotion.pipeline.factory import default_pipeline
        from omotion.pipeline.runner import ScanRunner
        from omotion.pipeline.sources import LiveUsbSource
        from omotion.pipeline.sinks import CsvSink as PipelineCsvSink, ScanDBSink, ScanMetadata
        from omotion.pipeline.pedestal import SensorPedestals, pedestal_for_fw

        # Clear the prior scan's outcome up front so a refusal below (busy, or
        # a pre-flight failure) never reports a stale error from an earlier run.
        self._last_scan_error = None

        with self._lock:
            if self._running or (self._thread and self._thread.is_alive()):
                logger.warning(
                    "start_scan refused: previous scan is still active "
                    "(_running=%s, thread alive=%s, thread=%s).",
                    self._running,
                    self._thread.is_alive() if self._thread else None,
                    self._thread,
                )
                return False
            self._running = True

        logger.info("start_scan: building pipeline for new scan")
        self._stop_evt = threading.Event()

        # ── Build ScanMetadata ────────────────────────────────────────────
        _now = datetime.datetime.now(datetime.timezone.utc)
        scan_id = _now.strftime("%Y%m%d_%H%M%S")
        meta = ScanMetadata(
            scan_id=scan_id,
            subject_id=request.subject_id,
            operator=getattr(self._interface, "operator_id", None) or "unknown",
            started_at_iso=_now.isoformat(),
            duration_sec=request.duration_sec,
            left_camera_mask=request.left_camera_mask,
            right_camera_mask=request.right_camera_mask,
            reduced_mode=request.reduced_mode,
        )
        # Mirror ScanDBSink.on_scan_start's label so callers can bind to this
        # exact session row (must match f"{scan_id}_{subject_id}" there).
        self._current_scan_label = f"{scan_id}_{request.subject_id}"

        # ── Build pedestals (safe: fall back to 64.0 if version unreadable) ─
        def _safe_pedestal(sensor) -> float:
            try:
                version_str = getattr(sensor, "_version", "v0.0.0")
                from omotion.MotionSensor import _parse_firmware_version
                return pedestal_for_fw(_parse_firmware_version(version_str))
            except Exception:
                return 64.0

        pedestals = SensorPedestals(
            left=_safe_pedestal(self._interface.left),
            right=_safe_pedestal(self._interface.right),
        )

        # ── Build pipeline ────────────────────────────────────────────────
        calibration = self._calibration
        pipeline = default_pipeline(
            metadata=meta,
            calibration=calibration,
            pedestals=pedestals,
            raw_save_max_duration_s=request.raw_save_max_duration_s,
        )

        # ── Auto-inject default storage sinks ─────────────────────────────
        default_sinks: list = []
        telemetry_writer: Optional[_TelemetryCsvWriter] = None
        if not request.skip_default_storage:
            data_dir = getattr(self._interface, "data_dir", None)
            scan_db_path = getattr(self._interface, "scan_db_path", None)
            if data_dir is not None:
                # Corrected CSV: honor request.write_corrected_csv when a
                # scan DB is configured (DB is the system of record, so
                # the CSV is opt-in). When NO DB is configured the CSV is
                # the only persisted record of corrected data — force it
                # on regardless of the request flag so a scan is never
                # silently unrecorded.
                write_corrected = (
                    True if scan_db_path is None
                    else bool(request.write_corrected_csv)
                )
                default_sinks.append(
                    PipelineCsvSink(output_dir=data_dir, write_corrected=write_corrected)
                )
            if scan_db_path is not None:
                # Pre-flight the scan DB before the laser fires. The DB is the
                # system of record for live per-camera BFI/BVI (and, when the
                # corrected CSV is opt-in and off, the ONLY record). If it
                # can't be opened there is nowhere to persist the scan, so
                # refuse now — failing fast, with the laser still off — rather
                # than run a scan whose data is silently lost. (ScanDBSink is
                # also marked ``critical`` so the runner aborts as a backstop
                # if the DB dies between this check and the worker start.)
                try:
                    from omotion.ScanDatabase import ScanDatabase
                    ScanDatabase(db_path=scan_db_path).close()
                except Exception as exc:
                    logger.exception(
                        "start_scan: scan database pre-flight failed (%s) — "
                        "aborting before laser start", scan_db_path,
                    )
                    self._last_scan_error = f"Scan database unavailable: {exc}"
                    with self._lock:
                        self._running = False
                    return False
                default_sinks.append(ScanDBSink(db_path=scan_db_path))
            # Telemetry CSV: per-scan snapshots from ConsoleTelemetryPoller.
            # Not a pipeline sink — the poller is its own daemon thread that
            # predates the sink-based architecture, so we register a listener
            # for the duration of the scan and unregister in _worker's finally.
            poller = getattr(getattr(self._interface, "console", None), "telemetry", None)
            if (
                request.write_telemetry_csv
                and data_dir is not None
                and poller is not None
            ):
                telemetry_path = os.path.join(
                    data_dir, f"{scan_id}_{request.subject_id}_telemetry.csv"
                )
                try:
                    telemetry_writer = _TelemetryCsvWriter(telemetry_path, poller)
                except Exception:
                    logger.exception("failed to open telemetry CSV %s", telemetry_path)
                    telemetry_writer = None
        all_sinks = default_sinks + list(request.sinks)

        # ── Build source + runner (set self._runner synchronously) ─────────
        source = LiveUsbSource(
            console=self._interface.console,
            left=self._interface.left,
            right=self._interface.right,
            batch_size_frames=request.batch_size_frames or 10,
            metadata=meta,
        )
        self._runner = ScanRunner(
            source=source,
            pipeline=pipeline,
            sinks=all_sinks,
        )

        # Anchor for TriggerStateEvent timestamps. Set before the worker
        # starts so the very first start_trigger emits at ~0.
        self._scan_t0_monotonic = time.monotonic()

        # ── Spawn worker thread ────────────────────────────────────────────
        def _worker():
            try:
                # ── Pre-flight hardware setup ─────────────────────────────
                active_sides = self._resolve_active_sides(
                    request.left_camera_mask, request.right_camera_mask
                )
                # Expose for cancel_scan's teardown.
                self._scan_active_sides = active_sides
                if not active_sides:
                    logger.warning(
                        "start_scan: no active sensors (demo mode or masks 0x00); "
                        "runner will iterate over empty source."
                    )
                else:
                    if not request.disable_laser:
                        for side, _, _ in active_sides:
                            res = self._interface.run_on_sensors(
                                "enable_camera_fsin_ext", target=side
                            )
                            if not self._ok_from_result(res, side):
                                logger.error(
                                    "Failed to enable external frame sync on %s.", side
                                )

                    time.sleep(0.1)

                    for side, mask, sensor in active_sides:
                        try:
                            q_side: queue.Queue = source._packet_queues.get(side)
                            if q_side is not None:
                                flushed = sensor.uart.histo.flush_stale_data(
                                    expected_size=request.expected_size
                                )
                                if flushed:
                                    logger.info(
                                        "Flushed %d stale bytes from %s USB endpoint "
                                        "before scan start.", flushed, side
                                    )
                        except Exception:
                            logger.warning(
                                "Could not flush stale USB data for %s.", side,
                                exc_info=True,
                            )

                    _scan_handles = (
                        [self._interface.console] + [s for _, _, s in active_sides]
                    )
                    self._scan_subscribe_state(handles=_scan_handles)

                    for side, mask, _ in active_sides:
                        res = self._interface.run_on_sensors(
                            "enable_camera", mask, target=side
                        )
                        if not self._ok_from_result(res, side):
                            logger.error(
                                "Failed to enable camera on %s (mask 0x%02X).", side, mask
                            )

                    # (Re)send the trigger config before starting the trigger.
                    # This resets the firmware fsync counter so its laser-skip
                    # (dark-frame) schedule starts fresh and aligns with the
                    # pipeline's dark-frame classification (boundary dark at
                    # abs_id == discard_count+1, then every dark_interval). The
                    # bloodflow app does this via QML; doing it here makes any
                    # direct SDK caller (contact-quality, examples) correct too.
                    # Idempotent for the app: its trigger ≈ this resolved config.
                    trigger_cfg = self._interface.resolve_trigger_config(
                        request.trigger_config
                    )
                    self._interface.console.set_trigger_json(data=trigger_cfg)
                    self._interface.console.start_trigger()
                    self._emit_trigger_event("ON")

                # ── Duration gate: stop the source when time is up ────────
                stop_after = float(request.duration_sec)

                def _duration_guard():
                    deadline = time.monotonic() + stop_after
                    while time.monotonic() < deadline:
                        if self._stop_evt.is_set():
                            break
                        time.sleep(0.2)
                    # Cancel race: cancel_scan() already closed the source
                    # (and likely called stop_trigger) before we got here.
                    # Don't re-run stop_trigger/close on a closed source —
                    # it can blow up on torn-down USB endpoints and would
                    # block on the already-drained _batch_queue's sentinel
                    # slot. Just exit.
                    source_already_closing = getattr(source, "_stop", None) is not None \
                        and source._stop.is_set()
                    if source_already_closing:
                        logger.debug(
                            "duration_guard: source already closing (cancel path); "
                            "skipping stop_trigger + close"
                        )
                        return

                    # Teardown order matches the legacy SciencePipeline path:
                    #
                    #   stop_trigger -> 0.5s -> disable_camera -> 0.35s -> close
                    #
                    # Why disable_camera BEFORE source.close (stop_streaming):
                    # After stop_trigger the laser is off but the camera FSIN
                    # scheduler keeps capturing frames (now darks) into the
                    # firmware DMA buffer. If we stop_streaming first, the
                    # camera continues filling DMA with no host reader, and
                    # by the time disable_camera fires (in the worker's
                    # `finally`) the firmware ends up in a half-state that
                    # fails the next scan's READY check at flash time —
                    # "left camera N not READY for FPGA/config." Calling
                    # disable_camera first stops the DMA being filled at the
                    # source so close() drains cleanly and the camera lands
                    # in standby ready for the next flash.
                    try:
                        self._interface.console.stop_trigger()
                        self._emit_trigger_event("OFF")
                    except Exception:
                        logger.warning("stop_trigger raised in duration guard", exc_info=True)
                    time.sleep(0.5)
                    for side, mask, _ in active_sides:
                        try:
                            self._interface.run_on_sensors(
                                "disable_camera", mask, target=side
                            )
                        except Exception:
                            logger.warning(
                                "disable_camera(%s) raised in duration guard",
                                side, exc_info=True,
                            )
                    time.sleep(0.35)
                    try:
                        source.close()
                    except Exception:
                        logger.warning("source.close raised in duration guard", exc_info=True)

                guard_thread = threading.Thread(
                    target=_duration_guard, daemon=True, name="ScanWorkflow-guard"
                )
                guard_thread.start()

                # ── Run the pipeline ──────────────────────────────────────
                try:
                    self._runner.run()
                finally:
                    # Ensure guard exits promptly if runner returned early.
                    self._stop_evt.set()
                    guard_thread.join(timeout=2.0)

                    # stop_trigger + disable_camera are normally done by
                    # _duration_guard above; the calls here are idempotent
                    # safety nets for the cancel-from-cancel_scan path where
                    # the guard short-circuits on the source_already_closing
                    # check and never runs them.
                    try:
                        self._interface.console.stop_trigger()
                    except Exception:
                        pass

                    if active_sides:
                        self._scan_unsubscribe_state()
                        for side, mask, _ in active_sides:
                            try:
                                self._interface.run_on_sensors(
                                    "disable_camera", mask, target=side
                                )
                            except Exception:
                                pass
            except Exception as e:
                logger.exception("ScanWorkflow worker raised")
                self._last_scan_error = str(e) or type(e).__name__
            finally:
                if telemetry_writer is not None:
                    telemetry_writer.close()
                with self._lock:
                    self._running = False
                    self._thread = None
                # _stop_evt is also set by the inner finally to wake the
                # duration guard on a clean exit, so checking it here would
                # mark every scan as canceled. Use the dedicated flag set
                # only by the user-cancel paths.
                self._last_scan_canceled = self._cancel_requested

        # Reset per-scan outcome before spawning the worker so callers
        # don't see stale state from a prior run. (_last_scan_error is cleared
        # at the top of start_scan so refusals report no stale error.)
        self._last_scan_canceled = False
        self._cancel_requested = False

        self._thread = threading.Thread(
            target=_worker, daemon=True, name="ScanWorkflow-scan"
        )
        self._thread.start()
        return True

    def _emit_trigger_event(self, state: str) -> None:
        """Push a TriggerStateEvent to the current runner's diagnostics channel.

        Wrapped in try/except because failure to emit a diagnostic event
        must never abort a scan.
        """
        try:
            from omotion.pipeline.batch import TriggerStateEvent
            runner = self._runner
            if runner is None:
                return
            t0 = getattr(self, "_scan_t0_monotonic", None)
            ts = (time.monotonic() - t0) if t0 is not None else 0.0
            runner.dispatch_event(TriggerStateEvent(state=state, timestamp_s=ts))
        except Exception:
            logger.warning("_emit_trigger_event raised", exc_info=True)

    @property
    def last_scan_error(self) -> str | None:
        """Error message from the most recent scan worker, or None on
        success. Cleared at the start of each new scan."""
        return self._last_scan_error

    @property
    def last_scan_canceled(self) -> bool:
        """True if the most recent scan was canceled (stop_evt was set
        before the worker finished). Cleared at the start of each new scan."""
        return self._last_scan_canceled

    @property
    def current_scan_label(self) -> str | None:
        """Scan-DB session label (``f"{scan_id}_{subject_id}"``) for the most
        recent start_scan, or None if no scan has started. Set synchronously
        while building scan metadata, so it's valid as soon as start_scan
        returns. Callers use it to bind to this scan's exact session row."""
        return self._current_scan_label

    def await_complete(self, *, timeout_sec: float | None = None) -> None:
        """Block until the current scan worker thread finishes (or the
        optional timeout elapses).  Useful for callers that start a scan
        and want synchronous completion — notably the sink-based workflow
        helpers added in Phase D of the pipeline cutover.

        Does nothing if no scan is running.
        """
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=timeout_sec)

    def cancel(self) -> None:
        """Signal the running scan to stop.

        Closes the source (stopping the USB reader loops). The worker thread's
        duration guard exits on the next poll and the runner's finally block fires.
        """
        self._cancel_requested = True
        self._stop_evt.set()
        if self._runner is not None:
            try:
                self._runner.source.close()
            except Exception:
                pass

    def cancel_scan(self, *, join_timeout: float = 5.0) -> None:
        """User-facing cancel. Mirrors the normal end-of-scan teardown order
        so a manual Stop returns in ~1s instead of ~3s.

        Order (matches _duration_guard above):
          stop_trigger -> 0.5s -> disable_camera -> 0.35s -> source.close()

        Without disable_camera before close(), drain_final fights the
        still-capturing firmware DMA and blocks for 2-3s while the host
        endpoint buffer drains. With it, drain_final finds almost nothing
        and close() returns quickly. cancel_scan then sets _stop_evt and
        joins the worker thread.
        """
        # Stop the laser FIRST (laser-safety: fastest response).
        try:
            if self._interface and self._interface.console:
                self._interface.console.stop_trigger()
                self._emit_trigger_event("OFF")
        except Exception:
            logger.warning("stop_trigger raised in cancel_scan", exc_info=True)

        # Stop cameras capturing so the firmware DMA stops being filled.
        # Use the snapshotted active_sides from the running worker. If the
        # worker hasn't snapshotted yet (very early cancel), this is empty
        # and we just fall through to the close() — the worker will hit
        # its own cancel paths.
        active = list(self._scan_active_sides)
        if active:
            time.sleep(0.5)
            for side, mask, _ in active:
                try:
                    self._interface.run_on_sensors(
                        "disable_camera", mask, target=side
                    )
                except Exception:
                    logger.warning(
                        "disable_camera(%s) raised in cancel_scan",
                        side, exc_info=True,
                    )
            time.sleep(0.35)

        # Signal worker + tear down source. cancel() sets _stop_evt and
        # calls source.close(); the duration_guard sees source._stop set
        # and short-circuits its own teardown.
        self.cancel()

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=join_timeout)

    def start_configure_camera_sensors(
        self,
        request: ConfigureRequest,
        *,
        on_progress_fn: Callable[[int], None] | None = None,
        on_log_fn: Callable[[str], None] | None = None,
        on_complete_fn: Callable[[ConfigureResult], None] | None = None,
    ) -> bool:
        with self._lock:
            if self._config_running or (
                self._config_thread and self._config_thread.is_alive()
            ):
                return False
            self._config_running = True

        self._config_stop_evt = threading.Event()

        def _emit_progress(pct: int) -> None:
            if on_progress_fn:
                on_progress_fn(int(pct))

        def _emit_log(msg: str) -> None:
            logger.info(msg)
            if on_log_fn:
                on_log_fn(msg)

        def _worker():
            ok = False
            err = ""
            try:
                active = self._resolve_active_sides(
                    request.left_camera_mask, request.right_camera_mask
                )
                if not active:
                    raise RuntimeError("No active sensors to configure.")

                if request.power_off_unused_cameras:
                    _emit_log("Powering on cameras before programming FPGAs...")
                    for side, mask, sensor in active:
                        try:
                            power_status = sensor.get_camera_power_status()
                            if not power_status or len(power_status) != 8:
                                _emit_log(f"{side}: could not get camera power status")
                                continue
                            off_mask = sum(
                                1 << i
                                for i in range(8)
                                if power_status[i] and not (mask & (1 << i))
                            )
                            on_mask = mask & 0xFF
                            if off_mask:
                                if sensor.disable_camera_power(off_mask):
                                    _emit_log(
                                        f"{side}: powered off cameras not in mask (0x{off_mask:02X})"
                                    )
                                time.sleep(0.05)
                            if on_mask:
                                if sensor.enable_camera_power(on_mask):
                                    _emit_log(
                                        f"{side}: powered on cameras (mask 0x{on_mask:02X})"
                                    )
                                else:
                                    raise RuntimeError(
                                        f"Failed to power on cameras on {side} (mask 0x{on_mask:02X})."
                                    )
                                time.sleep(0.5)
                        except Exception as e:
                            raise RuntimeError(
                                f"Error setting camera power for {side}: {e}"
                            ) from e

                side_positions: dict[str, list[int]] = {}
                side_sensors: dict = {}
                for side, mask, sensor in active:
                    positions = [i for i in range(8) if (mask & (1 << i))]
                    side_positions[side] = positions
                    side_sensors[side] = sensor

                total = sum(len(p) * 2 for p in side_positions.values())
                if not total:
                    raise RuntimeError("Empty camera masks (left & right)")

                done = [0]
                done_lock = threading.Lock()
                _emit_progress(1)

                side_errors: dict[str, str] = {}

                def _configure_side(side: str) -> None:
                    sensor = side_sensors[side]
                    for pos in side_positions[side]:
                        if self._config_stop_evt.is_set():
                            raise RuntimeError("Canceled")

                        if not sensor or not sensor.is_connected():
                            raise RuntimeError(
                                f"{side} sensor not connected during configure."
                            )

                        cam_mask_single = 1 << pos
                        pos1 = pos + 1

                        status_map = sensor.get_camera_status(cam_mask_single)
                        if not status_map or pos not in status_map:
                            raise RuntimeError(
                                f"Failed to read camera status for {side} camera {pos1}."
                            )
                        status = status_map[pos]
                        if not status & (1 << 0):
                            raise RuntimeError(
                                f"{side} camera {pos1} not READY for FPGA/config."
                            )

                        _emit_log(
                            f"Programming {side} camera FPGA at position {pos1} "
                            f"(mask 0x{cam_mask_single:02X})..."
                        )
                        if not sensor.program_fpga(
                            camera_position=cam_mask_single, manual_process=False
                        ):
                            raise RuntimeError(
                                f"Failed to program FPGA on {side} sensor (pos {pos1})."
                            )
                        with done_lock:
                            done[0] += 1
                            _emit_progress(int((done[0] / total) * 100))

                        if self._config_stop_evt.is_set():
                            raise RuntimeError("Canceled")

                        time.sleep(0.1)
                        _emit_log(
                            f"Configuring {side} camera sensor registers "
                            f"at position {pos1}..."
                        )
                        if not sensor.camera_configure_registers(
                            camera_position=cam_mask_single
                        ):
                            raise RuntimeError(
                                f"camera_configure_registers failed on {side} "
                                f"at position {pos1}."
                            )
                        with done_lock:
                            done[0] += 1
                            _emit_progress(int((done[0] / total) * 100))

                # Run each sensor's full configure sequence in parallel
                with concurrent.futures.ThreadPoolExecutor(
                    max_workers=len(side_positions)
                ) as executor:
                    fs = {
                        executor.submit(_configure_side, side): side
                        for side in side_positions
                    }
                    for future in concurrent.futures.as_completed(fs):
                        side = fs[future]
                        exc = future.exception()
                        if exc is not None:
                            side_errors[side] = str(exc)

                if side_errors:
                    raise RuntimeError(
                        "; ".join(
                            f"{side}: {e}" for side, e in sorted(side_errors.items())
                        )
                    )

                ok = True
                _emit_log("FPGAs programmed & registers configured")
            except Exception as e:
                err = str(e)
                logger.error("Camera configure workflow error: %s", err)
            finally:
                if on_complete_fn:
                    on_complete_fn(ConfigureResult(ok=ok, error=err))
                with self._lock:
                    self._config_running = False
                    self._config_thread = None

        self._config_thread = threading.Thread(target=_worker, daemon=True)
        self._config_thread.start()
        return True

    def cancel_configure_camera_sensors(self, *, join_timeout: float = 5.0) -> None:
        self._config_stop_evt.set()
        if self._config_thread and self._config_thread.is_alive():
            self._config_thread.join(timeout=join_timeout)

    def _resolve_active_sides(self, left_mask: int, right_mask: int):
        sides_info = [
            ("left", left_mask, self._interface.left),
            ("right", right_mask, self._interface.right),
        ]
        active = []
        for side, mask, sensor in sides_info:
            if int(mask) == 0x00:
                continue
            if not sensor.is_connected():
                continue
            active.append((side, int(mask), sensor))
        return active

    @staticmethod
    def _ok_from_result(result, side: str) -> bool:
        if isinstance(result, dict):
            return bool(result.get(side))
        return bool(result)

    # ──────────────────────────────────────────────────────────────────
    # Mid-scan disconnect handling: abort immediately
    # ──────────────────────────────────────────────────────────────────
    #
    # ScanWorkflow subscribes to signal_state_changed on every handle the
    # scan is using (always the console, plus the participating sensors).
    # If any of them transitions to DISCONNECTING during the scan, we set
    # _scan_abort_reason and trip _stop_evt — the main scan loop polls
    # _stop_evt and unwinds via its existing finally cleanup. There is no
    # recovery: a scan with a missing device is invalid, so the next scan
    # starts cleanly from a known-good state.
    #
    # The scan worker's exception handler picks up _scan_abort_reason
    # in place of "Capture canceled" so the result message names the
    # specific handle that dropped.

    def _scan_subscribe_state(self, handles: list) -> None:
        self._scan_active_handles = list(handles)
        self._scan_abort_reason = None

        # ScanWorkflow is not a QObject. Auto-connection sees that and
        # tries to deliver cross-thread emits via the receiver's event
        # loop — which doesn't exist for a plain Python callable, so the
        # signal is silently dropped. Force DirectConnection so the slot
        # runs synchronously on the emitter thread (the monitor thread,
        # which is exactly where _on_scan_handle_state belongs).
        try:
            from PyQt6.QtCore import Qt
            _conn_type = Qt.ConnectionType.DirectConnection
        except ImportError:
            _conn_type = None  # MotionSignal shim path; emit is direct anyway

        for h in handles:
            handler = self._make_state_handler(h)
            try:
                if _conn_type is not None:
                    h.signal_state_changed.connect(handler, type=_conn_type)
                else:
                    h.signal_state_changed.connect(handler)
                self._scan_subs.append((h.signal_state_changed, handler))
                logger.info("scan: subscribed to %s state changes", h.name)
            except Exception as e:
                logger.warning("scan: failed to subscribe to %s: %s", h.name, e)

    def _scan_unsubscribe_state(self) -> None:
        for sig, handler in self._scan_subs:
            try:
                sig.disconnect(handler)
            except Exception:
                pass
        self._scan_subs = []
        self._scan_active_handles = []

    def _make_state_handler(self, handle):
        # Bound closure so we can disconnect the same callable later.
        def _handler(h, old, new, reason):
            self._on_scan_handle_state(h, old, new, reason)
        return _handler

    def _on_scan_handle_state(self, handle, old, new, reason: str) -> None:
        if not self.running:
            return
        if handle not in self._scan_active_handles:
            return
        if new == ConnectionState.DISCONNECTING and not self._stop_evt.is_set():
            logger.error(
                "scan: %s disconnected mid-scan (%s); aborting",
                handle.name, reason,
            )
            self._scan_abort_reason = (
                f"{handle.name} disconnected mid-scan ({reason})"
            )
            # Treat mid-scan disconnect the same as user-cancel for
            # downstream "is partial data trustworthy" decisions.
            self._cancel_requested = True
            self._stop_evt.set()
