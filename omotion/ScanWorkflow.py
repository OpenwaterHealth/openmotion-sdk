import concurrent.futures
import datetime
import csv
import logging
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, TYPE_CHECKING

from omotion import _log_root
from omotion.connection_state import ConnectionState
from omotion.MotionProcessing import (
    CorrectedBatch,
    HISTO_SIZE_WORDS,
    create_science_pipeline,
    parse_histogram_stream,
)
from omotion.Calibration import Calibration
from omotion.CsvSink import CsvSink

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


@dataclass
class ScanRequest:
    subject_id: str
    duration_sec: int
    left_camera_mask: int
    right_camera_mask: int
    data_dir: str
    disable_laser: bool
    expected_size: int = 32837
    # CSV output flags — all enabled by default.  Flip to False once the
    # corresponding downstream consumer no longer needs the file, so the
    # pipeline avoids unnecessary disk I/O.
    write_raw_csv: bool = True
    write_corrected_csv: bool = True
    write_telemetry_csv: bool = True
    # Maximum number of seconds for which raw histogram CSVs are written.
    # None (default) means write for the full scan duration.
    # Has no effect when write_raw_csv is False.
    raw_csv_duration_sec: float | None = None
    # When True, the pipeline averages all active cameras per side into
    # single left/right BFI/BVI values.  The corrected CSV contains only
    # bfi_left, bfi_right, bvi_left, bvi_right columns.  Uncorrected
    # samples emitted to the UI are also averaged per-side per-frame.
    reduced_mode: bool = False
    # When True, the pipeline emits a rolling-mean Sample (mean + contrast
    # only) via on_rolling_avg_fn once per uncorrected light frame per
    # camera.  Window size is rolling_avg_window.  Dark frames are
    # excluded from the window.
    rolling_avg_enabled: bool = False
    rolling_avg_window: int = 10
    # Database sink (issue #92, see docs/superpowers/specs/2026-04-14-scan-db-sink-design.md).
    # The DB endpoint itself is opt-in at SDK construction via
    # ``MotionInterface(db_path=...)``; these per-scan fields are only
    # effective when that path is set.
    write_raw_to_db: bool = False
    notes: str = ""


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

        # Per-scan state for the disconnect-abort subscription. Reset at
        # each scan start by _scan_subscribe_state.
        self._scan_subs: list[tuple] = []  # (signal, handler) pairs
        self._scan_active_handles: list = []
        self._scan_abort_reason: str | None = None

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
        *,
        extra_cols_fn: Callable[[], list] | None = None,
        on_log_fn: Callable[[str], None] | None = None,
        on_progress_fn: Callable[[int], None] | None = None,
        on_trigger_state_fn: Callable[[str], None] | None = None,
        on_uncorrected_fn: Callable[[object], None] | None = None,
        on_corrected_batch_fn: Callable[[object], None] | None = None,
        # Real-time dark-corrected stream (data-pipeline-tweaks). Fires
        # per non-dark frame once the predictor has warmed up (~15 s in).
        # Forwarded straight to ``create_science_pipeline``; pass None
        # to keep today's behavior.
        on_realtime_corrected_fn: Callable[[object], None] | None = None,
        # Hooks for the ScanDBSink (issue #92). The workflow itself stays
        # DB-unaware — it just fires these callbacks if supplied.
        on_raw_frame_fn: Callable[..., None] | None = None,
        on_scan_start_fn: Callable[[str, float], None] | None = None,
        on_dark_frame_fn: Callable[[object], None] | None = None,
        on_rolling_avg_fn: Callable[[object], None] | None = None,
        on_error_fn: Callable[[Exception], None] | None = None,
        on_side_stream_fn: Callable[[str, str], None] | None = None,
        on_complete_fn: Callable[[ScanResult], None] | None = None,
        log_dark_endpoints: bool = False,
    ) -> bool:
        with self._lock:
            if self._running or (self._thread and self._thread.is_alive()):
                logger.warning(
                    "start_scan refused: previous scan is still active "
                    "(_running=%s, thread alive=%s, thread=%s). The previous "
                    "worker has not reached its finally cleanup yet — most "
                    "likely a writer or science-pipeline thread didn't exit "
                    "within its join timeout.",
                    self._running,
                    self._thread.is_alive() if self._thread else None,
                    self._thread,
                )
                return False
            self._running = True
        logger.info("start_scan: spawning worker thread for new scan")

        self._stop_evt = threading.Event()

        def _emit_log(msg: str) -> None:
            logger.info(msg)
            if on_log_fn:
                on_log_fn(msg)

        def _worker():
            ok = False
            err = ""
            left_path = ""
            right_path = ""
            corrected_path = ""
            telemetry_path = ""
            ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            # Fire the scan-start hook so a downstream sink (e.g. the
            # ScanDBSink for issue #92) can open its session with the
            # canonical label and wall-clock start used by the rest of
            # the workflow.
            session_start_ts = time.time()
            if on_scan_start_fn is not None:
                try:
                    on_scan_start_fn(ts, session_start_ts)
                except Exception:
                    logger.exception("on_scan_start_fn callback raised")
            active_sides = []
            writer_threads: dict[str, threading.Thread] = {}
            writer_stops: dict[str, threading.Event] = {}
            writer_row_counts: dict[str, int] = {}
            writer_queues: dict[str, queue.Queue] = {}
            science_pipeline = None
            # Issue #44: dropped the ``_corrected`` suffix from the
            # canonical output. The corrected stream is the default,
            # so naming it doesn't add information; the raw histo CSVs
            # below now carry ``_raw`` to disambiguate. Old scans
            # written before this change still use ``_corrected.csv``;
            # downstream consumers (bloodflow-app's get_scan_list /
            # get_scan_details) accept both.
            corrected_path = os.path.join(
                request.data_dir, f"{ts}_{request.subject_id}.csv"
            )
            telemetry_path = os.path.join(
                request.data_dir, f"{ts}_{request.subject_id}_telemetry.csv"
            )
            # Telemetry CSV state (populated in try block if console is available)
            _telem_poller = None
            _telem_listener = None
            _telem_fh = None
            _telem_lock = threading.Lock()
            _telem_stop = threading.Event()

            # CsvSink owns every per-scan CSV output (corrected + per-side
            # raw). It opens its own files, builds the per-frame merge,
            # enforces ``raw_csv_duration_sec``, and drains any partial
            # corrected frames at scan end. ScanWorkflow fans events into
            # it via the standard Sink hooks.
            _csv_sink = CsvSink()

            # Per-scan time origin: captured from the first sample emitted by
            # parse_histogram_stream (whichever side fires first wins) and
            # subtracted from every subsequent sample's timestamp_s. Result:
            # every downstream consumer — raw CSV, science pipeline,
            # corrected CSV, on_uncorrected_fn / on_corrected_batch_fn /
            # on_raw_frame_fn callbacks — sees the same 0-based per-scan
            # time origin. Replaces the per-output `_corr_base_ts` offset
            # the corrected CSV writer used to compute by itself.
            _session_t0: float | None = None
            _t0_lock = threading.Lock()

            def _normalize_ts(ts: float) -> float:
                """Capture-once t0 normalizer passed into parse_histogram_stream
                so the first sample any writer thread sees defines t=0 for the
                whole scan."""
                nonlocal _session_t0
                if _session_t0 is None:
                    with _t0_lock:
                        if _session_t0 is None:
                            _session_t0 = float(ts)
                return float(ts) - _session_t0
            # Reduced-mode uncorrected sample accumulator: buffers per-camera
            # samples and emits a single averaged sample per side per frame.
            _reduced_uncorr_buf: dict[tuple[str, int], dict] = {}
            # Number of active cameras per side (computed after active_sides
            # is resolved; filled in below). Used by the uncorrected-sample
            # reduced-mode aggregation only — corrected CSV's own per-side
            # cam counts live inside CsvSink.
            _reduced_cam_counts: dict[str, int] = {}

            try:
                os.makedirs(request.data_dir, exist_ok=True)

                # Open the telemetry CSV and register a listener on the poller.
                # The guard handles headless configs where there is no console module.
                _telem_poller = getattr(
                    getattr(self._interface, "console", None), "telemetry", None
                )
                if _telem_poller is not None and request.write_telemetry_csv:
                    try:
                        _telem_fh = open(  # noqa: WPS515
                            telemetry_path, "w", newline="", encoding="utf-8"
                        )
                        _telem_csv = csv.writer(_telem_fh)
                        _telem_csv.writerow(_TELEMETRY_HEADERS)
                        _telem_fh.flush()

                        def _on_telemetry(snap):
                            if _telem_stop.is_set():
                                return
                            with _telem_lock:
                                if _telem_stop.is_set():
                                    return
                                try:
                                    _telem_csv.writerow(_snap_to_row(snap))
                                    _telem_fh.flush()
                                except Exception as _te:
                                    logger.debug("Telemetry CSV write error: %s", _te)

                        _telem_listener = _on_telemetry
                        _telem_poller.add_listener(_telem_listener)
                    except Exception as _telem_err:
                        _emit_log(f"Failed to open telemetry CSV: {_telem_err}")
                        telemetry_path = ""
                else:
                    telemetry_path = ""

                active_sides = self._resolve_active_sides(
                    request.left_camera_mask, request.right_camera_mask
                )
                if not active_sides:
                    raise RuntimeError(
                        "No active sensors to capture (both masks 0x00 or disconnected)."
                    )

                # Per-side active camera counts for reduced-mode uncorrected
                # aggregation. (CsvSink computes its own from the request
                # masks; this one stays for the uncorrected-sample buffer.)
                for _s, _m, _ in active_sides:
                    _cam_count = 0
                    for _i in range(8):
                        if _m & (1 << _i):
                            _cam_count += 1
                    _reduced_cam_counts[_s] = _cam_count

                # Corrected CSV is now owned by CsvSink (issue #92, Step B4a).
                # It opens its own file handle, writes the header, and flushes
                # complete rows as cameras contribute. We just hand it the
                # scan-start context and read back the path it chose so
                # ScanResult.corrected_path keeps the same meaning ("" when the
                # writer is disabled or failed to open).
                try:
                    _csv_sink.on_scan_start(
                        ts=ts,
                        session_start_ts=session_start_ts,
                        request=request,
                        meta={},
                    )
                except Exception as _corr_open_err:
                    _emit_log(f"Failed to open corrected CSV: {_corr_open_err}")
                corrected_path = _csv_sink.corrected_path or ""

                _emit_log("Preparing capture...")

                if not request.disable_laser:
                    _emit_log("Enabling external frame sync...")
                    for side, _, _ in active_sides:
                        res = self._interface.run_on_sensors(
                            "enable_camera_fsin_ext", target=side
                        )
                        if not self._ok_from_result(res, side):
                            raise RuntimeError(
                                f"Failed to enable external frame sync on {side}."
                            )

                time.sleep(0.1)

                _emit_log("Setting up streaming...")

                # Build one unified SciencePipeline that handles both sides
                # before starting any per-side writer threads.
                if self._calibration is not None:
                    left_mask_active = next(
                        (m for s, m, _ in active_sides if s == "left"), 0x00
                    )
                    right_mask_active = next(
                        (m for s, m, _ in active_sides if s == "right"), 0x00
                    )

                    def _on_uncorrected_sample(sample):
                        # Per-sample real-time callback (fires immediately for
                        # GUI with uncorrected BFI/BVI).
                        if request.reduced_mode:
                            # Buffer samples per (side, frame_id), emit averaged
                            # result once all active cameras for that side report.
                            key = (sample.side, int(sample.absolute_frame_id))
                            entry = _reduced_uncorr_buf.get(key)
                            if entry is None:
                                entry = {
                                    "bfi_sum": 0.0, "bvi_sum": 0.0,
                                    "count": 0,
                                    "timestamp_s": float(sample.timestamp_s),
                                    "frame_id": int(sample.frame_id),
                                    "abs_frame_id": int(sample.absolute_frame_id),
                                    "side": sample.side,
                                }
                                _reduced_uncorr_buf[key] = entry
                            entry["bfi_sum"] += float(sample.bfi)
                            entry["bvi_sum"] += float(sample.bvi)
                            entry["count"] += 1

                            expected = _reduced_cam_counts.get(sample.side, 1)
                            if entry["count"] >= expected:
                                from omotion.MotionProcessing import Sample
                                avg_sample = Sample(
                                    side=entry["side"],
                                    cam_id=0,
                                    frame_id=entry["frame_id"],
                                    absolute_frame_id=entry["abs_frame_id"],
                                    timestamp_s=entry["timestamp_s"],
                                    row_sum=0,
                                    temperature_c=0.0,
                                    mean=0.0,
                                    std_dev=0.0,
                                    contrast=0.0,
                                    bfi=entry["bfi_sum"] / entry["count"],
                                    bvi=entry["bvi_sum"] / entry["count"],
                                    is_corrected=False,
                                )
                                del _reduced_uncorr_buf[key]
                                # Evict stale entries (>5 frames behind)
                                stale = [
                                    k for k in _reduced_uncorr_buf
                                    if k[0] == sample.side
                                    and k[1] < entry["abs_frame_id"] - 5
                                ]
                                for sk in stale:
                                    del _reduced_uncorr_buf[sk]
                                if on_uncorrected_fn:
                                    on_uncorrected_fn(avg_sample)
                            return
                        if on_uncorrected_fn:
                            on_uncorrected_fn(sample)

                    def _on_corrected_batch(batch: CorrectedBatch):
                        # Fires once per (side, cam_id) per dark-frame interval
                        # with properly corrected BFI/BVI for that interval.
                        # Sample timestamps are already normalized to scan-start
                        # at the source (parse_histogram_stream's t0_normalizer),
                        # so the corrected CSV can write them directly without
                        # its own per-output offset.
                        #
                        # CSV merging + write logic lives in CsvSink (issue #92,
                        # Step B4a). This function now only fans the batch out:
                        # one copy to the sink for on-disk merging, one to the
                        # user-facing callback (averaged in reduced mode).
                        _csv_sink.on_corrected_batch(batch)

                        if request.reduced_mode:
                            if on_corrected_batch_fn:
                                from omotion.MotionProcessing import Sample
                                _batch_buf: dict[tuple[str, int], dict] = {}
                                for s in batch.samples:
                                    bk = (s.side, int(s.absolute_frame_id))
                                    be = _batch_buf.get(bk)
                                    if be is None:
                                        be = {"bfi_sum": 0.0, "bvi_sum": 0.0, "count": 0,
                                              "ts": float(s.timestamp_s),
                                              "frame_id": int(s.frame_id),
                                              "abs_frame_id": int(s.absolute_frame_id),
                                              "side": s.side}
                                        _batch_buf[bk] = be
                                    be["bfi_sum"] += float(s.bfi)
                                    be["bvi_sum"] += float(s.bvi)
                                    be["count"] += 1
                                avg_samples = []
                                for be in _batch_buf.values():
                                    cnt = max(1, be["count"])
                                    avg_samples.append(Sample(
                                        side=be["side"], cam_id=0,
                                        frame_id=be["frame_id"],
                                        absolute_frame_id=be["abs_frame_id"],
                                        timestamp_s=be["ts"],
                                        row_sum=0, temperature_c=0.0,
                                        mean=0.0, std_dev=0.0, contrast=0.0,
                                        bfi=be["bfi_sum"] / cnt,
                                        bvi=be["bvi_sum"] / cnt,
                                        is_corrected=True,
                                    ))
                                on_corrected_batch_fn(CorrectedBatch(
                                    dark_frame_start=batch.dark_frame_start,
                                    dark_frame_end=batch.dark_frame_end,
                                    samples=avg_samples,
                                ))
                            return

                        if on_corrected_batch_fn:
                            on_corrected_batch_fn(batch)

                    science_pipeline = create_science_pipeline(
                        left_camera_mask=left_mask_active,
                        right_camera_mask=right_mask_active,
                        bfi_c_min=self._calibration.c_min,
                        bfi_c_max=self._calibration.c_max,
                        bfi_i_min=self._calibration.i_min,
                        bfi_i_max=self._calibration.i_max,
                        on_uncorrected_fn=_on_uncorrected_sample,
                        on_corrected_batch_fn=_on_corrected_batch,
                        on_realtime_corrected_fn=on_realtime_corrected_fn,
                        on_dark_frame_fn=on_dark_frame_fn,
                        on_rolling_avg_fn=on_rolling_avg_fn,
                        rolling_avg_enabled=request.rolling_avg_enabled,
                        rolling_avg_window=request.rolling_avg_window,
                        log_dark_endpoints=log_dark_endpoints,
                    )

                def _make_row_handler(current_side: str, p):
                    """Close over side so each writer thread feeds the right key."""
                    def _on_row(cam_id, frame_id, ts_val, hist, row_sum, temp):
                        if p is not None:
                            p.enqueue(
                                current_side,
                                cam_id,
                                frame_id,
                                ts_val,
                                hist,
                                row_sum,
                                temp,
                            )
                        # Resolve extras once; both sinks (the in-process
                        # CsvSink and any externally-wired raw-frame hook
                        # such as ScanDBSink) consume the same shape.
                        if extra_cols_fn is not None:
                            try:
                                extras = extra_cols_fn()
                            except Exception:
                                extras = []
                        else:
                            extras = []
                        tcm = float(extras[0]) if len(extras) > 0 else 0.0
                        tcl = float(extras[1]) if len(extras) > 1 else 0.0
                        pdc = float(extras[2]) if len(extras) > 2 else 0.0
                        temp_f = float(temp) if temp is not None else 0.0
                        row_sum_i = int(row_sum) if row_sum is not None else 0
                        ts_f = float(ts_val)
                        hist_b = bytes(hist)

                        # Raw CSVs (#92 B4b — owned by CsvSink, which closes
                        # its writer the first time it sees a frame past
                        # ``raw_csv_duration_sec``).
                        try:
                            _csv_sink.on_raw_frame(
                                current_side,
                                int(cam_id),
                                int(frame_id),
                                ts_f,
                                hist_b,
                                temp_f,
                                row_sum_i,
                                tcm, tcl, pdc,
                            )
                        except Exception:
                            logger.exception("CsvSink.on_raw_frame raised")

                        # External raw-frame hook (ScanDBSink, user
                        # callbacks). Honor the same duration cap so we
                        # don't accumulate raw rows for hour-long scans
                        # after the user-requested cap (#92, see commit
                        # c06263d). The cap used to be plumbed via a
                        # shared event set by the inline CSV writer when
                        # it hit its deadline; the writer is gone now,
                        # so we check the cap directly here.
                        if on_raw_frame_fn is None:
                            return
                        raw_dur = request.raw_csv_duration_sec
                        if raw_dur is not None and ts_f > float(raw_dur):
                            return
                        try:
                            on_raw_frame_fn(
                                current_side,
                                int(cam_id),
                                int(frame_id),
                                ts_f,
                                hist_b,
                                temp_f,
                                row_sum_i,
                                tcm, tcl, pdc,
                            )
                        except Exception:
                            logger.exception("on_raw_frame_fn callback raised")
                    return _on_row

                # The CSV deadline used to be implemented with two events
                # (_trigger_armed_evt + _csv_stop_evt) that gated the
                # inline per-side raw-CSV writer. Raw CSV writing now
                # lives in CsvSink (#92 B4b), which times its own cap
                # from the first frame it sees, so neither event is
                # needed anymore. parse_histogram_stream is called
                # with no csv_writer / no deadline.

                for side, mask, sensor in active_sides:
                    q = queue.Queue()
                    writer_queues[side] = q
                    stop_evt = threading.Event()

                    # Drain any USB data left over from the previous scan before
                    # arming the new writer thread.  This runs while the MCU
                    # trigger is still off, so only stale prior-scan frames can
                    # be in the endpoint buffer — no real data is discarded.
                    flushed = sensor.uart.histo.flush_stale_data(
                        expected_size=request.expected_size
                    )
                    if flushed:
                        _emit_log(
                            f"Flushed {flushed} stale bytes from {side} USB endpoint "
                            f"({flushed // request.expected_size} frame(s)) before scan start."
                        )

                    sensor.uart.histo.start_streaming(q, expected_size=request.expected_size)

                    _row_handler = _make_row_handler(side, science_pipeline)

                    def _writer(
                        q=q,
                        stop_evt=stop_evt,
                        on_row_fn=_row_handler,
                        s=side,
                    ):
                        # The per-side thread used to open the raw CSV
                        # itself; that's CsvSink's job now (#92, B4b).
                        # All this thread still does is drive
                        # parse_histogram_stream so the on_row_fn fires
                        # for every frame coming off USB — the row
                        # handler fans into CsvSink + the external
                        # raw-frame hook (ScanDBSink, user callback).
                        rows_written = 0
                        try:
                            rows_written = parse_histogram_stream(
                                q=q,
                                stop_evt=stop_evt,
                                buffer_accumulator=bytearray(),
                                on_row_fn=on_row_fn,
                                t0_normalizer=_normalize_ts,
                            )
                        except Exception as e:
                            _emit_log(f"Writer error ({s}): {e}")
                        finally:
                            writer_row_counts[s] = rows_written

                    t = threading.Thread(target=_writer, daemon=True)

                    t.start()
                    writer_threads[side] = t
                    writer_stops[side] = stop_evt

                    # CsvSink chose the path; surface it so the legacy
                    # left_path / right_path / on_side_stream_fn contract
                    # is preserved for ScanResult consumers.
                    filepath = _csv_sink.raw_paths.get(side, "")
                    if side == "left":
                        left_path = filepath
                    elif side == "right":
                        right_path = filepath
                    if filepath:
                        _emit_log(f"{side.capitalize()} raw CSV: {os.path.basename(filepath)}")
                    if on_side_stream_fn:
                        on_side_stream_fn(side, filepath)

                # Subscribe to state changes on the participating handles so
                # any mid-scan disconnect aborts the scan immediately.
                # Subscribed handles include the console (whose loss stops
                # the FSYNC trigger) and every active sensor.
                _scan_handles = [self._interface.console] + [s for _, _, s in active_sides]
                self._scan_subscribe_state(handles=_scan_handles)

                # Arm host-side streaming before enabling cameras so the first
                # frame packet is not missed at scan start.
                _emit_log("Enabling cameras...")
                for side, mask, _ in active_sides:
                    res = self._interface.run_on_sensors("enable_camera", mask, target=side)
                    if not self._ok_from_result(res, side):
                        raise RuntimeError(
                            f"Failed to enable camera on {side} (mask 0x{mask:02X})."
                        )

                _emit_log("Starting trigger...")
                if not self._interface.console.start_trigger():
                    raise RuntimeError("Failed to start trigger.")
                if on_trigger_state_fn:
                    on_trigger_state_fn("ON")

                start_t = time.time()
                last_emit = -1
                while not self._stop_evt.is_set():
                    elapsed = time.time() - start_t
                    pct = int(min(100, max(0, (elapsed / max(1, request.duration_sec)) * 100)))
                    if pct != last_emit:
                        if on_progress_fn:
                            on_progress_fn(pct if pct >= 1 else 1)
                        last_emit = pct
                    if elapsed >= request.duration_sec:
                        break
                    time.sleep(0.2)

                ok = not self._stop_evt.is_set()
                if not ok:
                    # If the stop was triggered by a mid-scan disconnect,
                    # prefer that specific reason over "Capture canceled".
                    err = self._scan_abort_reason or "Capture canceled"
            except Exception as e:
                ok = False
                err = str(e)
                if on_error_fn:
                    on_error_fn(e)
            finally:
                try:
                    self._interface.console.stop_trigger()
                except Exception:
                    pass
                if on_trigger_state_fn:
                    on_trigger_state_fn("OFF")

                # Stop reacting to handle state changes — the scan is
                # done, so any further disconnects shouldn't influence
                # this run.
                self._scan_unsubscribe_state()

                time.sleep(0.5)

                try:
                    for side, mask, _ in active_sides:
                        try:
                            self._interface.run_on_sensors("disable_camera", mask, target=side)
                        except Exception:
                            pass
                except Exception:
                    pass

                # After disabling cameras the MCU still needs up to ~250 ms to
                # flush its DMA buffer and complete the final USB bulk transfer.
                # Waiting here while _stream_loop is still running ensures that
                # transfer is received and queued BEFORE stop_streaming() signals
                # the loop to exit.
                time.sleep(0.35)

                for side, _, sensor in active_sides:
                    try:
                        sensor.uart.histo.stop_streaming()
                    except Exception:
                        pass
                    # Post-stop drain: _stream_loop exits when it gets a timeout
                    # while stop_event is set.  If the MCU's final USB transfer
                    # arrives after that timeout window (which can happen >350 ms
                    # after trigger-off), the frame lands in the host endpoint
                    # buffer with no reader.  drain_final() recovers it here,
                    # before the writer thread is told to stop.
                    q = writer_queues.get(side)
                    if q is not None:
                        try:
                            final_chunks = sensor.uart.histo.drain_final(
                                expected_size=request.expected_size
                            )
                            for chunk in final_chunks:
                                q.put(chunk)
                            if final_chunks:
                                _emit_log(
                                    f"{side.capitalize()}: post-stop drain recovered "
                                    f"{len(final_chunks)} late USB transfer(s) "
                                    f"({sum(len(c) for c in final_chunks)} bytes)"
                                )
                        except Exception as _drain_err:
                            logger.warning("%s: post-stop drain error: %s", side, _drain_err)

                for stop_evt in writer_stops.values():
                    stop_evt.set()
                for t in writer_threads.values():
                    t.join(timeout=5.0)

                # Per-side summary: USB read chunks received vs rows written to CSV.
                # Compare against the MCU's own frame-sent printout to locate
                # exactly where any frame loss is occurring. The sensor.uart
                # may be None here if the sensor disconnected mid-scan and
                # never recovered — in that case we can only report the
                # writer-side row count.
                for side, _, sensor in active_sides:
                    if sensor.uart is not None:
                        usb_pkts = sensor.uart.histo.packets_received
                    else:
                        usb_pkts = "n/a (disconnected)"
                    rows = writer_row_counts.get(side, 0)
                    side_path = left_path if side == "left" else right_path
                    _emit_log(
                        f"{side.capitalize()} — USB read chunks received: {usb_pkts} | "
                        f"CSV rows written: {rows}"
                        + (f" | {os.path.basename(side_path)}" if side_path else "")
                    )

                if science_pipeline is not None:
                    science_pipeline.stop()

                # Drain any partial corrected rows (frames where not all cameras
                # reported before the scan ended) and close the file. Owned by
                # CsvSink since #92 / Step B4a.
                try:
                    _csv_sink.on_complete()
                except Exception as _close_err:
                    _emit_log(f"Corrected CSV close error: {_close_err}")
                if corrected_path:
                    _emit_log(
                        f"Corrected CSV created: {os.path.basename(corrected_path)}"
                    )

                # Telemetry CSV teardown — signal the listener to stop writing,
                # wait for any in-flight write to drain, then close the file.
                _telem_stop.set()
                with _telem_lock:
                    pass  # acquire+release: ensures any in-flight write has exited
                if _telem_poller is not None and _telem_listener is not None:
                    try:
                        _telem_poller.remove_listener(_telem_listener)
                    except Exception:
                        pass
                if _telem_fh is not None:
                    try:
                        _telem_fh.close()
                    except Exception:
                        pass
                if telemetry_path:
                    _emit_log(f"Telemetry CSV created: {os.path.basename(telemetry_path)}")

                # Surface any dark-integrity warnings the science
                # pipeline collected so calibration callers can fail.
                integrity_warnings: list[str] = []
                if science_pipeline is not None:
                    try:
                        integrity_warnings = science_pipeline.dark_integrity_warnings
                    except Exception:
                        pass

                result = ScanResult(
                    ok=ok,
                    error=err,
                    left_path=left_path,
                    right_path=right_path,
                    corrected_path=corrected_path,
                    telemetry_path=telemetry_path,
                    canceled=self._stop_evt.is_set(),
                    scan_timestamp=ts,
                    dark_integrity_warnings=integrity_warnings,
                )
                if on_complete_fn:
                    on_complete_fn(result)
                with self._lock:
                    self._running = False
                    self._thread = None

        self._thread = threading.Thread(target=_worker, daemon=True)
        self._thread.start()
        return True

    def cancel_scan(self, *, join_timeout: float = 5.0) -> None:
        self._stop_evt.set()
        try:
            if self._interface and self._interface.console:
                self._interface.console.stop_trigger()
        except Exception:
            pass
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
            self._stop_evt.set()
