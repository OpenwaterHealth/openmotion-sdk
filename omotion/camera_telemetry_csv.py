"""1 Hz camera-telemetry CSV logger (sensor-fw#94 / #162).

Samples ``MotionSensor.get_camera_telemetry()`` — the firmware's cached,
background-collected per-camera condition snapshot (safe to poll mid-scan:
the query does no camera I2C) — once per second on a daemon thread and
writes one CSV per camera, following the console telemetry CSV conventions
(``_TelemetryCsvWriter`` in ScanWorkflow): header row, flush per sample,
best-effort writes that never raise into the caller.

File naming mirrors ``{scan_id}_{subject_id}_telemetry.csv``:

    {stem}_{side}_cam{N}_telemetry.csv        (N = 0..7)

Lifecycle mirrors ConsoleTelemetryPoller: ``start()`` / ``stop()`` are
idempotent; ScanWorkflow constructs it in ``start_scan`` when
``ScanRequest.write_camera_telemetry_csv`` is True (default False) and
closes it in the worker's ``finally``. It is equally usable standalone::

    log = CameraTelemetryCsvLogger([("left", sensor)], out_dir, "bench_20260718")
    log.start(); ...; log.stop()

Rows are written every tick for every camera, valid or not — a sampled log,
not an event log. ``updated_ms`` vs ``uptime_ms`` exposes firmware-side
staleness; ``read_ok``/``error`` record host-side query failures.
"""

from __future__ import annotations

import csv
import logging
import os
import threading
import time
from typing import Any, List, Optional, Sequence, Tuple

from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.CameraTelemetryCsv" if _log_root else "CameraTelemetryCsv")

CAMERA_TELEMETRY_CSV_INTERVAL_S: float = 1.0
_CAMERA_COUNT = 8

CAMERA_TELEMETRY_HEADERS: List[str] = [
    "host_time_s",          # host time.time() when the sample completed
    "side", "cam",
    "read_ok", "error",     # host-side query health
    "valid",                # firmware has >=1 completed sweep for this camera
    "uptime_ms", "updated_ms", "fsin_pulse_count",
    "sweep_count", "i2c_err_count",
    "avdd_v", "dovdd_v", "dvdd_v",
    "tpm_avg_c", "tpm0_c", "tpm1_c", "tpm_status",
    "vm_live", "vm_cp", "vm_latched", "vm_cp_latched",
    "wd_fault_a", "wd_fault_b", "wd_sticky", "wd_state",
    "sc_state", "otp_crc0", "otp_crc1",
    "trig_error", "yavg", "tc_row",
    "expo_cmd", "expo_applied", "again_x", "dgain_x",
    "aec_mode", "dcg_state", "blc_ctrl", "isp_ctrl",
    "isp_real_gain", "isp_dig_gain", "isp_blc", "isp_expo",
] + [f"blc_offset_{i}" for i in range(8)] + [
    # --- optical-black block (sensor-fw#103) ---
    # z_avg_00/01/10/11 are the zero-line (dark row) averages per Bayer
    # position; mono sensor, so all four should agree (z_avg_spread == 0).
    "z_avg_00", "z_avg_01", "z_avg_10", "z_avg_11",
    "z_avg_mean", "z_avg_spread",
    "blc_thres", "blk_lvl_target",
    "bl_start", "bl_end", "blk_ln_num", "blc_ln_mode",
    "zl_start", "zl_end", "zero_ln_num",
    "zavg_ctrl", "z_avg_sel", "zl_start2", "zl_end2",
    "blc_trig_ctrl", "blc_fault_latch", "blc_fault_state",
    "dig_test_fail", "dtr_fault",
] + [f"blc_offset_z_{i}" for i in range(4)]


def _cam_row(host_time: float, side: str, cam_id: int,
             telem: Optional[dict], error: str = "") -> List[Any]:
    """Build one CSV row; blanks + read_ok=0 when the query failed."""
    if telem is None:
        return ([round(host_time, 3), side, cam_id, 0, error or "no data"]
                + [""] * (len(CAMERA_TELEMETRY_HEADERS) - 5))
    c = telem["cameras"][cam_id]
    row: List[Any] = [
        round(host_time, 3), side, cam_id,
        1, "",
        int(c.get("valid", False)),
        telem.get("uptime_ms", ""), c.get("updated_ms", ""),
        telem.get("fsin_pulse_count", ""),
        c.get("sweep_count", ""), c.get("i2c_err_count", ""),
        round(c.get("avdd_v", 0.0), 4),
        round(c.get("dovdd_v", 0.0), 4),
        round(c.get("dvdd_v", 0.0), 4),
        round(c.get("tpm_avg_c", 0.0), 2),
        round(c.get("tpm0_c", 0.0), 2),
        round(c.get("tpm1_c", 0.0), 2),
        c.get("tpm_status", ""),
        c.get("vm_live", ""), c.get("vm_cp", ""),
        c.get("vm_latched", ""), c.get("vm_cp_latched", ""),
        c.get("wd_fault_a", ""), c.get("wd_fault_b", ""),
        c.get("wd_sticky", ""), c.get("wd_state", ""),
        c.get("sc_state", ""),
        (c.get("otp_crc") or ("", ""))[0], (c.get("otp_crc") or ("", ""))[1],
        c.get("trig_error", ""), c.get("yavg", ""), c.get("tc_row", ""),
        c.get("expo_cmd", ""), c.get("expo_applied", ""),
        round(c.get("again_x", 0.0), 3), round(c.get("dgain_x", 0.0), 3),
        c.get("aec_mode", ""), c.get("dcg_state", ""),
        c.get("blc_ctrl", ""), c.get("isp_ctrl", ""),
        c.get("isp_real_gain", ""), c.get("isp_dig_gain", ""),
        c.get("isp_blc", ""), c.get("isp_expo", ""),
    ]
    offsets = c.get("blc_offsets") or []
    for i in range(8):
        row.append(offsets[i] if i < len(offsets) else "")

    # Optical-black block (sensor-fw#103).
    z_avg = c.get("z_avg") or []
    row += [z_avg[i] if i < len(z_avg) else "" for i in range(4)]
    row += [
        round(c.get("z_avg_mean", 0.0), 2) if z_avg else "",
        c.get("z_avg_spread", ""),
        c.get("blc_thres", ""), c.get("blk_lvl_target", ""),
        c.get("bl_start", ""), c.get("bl_end", ""),
        c.get("blk_ln_num", ""), c.get("blc_ln_mode", ""),
        c.get("zl_start", ""), c.get("zl_end", ""), c.get("zero_ln_num", ""),
        c.get("zavg_ctrl", ""), c.get("z_avg_sel", ""),
        c.get("zl_start2", ""), c.get("zl_end2", ""),
        c.get("blc_trig_ctrl", ""), c.get("blc_fault_latch", ""),
        c.get("blc_fault_state", ""),
        c.get("dig_test_fail", ""), c.get("dtr_fault", ""),
    ]
    offsets_z = c.get("blc_offsets_z") or []
    row += [offsets_z[i] if i < len(offsets_z) else "" for i in range(4)]
    return row


class CameraTelemetryCsvLogger:
    """1 Hz per-camera telemetry CSV logger for one or more sensors.

    ``sensors`` is a sequence of ``(side, MotionSensor)``; one CSV per camera
    per sensor is opened immediately (header written) and sampled until
    ``stop()``. Query/write failures are logged and recorded in the row's
    ``read_ok``/``error`` columns — they never propagate to the caller.
    """

    def __init__(self, sensors: Sequence[Tuple[str, Any]], out_dir: str,
                 stem: str,
                 interval_s: float = CAMERA_TELEMETRY_CSV_INTERVAL_S) -> None:
        self._sensors = list(sensors)
        self._interval_s = max(0.05, float(interval_s))
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._wake = threading.Event()
        # (side, sensor) -> list of (file, csv.writer) per camera
        self._files: List[Tuple[str, Any, List[Tuple[Any, Any]]]] = []
        self.paths: List[str] = []

        for side, sensor in self._sensors:
            per_cam = []
            for cam_id in range(_CAMERA_COUNT):
                path = os.path.join(
                    out_dir, f"{stem}_{side}_cam{cam_id}_telemetry.csv")
                fh = open(path, "w", newline="", encoding="utf-8")
                w = csv.writer(fh)
                w.writerow(CAMERA_TELEMETRY_HEADERS)
                fh.flush()
                per_cam.append((fh, w))
                self.paths.append(path)
            self._files.append((side, sensor, per_cam))

    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the sampling thread (idempotent)."""
        with self._lock:
            if self._running:
                return
            self._running = True
            self._wake.clear()
            self._thread = threading.Thread(
                target=self._loop, name="CameraTelemetryCsvLogger", daemon=True)
            self._thread.start()
            logger.info("CameraTelemetryCsvLogger started (%d files, %.1f s interval)",
                        len(self.paths), self._interval_s)

    def stop(self) -> None:
        """Stop sampling and close all files (idempotent)."""
        thread_to_join: Optional[threading.Thread] = None
        with self._lock:
            if self._running:
                self._running = False
                self._wake.set()
                thread_to_join = self._thread
                self._thread = None
        if thread_to_join and thread_to_join.is_alive():
            thread_to_join.join(timeout=5.0)
        for _, _, per_cam in self._files:
            for fh, _w in per_cam:
                try:
                    fh.close()
                except Exception:
                    pass
        logger.info("CameraTelemetryCsvLogger stopped")

    # ScanWorkflow teardown symmetry with _TelemetryCsvWriter.
    close = stop

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    # ------------------------------------------------------------------

    def _loop(self) -> None:
        last = 0.0
        while True:
            with self._lock:
                if not self._running:
                    break
            now = time.time()
            if (now - last) >= self._interval_s:
                last = now
                self._sample_once()
            sleep_s = self._interval_s - (time.time() - last)
            self._wake.wait(timeout=max(0.05, min(self._interval_s, sleep_s)))
            self._wake.clear()

    def _sample_once(self) -> None:
        for side, sensor, per_cam in self._files:
            telem = None
            error = ""
            try:
                telem = sensor.get_camera_telemetry()
                if telem is None:
                    error = "get_camera_telemetry returned None"
            except Exception as exc:  # never let a bad poll kill the logger
                error = str(exc) or type(exc).__name__
                telem = None
            host_time = time.time()
            for cam_id in range(_CAMERA_COUNT):
                fh, w = per_cam[cam_id]
                try:
                    w.writerow(_cam_row(host_time, side, cam_id, telem, error))
                    fh.flush()
                except Exception:
                    logger.exception("camera telemetry CSV write failed (%s cam %d)",
                                     side, cam_id)

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
