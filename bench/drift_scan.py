#!/usr/bin/env python3
"""
drift_scan.py

Runs a long SDK-streamed scan on the connected OpenMOTION sensor module (8
cameras, mask 0xFF) while the Keysight-driven illumination source is held at
a fixed control voltage, periodically killing the light for ~1 s to record a
dark reference. Saves the SDK's raw per-frame histogram CSV (via the normal
ScanWorkflow/pipeline raw sink) plus a JSON sidecar describing exactly when
each dark window happened. All statistics (mean/std per frame, dark-pedestal
correction, drift plots) are computed afterward by analyze_drift_scan.py —
this script only acquires data.

Wiring assumed (same as illumination_sweep.py):
    PSU channel 2  -> illumination source supply rail (24 V, 2 A CV, constant)
    PSU channel 3  -> illumination source control input (0-10 V) -- held at
                       --control-voltage, only briefly pulled to 0 V for the
                       periodic dark references
    Sensor module  -> USB, 8 cameras (mask 0xFF) exposed to the source
    Console        -> UART, provides the camera FSYNC trigger for streaming
    Thorlabs meter -> USB, sampled continuously for the whole scan (its own
                       background thread, independent of the camera stream)
                       as a 9th, independent illumination reference

Note: apply_laser_power() IS called during bring-up. The console's FSYNC
trigger appears to require the laser driver armed to sustain streaming past
a few hundred ms to ~20s (observed experimentally: without it, scans cut
short at an inconsistent frame count well before the requested duration).
This means the console's own onboard NIR laser may emit light (at whatever
power is baked into the bundled laser_params.json) for the duration of the
scan, layered on top of the external Keysight-driven source -- confirmed
acceptable for this experiment. Camera intensity/drift readings reflect the
COMBINED illumination (external source + internal laser), not the external
source alone.

Usage
-----
    python bench/drift_scan.py
        Default: 2 V control level, 1800 s (30 min) scan, a 1 s dark window
        every 60 s starting at t=60s (first minute is left undisturbed to
        let the source settle).

    python bench/drift_scan.py --duration-sec 90 --dark-interval-sec 30 --dark-start-offset-sec 10
        Quick smoke test (~90 s, 2-3 dark windows) to validate the whole
        pipeline before committing to a full run.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "keysight-psu"))
sys.path.insert(0, str(Path(__file__).resolve().parent / "thorlabs-pm100"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import pyvisa

from keysight_psu import KeysightE36300, PSUError

SUPPLY_CHANNEL = 2
CONTROL_CHANNEL = 3
CAMERA_MASK = 0xFF
CAMERA_CROP = False   # set from --camera-crop; crops output to 1720x1280 at configure (sensor-fw#86)
CAMERA_RAW = False    # set from --camera-raw; disables on-sensor corrections at configure (sensor-fw#89)
CAMERA_REG_WRITES: list = []  # set from --camera-reg; (addr,val) written to each masked camera AFTER
                              # configure, before streaming (vendor register tests, e.g. 0x4001=0x2b)
CONFIGURE_TIMEOUT_S = 240.0
END_MARGIN_S = 5.0  # don't schedule a dark window this close to scan end
THORLABS_SAMPLE_INTERVAL_S = 0.1  # ~10 Hz -- plenty to resolve a 1s dark window
DROPOUT_TOLERANCE_S = 5.0  # a camera's last frame must land within this of duration_sec
DEFAULT_REGISTRY_PATH = Path("bench/test_registry.csv")


class _Tee:
    """Duplicates writes to multiple streams -- used to mirror stdout/stderr
    into a per-run log file without giving up the live terminal output."""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for s in self._streams:
            s.write(data)
            s.flush()

    def flush(self):
        for s in self._streams:
            s.flush()


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--control-voltage", type=float, default=2.0, help="Fixed channel 3 control voltage during light phases. Default: 2.0 V.")
    parser.add_argument("--supply-voltage", type=float, default=24.0, help="Channel 2 CV setpoint. Default: 24.0 V.")
    parser.add_argument("--supply-current-limit", type=float, default=2.0, help="Channel 2 current limit. Default: 2.0 A.")
    parser.add_argument("--control-current-limit", type=float, default=0.5, help="Channel 3 current limit. Default: 0.5 A.")
    parser.add_argument("--duration-sec", type=float, default=1800.0, help="Total scan duration. Default: 1800 (30 min).")
    parser.add_argument("--dark-interval-sec", type=float, default=60.0, help="Seconds between dark windows. Default: 60.")
    parser.add_argument("--dark-duration-sec", type=float, default=1.0, help="Seconds the light is off per dark window. Default: 1.0.")
    parser.add_argument("--dark-start-offset-sec", type=float, default=60.0, help="Elapsed time of the first periodic dark window. Default: 60.")
    parser.add_argument("--front-dark-sec", type=float, default=2.0, help="Hold the light OFF for this long at scan start so the analysis has a dark anchor at t~0 (covers the first minute). 0 disables. Default: 2.")
    parser.add_argument("--tail-dark-sec", type=float, default=2.0, help="Turn the light OFF this long before scan end for a final dark anchor (covers the last minute). 0 disables. Default: 2.")
    parser.add_argument("--subject-id", default="DRIFT", help="Scan subject_id (also used in the raw CSV filename). Default: DRIFT.")
    parser.add_argument("--data-dir", type=Path, default=Path("bench/drift_scan_out"), help="Output directory for the scan DB, telemetry CSV, raw CSV and JSON sidecar.")
    parser.add_argument("--psu-resource", default=None, help="VISA resource string for the PSU. Auto-discovered if omitted.")
    parser.add_argument("--meter-resource", default=None, help="VISA resource string for the Thorlabs meter. Auto-discovered if omitted.")
    parser.add_argument("--no-thorlabs", action="store_true", help="Skip the Thorlabs meter; run PSU+camera acquisition only.")
    parser.add_argument("--registry-path", type=Path, default=DEFAULT_REGISTRY_PATH, help="Central CSV that every run appends a summary row to. Default: bench/test_registry.csv.")
    parser.add_argument("--prewarm-min", type=float, default=0.0, help="Run a throwaway warmup scan of this many minutes first (no raw CSV), keeping cameras powered, then start the measurement scan immediately -- for isolating cold-start warmup effects. Default: 0 (off).")
    parser.add_argument("--leave-source-on", action="store_true", help="At scan end, leave the illumination source ON (control at --control-voltage, both PSU outputs on) instead of powering it off. Keeps the source thermally stable between runs -- tests whether the warm-up dip is source-side (a thermally-stable source would remove it) or camera-side (it would persist). The PSU is on separate mains from the Shelly rig plug, so it stays on through rig power-cycles.")
    parser.add_argument("--camera-mask", default="0xFF",
                        help="Which cameras to power + configure + stream, as hex (0xC3) or int (195). Only "
                             "these cameras are powered on. Default 0xFF (all 8). The clinical 'far 4' config "
                             "= 0xC3 (cams 1,2,7,8); research default = 0x99 (cams 1,4,5,8).")
    parser.add_argument("--sensor-fan-off-until-temp", type=float, default=0.0,
                        help="Accelerated warm-up: if >0, turn the sensor MODULE fan OFF at scan start so the "
                             "sensor self-heats through the cold-start dip faster, then turn it back ON when the "
                             "sentinel camera's die reaches this temp (deg C). 0 = leave the sensor fan in its "
                             "firmware default state (the control condition).")
    parser.add_argument("--sensor-fan-cap-temp", type=float, default=110.0,
                        help="Safety: force the sensor fan back ON immediately if the sentinel die exceeds this "
                             "(deg C). Default 110 (camera cutoff is ~115).")
    parser.add_argument("--sensor-fan-off-max-sec", type=float, default=360.0,
                        help="Time fail-safe: force the sensor fan back ON this many seconds after scan start "
                             "regardless of temperature (guards against a stuck/unreadable monitor). Default 360.")
    parser.add_argument("--fan-sentinel-cam", type=int, default=6,
                        help="cam_id (0-7) whose live die temp gates the fan restore. Default 6 (= cam 7, the "
                             "deepest responder and hottest clinical camera).")
    parser.add_argument("--camera-crop", action="store_true",
                        help="Set DEBUG_FLAG_CAMERA_CROP before camera configure: output cropped to "
                             "1720x1280 (rightmost 200 columns dropped; sensor-fw#86). Histogram sums "
                             "read ~2201600 instead of ~2457600. Needs the crop-capable firmware (next).")
    parser.add_argument("--camera-raw", action="store_true",
                        help="Set DEBUG_FLAG_CAMERA_RAW before camera configure: raw scientific-sensor mode, "
                             "all on-sensor pixel corrections off (BLC, DC-BLC, dither, OTP-DPC; sensor-fw#89). "
                             "Dark level sits at the raw ADC pedestal (~255+ DN) and may drift with temperature.")
    parser.add_argument("--camera-reg", action="append", default=[], metavar="ADDR=VAL",
                        help="Write a camera register to EVERY masked camera AFTER configure, before "
                             "streaming, e.g. --camera-reg 0x4001=0x2b (repeatable). Applied via I2C "
                             "passthrough; use for vendor register-change tests. Read back in telemetry.")
    parser.add_argument("--camera-telemetry", action="store_true",
                        help="Log 1 Hz per-camera condition telemetry (sensor-fw#94: on-die rails, dual die "
                             "temps, yavg frame mean, commanded/applied exposure + gains, DCG/BLC/ISP state, "
                             "fault latches) to {subject}_{side}_camN_telemetry.csv for the whole scan. "
                             "Firmware-cached, no camera I2C at query time -- safe mid-scan. Needs sensor-fw "
                             "with OW_CAMERA_GET_TELEMETRY (next / >1.8.2-dev.1).")
    return parser.parse_args()


def connect_psu(args) -> "KeysightE36300":
    psu = KeysightE36300.connect(resource_name=args.psu_resource)
    print(f"[+] PSU: {psu}")
    psu.set_current_limit(SUPPLY_CHANNEL, args.supply_current_limit)
    psu.set_voltage(SUPPLY_CHANNEL, args.supply_voltage)
    psu.set_current_limit(CONTROL_CHANNEL, args.control_current_limit)
    psu.set_voltage(CONTROL_CHANNEL, args.control_voltage)
    psu.set_output(SUPPLY_CHANNEL, True)
    psu.set_output(CONTROL_CHANNEL, True)
    print(
        f"[*] Outputs on: ch{SUPPLY_CHANNEL}={args.supply_voltage}V/{args.supply_current_limit}A (CV), "
        f"ch{CONTROL_CHANNEL}={args.control_voltage}V control"
    )
    return psu


def connect_and_configure_sensor(data_dir: Path):
    from omotion import MotionInterface
    from omotion.ScanWorkflow import ConfigureRequest

    iface = MotionInterface(data_dir=str(data_dir))
    iface.start()
    if not iface.wait_for_ready(console=True, sensors=1, timeout=20):
        iface.stop()
        raise RuntimeError("Console and/or sensor module not ready within 20s.")

    sensor = iface.connected_sensors()[0]
    side = "left" if sensor is iface.left else "right"
    serial = sensor.read_serial_number()
    print(f"[+] Console connected. Sensor module on {side}, serial={serial!r}.")

    if CAMERA_CROP or CAMERA_RAW:
        # Both flags are applied AT camera configure (sensor-fw#86/#89), so they must
        # be set before ConfigureRequest. Cold boots default to flags=0, so control
        # runs need no explicit clear.
        from omotion.config import DEBUG_FLAG_CAMERA_CROP, DEBUG_FLAG_CAMERA_RAW
        flags = (DEBUG_FLAG_CAMERA_CROP if CAMERA_CROP else 0) | \
                (DEBUG_FLAG_CAMERA_RAW if CAMERA_RAW else 0)
        if not sensor.set_debug_flags(flags):
            iface.stop()
            raise RuntimeError(f"set_debug_flags(0x{flags:03X}) failed")
        what = []
        if CAMERA_CROP:
            what.append("CROP 1720x1280 (right 200 columns dropped)")
        if CAMERA_RAW:
            what.append("RAW scientific mode (BLC/DC-BLC/dither/OTP-DPC OFF; pedestal = raw ADC level)")
        print(f"[*] Camera debug flags 0x{flags:03X} set: " + "; ".join(what))

    sensor.enable_camera_power(CAMERA_MASK)
    left_mask = CAMERA_MASK if side == "left" else 0x00
    right_mask = CAMERA_MASK if side == "right" else 0x00

    done = threading.Event()
    box: dict = {}

    def _on_complete(result) -> None:
        box["result"] = result
        done.set()

    accepted = iface.start_configure_camera_sensors(
        ConfigureRequest(left_camera_mask=left_mask, right_camera_mask=right_mask,
                          power_off_unused_cameras=False),
        on_complete_fn=_on_complete,
    )
    if not accepted:
        iface.stop()
        raise RuntimeError("Camera configure refused (already running?).")
    if not done.wait(CONFIGURE_TIMEOUT_S):
        iface.stop()
        raise RuntimeError(f"Camera configure timed out after {CONFIGURE_TIMEOUT_S:.0f}s")
    result = box.get("result")
    if result is None or not result.ok:
        iface.stop()
        raise RuntimeError(f"Camera configure failed: {result.error if result else 'no result'}")
    print(f"[+] Cameras configured (mask 0x{CAMERA_MASK:02X})")

    # Vendor register-change tests: write each requested register to every masked
    # camera now (post-configure, pre-stream), so the value is in place for the
    # whole scan. Applied per camera via the I2C passthrough (device 0x36), the
    # same path camera_set_gain uses. Read back in the telemetry CSV to confirm.
    if CAMERA_REG_WRITES:
        import time as _t
        from omotion.i2c_packet import I2C_Packet
        masked = [i for i in range(8) if CAMERA_MASK & (1 << i)]
        for addr, val in CAMERA_REG_WRITES:
            for cam_id in masked:
                sensor.switch_camera(cam_id)
                _t.sleep(0.05)
                sensor.camera_i2c_write(I2C_Packet(device_address=0x36,
                                                   register_address=addr, data=val))
                _t.sleep(0.05)
            print(f"[+] Camera reg 0x{addr:04X}=0x{val:02X} written to cams "
                  f"{[c + 1 for c in masked]}")

    # The console's FSYNC trigger needs the laser driver armed to sustain
    # streaming for the full requested duration (see module docstring) --
    # confirmed acceptable that this may cause real laser emission.
    if not iface.apply_laser_power():
        iface.stop()
        raise RuntimeError("apply_laser_power() failed.")
    print("[+] Laser driver armed (apply_laser_power)")

    return iface, sensor, side, serial


def connect_thorlabs_meter(resource: str | None):
    from read_thorlabs_powermeter import find_thorlabs_resource

    rm = pyvisa.ResourceManager("@py")
    resources = list(rm.list_resources())
    resource_str = resource or find_thorlabs_resource(resources)
    if not resource_str:
        raise RuntimeError(f"No Thorlabs meter auto-detected among {resources}; pass --meter-resource.")
    inst = rm.open_resource(resource_str)
    inst.timeout = 3000
    inst.read_termination = "\n"
    inst.write_termination = "\n"
    idn = inst.query("*IDN?").strip()
    unit = inst.query("SENS:POW:DC:UNIT?").strip()
    print(f"[+] Thorlabs meter: {idn} ({resource_str}), unit={unit}")
    return inst, unit


def thorlabs_logger_thread(meter, unit: str, csv_path: Path, stop_event: threading.Event,
                            scan_call_time: float, interval: float) -> None:
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "power", "unit"])
        while not stop_event.is_set():
            t0 = time.time()
            try:
                power = float(meter.query("MEAS:POW?"))
            except Exception as exc:
                print(f"[!] Thorlabs read error: {exc}")
                stop_event.wait(interval)
                continue
            writer.writerow([f"{t0 - scan_call_time:.4f}", power, unit])
            f.flush()
            sleep_for = interval - (time.time() - t0)
            if sleep_for > 0:
                stop_event.wait(sleep_for)


def build_dark_schedule(duration_sec: float, start_offset: float, interval: float, dark_dur: float) -> list[float]:
    schedule = []
    t = start_offset
    while t + dark_dur <= duration_sec - END_MARGIN_S:
        schedule.append(t)
        t += interval
    return schedule


def detect_camera_dropouts(raw_csv_path: str, duration_sec: float) -> list[str]:
    """Lightweight post-scan QC: flag any camera whose last frame landed
    well before the scan's nominal end. Only reads cam_id/timestamp_s, so
    it's cheap even on a multi-GB raw CSV."""
    import pandas as pd

    df = pd.read_csv(raw_csv_path, usecols=["cam_id", "timestamp_s"])
    notes = []
    for cam_id, sub in df.groupby("cam_id"):
        last_t = float(sub["timestamp_s"].max())
        if last_t < duration_sec - DROPOUT_TOLERANCE_S:
            notes.append(f"cam{int(cam_id) + 1}_stopped_at_{last_t:.0f}s")
    return notes


def append_registry_row(registry_path: Path, row: dict) -> None:
    registry_path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not registry_path.exists()
    with open(registry_path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if is_new:
            writer.writeheader()
        writer.writerow(row)


def read_imu_temp(sensor, label: str = "") -> "float | None":
    """Read the ICM-20948 IMU temperature -- a board-level probe away from the
    camera dies, so a cleaner measure of the module/deep-assembly temperature
    than any camera's die sensor. A single COMM command (not a stream); only
    call it when NOT 8-camera streaming (concurrent sensor I2C can wedge the
    HISTO endpoint). Returns degrees C, or None on failure."""
    try:
        sensor.imu_init()
        sensor.imu_on()
        time.sleep(0.2)
        t = float(sensor.imu_get_temperature())
        print(f"[+] IMU temp {label}: {t:.2f} C")
        return t
    except Exception as exc:
        print(f"[!] IMU temp read failed ({label}): {exc}")
        return None


def _tail_sentinel_temp(raw_path: Path, cam_id: int) -> "float | None":
    """Latest die temp (deg C) for cam_id from the tail of the LIVE raw CSV. Raw columns:
    cam_id(0), frame_id, timestamp_s, type, <1024 bins>, temperature([-5]), sum, tcm, tcl, pdc."""
    try:
        sz = raw_path.stat().st_size
        with open(raw_path, "rb") as f:
            f.seek(max(0, sz - 65536))
            chunk = f.read().decode("utf-8", "ignore")
    except Exception:
        return None
    prefix = f"{cam_id},"
    for line in reversed(chunk.split("\n")[1:]):   # drop first (partial) line
        if not line.startswith(prefix):
            continue
        parts = line.split(",")
        if len(parts) < 1033:
            continue
        try:
            return float(parts[-5])
        except ValueError:
            continue
    return None


def sensor_fan_warmup_monitor(sensor, raw_glob, cam_id, target_c, cap_c, max_off_s, stop_evt, result):
    """The sensor fan was turned OFF at scan start to accelerate warm-up through the dip.
    Tail the live raw CSV, read the sentinel camera's die temp, and turn the fan back ON
    at the target temp, a safety cap, or a time fail-safe -- whichever comes first.
    NOTE: set_fan_control here fires an I2C/controller command DURING streaming."""
    import glob as _glob
    t0 = time.time()
    raw_path = None
    peak = None
    while not stop_evt.is_set():
        if raw_path is None:
            cands = sorted(_glob.glob(raw_glob))
            raw_path = Path(cands[-1]) if cands else None
        temp = _tail_sentinel_temp(raw_path, cam_id) if raw_path is not None else None
        if temp is not None:
            peak = temp if peak is None else max(peak, temp)
        elapsed = time.time() - t0
        reason = None
        if temp is not None and temp >= cap_c:
            reason = f"SAFETY CAP {cap_c:.0f}C (cam {cam_id + 1} die {temp:.1f}C)"
        elif temp is not None and temp >= target_c:
            reason = f"target {target_c:.0f}C reached (cam {cam_id + 1} die {temp:.1f}C)"
        elif elapsed >= max_off_s:
            reason = f"time fail-safe {max_off_s:.0f}s (peak die {peak})"
        if reason is not None:
            ok = sensor.set_fan_control(True)
            result.update(restored=True, at_sec=round(elapsed, 1), at_temp=temp, reason=reason, set_ok=ok)
            print(f"[+] Sensor fan RE-ENABLED at t={elapsed:.0f}s -- {reason} (set_ok={ok})", flush=True)
            return
        stop_evt.wait(2.0)
    # scan ended before any trigger -> make sure the fan is back on
    ok = sensor.set_fan_control(True)
    result.update(restored=True, at_sec=round(time.time() - t0, 1), at_temp=peak,
                  reason="scan ended before target", set_ok=ok)
    print("[+] Sensor fan re-enabled (scan ended before target reached)", flush=True)


def main() -> int:
    args = parse_cli()
    global CAMERA_MASK, CAMERA_CROP, CAMERA_RAW
    CAMERA_MASK = int(args.camera_mask, 0)   # hex (0xC3) or int (195); powers/streams only these cameras
    CAMERA_CROP = bool(args.camera_crop)
    CAMERA_RAW = bool(args.camera_raw)
    global CAMERA_REG_WRITES
    CAMERA_REG_WRITES = []
    for spec in (args.camera_reg or []):
        addr_s, val_s = spec.split("=")
        CAMERA_REG_WRITES.append((int(addr_s, 0), int(val_s, 0)))
    args.data_dir.mkdir(parents=True, exist_ok=True)

    log_path = args.data_dir / f"{args.subject_id}_run.log"
    log_file = open(log_path, "w", encoding="utf-8")
    sys.stdout = _Tee(sys.__stdout__, log_file)
    sys.stderr = _Tee(sys.__stderr__, log_file)
    print(f"[*] Logging this run to {log_path}")

    print("[*] Connecting to PSU ...")
    psu = connect_psu(args)

    meter, meter_unit = (None, None)
    if not args.no_thorlabs:
        print("[*] Connecting to Thorlabs meter ...")
        meter, meter_unit = connect_thorlabs_meter(args.meter_resource)

    print("[*] Connecting to console + sensor module ...")
    iface, sensor, side, sensor_serial = connect_and_configure_sensor(args.data_dir)

    # Module (IMU) temperature entering the scan = the deep-assembly cold-soak
    # state. Read now (cameras configured but NOT yet streaming, so the IMU I2C
    # read is safe). imu_temp_end is filled in the finally after streaming stops.
    imu_temp_start = read_imu_temp(sensor, "start (cold-soak, pre-stream)")
    imu_temp_end = None

    # 1 Hz per-camera condition telemetry (sensor-fw#94). Started BEFORE the scan so
    # the cold pre-stream state and the entire warm-up are captured; firmware serves a
    # cached snapshot (no camera I2C at query time), so polling mid-scan is safe.
    cam_telem_logger = None
    if args.camera_telemetry:
        from omotion.camera_telemetry_csv import CameraTelemetryCsvLogger
        cam_telem_logger = CameraTelemetryCsvLogger(
            [(side, sensor)], str(args.data_dir), args.subject_id)
        cam_telem_logger.start()
        print(f"[*] Camera telemetry logging started (1 Hz, {len(cam_telem_logger.paths)} files)")

    from omotion.ScanWorkflow import ScanRequest

    schedule = build_dark_schedule(args.duration_sec, args.dark_start_offset_sec,
                                    args.dark_interval_sec, args.dark_duration_sec)
    print(f"[*] Dark-window schedule: {len(schedule)} windows, every {args.dark_interval_sec}s, "
          f"{args.dark_duration_sec}s each, starting at t={args.dark_start_offset_sec}s")

    if args.prewarm_min > 0:
        # Throwaway warmup scan: identical trigger/streaming load so the
        # cameras (and source) reach thermal plateau, but no raw CSV. The
        # cameras stay powered between this and the measurement scan.
        pre_secs = int(round(args.prewarm_min * 60))
        pre_req = ScanRequest(
            subject_id=f"{args.subject_id}PRE",
            duration_sec=pre_secs,
            left_camera_mask=(CAMERA_MASK if side == "left" else 0x00),
            right_camera_mask=(CAMERA_MASK if side == "right" else 0x00),
            disable_laser=False,
            write_corrected_csv=False,
            write_telemetry_csv=False,
            raw_save_max_duration_s=0,
        )
        print(f"[*] Pre-warm scan: {args.prewarm_min:g} min with source at {args.control_voltage}V ...")
        if not iface.start_scan(pre_req):
            raise RuntimeError("Pre-warm start_scan refused.")
        time.sleep(pre_secs)
        iface.scan_workflow.await_complete(timeout_sec=180)
        print("[+] Pre-warm complete; cameras still powered. Starting measurement scan.")

    dark_events = []
    scan_error: dict = {}

    def _on_scan_error(exc) -> None:
        scan_error["exc"] = exc

    req = ScanRequest(
        subject_id=args.subject_id,
        duration_sec=int(round(args.duration_sec)),
        left_camera_mask=(CAMERA_MASK if side == "left" else 0x00),
        right_camera_mask=(CAMERA_MASK if side == "right" else 0x00),
        disable_laser=False,          # external (console) FSYNC -- console is present
        write_telemetry_csv=True,
        raw_save_max_duration_s=None,  # unbounded raw histogram tee
        on_error=_on_scan_error,
    )

    thorlabs_csv_path = args.data_dir / f"{args.subject_id}_thorlabs.csv"
    thorlabs_stop = threading.Event()
    thorlabs_thread = None

    fan_stop = threading.Event()
    fan_monitor_thread = None
    fan_restore: dict = {}

    try:
        if args.front_dark_sec > 0:
            # Light off BEFORE the trigger starts, so the scan's very first
            # frames are dark regardless of scan-start latency.
            psu.set_voltage(CONTROL_CHANNEL, 0.0)
        scan_call_time = time.time()
        accepted = iface.start_scan(req)
        if not accepted:
            raise RuntimeError("start_scan refused (bad request or a scan already running).")
        print(f"[*] Scan started (subject={args.subject_id}, duration={args.duration_sec}s). "
              f"Running for {args.duration_sec:.0f}s with {len(schedule)} dark windows ...")

        # Accelerated warm-up: cut the sensor fan at scan start so the module self-heats
        # through the cold-start dip faster; a monitor thread restores it at the target
        # die temp (or a safety cap / time fail-safe). set_fan_control fires an I2C command
        # while streaming -- smoke-tested to not wedge HISTO in the 4-camera config.
        if args.sensor_fan_off_until_temp > 0:
            ok = sensor.set_fan_control(False)
            print(f"[*] Sensor fan OFF at scan start (accelerated warm-up); restore when cam "
                  f"{args.fan_sentinel_cam + 1} die hits {args.sensor_fan_off_until_temp:.0f}C "
                  f"(cap {args.sensor_fan_cap_temp:.0f}C, fail-safe {args.sensor_fan_off_max_sec:.0f}s). "
                  f"set_ok={ok}", flush=True)
            raw_glob = str(args.data_dir / f"*_{args.subject_id}_{side}_mask{CAMERA_MASK:02X}_raw.csv")
            fan_monitor_thread = threading.Thread(
                target=sensor_fan_warmup_monitor,
                args=(sensor, raw_glob, args.fan_sentinel_cam, args.sensor_fan_off_until_temp,
                      args.sensor_fan_cap_temp, args.sensor_fan_off_max_sec, fan_stop, fan_restore),
                daemon=True)
            fan_monitor_thread.start()

        if meter is not None:
            thorlabs_thread = threading.Thread(
                target=thorlabs_logger_thread,
                args=(meter, meter_unit, thorlabs_csv_path, thorlabs_stop, scan_call_time, THORLABS_SAMPLE_INTERVAL_S),
                daemon=True,
            )
            thorlabs_thread.start()
            print(f"[*] Thorlabs logging started -> {thorlabs_csv_path}")

        if args.front_dark_sec > 0:
            sleep_for = scan_call_time + args.front_dark_sec - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            psu.set_voltage(CONTROL_CHANNEL, args.control_voltage)
            on_wall = time.time()
            dark_events.append({
                "index": -1,
                "nominal_t_sec": 0.0,
                "elapsed_off_sec": 0.0,
                "elapsed_on_sec": on_wall - scan_call_time,
            })
            print(f"  front dark window: light on at t={on_wall - scan_call_time:.2f}s")

        for i, nominal_t in enumerate(schedule):
            target_wall = scan_call_time + nominal_t
            sleep_for = target_wall - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)

            off_wall = time.time()
            psu.set_voltage(CONTROL_CHANNEL, 0.0)
            time.sleep(args.dark_duration_sec)
            psu.set_voltage(CONTROL_CHANNEL, args.control_voltage)
            on_wall = time.time()

            dark_events.append({
                "index": i,
                "nominal_t_sec": nominal_t,
                "elapsed_off_sec": off_wall - scan_call_time,
                "elapsed_on_sec": on_wall - scan_call_time,
            })
            print(f"  dark window {i + 1}/{len(schedule)} at t~{nominal_t:.0f}s "
                  f"(actual off={off_wall - scan_call_time:.2f}s on={on_wall - scan_call_time:.2f}s)")

        if args.tail_dark_sec > 0:
            t_off = args.duration_sec - args.tail_dark_sec
            sleep_for = scan_call_time + t_off - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)
            off_wall = time.time()
            psu.set_voltage(CONTROL_CHANNEL, 0.0)
            dark_events.append({
                "index": len(schedule),
                "nominal_t_sec": t_off,
                "elapsed_off_sec": off_wall - scan_call_time,
                "elapsed_on_sec": None,  # stays off through scan end
            })
            print(f"  tail dark window: light off at t={off_wall - scan_call_time:.2f}s (stays off)")

        remaining = args.duration_sec - (time.time() - scan_call_time)
        if remaining > 0:
            print(f"[*] Waiting {remaining:.0f}s for scan tail ...")
            time.sleep(remaining)

        print("[*] Waiting for scan to finish flushing ...")
        iface.scan_workflow.await_complete(timeout_sec=args.duration_sec + 60)

    finally:
        # Stop the sensor-fan monitor and GUARANTEE the fan is back ON, however the
        # scan ended (target reached, exception, or early exit).
        fan_stop.set()
        if fan_monitor_thread is not None:
            fan_monitor_thread.join(timeout=6)
        if args.sensor_fan_off_until_temp > 0 and not fan_restore.get("restored"):
            try:
                sensor.set_fan_control(True)
                print("[*] Sensor fan restored ON in cleanup.")
            except Exception as _e:
                print(f"[!] Sensor fan cleanup restore failed: {_e}")
        if cam_telem_logger is not None:
            try:
                cam_telem_logger.stop()
                print("[*] Camera telemetry logging stopped.")
            except Exception as _e:
                print(f"[!] Camera telemetry stop failed: {_e}")
        # Module temperature at scan end (streaming has stopped, so the IMU
        # read is safe again) -- the warm end-state of the deep assembly.
        imu_temp_end = read_imu_temp(sensor, "end (post-scan)")
        thorlabs_stop.set()
        if thorlabs_thread is not None:
            thorlabs_thread.join(timeout=5)
            print(f"[*] Thorlabs logging stopped -> {thorlabs_csv_path}")
        if meter is not None:
            try:
                meter.close()
            except Exception as exc:
                print(f"[!] Error while closing Thorlabs meter: {exc}")

        if args.leave_source_on:
            print(f"[*] Leaving illumination source ON between runs (control={args.control_voltage} V) ...")
            try:
                # Restore the light level (the tail-dark window left control at 0 V).
                psu.set_output(SUPPLY_CHANNEL, True)
                psu.set_output(CONTROL_CHANNEL, True)
                psu.set_voltage(CONTROL_CHANNEL, args.control_voltage)
            except PSUError as exc:
                print(f"[!] Error keeping PSU outputs on: {exc}")
        else:
            print("[*] Shutting down PSU outputs ...")
            try:
                psu.set_voltage(CONTROL_CHANNEL, 0.0)
                psu.set_output(CONTROL_CHANNEL, False)
                psu.set_output(SUPPLY_CHANNEL, False)
            except PSUError as exc:
                print(f"[!] Error while shutting down PSU outputs: {exc}")
        psu.close()
        try:
            sensor.disable_camera_power(CAMERA_MASK)
        except Exception as exc:
            print(f"[!] Error while powering off cameras: {exc}")
        iface.stop()

    if scan_error:
        print(f"[!] Scan worker reported an error: {scan_error['exc']}")

    mask_hex = f"{CAMERA_MASK:02X}"
    raw_candidates = sorted(args.data_dir.glob(f"*_{args.subject_id}_{side}_mask{mask_hex}_raw.csv"))
    raw_csv_path = str(raw_candidates[-1]) if raw_candidates else None
    if raw_csv_path:
        print(f"[+] Raw histogram CSV: {raw_csv_path}")
    else:
        print(f"[!] Could not locate raw CSV in {args.data_dir} matching subject={args.subject_id} side={side} mask={mask_hex}")

    dropout_notes: list[str] = []
    if raw_csv_path:
        print("[*] Checking for camera dropouts ...")
        try:
            dropout_notes = detect_camera_dropouts(raw_csv_path, args.duration_sec)
        except Exception as exc:
            # Best-effort QC only -- must never cost us the meta JSON/registry
            # row for a scan that otherwise completed fine.
            print(f"[!] Dropout check failed (non-fatal): {exc!r}")
            dropout_notes = ["dropout_check_failed"]
        if dropout_notes:
            print(f"[!] Dropouts detected: {', '.join(dropout_notes)}")
        else:
            print("[+] No camera dropouts detected.")

    meta = {
        "subject_id": args.subject_id,
        "side": side,
        "sensor_serial": sensor_serial,
        "camera_mask": CAMERA_MASK,
        "control_voltage": args.control_voltage,
        "supply_voltage": args.supply_voltage,
        "duration_sec": args.duration_sec,
        "dark_interval_sec": args.dark_interval_sec,
        "dark_duration_sec": args.dark_duration_sec,
        "dark_start_offset_sec": args.dark_start_offset_sec,
        "front_dark_sec": args.front_dark_sec,
        "tail_dark_sec": args.tail_dark_sec,
        "prewarm_min": args.prewarm_min,
        "scan_call_time_epoch": scan_call_time,
        "dark_events": dark_events,
        "raw_csv_path": raw_csv_path,
        "thorlabs_csv_path": str(thorlabs_csv_path) if meter is not None else None,
        "camera_dropouts": dropout_notes,
        "imu_temp_start_c": imu_temp_start,
        "imu_temp_end_c": imu_temp_end,
        "leave_source_on": args.leave_source_on,
        "sensor_fan_off_until_temp_c": args.sensor_fan_off_until_temp or None,
        "sensor_fan_sentinel_cam": args.fan_sentinel_cam if args.sensor_fan_off_until_temp > 0 else None,
        "sensor_fan_restore": fan_restore or None,   # {restored, at_sec, at_temp, reason, set_ok}
        "camera_telemetry_csvs": (cam_telem_logger.paths if cam_telem_logger is not None else None),
        "camera_crop": CAMERA_CROP,
        "camera_raw": CAMERA_RAW,
        "camera_reg_writes": [f"0x{a:04X}=0x{v:02X}" for a, v in CAMERA_REG_WRITES] or None,
    }
    meta_path = args.data_dir / f"{args.subject_id}_drift_meta.json"
    meta_path.write_text(json.dumps(meta, indent=2))
    print(f"[+] Wrote {meta_path}")

    notes_parts = list(dropout_notes)
    if args.prewarm_min > 0:
        notes_parts.append(f"prewarm_{args.prewarm_min:g}min")
    if scan_error:
        notes_parts.append(f"scan_error={scan_error['exc']!r}")
    raw_csv_size_mb = round(Path(raw_csv_path).stat().st_size / 1e6, 1) if raw_csv_path else None
    registry_row = {
        "run_timestamp": datetime.now().isoformat(timespec="seconds"),
        "subject_id": args.subject_id,
        "sensor_serial": sensor_serial,
        "side": side,
        "control_voltage_v": args.control_voltage,
        "supply_voltage_v": args.supply_voltage,
        "duration_sec": args.duration_sec,
        "dark_interval_sec": args.dark_interval_sec,
        "dark_duration_sec": args.dark_duration_sec,
        "n_dark_events": len(dark_events),
        "raw_csv_path": raw_csv_path,
        "raw_csv_size_mb": raw_csv_size_mb,
        "thorlabs_csv_path": str(thorlabs_csv_path) if meter is not None else None,
        "notes": "; ".join(notes_parts) if notes_parts else "clean",
    }
    append_registry_row(args.registry_path, registry_row)
    print(f"[+] Appended registry row to {args.registry_path}")

    return 0 if raw_csv_path else 1


if __name__ == "__main__":
    sys.exit(main())
