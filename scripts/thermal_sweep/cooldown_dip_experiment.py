#!/usr/bin/env python3
"""Cooldown vs warm-up-dip experiment (2026-07-15 night).

GOAL: verify that cooldown time controls the DEPTH and PROBABILITY of the
cold-start intensity dip, using a fan to accelerate cooling and the IMU
temperature as a board-level (deep-assembly) probe.

FIXED CONDITIONS
  * Illumination source ON the whole night (Keysight, separate mains) so the
    source is thermally stable -- rules out any source warm-up confound
    (drift_scan is launched with --leave-source-on).
  * 30-min drift scans, mask 0xFF. The dip lives in the first ~5 min; the rest
    is the plateau reference. Photodiode logged; dip measured later on the
    dark-corrected camera signal (mean_dc) with a photodiode-stability gate.
  * IMU temperature read at each scan's bring-up (cold-soak) and end -- a probe
    away from the camera dies, and (verified) it reads true ambient ~25 C when
    the module is fully cold, unlike the camera die which self-heats to ~35 C
    during configure. Read only when NOT streaming (8-cam + I2C wedges HISTO).
  * FAN on Shelly 192.168.1.214; RIG on Shelly 192.168.1.79 (separate plugs).
    Fan is OFF during scans (so it doesn't cool the cameras mid-scan) and ON
    during cooldowns. Cooldowns power the RIG OFF (no board self-heat) + fan ON
    for the fastest, coldest soak.

PLAN
  Phase 1 -- fan cooling characterization (~1 h):
    One cold-start scan (module is cold from the day), which also HEATS the
    module, then a continuous IMU cooling curve with the rig powered but
    cameras off and the fan on (poll IMU every 15 s for FAN_PROBE_MIN). Gives
    the fan's cooling rate + floor. (Rig-on has some board self-heat, so this
    floor is an upper bound; the rig-OFF cooldowns in Phase 2 go colder.)

  Phase 2 -- cooldown dose-response + probability (~9-10 h):
    A LADDER of 30-min scans, each preceded by a rig-OFF cooldown of a given
    duration with the fan on/off (see LADDER). Reading the IMU temp at each
    scan start ties dip depth to the actual achieved module temperature, not
    just the wall-clock cooldown. Durations are replicated to measure the dip
    PROBABILITY, and a few fan-OFF cooldowns give the fan-vs-natural speedup.

Durations below are first-night guesses (the fan's cooling rate is unknown);
the scan-start IMU temps will show the real cold-soaks and we retune next night.
Drop a file named STOP in the data dir to end cleanly after the current step.
Analysis is separate (analyze_drift_scan.py + dip_from_meandc.py + IMU CSVs).
"""
import argparse
import csv
import datetime
import subprocess
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parents[2]
DRIFT = WORKTREE / "bench" / "drift_scan.py"
sys.path.insert(0, r"C:\Users\openwater\Projects\openmotion-bloodflow-app\tests")
sys.path.insert(0, str(WORKTREE / "bench" / "keysight-psu"))
from shelly import ShellyOutlet  # noqa: E402

RIG_HOST = "192.168.1.79"
FAN_HOST = "192.168.1.214"
SUPPLY_CH, CONTROL_CH = 2, 3          # Keysight: illumination supply + control
CONTROL_VOLTAGE = 2.0
SCAN_MIN = 30.0
ENUM_WAIT_S = 60.0
FAN_PROBE_MIN = 30.0                  # Phase-1 continuous fan-cooling curve
IMU_POLL_S = 15.0

# Phase-2 ladder: (cooldown_minutes, fan_on). Each entry = one rig-OFF cooldown
# followed by a 30-min scan. Deep first (validation), then a 10/20/30/45 curve,
# replicates for probability, and two fan-OFF baselines.
LADDER = [
    (45, True), (10, True), (20, True), (30, True),
    (45, True), (20, True), (30, True), (10, True),
    (45, False), (20, False),
    (30, True), (45, True),
]

DATA_DIR = WORKTREE / "bench" / "cooldown_dip_out"


def log(msg: str) -> None:
    print(f"{datetime.datetime.now():%Y-%m-%d %H:%M:%S} EXP {msg}", flush=True)


def _shelly(outlet: ShellyOutlet, on: bool, name: str) -> None:
    for attempt in range(1, 4):
        try:
            outlet.on() if on else outlet.off()
            return
        except Exception as e:
            log(f"{name} {'on' if on else 'off'} attempt {attempt} failed: {e}")
            time.sleep(5)
    log(f"{name} {'on' if on else 'off'} FAILED 3x")


def source_on() -> None:
    """Turn the Keysight illumination on and release the handle (outputs persist)."""
    try:
        from keysight_psu import KeysightE36300
        psu = KeysightE36300.connect()
        psu.set_current_limit(SUPPLY_CH, 2.0); psu.set_voltage(SUPPLY_CH, 24.0)
        psu.set_current_limit(CONTROL_CH, 0.5); psu.set_voltage(CONTROL_CH, CONTROL_VOLTAGE)
        psu.set_output(SUPPLY_CH, True); psu.set_output(CONTROL_CH, True)
        psu.close()
        log(f"illumination source ON (24 V, {CONTROL_VOLTAGE} V control) -- kept on all night")
    except Exception as e:
        log(f"source_on FAILED (continuing): {e}")


CAMERA_TELEMETRY = False   # set by main() from --camera-telemetry; passed to every scan


def run_scan(rig: ShellyOutlet, fan: ShellyOutlet, subject: str, data_dir: Path,
             mask: int, warmup_temp: float = 0.0, crop: bool = False, raw: bool = False,
             reg_write: str = "") -> int:
    """Fan OFF, rig ON (cold boot), enumerate, run one 30-min drift scan on the given
    camera mask with the source held on. drift_scan reads the IMU temp at start + end.
    Only the masked cameras are powered, so the thermal load matches that config.
    warmup_temp > 0 = accelerated warm-up: drift_scan cuts the SENSOR fan at scan start
    and restores it when cam 7's die reaches warmup_temp (deg C).
    crop = configure the cameras cropped to 1720x1280 (DEBUG_FLAG_CAMERA_CROP)."""
    _shelly(fan, False, "fan")            # never cool the cameras during a scan (external .214 fan)
    already_on = rig.is_on()
    _shelly(rig, True, "rig")             # no-op if a powered-idle gap left it on (no power cycle)
    log(f"rig ON ({'already powered' if already_on else 'cold boot'}); "
        f"waiting {ENUM_WAIT_S:.0f}s for enumeration")
    time.sleep(ENUM_WAIT_S)
    cmd = [sys.executable, "-u", str(DRIFT),
           "--duration-sec", str(SCAN_MIN * 60.0),
           "--control-voltage", str(CONTROL_VOLTAGE),
           "--camera-mask", f"0x{mask:02X}",
           "--leave-source-on",
           "--subject-id", subject, "--data-dir", str(data_dir)]
    if warmup_temp > 0:
        cmd += ["--sensor-fan-off-until-temp", str(warmup_temp)]
    if crop:
        cmd += ["--camera-crop"]
    if raw:
        cmd += ["--camera-raw"]
    if reg_write:
        cmd += ["--camera-reg", reg_write]
    if CAMERA_TELEMETRY:
        cmd += ["--camera-telemetry"]
    wu = f", sensor-fan-off->{warmup_temp:g}C" if warmup_temp > 0 else ""
    log(f"launch drift_scan {subject}: {SCAN_MIN:g} min, mask 0x{mask:02X}{wu}"
        f"{', CROP 1720x1280' if crop else ''}{', RAW mode' if raw else ''}"
        f"{', cam-telemetry' if CAMERA_TELEMETRY else ''}")
    t0 = time.time()
    try:
        rc = subprocess.run(cmd, cwd=str(WORKTREE)).returncode
    except Exception as e:
        rc = -999
        log(f"drift_scan raised: {e}")
    log(f"drift_scan {subject} exited rc={rc} after {(time.time()-t0)/60:.1f} min")
    return rc


def cooldown(rig: ShellyOutlet, fan: ShellyOutlet, minutes: float, fan_on: bool,
             powered: bool = False) -> None:
    """Gap between scans.

    powered=False (default): rig OFF -- the whole assembly cools with no board
    self-heat. This is the cold-soak protocol every dip campaign has used.

    powered=True: rig stays ON (module powered but not streaming, the real
    between-scans state in the field), with the fan following fan_on. Used for
    the warm-restart ladder: the next scan starts from a partially-warm assembly
    rather than a cold one, and fan_on sets how far it cools during the gap.
    """
    if powered:
        _shelly(rig, True, "rig")
        _shelly(fan, fan_on, "fan")
        log(f"powered idle {minutes:g} min (rig ON, fan {'ON' if fan_on else 'OFF'} "
            f"-- assembly cools from the streaming plateau but stays powered)")
        time.sleep(minutes * 60.0)
        _shelly(fan, False, "fan")
        return
    _shelly(rig, False, "rig")
    _shelly(fan, fan_on, "fan")
    log(f"cooldown {minutes:g} min (rig OFF, fan {'ON' if fan_on else 'OFF'})")
    time.sleep(minutes * 60.0)
    _shelly(fan, False, "fan")


def fan_cooling_probe(fan: ShellyOutlet, minutes: float, out_csv: Path) -> None:
    """Rig already ON, cameras OFF, fan ON: poll IMU temp to trace the cooling
    curve. Cameras are not streaming so the IMU I2C read is safe."""
    from omotion import MotionInterface
    _shelly(fan, True, "fan")
    log(f"fan cooling probe: {minutes:g} min, polling IMU every {IMU_POLL_S:g}s -> {out_csv.name}")
    iface = MotionInterface()
    iface.start()
    if not iface.wait_for_ready(console=True, sensors=1, timeout=90):
        log("fan probe: sensor not ready, skipping"); iface.stop(); return
    sensor = iface.connected_sensors()[0]
    try:
        sensor.imu_init(); sensor.imu_on(); time.sleep(0.2)
        t0 = time.time()
        with open(out_csv, "w", newline="") as f:
            w = csv.writer(f); w.writerow(["elapsed_s", "imu_temp_c", "fan_on", "rig_on"])
            while time.time() - t0 < minutes * 60.0:
                try:
                    t = float(sensor.imu_get_temperature())
                    w.writerow([f"{time.time()-t0:.1f}", f"{t:.2f}", 1, 1]); f.flush()
                except Exception as e:
                    log(f"fan probe IMU read error: {e}")
                time.sleep(IMU_POLL_S)
    finally:
        iface.stop()
    log("fan cooling probe complete")


def _parse_ladder(spec: str) -> "list[tuple[float, bool, int | None, float, bool, bool]]":
    """Parse 'cooldownMin:fanOn[:mask[:warmupC[:crop]]]' tokens, e.g.
    '120:1:0xC3:0:0,120:1:0xC3:0:1' -> [(120.0,True,195,0.0,False),(120.0,True,195,0.0,True)].
    mask (hex/int), warmupC and crop are optional per step; mask omitted -> --camera-mask
    default; warmupC 0/omitted -> control (sensor fan left at firmware default), >0 ->
    accelerated warm-up (sensor fan off until cam 7 die reaches warmupC); crop 1 ->
    DEBUG_FLAG_CAMERA_CROP (1720x1280) for that scan."""
    out = []
    for tok in spec.split(","):
        parts = tok.split(":")
        cd, fan = parts[0], parts[1]
        mask = int(parts[2], 0) if len(parts) > 2 and parts[2] else None
        warmup = float(parts[3]) if len(parts) > 3 and parts[3] else 0.0
        crop = bool(int(parts[4])) if len(parts) > 4 and parts[4] else False
        raw = bool(int(parts[5])) if len(parts) > 5 and parts[5] else False
        powered = bool(int(parts[6])) if len(parts) > 6 and parts[6] else False
        regw = bool(int(parts[7])) if len(parts) > 7 and parts[7] else False
        out.append((float(cd), bool(int(fan)), mask, warmup, crop, raw, powered, regw))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data-dir", default=str(DATA_DIR))
    ap.add_argument("--skip-phase1", action="store_true", help="skip the warm-up scan + fan cooling probe")
    ap.add_argument("--ladder", default=None,
                    help="override the Phase-2 ladder: comma list of cooldownMin:fanOn[:mask[:warmupC]] steps, "
                         "e.g. '120:1:0xC3:0,120:1:0xC3:80' (fanOn=1/0; mask hex/int optional -> --camera-mask; "
                         "warmupC optional, 0=control, >0=accelerated warm-up = sensor fan off until cam 7 die "
                         "hits warmupC). Every step still gets its full cooldown, so the cold soak is held "
                         "constant across control and warm-up runs.")
    ap.add_argument("--subject-prefix", default="CDDIP",
                    help="subject/file prefix (use a distinct one to avoid clobbering a prior run's files)")
    ap.add_argument("--start-index", type=int, default=2,
                    help="index of the first ladder scan (Phase 1, when run, uses _01)")
    ap.add_argument("--camera-mask", default="0xFF",
                    help="default camera mask (hex 0xC3 or int) for scans; per-ladder-step masks override it. "
                         "Clinical 'far 4' = 0xC3 (cams 1,2,7,8); all-8 = 0xFF.")
    ap.add_argument("--camera-reg", default="", metavar="ADDR=VAL",
                    help="camera register write (e.g. 0x4001=0x2b) applied on ladder steps whose 8th "
                         "field (regwrite) is 1. Written to every masked camera post-configure via "
                         "drift_scan --camera-reg. For vendor register-change tests.")
    ap.add_argument("--camera-telemetry", action="store_true",
                    help="pass --camera-telemetry to every drift_scan (1 Hz per-camera condition CSVs; "
                         "needs the sensor-fw#94 telemetry firmware).")
    args = ap.parse_args()
    global CAMERA_TELEMETRY
    CAMERA_TELEMETRY = args.camera_telemetry
    data_dir = Path(args.data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    stop_file = data_dir / "STOP"
    default_mask = int(args.camera_mask, 0)
    ladder = _parse_ladder(args.ladder) if args.ladder else [(cd, fan, None, 0.0, False, False, False, False) for cd, fan in LADDER]
    prefix = args.subject_prefix

    # Powered-idle gaps keep the rig ON between scans, so there is no power cycle
    # to clear a stalled COMM endpoint. Sustained camera-telemetry polling stalls
    # that endpoint (Errno 10060) partway through a scan (sensor-fw #96); with a
    # rig-OFF cooldown the next scan's power cycle recovers it, but a powered-idle
    # gap carries the stall forward and the next connect times out. Until #96 is
    # fixed the two are incompatible -- disable telemetry when any step is powered.
    if CAMERA_TELEMETRY and any(step[6] for step in ladder):
        log("WARNING: camera telemetry disabled -- a powered-idle step is present and "
            "sustained telemetry polling stalls the COMM endpoint with no power cycle to "
            "recover it (sensor-fw #96). Die temp + intensity come from the raw stream anyway.")
        CAMERA_TELEMETRY = False

    rig = ShellyOutlet(RIG_HOST)
    fan = ShellyOutlet(FAN_HOST)

    log(f"=== cooldown-dip experiment: {'Phase1 + ' if not args.skip_phase1 else ''}"
        f"{len(ladder)} ladder scans (prefix {prefix}) -> {data_dir} ===")
    source_on()

    # Phase 1: a cold-start scan (module is cold from the day; also heats it),
    # then the continuous fan cooling curve.
    if not args.skip_phase1 and not stop_file.exists():
        log("--- Phase 1: cold-start scan (heats module) + fan cooling probe ---")
        run_scan(rig, fan, f"{prefix}_01", data_dir, default_mask)
        fan_cooling_probe(fan, FAN_PROBE_MIN, data_dir / "fan_cooling_probe.csv")

    # Phase 2: cooldown ladder.
    log("--- Phase 2: cooldown ladder ---")
    for k, (cd_min, fan_on, step_mask, warmup, crop, raw, powered, regw) in enumerate(ladder):
        if stop_file.exists():
            log("STOP file present -> exiting"); break
        idx = args.start_index + k
        mask = step_mask if step_mask is not None else default_mask
        reg = args.camera_reg if regw else ""
        # describe every lever this step actually pulls (a step with none is the control)
        levers = ", ".join(filter(None, [
            f"warm-up->{warmup:g}C" if warmup > 0 else "",
            "CROP" if crop else "",
            "RAW" if raw else "",
            f"REG {reg}" if reg else "",
        ])) or "control"
        gap = (f"{cd_min:g} min powered idle (rig ON)" if powered
               else f"{cd_min:g} min cooldown (fan {'ON' if fan_on else 'OFF'})")
        log(f"=== ladder {k+1}/{len(ladder)}: {gap}, mask 0x{mask:02X} "
            f"[{levers}] -> {prefix}_{idx:02d} ===")
        cooldown(rig, fan, cd_min, fan_on, powered)
        run_scan(rig, fan, f"{prefix}_{idx:02d}", data_dir, mask, warmup, crop, raw, reg)

    _shelly(rig, False, "rig")     # leave rig off (module cool) at the end
    _shelly(fan, False, "fan")
    log("=== experiment complete (rig + fan OFF; source left ON) ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
