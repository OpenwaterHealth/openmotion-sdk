#!/usr/bin/env python3
"""Full-frame images through the cold-start warm-up dip, lit by the Keysight source.

Wraps the stride-composite thermal soak (openmotion-sdk feature/296,
scripts/thermal_soak.py, run as its own python process) with the drift-scan
bench conditions, so the images line up with the histogram dip runs:

  * Illumination = Keysight source (ch2 24 V supply, ch3 control at
    --control-voltage), held on between runs. The console laser is NOT fired
    (thermal_soak --no-laser: camera sync still runs at 40 Hz, TA trigger off).
  * Dark reference as in drift_scan.py: source off for --front-dark-s at the
    start of imaging, for --dark-dur-s every --dark-interval-s, and for
    --tail-dark-s at the end. Every toggle is logged with its wall-clock time
    (dark_events.csv) so analysis can mark the composite rows that were
    exposed while the source was off (npz row_t_s + run.json start).
  * Thorlabs photodiode logged the whole time (thorlabs.csv, epoch seconds).
  * Optional cold start per step: rig OFF + fan ON for --cooldown-min, then
    fan OFF, rig ON (cold boot), wait for enumeration, capture.

Each --pairs entry (0-based camera indices, e.g. "0,1;6,7") is one cold-start
step: all cameras of --scan-mask are powered and triggered as in a clinical
scan; the pair is imaged concurrently (one composite per ~4 s per camera at
production timing).

Output per step: <out>/<prefix>_<NN>_cams<a><b>/
    soak/                thermal_soak output (composites.csv, camera_telemetry.csv,
                         events.csv, images/..., run.json, soak.log)
    soak_console.log     thermal_soak stdout/stderr
    dark_events.csv      kind, off/on epoch, off/on seconds since imaging start
    thorlabs.csv         epoch_s, power_w
    capture_meta.json    t0 (imaging start, epoch), args, soak return code

Never hard-kill thermal_soak (it leaves trigger/laser running): drop a file
named STOP in --out to end after the current step instead.

Usage:
  python bench/ff_dip_capture.py --pairs "0,1" --capture-min 3 --out bench/ff_dip_out --prefix FFVAL
  python bench/ff_dip_capture.py --pairs "0,1;6,7" --cooldown-min 120 --out bench/ff_dip_out --prefix FFDIP
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "keysight-psu"))
sys.path.insert(0, str(HERE / "thorlabs-pm100"))
sys.path.insert(0, r"C:\Users\openwater\Projects\openmotion-bloodflow-app\tests")

SOAK_SDK = Path(r"C:\Users\openwater\Projects\openmotion-sdk\.claude\worktrees\ff-thermal-soak")
SUPPLY_CH, CONTROL_CH = 2, 3
RIG_HOST, FAN_HOST = "192.168.1.79", "192.168.1.214"
ENUM_WAIT_S = 60.0


def log(msg: str) -> None:
    print(f"{dt.datetime.now():%Y-%m-%d %H:%M:%S} FFDIP {msg}", flush=True)


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pairs", required=True, help='cameras imaged per step, 0-based, e.g. "0,1;6,7"')
    ap.add_argument("--scan-mask", default="0xC3", help="cameras powered + triggered (clinical 0xC3)")
    ap.add_argument("--capture-min", type=float, default=30.0, help="minutes of imaging per step")
    ap.add_argument("--cooldown-min", type=float, default=0.0,
                    help="rig-OFF + fan-ON cold soak before each step (0 = capture now, no power cycle)")
    ap.add_argument("--control-voltage", type=float, default=2.0)
    ap.add_argument("--dark-interval-s", type=float, default=60.0)
    ap.add_argument("--dark-dur-s", type=float, default=1.0)
    ap.add_argument("--front-dark-s", type=float, default=2.0)
    ap.add_argument("--tail-dark-s", type=float, default=2.0)
    ap.add_argument("--setup-margin-s", type=float, default=90.0,
                    help="added to --capture-min for thermal_soak's --duration-h, which also counts "
                         "connect + camera power-on + FPGA load before imaging starts")
    # 10 s = thermal_soak default. 1 s polling stalled the sensor COMM endpoint ~3 min into imaging
    # (FFDIP_01, USBTimeoutError 10060, cf. sensor-fw #96) -- and the --max-die-c stop needs telemetry.
    ap.add_argument("--telemetry-s", type=float, default=10.0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--prefix", default="FFDIP")
    ap.add_argument("--start-index", type=int, default=1)
    ap.add_argument("--rig-off-after", action="store_true", help="power the rig off after the last step")
    return ap.parse_args()


class Source:
    """Keysight illumination: connect once, toggle the control channel for darks."""

    def __init__(self, control_v: float):
        from keysight_psu import KeysightE36300
        self.v = control_v
        self.psu = KeysightE36300.connect()
        self.psu.set_current_limit(SUPPLY_CH, 2.0)
        self.psu.set_voltage(SUPPLY_CH, 24.0)
        self.psu.set_current_limit(CONTROL_CH, 0.5)
        self.psu.set_voltage(CONTROL_CH, control_v)
        self.psu.set_output(SUPPLY_CH, True)
        self.psu.set_output(CONTROL_CH, True)
        log(f"source ON: {self.psu} (24 V, control {control_v} V)")

    def off(self) -> float:
        self.psu.set_voltage(CONTROL_CH, 0.0)
        return time.time()

    def on(self) -> float:
        self.psu.set_voltage(CONTROL_CH, self.v)
        return time.time()

    def close(self) -> None:
        self.on()                       # standing instruction: leave the source ON
        self.psu.close()
        log("source left ON")


def thorlabs_logger(csv_path: Path, stop: threading.Event) -> None:
    import pyvisa
    from read_thorlabs_powermeter import find_thorlabs_resource
    rm = pyvisa.ResourceManager("@py")
    res = find_thorlabs_resource(list(rm.list_resources()))
    if not res:
        log("WARNING: no Thorlabs meter found -- photodiode not logged")
        return
    inst = rm.open_resource(res)
    inst.timeout, inst.read_termination, inst.write_termination = 3000, "\n", "\n"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["epoch_s", "power_w"])
        while not stop.is_set():
            try:
                w.writerow([f"{time.time():.4f}", float(inst.query("MEAS:POW?"))])
                f.flush()
            except Exception as exc:
                log(f"Thorlabs read error: {exc}")
            stop.wait(0.05)
    inst.close()


def wait_for_event(events_csv: Path, name: str, proc: subprocess.Popen, timeout_s: float):
    """Poll thermal_soak's events.csv for the first row with event == name -> (epoch, row)."""
    t_lim = time.time() + timeout_s
    while time.time() < t_lim and proc.poll() is None:
        if events_csv.exists():
            with open(events_csv, newline="", encoding="utf-8") as f:
                for row in csv.DictReader(f):
                    if row.get("event") == name:
                        return dt.datetime.fromisoformat(row["wall_time"]).timestamp(), row
        time.sleep(0.05)
    return None, None


def cold_start(minutes: float) -> None:
    from shelly import ShellyOutlet
    rig, fan = ShellyOutlet(RIG_HOST), ShellyOutlet(FAN_HOST)
    rig.off(); fan.on()
    log(f"cold soak {minutes:g} min (rig OFF, fan ON)")
    time.sleep(minutes * 60.0)
    fan.off(); rig.on()
    log(f"rig ON (cold boot), fan OFF; waiting {ENUM_WAIT_S:.0f} s for enumeration")
    time.sleep(ENUM_WAIT_S)


def capture(a, src: Source, cams: list[int], step_dir: Path) -> int:
    step_dir = step_dir.resolve()      # thermal_soak runs with cwd=SOAK_SDK: pass absolute paths
    step_dir.mkdir(parents=True, exist_ok=True)
    soak_dir = step_dir / "soak"
    duration_h = (a.capture_min * 60.0 + a.setup_margin_s) / 3600.0
    cmd = [sys.executable, "-u", str(SOAK_SDK / "scripts" / "thermal_soak.py"),
           "--sides", "left", "--scan-mask", a.scan_mask,
           "--cams", *[str(c) for c in cams], "--concurrent", str(len(cams)),
           "--no-laser", "--telemetry-s", str(a.telemetry_s),
           "--duration-h", f"{duration_h:.5f}", "--out", str(soak_dir)]
    env = dict(os.environ, PYTHONPATH=str(SOAK_SDK), PYTHONIOENCODING="utf-8")
    stop_pd = threading.Event()
    pd_thread = threading.Thread(target=thorlabs_logger, args=(step_dir / "thorlabs.csv", stop_pd), daemon=True)
    pd_thread.start()
    darks = []
    launch = time.time()
    log(f"launch thermal_soak: cams {cams} (0-based), {a.capture_min:g} min imaging -> {soak_dir}")
    with open(step_dir / "soak_console.log", "w", encoding="utf-8") as con:
        # python itself (not a shell): the soak tears down cleanly on its own duration end
        proc = subprocess.Popen(cmd, cwd=str(SOAK_SDK), env=env, stdout=con, stderr=subprocess.STDOUT)
        t0, _ = wait_for_event(soak_dir / "events.csv", "segment_start", proc, timeout_s=600)
        if t0 is None:
            log(f"imaging never started (thermal_soak rc={proc.poll()}); see soak_console.log")
        else:
            log(f"imaging started {t0 - launch:.1f} s after launch")
            t_end = launch + duration_h * 3600.0          # thermal_soak's own end (approx.)
            if a.front_dark_s > 0:
                off = src.off()
                time.sleep(max(0.0, t0 + a.front_dark_s - time.time()))
                darks.append(("front", off, src.on()))
            k = 1
            while proc.poll() is None:
                t_win = t0 + k * a.dark_interval_s
                if t_win + a.dark_dur_s > t_end - a.tail_dark_s - 5:
                    break
                while time.time() < t_win and proc.poll() is None:
                    time.sleep(0.01)
                if proc.poll() is not None:
                    break
                off = src.off()
                time.sleep(a.dark_dur_s)
                darks.append(("window", off, src.on()))
                k += 1
            if a.tail_dark_s > 0 and proc.poll() is None:
                while time.time() < t_end - a.tail_dark_s and proc.poll() is None:
                    time.sleep(0.05)
                if proc.poll() is None:
                    darks.append(("tail", src.off(), None))
        rc = proc.wait()                                   # soak ends itself at --duration-h
    if darks and darks[-1][2] is None:
        src.on()                                           # end the tail dark
    stop_pd.set()
    pd_thread.join(timeout=5)
    with open(step_dir / "dark_events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kind", "off_epoch", "on_epoch", "off_s", "on_s"])
        for kind, off, on in darks:
            w.writerow([kind, f"{off:.4f}", "" if on is None else f"{on:.4f}",
                        f"{off - t0:.4f}", "" if on is None else f"{on - t0:.4f}"])
    (step_dir / "capture_meta.json").write_text(json.dumps({
        "cams_0based": cams, "scan_mask": a.scan_mask, "t0_epoch": t0, "launch_epoch": launch,
        "capture_min": a.capture_min, "duration_h": duration_h, "soak_rc": rc,
        "n_dark_windows": sum(1 for d in darks if d[0] == "window"), "cmd": cmd,
        "control_voltage": a.control_voltage, "laser": "off (--no-laser); Keysight illumination",
    }, indent=1))
    log(f"thermal_soak exited rc={rc}; {len(darks)} dark events logged")
    return rc


def main() -> int:
    a = parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    pairs = [[int(c) for c in p.split(",")] for p in a.pairs.split(";")]
    src = Source(a.control_voltage)
    try:
        for i, cams in enumerate(pairs):
            if (out / "STOP").exists():
                log("STOP file present -> exiting")
                break
            name = f"{a.prefix}_{a.start_index + i:02d}_cams{''.join(str(c + 1) for c in cams)}"
            log(f"=== step {i + 1}/{len(pairs)}: {name} ===")
            if a.cooldown_min > 0:
                cold_start(a.cooldown_min)
            capture(a, src, cams, out / name)
    finally:
        src.close()
        if a.rig_off_after:
            from shelly import ShellyOutlet
            ShellyOutlet(RIG_HOST).off()
            log("rig OFF")
    log("=== done ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
