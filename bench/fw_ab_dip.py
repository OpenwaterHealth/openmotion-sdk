#!/usr/bin/env python3
"""Sensor-firmware A/B on the cold-start warm-up dip.

Question: does sensor fw 1.8.3-rc.4 really remove most of the dip that 1.8.2
showed (VREG3 baselines: cam 7 ~2 % vs REBASE 10-13 %)?

Both images are the official bare-metal release assets
(motion-sensor-fw-baremetal-fpga.bin). Their camera bitstream at 0x081A0000 is
byte-identical, so only the MCU application differs. The flash never reaches
the last sector (serial / config), which is preserved.

Sequence (each scan = 30 min, mask 0xC3, source held on, no telemetry):
  flash 1.8.2 -> conditioning scan (unscored; equalizes thermal history)
  -> 120 min cold soak -> FWAB_01 (1.8.2)
  flash 1.8.3-rc.4 -> soak -> FWAB_02 (1.8.3-rc.4) -> soak -> FWAB_03 (1.8.3-rc.4)
  flash 1.8.2 -> soak -> FWAB_04 (1.8.2)
  flash 1.8.3-rc.4 (bench back to the state it was found in) -> rig OFF.
Flashing happens before each soak, so the soak resets the thermal state.

Flash step: power-cycle the rig, enter ROM DFU via MotionSensor.enter_dfu(),
STM32_Programmer_CLI write + verify at 0x08000000, power-cycle, then read the
firmware version back and abort the experiment on a mismatch (rig left ON in
whatever state, for a human to look at).

Drop a file named STOP in the data dir to end after the current step (the
final restore flash still runs).
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import subprocess
import sys
import time
from pathlib import Path

WORKTREE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKTREE / "scripts" / "thermal_sweep"))
import cooldown_dip_experiment as cde  # noqa: E402  (run_scan, cooldown, source_on, Shelly)

CLI = r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
FW_DIR = Path(r"C:\Users\openwater\Projects\ff_fw\fw_ab")
IMAGES = {"1.8.2": FW_DIR / "fw_1.8.2.bin", "1.8.3-rc.4": FW_DIR / "fw_1.8.3-rc.4.bin"}
PLAN = [  # (subject suffix or None for conditioning, firmware, cooldown minutes before the scan)
    ("00", "1.8.2", 0),
    ("01", "1.8.2", 120),
    ("02", "1.8.3-rc.4", 120),
    ("03", "1.8.3-rc.4", 120),
    ("04", "1.8.2", 120),
]
RESTORE_FW = "1.8.3-rc.4"

_SDK_SNIPPET = r"""
import sys, time
from omotion import MotionInterface
iface = MotionInterface(); iface.start()
if not iface.wait_for_ready(console=True, sensors=1, timeout=45):
    print("RESULT notready"); iface.stop(); sys.exit(2)
s = None
for _ in range(60):
    c = iface.connected_sensors()
    if c and c[0].read_serial_number():
        s = c[0]; break
    time.sleep(0.5)
if s is None:
    print("RESULT nohandle"); iface.stop(); sys.exit(3)
print("RESULT version", s.get_version())
if sys.argv[1] == "dfu":
    print("RESULT dfu", s.enter_dfu()); time.sleep(1.0)
iface.stop()
"""


def log(msg: str) -> None:
    print(f"{dt.datetime.now():%Y-%m-%d %H:%M:%S} FWAB {msg}", flush=True)


def sdk(action: str) -> dict:
    """Run a short SDK session in its own process ('version' or 'dfu')."""
    r = subprocess.run([sys.executable, "-c", _SDK_SNIPPET, action], cwd=str(WORKTREE),
                       capture_output=True, text=True, timeout=180)
    out = {}
    for line in r.stdout.splitlines():
        if line.startswith("RESULT "):
            parts = line.split(" ", 2)
            out[parts[1]] = parts[2] if len(parts) > 2 else ""
    return out


def power_cycle(rig) -> None:
    rig.off(); time.sleep(5.0); rig.on()
    log(f"rig power-cycled; waiting {cde.ENUM_WAIT_S:.0f} s for enumeration")
    time.sleep(cde.ENUM_WAIT_S)


def dfu_present() -> bool:
    r = subprocess.run([CLI, "-l", "usb"], capture_output=True, text=True, timeout=60)
    return "Device ID" in r.stdout and "0x0450" in r.stdout


def flash(rig, fw: str) -> str:
    """Flash + verify + reboot + version check. Returns the verified version or raises."""
    img = IMAGES[fw]
    sha = hashlib.sha256(img.read_bytes()).hexdigest()[:16]
    for attempt in (1, 2):
        log(f"flash {fw} ({img.name}, sha256 {sha}) attempt {attempt}")
        power_cycle(rig)
        if not dfu_present():
            res = sdk("dfu")
            log(f"  enter DFU: {res}")
            for _ in range(30):
                if dfu_present():
                    break
                time.sleep(1.0)
        if not dfu_present():
            log("  ROM DFU device not found")
            continue
        r = subprocess.run([CLI, "-c", "port=usb1", "-w", str(img), "0x08000000", "-v"],
                           capture_output=True, text=True, timeout=600)
        if "Download verified successfully" not in r.stdout:
            log("  write/verify FAILED: " + " | ".join(l.strip() for l in r.stdout.splitlines()[-6:]))
            continue
        log("  written and verified")
        power_cycle(rig)
        ver = sdk("version").get("version", "")
        log(f"  running firmware reports {ver!r}")
        if ver == fw:
            return ver
        log(f"  version mismatch (want {fw})")
    raise RuntimeError(f"could not flash and verify {fw}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data-dir", type=Path, default=WORKTREE / "bench" / "fw_ab_out")
    ap.add_argument("--prefix", default="FWAB")
    ap.add_argument("--plan", default=None,
                    help='override PLAN: comma list of suffix:firmware:cooldown_min, e.g. "03:1.8.3-rc.4:120,04:1.8.2:120"')
    ap.add_argument("--current-fw", default=None, choices=sorted(IMAGES),
                    help="firmware already on the sensor (skips a redundant first flash)")
    a = ap.parse_args()
    plan = PLAN if not a.plan else [(x.split(":")[0], x.split(":")[1], float(x.split(":")[2])) for x in a.plan.split(",")]
    if any(fw not in IMAGES for _, fw, _ in plan):
        ap.error(f"firmware must be one of {sorted(IMAGES)}")
    a.data_dir.mkdir(parents=True, exist_ok=True)
    stop_file = a.data_dir / "STOP"
    cde.CAMERA_REG_READS = ["0x3400", "0x4001"]       # documents camera init under each firmware
    cde.CAMERA_TELEMETRY = False
    rig, fan = cde.ShellyOutlet(cde.RIG_HOST), cde.ShellyOutlet(cde.FAN_HOST)
    steps_csv = a.data_dir / "fw_ab_steps.csv"
    new = not steps_csv.exists()
    with open(steps_csv, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "subject", "firmware", "firmware_verified", "cooldown_min", "drift_scan_rc"])
    log(f"=== firmware A/B: {len(plan)} scans -> {a.data_dir} ===")
    cde.source_on()
    current = a.current_fw
    try:
        for suffix, fw, cool in plan:
            if stop_file.exists():
                log("STOP file present -> ending")
                break
            if fw != current:
                current = flash(rig, fw)
            subject = f"{a.prefix}_{suffix}"
            log(f"=== {subject}: firmware {fw}, {cool} min cold soak first ===")
            cde.cooldown(rig, fan, cool, True)          # rig OFF (+ fan ON); 0 min = plain power cycle
            rc = cde.run_scan(rig, fan, subject, a.data_dir, 0xC3)
            with open(steps_csv, "a", newline="") as f:
                csv.writer(f).writerow([dt.datetime.now().isoformat(timespec="seconds"), subject, fw, current, cool, rc])
    except Exception as exc:
        log(f"ABORTED: {exc} -- rig left ON for inspection")
        return 1
    if current != RESTORE_FW:
        try:
            flash(rig, RESTORE_FW)
            log(f"bench restored to {RESTORE_FW}")
        except Exception as exc:
            log(f"RESTORE FAILED: {exc} -- rig left ON for inspection")
            return 1
    cde._shelly(rig, False, "rig")
    cde._shelly(fan, False, "fan")
    log("=== done (rig + fan OFF; source left ON) ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
