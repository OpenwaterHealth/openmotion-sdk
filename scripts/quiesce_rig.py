"""Put the rig in a safe idle state after a capture script died uncleanly.

usage: quiesce_rig.py [--sides left right] [--power-off]

A capture killed hard (task kill, crash, closed terminal) skips its teardown:
the console trigger -- and the laser -- keep running, and the sensor may stay
in image mode. This stops the trigger, exits image mode, disables the camera
streams and external frame sync, and optionally powers the cameras off (lets
hot cameras cool). It prints the cameras' last die temperatures first.

Make sure the dead capture process is really gone first (it holds the USB
devices): e.g. PowerShell
  Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'thermal_soak|full_frame' }
"""
import argparse
import time

from omotion import MotionInterface


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sides", nargs="+", choices=["left", "right"], default=["left", "right"])
    ap.add_argument("--power-off", action="store_true", help="also power every camera off")
    a = ap.parse_args()
    iface = MotionInterface(data_dir="quiesce_sdkdata")
    iface.start(wait=True, wait_timeout=3.0)
    iface.wait_for_ready(console=True, sensors=1, timeout=30)
    time.sleep(2.0)
    try:
        try:
            print("console stop_trigger:", iface.console.stop_trigger(), flush=True)
        except Exception as e:
            print("console stop_trigger FAILED:", e, flush=True)
        for side in a.sides:
            s = getattr(iface, side)
            if s is None or not s.is_connected():
                print(side, "not connected")
                continue
            time.sleep(0.5)
            steps = [
                ("image mode exit", s.image_mode_exit_status),
                ("disable camera streams", lambda: s.disable_camera(0xFF)),
                ("disable external frame sync", s.disable_camera_fsin_ext),
                ("die temps (C, stale for cameras already off)", lambda: [
                    round(c["tpm_avg_c"], 1) if c.get("valid") else None
                    for c in (s.get_camera_telemetry() or {}).get("cameras", [])]),
            ]
            if a.power_off:
                steps.append(("camera power off", lambda: s.disable_camera_power(0xFF)))
            for name, fn in steps:
                try:
                    print(f"{side} {name}: {fn()}", flush=True)
                except Exception as e:
                    print(f"{side} {name}: FAILED {e}", flush=True)
    finally:
        iface.stop()


if __name__ == "__main__":
    main()
