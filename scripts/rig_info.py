"""Read-only preflight for full-frame capture: what is connected and is it ready?

usage: rig_info.py

Prints, per connected device:
  - console: firmware version, trigger config;
  - each sensor: firmware version (NOTE: stamped at the last CMake configure,
    so it can be stale), boot mode (bare-metal / bootloader), whether the
    firmware answers the image-mode command (feature/99 builds do; stock
    firmware NAKs it), camera power status and die temperatures.
Changes nothing on the rig (the image-mode probe is the idempotent "exit").
"""
import time

from omotion import MotionInterface


def main():
    iface = MotionInterface(data_dir="rig_info_sdkdata")
    iface.start(wait=True, wait_timeout=3.0)
    try:
        iface.wait_for_ready(console=True, sensors=1, timeout=30)
    except Exception as e:
        print("wait_for_ready:", e)
    time.sleep(2.0)
    try:
        con = iface.console
        if con is None or not con.is_connected():
            print("console: NOT connected")
        else:
            for name, fn in (("firmware", con.get_version), ("trigger", con.get_trigger_json)):
                try:
                    print(f"console {name}: {fn()}")
                except Exception as e:
                    print(f"console {name}: FAILED {e}")
        for side in ("left", "right"):
            s = getattr(iface, side)
            if s is None or not s.is_connected():
                print(f"{side} sensor: NOT connected")
                continue
            print(f"{side} sensor:")
            for name, fn in (
                ("firmware (stamp may be stale)", s.get_version),
                ("boot mode", lambda: s.get_boot_mode().name),
                ("image mode supported", lambda: s.image_mode_exit_status() is not None),
                ("camera power", s.get_camera_power_status),
                ("die temps C (only meaningful for powered cameras)", lambda: [
                    round(c["tpm_avg_c"], 1) if c.get("valid") else None
                    for c in (s.get_camera_telemetry() or {}).get("cameras", [])]),
            ):
                try:
                    print(f"  {name}: {fn()}")
                except Exception as e:
                    print(f"  {name}: FAILED {e}")
    finally:
        iface.stop()


if __name__ == "__main__":
    main()
