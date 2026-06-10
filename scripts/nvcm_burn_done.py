#!/usr/bin/env python3
"""
nvcm_burn_done.py - Retry burning the NVCM Done fuse on a CrossLink FPGA.

Use after NVCM programming completed successfully (content written) but the
Done fuse didn't take. Enters forced slave config, verifies IDCODE, enters
ISC mode, sends ISC_PROGRAM_DONE (0x5E), and verifies Done=1 in STATUS.

Usage:
    python scripts/nvcm_burn_done.py --camera 8
    python scripts/nvcm_burn_done.py --camera 8 --attempts 3
"""

import argparse
import logging
import os
import sys
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SDK_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from omotion import MotionInterface
from omotion.config import DEBUG_FLAG_USB_PRINTF, DEBUG_FLAG_CMD_VERBOSE

_CONNECT_TIMEOUT = 12.0
FPGA_ADDR = 0x40
EXPECTED_IDCODE = bytes([0x01, 0x2C, 0x00, 0x43])
ACTIVATION_KEY = bytes([0xFF, 0xA4, 0xC6, 0xF4, 0x8A])


def _hex(b):
    return " ".join(f"{x:02X}" for x in b)


def enter_forced_config(sensor):
    """CRESETB low -> activation key -> CRESETB high -> wait."""
    sensor.creset(False)
    time.sleep(0.001)
    sensor.i2c_write(FPGA_ADDR, ACTIVATION_KEY)
    sensor.creset(True)
    time.sleep(0.010)


def check_idcode(sensor):
    """Read IDCODE (0xE0) and return True if it matches expected."""
    result = sensor.i2c_write_read(FPGA_ADDR, bytes([0xE0, 0x00, 0x00, 0x00]), 4)
    ok = result == EXPECTED_IDCODE
    print(f"  IDCODE: {_hex(result)}  {'OK' if ok else 'MISMATCH'}")
    return ok


def read_status(sensor):
    """Read STATUS register (0x3C), return (raw_bytes, done_bit)."""
    result = sensor.i2c_write_read(FPGA_ADDR, bytes([0x3C, 0x00, 0x00, 0x00]), 4)
    s = int.from_bytes(result, "big")
    done = (s >> 8) & 1
    busy = (s >> 12) & 1
    fail = (s >> 13) & 1
    print(f"  STATUS: {_hex(result)}  (0x{s:08X})  Done={done} Busy={busy} Fail={fail}")
    return result, done


def isc_enable(sensor, operand=0x02):
    """ISC_ENABLE with given operand."""
    sensor.i2c_write(FPGA_ADDR, bytes([0xC6, operand, 0x00, 0x00]))
    time.sleep(0.005)


def isc_program_done(sensor):
    """Send ISC_PROGRAM_DONE (0x5E) to burn the Done fuse."""
    sensor.i2c_write(FPGA_ADDR, bytes([0x5E, 0x00, 0x00, 0x00]))


def isc_disable(sensor):
    """Send ISC_DISABLE (0x26)."""
    sensor.i2c_write(FPGA_ADDR, bytes([0x26, 0x00, 0x00, 0x00]))
    time.sleep(0.001)


def attempt_done_burn(sensor, attempt, wait_ms=200):
    """One attempt at burning the Done fuse. Returns True if Done=1 after."""
    print(f"\n--- Attempt {attempt} (wait={wait_ms}ms) ---")

    print("Entering forced slave config...")
    enter_forced_config(sensor)

    if not check_idcode(sensor):
        print("IDCODE failed, aborting attempt.")
        sensor.creset(False)
        return False

    print("ISC_ENABLE (operand=0x02, NVCM read-enable)...")
    isc_enable(sensor, 0x02)

    print("STATUS before ISC_PROGRAM_DONE:")
    _, done_before = read_status(sensor)
    if done_before:
        print("Done bit is ALREADY SET! No burn needed.")
        isc_disable(sensor)
        sensor.creset(False)
        time.sleep(0.001)
        sensor.creset(True)
        return True

    print(f"Sending ISC_PROGRAM_DONE (0x5E), waiting {wait_ms}ms...")
    isc_program_done(sensor)
    time.sleep(wait_ms / 1000.0)

    print("STATUS after ISC_PROGRAM_DONE:")
    _, done_after = read_status(sensor)

    print("ISC_DISABLE...")
    isc_disable(sensor)

    # Re-enter ISC and re-read STATUS to confirm it persisted
    print("Re-entering ISC to verify Done persisted...")
    isc_enable(sensor, 0x02)
    print("STATUS (re-read):")
    _, done_reread = read_status(sensor)
    isc_disable(sensor)

    # Clean exit: toggle CRESETB
    sensor.creset(False)
    time.sleep(0.001)
    sensor.creset(True)

    if done_after and done_reread:
        print(f"SUCCESS: Done fuse burned on attempt {attempt}!")
        return True
    elif done_after and not done_reread:
        print("WARNING: Done=1 immediately after burn but 0 on re-read (volatile only).")
        return False
    else:
        print(f"FAILED: Done still 0 after ISC_PROGRAM_DONE (attempt {attempt}).")
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sensor", choices=["left", "right"], default="left")
    ap.add_argument("--camera", type=int, default=8, help="1-8 (default 8)")
    ap.add_argument("--attempts", type=int, default=3,
                    help="Number of burn attempts (default 3)")
    ap.add_argument("--wait-ms", type=int, default=200,
                    help="Wait after ISC_PROGRAM_DONE in ms (default 200)")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    cam_idx = args.camera - 1
    cam_mask = 1 << cam_idx

    iface = MotionInterface()
    iface.start(wait=True, wait_timeout=_CONNECT_TIMEOUT)

    deadline = time.monotonic() + _CONNECT_TIMEOUT
    sensor = iface.left if args.sensor == "left" else iface.right
    while time.monotonic() < deadline and not sensor.is_connected():
        time.sleep(0.1)

    if not sensor.is_connected():
        print(f"Sensor '{args.sensor}' not connected.")
        iface.stop()
        sys.exit(1)

    print(f"Connected to {args.sensor} sensor.")

    try:
        sensor.set_debug_flags(DEBUG_FLAG_USB_PRINTF | DEBUG_FLAG_CMD_VERBOSE)

        print(f"Powering on camera {args.camera}...")
        sensor.enable_camera_power(cam_mask)
        time.sleep(0.3)

        print(f"Selecting camera {args.camera} (TCA channel {cam_idx})...")
        sensor.switch_camera(cam_idx)
        time.sleep(0.1)

        for attempt in range(1, args.attempts + 1):
            if attempt_done_burn(sensor, attempt, args.wait_ms):
                print("\n+============================+")
                print("| DONE FUSE BURNED - SUCCESS |")
                print("+============================+\n")
                sys.exit(0)

            if attempt < args.attempts:
                print("Retrying after 500ms cooldown...")
                time.sleep(0.5)

        print(f"\nAll {args.attempts} attempts failed. Done fuse did not burn.")
        print("+=======+")
        print("| FAIL! |")
        print("+=======+\n")
        sys.exit(1)

    finally:
        iface.stop()


if __name__ == "__main__":
    main()
