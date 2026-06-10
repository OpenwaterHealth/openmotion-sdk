#!/usr/bin/env python3
"""
nvcm_probe.py - Detect whether a CrossLink FPGA's NVCM has been programmed.

Talks to ONE camera (default: camera 8 on the left sensor) and reads every NVCM
discriminator directly over I2C via the firmware OW_FACTORY_NVCM_CHECK command.
This never boots the FPGA and never toggles camera power beyond a single standard
power-on, so it can't upset the TCA9548A mux.

Flow:
  connect left -> debug flags (USB printf + cmd verbose) -> power on camera
  -> switch_camera (routes TCA mux) -> nvcm_check -> parse + interpret.

Usage:
  python scripts/nvcm_probe.py                 # camera 8, NVCM mode, 1 row
  python scripts/nvcm_probe.py --rows 4        # read 4 NVCM array rows
  python scripts/nvcm_probe.py --operand 0x00  # try SRAM access mode instead
  python scripts/nvcm_probe.py --no-power      # don't power-cycle, assume powered
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

from omotion import MotionInterface  # noqa: E402
from omotion.config import (  # noqa: E402
    DEBUG_FLAG_USB_PRINTF,
    DEBUG_FLAG_CMD_VERBOSE,
)

_CONNECT_TIMEOUT = 12.0
EXPECTED_IDCODE = bytes([0x01, 0x2C, 0x00, 0x43])

_STEP_NAMES = [
    (1 << 0, "ACTIVATION"),
    (1 << 1, "IDCODE"),
    (1 << 2, "ISC_ENABLE"),
    (1 << 3, "STATUS"),
    (1 << 4, "FEATROW"),
    (1 << 5, "FEABITS"),
    (1 << 6, "USERCODE"),
]


def _hex(b):
    return " ".join(f"{x:02X}" for x in b)


def parse_blob(blob: bytes) -> dict:
    """Parse the fixed-layout OW_FACTORY_NVCM_CHECK response."""
    if len(blob) < 27:
        raise ValueError(f"response too short: {len(blob)} bytes ({_hex(blob)})")
    d = {}
    d["idcode"] = blob[0:4]
    d["idcode_ok"] = blob[4]
    d["step_status"] = blob[5]
    d["status"] = blob[6:10]
    d["feature_row"] = blob[10:18]
    d["feabits"] = blob[18:20]
    d["usercode"] = blob[20:24]
    d["boot_probe_done"] = blob[24]
    d["boot_0x40_responds"] = blob[25]
    d["num_rows_read"] = blob[26]
    rows = []
    off = 27
    for _ in range(d["num_rows_read"]):
        if off + 16 <= len(blob):
            rows.append(blob[off:off + 16])
            off += 16
    d["nvcm_rows"] = rows
    return d


def interpret(d: dict) -> None:
    print("\n================ NVCM PROBE RESULT ================")
    steps = [name for bit, name in _STEP_NAMES if d["step_status"] & bit]
    print(f"  step_status : 0x{d['step_status']:02X}  ({', '.join(steps) or 'none'})")
    print(f"  IDCODE      : {_hex(d['idcode'])}  ok={d['idcode_ok']}"
          f"  (expected {_hex(EXPECTED_IDCODE)})")
    print(f"  STATUS      : {_hex(d['status'])}")
    # Decode status both ways; spec lists STATUS[31..0], byte 0 = MSB.
    s_msb = int.from_bytes(d["status"], "big")
    print(f"                msb-first=0x{s_msb:08X}  "
          f"Done(bit8)={(s_msb >> 8) & 1}  OTP(bit6)={(s_msb >> 6) & 1}  "
          f"Busy(bit12)={(s_msb >> 12) & 1}  Fail(bit13)={(s_msb >> 13) & 1}")
    print(f"  FEATURE_ROW : {_hex(d['feature_row'])}")
    print(f"  FEABITS     : {_hex(d['feabits'])}")
    print(f"  USERCODE    : {_hex(d['usercode'])}")
    boot_done = d["boot_probe_done"]
    boot_ack = d["boot_0x40_responds"]
    print(f"  BOOT TEST   : done={boot_done}  0x40_responds={boot_ack}"
          f"  ({'ran' if boot_done else 'skipped'})")
    print(f"  NVCM rows   : {d['num_rows_read']} read")
    for i, row in enumerate(d["nvcm_rows"]):
        print(f"    row{i}: {_hex(row)}")

    # ---- signals -------------------------------------------------------
    # NOTE: the content reads (feature_row / NVCM array) come back as floating
    # 0xFF on this part because a bare ISC_ENABLE 0x08 doesn't read-enable the
    # NVCM array — they are NOT trustworthy yet.  The auto-boot test is the
    # primary, behaviorally-definitive signal.
    featrow_real = any(d["feature_row"]) and not all(b == 0xFF for b in d["feature_row"])
    usercode_nz = any(d["usercode"])
    nvcm_real = any(any(b for b in r if b != 0xFF) for r in d["nvcm_rows"])
    done_bit = bool((s_msb >> 8) & 1)

    print("\n  --- signals ---")
    print(f"    [primary] boot test ran   : {bool(boot_done)}")
    print(f"    [primary] 0x40 after boot : "
          f"{'ACKs (blank)' if boot_ack else 'gone (programmed)'}")
    print(f"    feature_row real (non-FF) : {featrow_real}")
    print(f"    usercode    != 0          : {usercode_nz}")
    print(f"    nvcm row real (non-FF)    : {nvcm_real}")
    print(f"    status Done bit           : {done_bit}")

    print()
    if d["idcode_ok"] != 1:
        print("  VERDICT: INCONCLUSIVE — IDCODE mismatch; config port not "
              "answering in forced config mode. Check power / mux / CRESETB.")
    elif boot_done:
        # Primary, behaviorally-definitive signal.
        if boot_ack == 0:
            print("  VERDICT: *** NVCM PROGRAMMED *** -- config port reachable when "
                  "forced (IDCODE ok), but 0x40 DISAPPEARS after auto-boot, i.e. "
                  "the FPGA booted a user design from NVCM.")
        else:
            print("  VERDICT: BLANK -- 0x40 still ACKs after releasing CRESETB "
                  "without the activation key, i.e. nothing auto-booted.")
        if featrow_real or nvcm_real or usercode_nz or done_bit:
            print("  (corroborated by a non-blank content read)")
    else:
        print("  VERDICT: boot test was skipped; content reads on this part are "
              "untrustworthy (floating 0xFF). Re-run without --no-boot-test.")
    print("==================================================\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sensor", choices=["left", "right"], default="left")
    ap.add_argument("--camera", type=int, default=8, help="1-8 (default 8)")
    ap.add_argument("--operand", type=lambda x: int(x, 0), default=0x08,
                    help="ISC_ENABLE operand1: 0x08=NVCM (default), 0x00=SRAM")
    ap.add_argument("--rows", type=int, default=1,
                    help="NVCM array rows to read (0-8, default 1)")
    ap.add_argument("--no-power", action="store_true",
                    help="skip power-on (assume the camera is already powered)")
    ap.add_argument("--no-boot-test", action="store_true",
                    help="skip the auto-boot 0x40-disappearance test")
    ap.add_argument("--verbose", action="store_true",
                    help="show all SDK debug logging")
    args = ap.parse_args()

    # Surface firmware [PRINTF] lines (logged at WARNING by CommInterface).
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    if not (1 <= args.camera <= 8):
        print("camera must be 1-8")
        sys.exit(2)
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

    print(f"Connected to {args.sensor} sensor. Probing camera {args.camera} "
          f"(idx {cam_idx}, mask 0x{cam_mask:02X}).")
    print(f"ISC_ENABLE operand=0x{args.operand:02X}  rows={args.rows}\n")

    try:
        # Enable firmware printf + verbose command logging over USB.
        sensor.set_debug_flags(DEBUG_FLAG_USB_PRINTF | DEBUG_FLAG_CMD_VERBOSE)

        if not args.no_power:
            print(f"Powering on camera {args.camera}...")
            ok = sensor.enable_camera_power(cam_mask)
            print(f"  power-on -> {ok}")
            time.sleep(0.3)

        print(f"Selecting camera {args.camera} (routes TCA mux channel {cam_idx})...")
        sensor.switch_camera(cam_idx)
        time.sleep(0.1)

        print("Running NVCM probe...\n")
        blob = sensor.nvcm_check(isc_operand=args.operand, num_rows=args.rows,
                                 boot_test=not args.no_boot_test)
        if not blob:
            print("nvcm_check returned no data (firmware error). "
                  "Re-run with --verbose to see firmware printf.")
            sys.exit(3)

        print(f"raw response ({len(blob)} bytes): {_hex(blob)}")
        d = parse_blob(blob)
        interpret(d)
    finally:
        iface.stop()


if __name__ == "__main__":
    main()
