#!/usr/bin/env python3
"""validate_scan_integrity.py - Run a short live scan and verify histogram integrity.

Runs the full scan bring-up (power -> FPGA program -> sensor config ->
stream -> trigger) on one sensor side, parses the live HISTO stream with
the production parser, and reports how many samples passed or failed the
histogram-sum invariant (every valid frame must sum to
EXPECTED_HISTOGRAM_SUM = 1920*1280 + metadata).

A bit-shifted serial link (e.g. stray bits clocked into a USART before
the scan) multiplies every bin by a power of two, so failures are
reported with their got/expected ratio to make that signature obvious.

Usage:
    python scripts/validate_scan_integrity.py --sensor right --camera-mask 0x01
    python scripts/validate_scan_integrity.py --sensor left --camera-mask 0x80 --duration 15
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from collections import Counter, defaultdict

from omotion import MotionInterface
from omotion.MotionProcessing import (
    EXPECTED_HISTOGRAM_SUM,
    HISTOGRAM_BYTES,
    parse_histogram_stream,
)

import queue

_CONNECT_TIMEOUT = 12.0
FRAME_RATE_HZ = 40


class MismatchCounter(logging.Handler):
    """Counts 'Histogram sum mismatch' warnings from the stream parser."""

    def __init__(self):
        super().__init__(level=logging.WARNING)
        self.count = 0
        self.ratios = Counter()

    def emit(self, record):
        msg = record.getMessage()
        if "Histogram sum mismatch" not in msg:
            return
        self.count += 1
        try:
            # args: (cam_id, frame_id, row_sum, expected)
            got, expected = record.args[2], record.args[3]
            self.ratios[round(got / expected, 3)] += 1
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sensor", choices=["left", "right"], required=True)
    ap.add_argument("--camera-mask", type=lambda x: int(x, 0), default=0x01)
    ap.add_argument("--duration", type=float, default=10.0,
                    help="Scan duration in seconds (default 10)")
    ap.add_argument("--skip-bringup", action="store_true",
                    help="Skip power/program/configure (cameras already set up)")
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    counter = MismatchCounter()
    logging.getLogger().addHandler(counter)

    # Hard watchdog: this script must never hang the bench. If anything
    # below blocks (USB teardown, console command, parser join), kill the
    # whole process well after the scan should have finished.
    watchdog_s = args.duration + 60
    def _watchdog():
        print(f"\nWATCHDOG: still running after {watchdog_s:.0f}s — aborting", flush=True)
        os._exit(2)
    wd = threading.Timer(watchdog_s, _watchdog)
    wd.daemon = True
    wd.start()

    iface = MotionInterface()
    iface.start(wait=False)

    deadline = time.monotonic() + _CONNECT_TIMEOUT
    sensor = iface.left if args.sensor == "left" else iface.right
    while time.monotonic() < deadline and not (
            sensor.is_connected() and iface.console.is_connected()):
        time.sleep(0.2)
    if not sensor.is_connected():
        print(f"FAIL: {args.sensor} sensor not connected")
        iface.stop()
        return 1
    if not iface.console.is_connected():
        print("FAIL: console not connected (needed for trigger)")
        iface.stop()
        return 1

    mask = args.camera_mask
    ok = True
    valid_per_cam: dict[int, int] = defaultdict(int)
    parser_thread = None
    streaming_started = False
    stop_evt = threading.Event()
    q: queue.Queue = queue.Queue()

    try:
        if not args.skip_bringup:
            print(f"Powering cameras (mask 0x{mask:02X})...")
            if not sensor.enable_camera_power(mask):
                print("FAIL: enable_camera_power")
                return 1
            time.sleep(1.0)

            print("Programming FPGA(s)...")
            if not sensor.program_fpga(mask, False):
                print("FAIL: program_fpga")
                return 1

            print("Configuring camera sensor registers...")
            if not sensor.camera_configure_registers(mask):
                print("FAIL: camera_configure_registers")
                return 1

        def on_row(cam_id, frame_id, ts, hist, row_sum, temp):
            valid_per_cam[int(cam_id)] += 1

        buf = bytearray()
        sensor.uart.histo.start_streaming(q, expected_size=HISTOGRAM_BYTES)
        streaming_started = True
        parser_thread = threading.Thread(
            target=parse_histogram_stream,
            args=(q, stop_evt, buf),
            kwargs={"on_row_fn": on_row,
                    "expected_row_sum": EXPECTED_HISTOGRAM_SUM},
            daemon=True,
        )
        parser_thread.start()

        # The console loses its trigger config on power cycle; without it
        # start_trigger produces no FSIN and the scan yields zero frames.
        print("Configuring console trigger (40 Hz)...")
        trig = iface.console.set_trigger_json(data={
            "TriggerFrequencyHz": 40,
            "TriggerPulseWidthUsec": 500,
            "LaserPulseDelayUsec": 100,
            "LaserPulseWidthUsec": 500,
            "LaserPulseSkipInterval": 600,
            "EnableSyncOut": True,
            "EnableTaTrigger": True,
        })
        if not trig:
            print("FAIL: set_trigger_json")
            ok = False

        print("Enabling external frame sync on sensor...")
        if ok and not sensor.enable_camera_fsin_ext():
            print("FAIL: enable_camera_fsin_ext")
            ok = False

        print("Enabling camera stream...")
        if not ok:
            pass
        elif not sensor.enable_camera(mask):
            print("FAIL: enable_camera")
            ok = False
        elif not iface.console.start_trigger():
            print("FAIL: start_trigger")
            ok = False
        else:
            print(f"Scanning for {args.duration:.0f}s...")
            time.sleep(args.duration)

            iface.console.stop_trigger()
            sensor.disable_camera(mask)
    finally:
        if streaming_started:
            try:
                # No drain_final here: it can block indefinitely when the
                # scan produced no data. Final partial frames don't matter
                # for an integrity verdict.
                sensor.uart.histo.stop_streaming()
            except Exception:
                pass
        stop_evt.set()
        if parser_thread is not None:
            parser_thread.join(timeout=5)
        iface.stop()

    # ---- report ----
    expected_frames = int(args.duration * FRAME_RATE_HZ)
    n_cams = bin(mask).count("1")
    total_valid = sum(valid_per_cam.values())

    print()
    print("=========== SCAN INTEGRITY RESULT ===========")
    print(f"  duration          : {args.duration:.0f}s  (~{expected_frames} frames/cam expected)")
    print(f"  cameras in mask   : {n_cams}")
    for cam, n in sorted(valid_per_cam.items()):
        print(f"  cam {cam}: valid samples = {n}")
    print(f"  total valid       : {total_valid}")
    print(f"  sum mismatches    : {counter.count}")
    if counter.ratios:
        for ratio, n in counter.ratios.most_common():
            print(f"    ratio got/expected = {ratio} x{n}"
                  + ("  <-- power-of-2: bit-shifted serial link" if ratio in (2.0, 4.0, 8.0, 16.0, 0.5, 0.25) else ""))

    min_valid = int(expected_frames * n_cams * 0.5)
    if counter.count > 0:
        print("  VERDICT: FAIL — corrupted frames detected")
        ok = False
    elif total_valid < min_valid:
        print(f"  VERDICT: FAIL — too few valid samples ({total_valid} < {min_valid})")
        ok = False
    else:
        print("  VERDICT: PASS")
    print("=============================================")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
