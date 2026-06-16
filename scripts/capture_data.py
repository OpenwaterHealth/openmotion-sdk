#!/usr/bin/env python3
"""Capture raw MOTION histogram stream data to .raw files.

Enables the cameras, starts the console trigger, and streams each
connected sensor's histogram endpoint to a per-side .raw file.

Usage
-----
    python scripts/capture_data.py --camera-mask 0x01 --subject-id Test --duration 15
"""

import argparse
import os
import queue
import sys
import threading
import time
from datetime import datetime

from omotion import MotionInterface

MAX_DURATION = 120  # seconds


def parse_args():
    def parse_mask(x):
        try:
            return int(x, 0)  # allows hex like 0x11 or decimal like 17
        except ValueError:
            raise argparse.ArgumentTypeError(f"Invalid camera mask value: {x}")

    parser = argparse.ArgumentParser(description="Capture MOTION camera data")

    parser.add_argument(
        "--camera-mask",
        type=parse_mask,
        default=0xFF,
        help="Bitmask for cameras (e.g., 0x11 for cameras 0 and 4, default 0xFF)"
    )
    parser.add_argument(
        "--duration",
        type=int,
        required=True,
        help=f"Duration in seconds (max {MAX_DURATION})"
    )
    parser.add_argument(
        "--subject-id",
        type=str,
        required=True,
        help="Subject or patient identifier"
    )
    parser.add_argument(
        "--disable-laser",
        action="store_true",
        help="If set, disables external frame sync (laser). Otherwise, enables external frame sync by default."
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default="scan_data",
        help="Directory to save scan files (default: scan_data)"
    )
    return parser.parse_args()


def write_stream_to_file(queue_obj, stop_event, filename):
    with open(filename, "wb") as f:
        while not stop_event.is_set() or not queue_obj.empty():
            try:
                data = queue_obj.get(timeout=0.100)
                if data:
                    f.write(data)
                queue_obj.task_done()
            except queue.Empty:
                continue


def main() -> int:
    args = parse_args()

    if args.duration > MAX_DURATION:
        print(f"Error: Duration cannot exceed {MAX_DURATION} seconds.")
        return 1

    print("Starting MOTION Capture Data Script...")
    print(f"Camera Mask: 0x{args.camera_mask:02X}")
    print(f"Duration: {args.duration} seconds")

    data_dir = args.data_dir
    os.makedirs(data_dir, exist_ok=True)

    interface = MotionInterface()
    interface.start()

    try:
        console_connected, left_connected, right_connected = interface.is_device_connected()
        if console_connected and left_connected and right_connected:
            print("MOTION System fully connected.")
            target = "all"
        elif console_connected and (left_connected or right_connected):
            target = "left" if left_connected else "right"
        else:
            print(
                f"MOTION System NOT Fully Connected. CONSOLE: {console_connected}, "
                f"SENSOR (LEFT,RIGHT): {left_connected}, {right_connected}"
            )
            return 1

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        stream_threads = []
        stop_events = []

        if not args.disable_laser:
            # enable external frame sync
            print("\nEnabling external frame sync...")
            results = interface.run_on_sensors("enable_camera_fsin_ext", target=target)
            for side, success in results.items():
                if not success:
                    print(f"Failed to enable external frame sync on {side}.")
                    return 1
        else:
            print("\nLaser sync disabled. Using internal sensor-generated frame sync.")

        # Enable cameras
        print("\nEnabling cameras...")
        results = interface.run_on_sensors("enable_camera", args.camera_mask, target=target)
        for side, success in results.items():
            if not success:
                print(f"Failed to enable camera on {side}.")
                return 1

        # Setup streaming
        sensors = {"left": interface.left, "right": interface.right}
        for side, sensor in sensors.items():
            if sensor.is_connected() and sensor.uart is not None:
                q = queue.Queue()
                stop_evt = threading.Event()
                sensor.uart.histo.start_streaming(q, expected_size=32833)  # adjust size if needed probably should be exact based on mask

                filename = f"scan_{args.subject_id}_{timestamp}_{side}_mask{args.camera_mask:02X}.raw"
                filepath = os.path.join(data_dir, filename)

                t = threading.Thread(target=write_stream_to_file, args=(q, stop_evt, filepath), daemon=True)
                t.start()
                stream_threads.append((t, stop_evt))
                stop_events.append(stop_evt)
                print(f"[{side.upper()}] Streaming to: {filename}")

        # Activate Laser
        print("\nStart trigger...")
        if not interface.console.start_trigger():
            print("Failed to start trigger.")

        # Start capture loop
        start_time = time.time()
        elapsed = 0

        try:
            while elapsed < args.duration:
                elapsed = time.time() - start_time
                time.sleep(1)

        except KeyboardInterrupt:
            print("\n🛑 Capture interrupted by user.")

        print("\nStop trigger...")
        if not interface.console.stop_trigger():
            print("Failed to stop trigger.")

        results = interface.run_on_sensors("disable_camera", args.camera_mask, target=target)
        for side, success in results.items():
            if not success:
                print(f"Failed to disable camera on {side}.")

        # Stop streaming threads
        for side, sensor in sensors.items():
            if sensor.uart is not None:
                sensor.uart.histo.stop_streaming()

        # Signal all threads to stop
        for evt in stop_events:
            evt.set()

        # Wait for all threads to finish
        for t, _ in stream_threads:
            t.join()

        print("Capture session complete.")
        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
