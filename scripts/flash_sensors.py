#!/usr/bin/env python3
"""Program the camera FPGAs on connected MOTION sensors.

Auto mode uses the bitstream embedded in the sensor firmware
(``program_fpga(manual_process=False)``); manual mode pushes a local
bitstream file through the SRAM programming sequence step by step.

Usage
-----
    python scripts/flash_sensors.py --camera-mask 0x01 --target left
    python scripts/flash_sensors.py --manual-upload --bit-file bitstream/HistoFPGAFw_impl1_agg.bit
"""

import argparse
import sys
import time

from omotion import MotionInterface


def parse_args():
    def parse_mask(x):
        try:
            return int(x, 0)  # allows hex like 0x11 or decimal like 17
        except ValueError:
            raise argparse.ArgumentTypeError(f"Invalid camera mask value: {x}")

    parser = argparse.ArgumentParser(description="MOTION Sensor FPGA programming")

    parser.add_argument(
        "--camera-mask",
        type=parse_mask,
        default=0xFF,
        help="Bitmask for cameras (e.g., 0x11 for cameras 0 and 4, default 0xFF)"
    )
    parser.add_argument(
        "--manual-upload",
        action="store_true",
        help="Push a local bitstream file step by step instead of the firmware-embedded auto upload"
    )
    parser.add_argument(
        "--bit-file",
        type=str,
        help="Path to FPGA bitstream file (required if manual upload)"
    )
    parser.add_argument(
        "--target",
        type=str,
        choices=["left", "right", "all"],
        default="left",
        help="Which side to flash (default: left)"
    )
    return parser.parse_args()


def program_sensor_bitstream(interface, camera_position, bit_file, target: str):
    steps = [
        ("reset_camera_sensor", "Failed to reset camera sensor."),
        ("activate_camera_fpga", "Failed to activate camera FPGA."),
        ("check_camera_fpga", "Failed to check ID of camera FPGA."),
        ("enter_sram_prog_fpga", "Failed to enter SRAM programming mode for camera FPGA."),
        ("erase_sram_fpga", "Failed to erase SRAM for camera FPGA."),
    ]

    # Run the initial steps
    for method, error_msg in steps:
        results = interface.run_on_sensors(method, camera_position, target=target)
        for side, success in results.items():
            if not success:
                print(f"{error_msg} ({side})")
                return False

    # Send bitstream
    print("Sending bitstream to camera FPGA")
    results = interface.run_on_sensors("send_bitstream_fpga", target=target, filename=bit_file)
    for side, success in results.items():
        if not success:
            print(f"Failed to send bitstream to camera FPGA ({side})")
            return False

    # Status after bitstream
    results = interface.run_on_sensors("get_status_fpga", camera_position, target=target)
    for side, success in results.items():
        if not success:
            print(f"Failed to get status for camera FPGA ({side})")
            return False

    # Program FPGA
    results = interface.run_on_sensors("program_fpga", camera_position=camera_position, manual_process=True, target=target)
    for side, success in results.items():
        if not success:
            print(f"Failed to program FPGA ({side})")
            return False

    # Get usercode
    results = interface.run_on_sensors("get_usercode_fpga", camera_position, target=target)
    for side, success in results.items():
        if not success:
            print(f"Failed to get usercode for camera FPGA ({side})")
            return False

    # Final status
    results = interface.run_on_sensors("get_status_fpga", camera_position, target=target)
    for side, success in results.items():
        if not success:
            print(f"Failed to get status for camera FPGA ({side})")
            return False

    print("✅ Camera FPGA programming complete for all connected sensors.")
    return True


def upload_camera_bitstream(interface, auto_upload: bool, camera_position: int, target: str, bit_file: str) -> bool:
    print("FPGA Configuration Started")

    if auto_upload:
        print("Programming camera FPGA")
        results = interface.run_on_sensors(
            "program_fpga",
            camera_position=camera_position,
            target=target,
            manual_process=False
        )

        for side, success in results.items():
            if not success:
                print(f"❌ Failed to program FPGA on {side} sensor.")
                return False
    else:
        # Manual upload process
        if not program_sensor_bitstream(interface, camera_position, bit_file, target=target):
            return False

    return True


def configure_camera_sensors(interface, camera_mask, auto_upload: bool, target: str, bit_file: str) -> bool:
    # turn camera mask into camera positions
    camera_positions = [i for i in range(8) if camera_mask & (1 << i)]

    print(f"Using filtered camera mask: 0x{camera_mask:02X} (sensors {camera_positions})")

    for pos in camera_positions:
        print(f"\nProgramming camera FPGA at position {pos + 1}...")
        cam_mask_single = 1 << pos
        start_time = time.time()
        if not upload_camera_bitstream(interface, auto_upload, cam_mask_single, target, bit_file):
            print("Failed to upload camera bitstream.")
            return False
        print(f"FPGAs programmed | Time: {(time.time() - start_time)*1000:.2f} ms")

        print("Programming camera sensor registers.")
        results = interface.run_on_sensors("camera_configure_registers", cam_mask_single, target=target)
        for side, success in results.items():
            if success is False:
                print(f"❌ Failed to program Camera sensor on {side} sensor.")

    return True


def main() -> int:
    args = parse_args()

    if args.manual_upload and not args.bit_file:
        print("❌ --bit-file is required when manual upload is selected.")
        return 1

    print("Starting MOTION Sensor FPGA programming...")

    interface = MotionInterface()
    interface.start()

    try:
        console_connected, left_connected, right_connected = interface.is_device_connected()

        if console_connected and left_connected and right_connected:
            print("MOTION System fully connected.")
        else:
            print(
                f"MOTION System NOT Fully Connected. CONSOLE: {console_connected}, "
                f"SENSOR (LEFT,RIGHT): {left_connected}, {right_connected}"
            )

        if args.target == "left" and not left_connected:
            print("Left sensor module not connected.")
            return 1
        if args.target == "right" and not right_connected:
            print("Right sensor module not connected.")
            return 1
        if args.target == "all" and not (left_connected and right_connected):
            print("Both sensor modules not connected.")
            return 1

        if not configure_camera_sensors(
            interface, args.camera_mask, not args.manual_upload, args.target, args.bit_file
        ):
            return 1

        print("\nSensor FPGA programming completed.")
        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
