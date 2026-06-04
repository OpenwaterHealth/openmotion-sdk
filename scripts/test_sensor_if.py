import time
import argparse
from omotion import MotionInterface

# Run this script with:
# python scripts\test_sensor_if.py --camera-mask 0x01

print("Starting MOTION Sensor Module Test Script...")
BIT_FILE = "bitstream/HistoFPGAFw_impl1_agg.bit"
#BIT_FILE = "bitstream/testcustom_agg.bit"
CAMERA_MASK = 0xFF

_CONNECT_TIMEOUT = 12.0


def parse_args():
    parser = argparse.ArgumentParser(description="MOTION Sensor FPGA Test")
    parser.add_argument(
        "--camera-mask",
        type=lambda x: int(x, 0),  # allows 0xFF or decimal
        default=0xFF,
        help="Bitmask for cameras (default 0xFF)"
    )
    parser.add_argument(
        "--auto-upload",
        action="store_true",
        default=True,
        help="Enable auto-upload of FPGA bitstream (default True)"
    )
    parser.add_argument(
        "--manual-upload",
        action="store_true",
        help="Force manual upload (overrides --auto-upload)"
    )
    parser.add_argument(
        "--bit-file",
        type=str,
        help="Path to FPGA bitstream file (required if manual upload)"
    )
    return parser.parse_args()


def _await_connected(handle, label):
    """Poll until handle.is_connected() or timeout; return True if connected."""
    deadline = time.monotonic() + _CONNECT_TIMEOUT
    while time.monotonic() < deadline:
        if handle.is_connected():
            return True
        time.sleep(0.1)
    print(f"  {label} not connected after {_CONNECT_TIMEOUT:.0f}s")
    return False


def _connected_sensors(iface):
    """Return a dict of {side_label: sensor} for each connected sensor."""
    sensors = {}
    if iface.left.is_connected():
        sensors["left"] = iface.left
    if iface.right.is_connected():
        sensors["right"] = iface.right
    return sensors


def program_all_sensors(sensors, camera_position, bit_file):
    for side, sensor in sensors.items():
        print(f"  [{side}] reset_camera_sensor")
        if not sensor.reset_camera_sensor(camera_position):
            print(f"  Failed to reset camera sensor. ({side})")
            return False

        print(f"  [{side}] activate_camera_fpga")
        if not sensor.activate_camera_fpga(camera_position):
            print(f"  Failed to activate camera FPGA. ({side})")
            return False

        print(f"  [{side}] check_camera_fpga")
        if not sensor.check_camera_fpga(camera_position):
            print(f"  Failed to check ID of camera FPGA. ({side})")
            return False

        print(f"  [{side}] enter_sram_prog_fpga")
        if not sensor.enter_sram_prog_fpga(camera_position):
            print(f"  Failed to enter SRAM programming mode for camera FPGA. ({side})")
            return False

        print(f"  [{side}] erase_sram_fpga")
        if not sensor.erase_sram_fpga(camera_position):
            print(f"  Failed to erase SRAM for camera FPGA. ({side})")
            return False

        print(f"  [{side}] send_bitstream_fpga")
        if not sensor.send_bitstream_fpga(filename=bit_file):
            print(f"  Failed to send bitstream to camera FPGA. ({side})")
            return False

        print(f"  [{side}] get_status_fpga (post-bitstream)")
        if not sensor.get_status_fpga(camera_position):
            print(f"  Failed to get status for camera FPGA. ({side})")
            return False

        print(f"  [{side}] program_fpga (manual)")
        if not sensor.program_fpga(camera_position=camera_position, manual_process=True):
            print(f"  Failed to program FPGA. ({side})")
            return False

        print(f"  [{side}] get_usercode_fpga")
        if not sensor.get_usercode_fpga(camera_position):
            print(f"  Failed to get usercode for camera FPGA. ({side})")
            return False

        print(f"  [{side}] get_status_fpga (final)")
        if not sensor.get_status_fpga(camera_position):
            print(f"  Failed to get final status for camera FPGA. ({side})")
            return False

    print("✅ Camera FPGA programming complete for all connected sensors.")
    return True


def upload_camera_bitstream(sensors, auto_upload: bool, camera_position: int, bit_file: str) -> bool:
    print("FPGA Configuration Started")

    if auto_upload:
        print("Programming camera FPGA")
        for side, sensor in sensors.items():
            if not sensor.program_fpga(camera_position=camera_position, manual_process=False):
                print(f"❌ Failed to program FPGA on {side} sensor.")
                return False
    else:
        if not program_all_sensors(sensors, camera_position, bit_file):
            return False

    return True


def run_sensor_tests(sensors, camera_mask, auto_upload, bit_file) -> bool:
    # Ping Test
    print("\n[1] Ping Sensor Module...")
    for side, sensor in sensors.items():
        result = sensor.ping()
        print(f"  {side}: {result}")

    # Get Firmware Version
    print("\n[2] Reading Firmware Version...")
    for side, sensor in sensors.items():
        result = sensor.get_version()
        print(f"  {side}: {result}")

    # Get HWID
    print("\n[5] Read Hardware ID...")
    for side, sensor in sensors.items():
        result = sensor.get_hardware_id()
        print(f"  {side}: {result}")

    # turn camera mask into camera positions
    camera_positions = [i for i in range(8) if camera_mask & (1 << i)]

    for pos in camera_positions:
        print(f"\nProgramming camera FPGA at position {pos + 1}...")
        cam_mask_single = 1 << pos
        start_time = time.time()
        if not upload_camera_bitstream(sensors, auto_upload, cam_mask_single, bit_file):
            print("Failed to upload camera bitstream.")
            exit(1)
        print(f"FPGAs programmed | Time: {(time.time() - start_time)*1000:.2f} ms")

        print("Programming camera sensor registers.")
        for side, sensor in sensors.items():
            result = sensor.camera_configure_registers(cam_mask_single)
            print(f"  {side}: {result}")

        # print ("Programming camera sensor set test pattern.")
        # sensor.camera_configure_test_pattern(CAMERA_MASK)

        # print("Capture histogram frame.")
        # sensor.camera_capture_histogram(CAMERA_MASK)

    return True


def main():
    args = parse_args()

    if not args.auto_upload and not args.bit_file:
        print("❌ --bit-file is required when manual upload is selected.")
        exit(1)

    iface = MotionInterface()
    iface.start(wait=True, wait_timeout=_CONNECT_TIMEOUT)

    console_connected = _await_connected(iface.console, "Console module")
    left_connected = _await_connected(iface.left, "Left sensor")
    right_connected = _await_connected(iface.right, "Right sensor")

    if console_connected and left_connected and right_connected:
        print("MOTION System fully connected.")
    else:
        print(f"MOTION System NOT Fully Connected. CONSOLE: {console_connected}, SENSOR (LEFT,RIGHT): {left_connected}, {right_connected}")

    sensors = _connected_sensors(iface)
    if not sensors:
        print("Sensor Module not connected.")
        iface.stop()
        exit(1)

    try:
        run_sensor_tests(sensors, args.camera_mask, not args.manual_upload, args.bit_file)
    finally:
        iface.stop()

    print("\nSensor Module Test Completed.")


if __name__ == "__main__":
    main()
