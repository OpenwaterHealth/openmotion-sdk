import time

from omotion import MotionInterface

# Run this script with:
# set PYTHONPATH=%cd%;%PYTHONPATH%
# python scripts\test_console_trigger.py


def main():
    print("Starting MOTION Console Module Test Script...")

    interface = MotionInterface()
    interface.start()

    # start() only waits ~2s for devices already mid-CONNECTING. Block
    # explicitly until the console reaches CONNECTED (or give up).
    interface.wait_for_ready(console=True, sensors=0, timeout=10.0)

    console_connected, left_connected, right_connected = (
        interface.is_device_connected()
    )

    if console_connected and left_connected and right_connected:
        print("MOTION System fully connected.")
    else:
        print(
            f"MOTION System NOT Fully Connected. "
            f"CONSOLE: {console_connected}, "
            f"SENSOR (LEFT,RIGHT): {left_connected}, {right_connected}"
        )

    if not console_connected:
        print("Console Module not connected.")
        interface.stop()
        exit(1)

    try:
        # Ping Test
        print("\n[1] Ping Console Module...")
        response = interface.console.ping()
        print("Ping successful." if response else "Ping failed.")

        print("\n[0] Set trigger...")
        json_trigger_data = {
            "TriggerFrequencyHz": 40,
            "TriggerPulseWidthUsec": 500,
            "LaserPulseDelayUsec": 100,
            "LaserPulseWidthUsec": 500,
            "LaserPulseSkipInterval": 0,
            "EnableSyncOut": True,
            "EnableTaTrigger": True,
        }

        new_setting = interface.console.set_trigger_json(data=json_trigger_data)
        if new_setting:
            print(f"Trigger Setting: {new_setting}")
        else:
            print("Failed to get trigger setting.")

        print("\n[1] Get trigger...")
        trigger_setting = interface.console.get_trigger_json()
        if trigger_setting:
            print(f"Trigger Setting: {trigger_setting}")
        else:
            print("Failed to get trigger setting.")

        print("\n[2] Start trigger...")
        if not interface.console.start_trigger():
            print("Failed to start trigger.")
        else:
            print("Press [ENTER] to stop trigger...")
            input()
            interface.console.stop_trigger()

        print("\n[3] Set trigger...")
        json_trigger_data = {
            "TriggerFrequencyHz": 25,
            "TriggerPulseWidthUsec": 500,
            "LaserPulseDelayUsec": 100,
            "LaserPulseWidthUsec": 500,
            "LaserPulseSkipInterval": 5,
            "EnableSyncOut": True,
            "EnableTaTrigger": True,
        }

        new_setting = interface.console.set_trigger_json(data=json_trigger_data)
        if new_setting:
            print(f"Trigger Setting: {new_setting}")
        else:
            print("Failed to get trigger setting.")

        print("\n[4] Start trigger...")
        if not interface.console.start_trigger():
            print("Failed to start trigger.")
        else:
            trigger_setting = interface.console.get_trigger_json()
            if trigger_setting:
                print(f"Trigger Setting: {trigger_setting}")
            else:
                print("Failed to get trigger setting.")

            time.sleep(1)
            fsync_pulsecount = interface.console.get_fsync_pulsecount()
            print(f"FSYNC PulseCount: {fsync_pulsecount}")

            print("Press [ENTER] to stop trigger...")
            input()
            interface.console.stop_trigger()
    finally:
        interface.stop()


if __name__ == "__main__":
    main()
