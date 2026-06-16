#!/usr/bin/env python3
"""Interactive TEC bench console for the MOTION console module.

Connects to the console, prints board ID / temperatures / PDU monitor
readings, then drops into a small REPL for TEC control.

Usage
-----
    python scripts/tec_console.py
"""

import sys
import time

from omotion import MotionInterface

HELP = """\
Commands:
  get                 - Read current TEC setpoint (volts)
  set <volts>         - Set TEC setpoint to <volts> and read back
  read <channel>      - Read TEC ADC voltage on specified channel (0-3 or 4 for all)
  status              - Get TEC status information
  help                - Show this help
  quit / exit         - Leave the console
"""


def main() -> int:
    print("Starting MOTION TEC Console…")

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

        if not console_connected:
            print("Console Module not connected.")
            return 1

        console = interface.console

        # Quick ping
        print("\nPinging Console Module…")
        ok = False
        try:
            ok = bool(console.ping())
            print(f"SDK Version: {MotionInterface.get_sdk_version()}")
        except Exception as e:
            print(f"Ping failed: {e}")
        print("Ping successful." if ok else "Ping failed (continuing).")

        print(f"BoardID: {console.read_board_id()}")
        print("\nType 'help' for commands.\n")

        # Initial temperature readout
        try:
            mcu, safety, ta = console.get_temperatures()
            print(f"Temps → MCU: {mcu:.2f} °C | Safety: {safety:.2f} °C | TA: {ta:.2f} °C")
        except Exception as e:
            print(f"Temperature read failed: {e}")

        # PDU MON readout
        try:
            pdu = console.read_pdu_mon()
            if pdu is None:
                print("PDU MON: no data")
            else:
                # First 8 channels belong to ADC0, next 8 to ADC1
                print("ADC0 (ch 0-7)")
                print(f"{'Ch':>2}  {'Raw':>6}  {'Value':>10}")
                for i in range(8):
                    print(f"{i:>2}  {pdu.raws[i]:>6}  {pdu.volts[i]:>10.3f}")

                print("\nADC1 (ch 0-7)")
                print(f"{'Ch':>2}  {'Raw':>6}  {'Value':>10}")
                for i in range(8, 16):
                    ch = i - 8
                    print(f"{ch:>2}  {pdu.raws[i]:>6}  {pdu.volts[i]:>10.3f}")

        except Exception as e:
            print(f"PDU MON read failed: {e}")

        while True:
            try:
                line = input("tec> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nBye.")
                break

            if not line:
                continue

            cmd, *args = line.split()

            if cmd.lower() in ("quit", "exit"):
                print("Bye.")
                break

            if cmd.lower() == "help":
                print(HELP)
                continue

            if cmd.lower() == "get":
                try:
                    volts = console.tec_voltage()  # GET (no arg)
                    print(f"Current TEC setpoint: {volts:.6f} V")
                except Exception as e:
                    print(f"GET failed: {e}")
                continue

            if cmd.lower() == "set":
                if not args:
                    print("Usage: set <volts>")
                    continue
                try:
                    target = float(args[0])
                except ValueError:
                    print("Invalid number; try e.g. 'set 1.25'")
                    continue

                try:
                    console.tec_voltage(target)  # SET

                    time.sleep(0.02)
                    readback = console.tec_voltage()  # GET
                    print(f"Setpoint requested: {target:.6f} V; readback: {readback:.6f} V")
                except Exception as e:
                    print(f"SET failed: {e}")
                continue

            if cmd.lower() == "read":
                if not args:
                    print("Usage: read <channel>")
                    continue
                try:
                    channel = int(args[0])
                except ValueError:
                    print("Invalid number; try e.g. 'read 1'")
                    continue

                try:
                    ch_volts = console.tec_adc(channel)  # read channel

                    if channel == 4:
                        formatted = ", ".join(f"{v:.6f} V" for v in ch_volts)
                        print(f"CHANNELS 0-3: {formatted}")
                    else:
                        print(f"CHANNEL {channel}: {ch_volts:.6f} V")
                except Exception as e:
                    print(f"TEC ADC read failed: {e}")
                continue

            if cmd.lower() == "status":
                try:
                    status_arr = console.tec_status()
                    print(f"TEC STATUS {status_arr}")
                except Exception as e:
                    print(f"TEC status read failed: {e}")
                continue

            print("Unknown command. Type 'help' for a list of commands.")

        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
