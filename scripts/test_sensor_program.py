#!/usr/bin/env python3
"""Put a MOTION sensor STM32 into DFU mode and flash a .bin file.

Uses the vendored dfu-util via :class:`omotion.DFUProgrammer.DFUProgrammer`.

Usage
-----
    python scripts/test_sensor_program.py <path_to_bin_file> [--sensor left|right]
"""

import argparse
import sys
import time
from pathlib import Path

from omotion import MotionInterface
from omotion.DFUProgrammer import DFUProgrammer, DFUProgress


class _LiveStatus:
    def __init__(self, *, enabled: bool = True):
        self.enabled = enabled
        self._spinner = "|/-\\"
        self._spinner_index = 0
        self._last_render = 0.0
        self._phase = "working"
        self._percent: int | None = None

    def clear(self) -> None:
        if not self.enabled:
            return
        sys.stderr.write("\r" + (" " * 80) + "\r")
        sys.stderr.flush()

    def update(self, p: DFUProgress) -> None:
        if not self.enabled:
            return

        if p.phase == "erase":
            self._phase = "erasing"
        elif p.phase == "download":
            self._phase = "downloading"

        if p.percent is not None:
            self._percent = p.percent

        now = time.monotonic()
        if now - self._last_render < 0.1:
            return

        ch = self._spinner[self._spinner_index % len(self._spinner)]
        self._spinner_index += 1
        pct = f" {self._percent:3d}%" if self._percent is not None else ""
        sys.stderr.write(f"\r   … {self._phase} {ch}{pct}  ({p.elapsed_s:0.1f}s)")
        sys.stderr.flush()
        self._last_render = now


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Put the MOTION STM32 into DFU mode and flash a .bin file."
    )
    parser.add_argument(
        "bin_file",
        type=Path,
        help="Path to bin file that shall be programmed onto the STM32.",
    )
    parser.add_argument(
        "--no-confirm",
        action="store_true",
        help="Skip the interactive confirmation before entering DFU mode.",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=5.0,
        help="Seconds to wait after issuing the DFU command before polling for the USB device.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for DFU device to appear.",
    )
    parser.add_argument(
        "--vidpid",
        default="0483:df11",
        help="VID:PID for the DFU device (default 0483:df11).",
    )
    parser.add_argument(
        "--addr",
        default=DFUProgrammer.DEFAULT_ADDRESS,
        help="Flash start address (default 0x08000000).",
    )
    parser.add_argument(
        "--alt",
        type=int,
        default=0,
        help="DFU alt setting (default 0).",
    )
    parser.add_argument(
        "--dfu-verbose",
        action="count",
        default=0,
        help="Pass -v to dfu-util (repeat for more verbosity).",
    )
    parser.add_argument(
        "--no-spinner",
        action="store_true",
        help="Disable live status line.",
    )
    parser.add_argument(
        "--sensor",
        choices=("left", "right"),
        default=None,
        help="Select which sensor to target (left or right). If omitted, the first present sensor is used.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_cli()

    print("[*] Starting MOTION interface …")
    interface = MotionInterface()
    interface.start()

    try:
        _console_connected, left_connected, right_connected = interface.is_device_connected()

        # Ensure at least one sensor module is present.
        if not (left_connected or right_connected):
            print("❌  No sensor modules connected – cannot continue.")
            return 1

        selected_sensor = None
        # If the user requested a specific side, honor it (fail if not present).
        if args.sensor == "left":
            if not left_connected:
                print("❌  LEFT sensor not connected – cannot continue.")
                return 1
            print("Running firmware update on LEFT sensor")
            selected_sensor = interface.left
        elif args.sensor == "right":
            if not right_connected:
                print("❌  RIGHT sensor not connected – cannot continue.")
                return 1
            print("Running firmware update on RIGHT sensor")
            selected_sensor = interface.right
        else:
            # Auto-select: prefer left if present, otherwise right.
            if left_connected:
                print("Running firmware update on LEFT sensor (auto-selected)")
                selected_sensor = interface.left
            elif right_connected:
                print("Running firmware update on RIGHT sensor (auto-selected)")
                selected_sensor = interface.right

        if selected_sensor is None:
            print("❌  Sensor module not connected – cannot continue.")
            return 1

        dfu = DFUProgrammer(vidpid=args.vidpid)
        status = _LiveStatus(enabled=not args.no_spinner)

        if not args.no_confirm:
            answer = input("Do you really want to put the board into DFU mode? (y/N): ").strip().lower()
            if answer != "y":
                print("Aborted by user.")
                return 0

        print("\n[+] Requesting DFU mode from the Sensor module …")
        try:
            ok = selected_sensor.enter_dfu()
        except Exception as exc:  # pragma: no cover
            print(f"   ❌  Exception while calling enter_dfu(): {exc}")
            ok = False

        if ok:
            print("   ✅  Sensor module reported success.")
        else:
            print("   ❌  Sensor module reported failure.")
            print("❌  Failed to request DFU mode – aborting.")
            return 1

        print(f"\n[*] Sleeping {args.wait:.1f}s to give the bootloader time to re‑enumerate …")
        time.sleep(args.wait)

        print(f"[+] Waiting up to {args.timeout:.0f}s for DFU device …")
        if not dfu.wait_for_dfu_device(timeout_s=args.timeout):
            print("❌  DFU device never appeared – aborting.")
            return 1
        print("   ✅  DFU device detected.")

        def on_line(line: str) -> None:
            # Ensure the status line doesn't collide with printed output.
            status.clear()
            print("   |", line)
            status._last_render = 0.0

        print("\n[+] Flashing with dfu-util …")
        result = dfu.flash_bin(
            args.bin_file,
            address=args.addr,
            alt=args.alt,
            verbose=args.dfu_verbose,
            normalize_dfu_suffix=True,
            progress=status.update,
            line_callback=on_line,
            echo_output=False,
            echo_progress_lines=False,
        )

        status.clear()
        if not result.success:
            print(f"❌  Flash failed (exit code {result.returncode}).")
            # Print any non-progress lines from captured stdout for debugging.
            for ln in (result.stdout or "").splitlines():
                t = ln.strip()
                if (t.startswith("Erase") or t.startswith("Download")) and "%" in t:
                    continue
                print("   |", ln)
            return 1

        print("   ✅  Flash successful.")
        print("   ℹ️  DFU bootloader already left – device should be running now.")
        print("\n🎉  All done! The STM32 should now be running the newly‑flashed firmware.\n")
        return 0
    finally:
        interface.stop()


if __name__ == "__main__":
    sys.exit(main())
