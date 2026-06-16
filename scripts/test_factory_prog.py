#!/usr/bin/env python3
"""test_factory_prog.py - Burn a CrossLink NVCM image via the sensor board.

Thin CLI over omotion.NvcmProgrammer. With no file arguments, burns the
image bundled with the SDK (omotion/nvcm/).

Usage:
    python test_factory_prog.py [ALGO.IEA DATA.IED] [--sensor left|right] [--cam N]
"""

import argparse
import logging
import sys
import time

from omotion import MotionInterface
from omotion.NvcmProgrammer import (
    NvcmProgrammer, DEFAULT_ALGO_PATH, DEFAULT_DATA_PATH,
)

logger = logging.getLogger(__name__)
_CONNECT_TIMEOUT = 12.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("algo", nargs="?", default=str(DEFAULT_ALGO_PATH),
                        metavar="ALGO.IEA", help="Lattice algorithm file")
    parser.add_argument("data", nargs="?", default=str(DEFAULT_DATA_PATH),
                        metavar="DATA.IED", help="Lattice data file")
    parser.add_argument("--sensor", default="left", choices=("left", "right"))
    parser.add_argument("--cam", default=1, type=int, help="camera 1-8")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s")

    if not (1 <= args.cam <= 8):
        print(f"Error: --cam must be 1-8, got {args.cam}", file=sys.stderr)
        return 1

    print("Connecting to Motion Sensor...")
    iface = MotionInterface()
    iface.start(wait=True, wait_timeout=_CONNECT_TIMEOUT)
    sensor = iface.left if args.sensor == "left" else iface.right
    deadline = time.monotonic() + _CONNECT_TIMEOUT
    while time.monotonic() < deadline and not sensor.is_connected():
        time.sleep(0.1)
    if not sensor.is_connected():
        print(f"Requested sensor '{args.sensor}' is not connected.")
        iface.stop()
        return 1

    print(f"Connected.  Sensor: {args.sensor}  Camera: {args.cam}")
    print(f"  algo: {args.algo}\n  data: {args.data}\n")

    last = [-1]

    def progress(done, total):
        pct = done * 100 // total
        if pct != last[0]:
            last[0] = pct
            print(f"\r  {pct:3d}%  ({done:,}/{total:,})", end="", flush=True)

    try:
        result = NvcmProgrammer(sensor).burn(
            args.cam, algo_path=args.algo, data_path=args.data,
            progress_cb=progress)
    finally:
        iface.stop()
    print()

    if not result.success:
        print(f"\nProgramming failed: {result.error}", file=sys.stderr)
        print("+=======+\n| FAIL! |\n+=======+\n")
        return 1
    print("\n+=========+\n| PASSED! |\n+=========+\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
