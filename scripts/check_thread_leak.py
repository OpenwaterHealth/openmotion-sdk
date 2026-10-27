#!/usr/bin/env python
"""Bench hand-test for the transport thread-leak fix (PR #243).

Cycles sensor release/reacquire against real hardware and reports the
process thread count each cycle. Every transport thread is named
(``{SIDE}-COMM-read``, ``{SIDE}-COMM-resp``, ``{SIDE}-HISTO-stream``, ...),
so a leak is attributable at a glance.

Usage (close any app first -- USB access is exclusive)::

    python scripts/check_thread_leak.py --cycles 10 --sensors 2

Each cycle calls ``request_disconnect()`` on the connected sensors and
waits for the ConnectionMonitor to reconnect them (the 200 ms poll sweep
sees the device still attached) -- exactly the app's procedure
release/reacquire cycle. PASS means the steady-state thread count never
grew above the first-connect baseline.

To also exercise the historical leak path (connect attempts failing in
the post-enumeration "resource busy" window -- the source of the ~52
leaked ``_process_responses`` threads in the 2026-08-17 fault dump):
start this script while another app still holds the sensors, watch the
"waiting" reports for a while, then close that app. Pre-fix SDKs grow by
up to 5 threads per retry burst during the busy period; the fixed SDK
stays flat.

Exit codes: 0 = PASS, 1 = leak detected, 2 = hardware/setup problem.
"""

import argparse
import logging
import sys
import threading
import time

from omotion import MotionInterface
from omotion.connection_state import ConnectionState

_TRANSPORT_SUFFIXES = ("-read", "-resp", "-stream")


def _transport_thread_names() -> list[str]:
    return sorted(
        t.name for t in threading.enumerate()
        if t.name.endswith(_TRANSPORT_SUFFIXES)
    )


def _report(tag: str) -> tuple[int, int]:
    names = _transport_thread_names()
    total = threading.active_count()
    print(f"[{tag:>12}] total={total:3d}  transport={len(names)}  {names}")
    return total, len(names)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Count threads across sensor release/reacquire cycles."
    )
    ap.add_argument("--cycles", type=int, default=10,
                    help="release/reacquire cycles to run (default 10)")
    ap.add_argument("--sensors", type=int, default=1, choices=(1, 2),
                    help="sensors that must be connected each cycle (default 1)")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="seconds to settle before each count (default 1.0)")
    ap.add_argument("--connect-timeout", type=float, default=120.0,
                    help="seconds to wait for the first connect (default 120; "
                         "generous so the resource-busy scenario can be watched)")
    ap.add_argument("--verbose", action="store_true",
                    help="show SDK INFO logs (connect/disconnect flow)")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
    )

    iface = MotionInterface()
    iface.start()

    # Wait for first connect, reporting while we wait so a resource-busy
    # period (another app holding the device) is visible: pre-fix SDKs
    # grow here, the fixed SDK stays flat.
    deadline = time.monotonic() + args.connect_timeout
    while len(iface.connected_sensors()) < args.sensors:
        if time.monotonic() >= deadline:
            print(f"SETUP FAIL: {args.sensors} sensor(s) did not connect in "
                  f"{args.connect_timeout:.0f}s -- close any app holding the "
                  f"USB device (TestApp/bloodflow) and check cabling")
            iface.stop()
            return 2
        _report("waiting")
        time.sleep(5.0)

    # Let the console (if attached) finish connecting too, so the baseline
    # is a steady state that later cycles can be compared against.
    iface.wait_for_ready(require_attached_only=True, timeout=15.0)
    time.sleep(args.settle)
    base_total, base_transport = _report("baseline")

    worst_total, worst_transport = base_total, base_transport
    for i in range(1, args.cycles + 1):
        for s in iface.connected_sensors():
            s.request_disconnect()
        # The DISCONNECTED window is transient (the poll sweep reconnects
        # within ~200 ms); a missed window just means reconnect already won.
        for s in (iface.left, iface.right):
            s.wait_for(ConnectionState.DISCONNECTED, timeout=5.0)
        if not iface.wait_for_ready(console=False, sensors=args.sensors,
                                    timeout=30.0):
            print(f"SETUP FAIL: cycle {i}: sensors did not reconnect")
            iface.stop()
            return 2
        time.sleep(args.settle)
        total, transport = _report(f"cycle {i:02d}")
        worst_total = max(worst_total, total)
        worst_transport = max(worst_transport, transport)

    iface.stop()
    time.sleep(1.0)
    _report("after stop")

    grew = max(worst_total - base_total, worst_transport - base_transport)
    if grew > 0:
        print(f"FAIL: thread count grew by {grew} above the baseline across "
              f"{args.cycles} release/reacquire cycles -- transport threads "
              f"are leaking")
        return 1
    print(f"PASS: thread count flat across {args.cycles} release/reacquire "
          f"cycles ({args.sensors} sensor(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
