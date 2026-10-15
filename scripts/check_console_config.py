#!/usr/bin/env python3
"""
check_console_config.py

Check the user config stored on a console before updating it to firmware
with the strict-config check (console-fw fix/jsmn-strict, CFW-12/CFW-15).

That firmware parses the stored config in jsmn strict mode at boot. A
stored config that is not strict JSON (NaN, Infinity, -Infinity, a bare
word, an out-of-range number) is no longer applied, which leaves the TEC
thermal trip and the safety limits unset. Run this on each console while it
still runs the old firmware; any console that fails needs its config fixed
and rewritten first.

The raw stored JSON text is checked, not Python's view of it: json.loads
accepts NaN and Infinity, the console does not.

Reads only; writes nothing.

Usage
-----
    python scripts/check_console_config.py

Exit code 0 when the stored config passes, 1 when it does not, 2 when the
console cannot be read.
"""

import json
import math
import struct
import sys

from omotion import MotionInterface
from omotion.MotionConfig import MOTION_CFG_JSON_MAX_LEN, strict_json_problem
from omotion.config import OW_CMD, OW_CMD_USR_CFG, OW_ERROR


def read_raw_config(console) -> str:
    """The stored config JSON exactly as the console holds it."""
    r = console.uart.send_packet(
        id=None, packetType=OW_CMD, command=OW_CMD_USR_CFG, reserved=0
    )
    console.uart.clear_buffer()
    if r is None or r.packetType == OW_ERROR or r.data_len < 16:
        raise RuntimeError("console did not return its config")
    data = bytes(r.data)
    _magic, _version, _seq, _crc, json_len = struct.unpack("<IIIHH", data[:16])
    return data[16:16 + json_len].decode("utf-8", errors="replace").rstrip("\x00")


def main() -> int:
    iface = MotionInterface()
    iface.start(wait=False)
    try:
        if not iface.wait_for_ready(console=True, sensors=0, timeout=15):
            print("console not connected")
            return 2
        console = iface.console
        try:
            console.telemetry.stop()
        except Exception:
            pass
        print(f"console firmware {console.get_version()}")
        try:
            text = read_raw_config(console)
        except Exception as e:
            print(f"cannot read the stored config: {e}")
            return 2

        print(f"stored config: {len(text.encode('utf-8'))} bytes (limit {MOTION_CFG_JSON_MAX_LEN})")
        problems = []
        why = strict_json_problem(text)
        if why:
            problems.append(f"not strict JSON: {why}")
        else:
            data = json.loads(text)
            tec = data.get("TEC_TRIP")
            if tec is None:
                problems.append("TEC_TRIP missing: the TEC thermal trip would not be armed")
            elif isinstance(tec, bool) or not isinstance(tec, (int, float)) or not math.isfinite(tec) or tec <= 0:
                problems.append(f"TEC_TRIP {tec!r} is not a positive number")
        if problems:
            print("FAIL")
            for p in problems:
                print(f"  - {p}")
            print("Fix the config and rewrite it (write_config / write_config_json) before "
                  "updating this console's firmware.")
            return 1
        print("OK: strict JSON object with a TEC_TRIP threshold")
        return 0
    finally:
        iface.stop()


if __name__ == "__main__":
    sys.exit(main())
