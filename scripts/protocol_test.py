#!/usr/bin/env python3
"""Packet-protocol verification utility (console UART path).

Exercises the framing, CRC, transaction-ID, response-type and payload-bound
clauses of SWREQ-196 / SWREQ-197 / SWREQ-202 by constructing frames directly
(including deliberately malformed ones) and reporting the response packet type.

Every check prints the transmitted frame and the received response type, so the
console output is the test record.

Usage:
    python scripts/protocol_test.py --list
    python scripts/protocol_test.py --check crc-mismatch
    python scripts/protocol_test.py --check all
"""

from __future__ import annotations

import argparse
import sys

from omotion import MotionInterface
from omotion.UartPacket import UartPacket
from omotion.config import (
    OW_ACK, OW_NAK, OW_CMD, OW_RESP, OW_DATA, OW_JSON,
    OW_CONTROLLER, OW_FPGA_PROG,
    OW_BAD_PARSE, OW_BAD_CRC, OW_UNKNOWN, OW_ERROR,
    OW_START_BYTE, OW_END_BYTE,
    OW_CMD_PING, OW_CMD_VERSION, OW_CMD_ECHO,
)

DEFINED_RESPONSES = {
    OW_ACK: "OW_ACK", OW_NAK: "OW_NAK", OW_RESP: "OW_RESP",
    OW_DATA: "OW_DATA", OW_JSON: "OW_JSON", OW_BAD_PARSE: "OW_BAD_PARSE",
    OW_BAD_CRC: "OW_BAD_CRC", OW_UNKNOWN: "OW_UNKNOWN", OW_ERROR: "OW_ERROR",
}

_observed: set[int] = set()


def name_of(pt) -> str:
    if pt is None:
        return "NO RESPONSE"
    return f"{DEFINED_RESPONSES.get(pt, 'UNDEFINED')} (0x{pt:02X})"


def build(tid=0x0001, ptype=OW_CMD, command=OW_CMD_PING, addr=0, reserved=0, data=None):
    return UartPacket(id=tid, packetType=ptype, command=command,
                      addr=addr, reserved=reserved, data=data or [])


def show_tx(raw: bytes, label: str = "TX") -> None:
    head = " ".join(f"{b:02X}" for b in raw[:12])
    tail = " ".join(f"{b:02X}" for b in raw[-3:])
    print(f"  {label} ({len(raw)} bytes): {head}{' ... ' if len(raw) > 15 else ' '}{tail}")


class Transport:
    """Uniform raw-frame send/receive over either the console UART or a
    sensor's COMMS bulk endpoint.

    Both paths speak the same `UartPacket` frame; only the byte pipe differs.
    """

    def __init__(self, kind: str, handle):
        self.kind = kind          # "console" | "sensor"
        self.handle = handle      # MotionUart | CommInterface
        self._paused_reader = False

    def __enter__(self):
        # A sensor in async mode runs a reader thread that would consume the
        # responses we want to inspect; pause it for the duration.
        if self.kind == "sensor" and getattr(self.handle, "async_mode", False):
            try:
                self.handle.stop_read_thread()
                self._paused_reader = True
            except Exception:
                pass
        return self

    def __exit__(self, *exc):
        if self._paused_reader:
            try:
                self.handle.start_read_thread()
            except Exception:
                pass
        return False

    def send(self, raw: bytes, timeout: int = 5):
        if self.kind == "console":
            self.handle._tx(bytes(raw))
            return self.handle.read_packet(timeout=timeout)
        # sensor: raw bulk write, then read and parse one frame
        self.handle.write(bytes(raw), timeout=max(100, timeout * 1000))
        data = self.handle.receive(length=4096, timeout=max(100, timeout * 1000))
        if not data:
            return None
        return UartPacket(buffer=bytes(data))


def send_raw(tp: "Transport", raw: bytes, timeout: int = 5):
    """Write raw bytes and read whatever comes back. None on timeout."""
    show_tx(raw)
    try:
        resp = tp.send(raw, timeout=timeout)
    except Exception as exc:                      # timeout / parse failure
        print(f"  RX: none ({type(exc).__name__}: {exc})")
        return None
    if resp is None:
        print("  RX: none")
        return None
    _observed.add(resp.packetType)
    print(f"  RX: type={name_of(resp.packetType)} tid=0x{resp.id:04X} len={resp.data_len}")
    return resp


# --------------------------------------------------------------------------- #
# Checks
# --------------------------------------------------------------------------- #

def check_frame_format(uart):
    """Transmitted frame matches the specified fixed layout."""
    pkt = build(tid=0x0101, command=OW_CMD_VERSION)
    raw = pkt.to_bytes()
    show_tx(raw, "FRAME")
    print(f"  start=0x{raw[0]:02X} tid=0x{raw[1]:02X}{raw[2]:02X} type=0x{raw[3]:02X} "
          f"cmd=0x{raw[4]:02X} addr=0x{raw[5]:02X} reserved=0x{raw[6]:02X} "
          f"len={int.from_bytes(raw[7:9],'big')} crc=0x{raw[-3]:02X}{raw[-2]:02X} end=0x{raw[-1]:02X}")
    ok = raw[0] == OW_START_BYTE and raw[-1] == OW_END_BYTE and raw[6] == 0
    resp = send_raw(uart, raw)
    return ok and resp is not None


def check_tid_echo(uart):
    """Transaction ID echoed unchanged."""
    resp = send_raw(uart, build(tid=0x1234, command=OW_CMD_VERSION).to_bytes())
    return resp is not None and resp.id == 0x1234


def check_crc_mismatch(uart):
    """Corrupted CRC -> OW_BAD_CRC, command not executed."""
    raw = bytearray(build(tid=0x0202, command=OW_CMD_VERSION).to_bytes())
    raw[-3] ^= 0xFF                                # corrupt CRC field
    resp = send_raw(uart, raw)
    return resp is not None and resp.packetType == OW_BAD_CRC


def check_no_start(uart):
    """Absent start byte -> framing error."""
    raw = bytearray(build(tid=0x0303).to_bytes())
    raw[0] = 0x55
    resp = send_raw(uart, raw)
    return resp is None or resp.packetType in (OW_BAD_PARSE, OW_UNKNOWN)


def check_no_end(uart):
    """Absent end byte -> framing error."""
    raw = bytearray(build(tid=0x0404).to_bytes())[:-1]
    resp = send_raw(uart, raw)
    return resp is None or resp.packetType in (OW_BAD_PARSE, OW_UNKNOWN)


def check_bad_length(uart):
    """Declared length inconsistent with frame -> framing error."""
    raw = bytearray(build(tid=0x0505, data=[1, 2, 3, 4]).to_bytes())
    raw[7:9] = (99).to_bytes(2, "big")             # claim 99 bytes, send 4
    resp = send_raw(uart, raw)
    return resp is None or resp.packetType in (OW_BAD_PARSE, OW_UNKNOWN)


def check_oversize(uart):
    """Declared payload > 2048 -> OW_BAD_PARSE, payload not buffered."""
    raw = bytearray(build(tid=0x0606, data=[0] * 16).to_bytes())
    raw[7:9] = (2049).to_bytes(2, "big")
    resp = send_raw(uart, raw)
    return resp is None or resp.packetType == OW_BAD_PARSE


def check_payload_bounds(uart):
    """0-byte and 2048-byte payloads accepted."""
    ok = True
    for n in (0, 2048):
        print(f"  -- echo payload {n} bytes")
        resp = send_raw(uart, build(tid=0x0700 + (n & 0xFF),
                                    command=OW_CMD_ECHO,
                                    data=[0xA5] * n).to_bytes(), timeout=10)
        ok = ok and resp is not None and resp.packetType not in (OW_BAD_PARSE, OW_BAD_CRC)
    return ok


def check_unknown_opcode(uart):
    """Undefined opcode and undefined packet type -> OW_UNKNOWN."""
    r1 = send_raw(uart, build(tid=0x0801, command=0x7E).to_bytes())
    r2 = send_raw(uart, build(tid=0x0802, ptype=0xC3).to_bytes())
    return all(r is not None and r.packetType == OW_UNKNOWN for r in (r1, r2))


def check_reserved_nonzero(uart):
    """Reserved byte non-zero -> rejected, no effect."""
    resp = send_raw(uart, build(tid=0x0901, reserved=0x01).to_bytes())
    return resp is None or resp.packetType in (OW_UNKNOWN, OW_BAD_PARSE)


def check_bad_subtarget(uart):
    """Non-existent sub-target -> rejected."""
    resp = send_raw(uart, build(tid=0x0A01, ptype=OW_CONTROLLER,
                                command=0x19, addr=0x7F).to_bytes())
    return resp is None or resp.packetType in (OW_UNKNOWN, OW_ERROR)


def check_ordering(uart):
    """Responses returned in the order commands were sent."""
    sent, got = [], []
    for i in range(10):
        tid = 0x0B00 + i
        sent.append(tid)
        r = send_raw(uart, build(tid=tid, command=OW_CMD_VERSION).to_bytes())
        got.append(r.id if r else None)
    print(f"  sent tids: {[hex(t) for t in sent]}")
    print(f"  recv tids: {[hex(t) if t else None for t in got]}")
    return sent == got


def check_command_sweep(uart):
    """Send every defined opcode in each packet type; record responses."""
    sets = {
        "OW_CMD": (OW_CMD, list(range(0x00, 0x10))),
        "OW_CONTROLLER": (OW_CONTROLLER, list(range(0x10, 0x2B))),
        "OW_FPGA_PROG": (OW_FPGA_PROG, list(range(0x30, 0x40))),
    }
    ok = True
    for label, (ptype, opcodes) in sets.items():
        print(f"  -- {label}")
        for op in opcodes:
            r = send_raw(uart, build(tid=0x0C00 + op, ptype=ptype, command=op).to_bytes())
            if r is None or r.packetType == OW_UNKNOWN:
                print(f"     opcode 0x{op:02X}: NOT IMPLEMENTED")
                ok = False
    return ok


def check_response_set(uart):
    """Every response type observed so far is within the defined set."""
    undefined = [pt for pt in _observed if pt not in DEFINED_RESPONSES]
    print(f"  observed: {sorted(name_of(p) for p in _observed)}")
    if undefined:
        print(f"  UNDEFINED TYPES SEEN: {[hex(p) for p in undefined]}")
    return not undefined


CHECKS = {
    "frame-format": check_frame_format,
    "tid-echo": check_tid_echo,
    "crc-mismatch": check_crc_mismatch,
    "no-start": check_no_start,
    "no-end": check_no_end,
    "bad-length": check_bad_length,
    "oversize": check_oversize,
    "payload-bounds": check_payload_bounds,
    "unknown-opcode": check_unknown_opcode,
    "reserved-nonzero": check_reserved_nonzero,
    "bad-subtarget": check_bad_subtarget,
    "ordering": check_ordering,
    "command-sweep": check_command_sweep,
    "response-set": check_response_set,
}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", default="all", help="check name, or 'all'")
    p.add_argument("--list", action="store_true", help="list available checks")
    p.add_argument("--target", default="console", choices=("console", "sensor"),
                   help="device under test (default: console)")
    p.add_argument("--side", default="left", choices=("left", "right"),
                   help="sensor side when --target sensor (default: left)")
    args = p.parse_args()

    if args.list:
        for k, fn in CHECKS.items():
            print(f"  {k:20s} {(fn.__doc__ or '').strip()}")
        return 0

    if args.check != "all" and args.check not in CHECKS:
        print(f"Unknown check '{args.check}'. Use --list.")
        return 2

    iface = MotionInterface()
    iface.start()
    want_sensors = 1 if args.target == "sensor" else 0
    iface.wait_for_ready(console=args.target == "console",
                         sensors=want_sensors, timeout=15)
    console_ok, left_ok, right_ok = iface.is_device_connected()

    if args.target == "console":
        if not console_ok:
            print("Console not connected — check the cable and that no other app holds the port.")
            iface.stop()
            return 1
        handle, kind = iface.console.uart, "console"
    else:
        sensor = iface.left if args.side == "left" else iface.right
        if not (left_ok if args.side == "left" else right_ok):
            print(f"{args.side.capitalize()} sensor not connected — check the cable "
                  "and that no other app holds the device.")
            iface.stop()
            return 1
        if sensor.uart is None or getattr(sensor.uart, "comm", None) is None:
            print("Sensor COMMS interface unavailable.")
            iface.stop()
            return 1
        handle, kind = sensor.uart.comm, "sensor"

    print(f"Target: {kind}" + (f" ({args.side})" if kind == "sensor" else ""))
    selected = list(CHECKS) if args.check == "all" else [args.check]
    results = {}

    try:
        with Transport(kind, handle) as tp:
            for name in selected:
                print(f"\n=== {name} ===")
                try:
                    results[name] = bool(CHECKS[name](tp))
                except Exception as exc:
                    print(f"  EXCEPTION: {type(exc).__name__}: {exc}")
                    results[name] = False
                print(f"  RESULT: {'PASS' if results[name] else 'REVIEW'}")
    finally:
        iface.stop()

    print("\n=== SUMMARY ===")
    for name, ok in results.items():
        print(f"  {name:20s} {'PASS' if ok else 'REVIEW'}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
