#!/usr/bin/env python3
"""
hardware_unit_test.py - Functional smoke test of the console and sensor modules.

Walks each module through a quick "is every section talking and working"
check - not a characterisation - and writes a structured JSON result for the
report generator. The checks mirror the test app's Console and Sensor pages
(openmotion-test-app pages/Console.qml, pages/Sensor.qml, motion_connector.py).

Console
  comms      ping, echo
  info       firmware version, serial number, device id, board rev id
  temps      MCU / Safety / TA board temperature (MAX31875 sensors) within
             per-sensor limits (TEMP_LIMITS_C); --temp-limit overrides one
  i2c        PDU (0x20/0x48/0x4B on mux 1 ch 0 + PDU MON read),
             TA / Seed / Safety EE / Safety OPT FPGA (0x41 on mux 1 ch 4-7),
             TEC (DAC setpoint + status read; a latched TEC trip is a FAIL -
             the firmware refuses trigger start while it is set - and the
             laser temperature from the TEC thermistor, the reading the trip
             acts on, must be below the 40 C trip point)
  odometer   system minutes + laser pulses
  indicator  step the RGB indicator through every state, read each back,
             then restore the original state; a second check sends the
             out-of-range state 4 and expects the firmware to refuse it
             (KNOWN on console 1.8.2 and earlier, see KNOWN_ANOMALIES)
  fan        drive --fan-speed, all three tachs >= --fan-min-rpm; the fan is
             left at --fan-speed (the bloodflow app runs it at 100)
  fpga       version register of each FPGA; one full-width write to a
             firmware-owned register per FPGA, which the firmware must REFUSE
             with OW_ERROR and the value left unchanged (console 1.8.2+
             Release/Debug images own the safety state; a successful write is
             itself the failure). Run only when boot-info reports a console
             that owns the safety state and is not a Factory image; skipped
             while the trigger runs.

Sensor (left / right)
  comms      ping, echo
  info       firmware version, serial number, device id
  temp       sensor (IMU) temperature within TEMP_LIMITS_C["imu"]
  imu        accelerometer + gyroscope read, accelerometer non-zero
  nvcm       NVCM pin-drive boot probe on all 8 cameras (PROGRAMMED = pass;
             INCONCLUSIVE when the firmware omits the verdict byte)
  capture    one histogram frame from each of the 8 cameras

Close the test app first - USB access is exclusive. Cameras are left powered
and the fan at --fan-speed; both are recorded under "left_as" in the result.

Progress is printed as each check runs (--progress jsonl emits one JSON event
per line on stdout instead, for a wrapping GUI); the result document is
written to --output.

Usage:
  python scripts/hardware_unit_test.py
  python scripts/hardware_unit_test.py --sensors left --no-console
  python scripts/hardware_unit_test.py --skip-capture -o result.json
  python scripts/hardware_unit_test.py --progress jsonl

Status per check: PASS, FAIL, SKIP, or KNOWN - a failure that matches a
recorded firmware anomaly on the affected versions (KNOWN_ANOMALIES). KNOWN is
reported but does not fail the run.

Exit code: 0 no check failed, 1 a check failed, 2 could not start.
"""

import argparse
import contextlib
import datetime as _dt
import io
import json
import logging
import math
import os
import platform
import re
import struct
import sys
import time
import traceback
from dataclasses import dataclass, field
from typing import Callable

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SDK_ROOT = os.path.abspath(os.path.join(_THIS_DIR, ".."))
if _SDK_ROOT not in sys.path:
    sys.path.insert(0, _SDK_ROOT)

from omotion.config import (  # noqa: E402
    OW_CONTROLLER, OW_CTRL_I2C_WR, OW_CTRL_SET_IND, OW_ERROR, OW_RESP,
    TEST_PATTERN_DISABLED)
from omotion.console_telemetry_conversions import (  # noqa: E402
    tec_thermistor_voltage_to_celsius)

SCHEMA = "openmotion.hardware_unit_test/1"
PASS, FAIL, SKIP, KNOWN = "PASS", "FAIL", "SKIP", "KNOWN"

# Failures that match a recorded firmware anomaly. A check listed here that
# fails on a firmware version at or below "through" is reported KNOWN, not
# FAIL; on later firmware it is an ordinary FAIL (the fix regressed).
KNOWN_ANOMALIES = {
    "console.indicator.reject": {
        "through": (1, 8, 2),
        "note": ("console fw if_commands.c OW_CTRL_SET_IND range check tests "
                 "uartResp->reserved (always 0) instead of cmd->reserved, so state 4 "
                 "is stored and all indicator LEDs go off; cosmetic"),
    },
}

# Laser (TEC thermistor) trip point, C: console fw safety_envelope.h
# SAFETY_TEC_TRIP_C. uart_comms.c tec_trip_evaluate() compares the thermistor
# sense voltage against it and opens the safety disconnect.
TEC_TRIP_C = 40.0

ECHO_PAYLOAD = b"Hello FROM Test Application!"
FAN_SPINUP_S = 3.0
INDICATOR_STATES = (1, 2, 3, 0)  # IND1, IND2, IND3, OFF
INDICATOR_INVALID = 4            # first out-of-range state; firmware must refuse it

# Per-sensor board temperature windows, C. Lower bound is the 15 C ambient
# minimum of TR-00052 s4.3. These are the MAX31875 board sensors, not the laser:
# the TEC trip acts on the TEC thermistor, which the TEC check reads.
TEMP_LIMITS_C = {
    "mcu": (15.0, 70.0),
    "safety": (15.0, 60.0),
    "ta": (15.0, 60.0),
    "imu": (15.0, 60.0),
}
CAMERAS = 8

# Console I2C topology (Console.qml PDU / TA / Seed / Safety buttons).
PDU_MUX, PDU_CHANNEL = 1, 0
PDU_ADDRS = (0x20, 0x48, 0x4B)
FPGA_MUX, FPGA_ADDR = 1, 0x41


@dataclass(frozen=True)
class Fpga:
    key: str
    name: str
    channel: int      # console mux 1 channel
    version_reg: int  # 4 bytes: REV, MINOR, MAJOR, ID
    rw_reg: int       # firmware-owned register for the refused-write check
    rw_reg_name: str  # name in console-fw Core/Src/fpga_register_map.c
    rw_width: int     # data_len in that map; the firmware checks the length first


# Refused-write registers, named and sized as in the console firmware's
# fpga_register_map.c. The width matters: fpga_passthrough_write_problem()
# refuses a wrong length with "length does not match the register" BEFORE it
# reaches the ownership rule, so a short write would pass for the wrong
# reason. TA_PWM_MON_CL is CFG_KEY_SAFETY; SEED_DDS_CTRL is unlisted (owned);
# EE/OPT_PULSE_WIDTH_UL are 4-byte CFG_KEY_SAFETY limits.
FPGAS = (
    Fpga("ta", "TA", 4, 0x14, 0x0A, "TA_PWM_MON_CL", 2),
    Fpga("seed", "Seed", 5, 0x13, 0x00, "SEED_DDS_CTRL", 2),
    Fpga("safety_ee", "Safety EE", 6, 0x25, 0x04, "EE_PULSE_WIDTH_UL", 4),
    Fpga("safety_opt", "Safety OPT", 7, 0x25, 0x04, "OPT_PULSE_WIDTH_UL", 4),
)

# Byte repeated to the register width for the refused write. The check runs
# only where boot-info says the console owns the safety state and is not a
# Factory image, so the write is never applied; if it ever were (a regression),
# the original value is written back at once. SEED_DDS_CTRL is a control word,
# not a limit, so the gating is what keeps this write off older firmware.
RW_PATTERN_BYTE = 0xA5

# OW_FACTORY_NVCM_CHECK blob: 27 fixed bytes, 16 B per NVCM row read, then
# (sensor-fw #91) one pin-drive boot verdict byte.
_NVCM_FIXED_LEN = 27
_NVCM_ROW_LEN = 16
_NVCM_STEP_STATUS = 1 << 3

_B58_ALPHABET = "123456789ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz"


# ---------------------------------------------------------------------------
# Check plumbing
# ---------------------------------------------------------------------------

class Skip(Exception):
    """Raised by a check to record SKIP instead of PASS/FAIL."""


@dataclass
class Outcome:
    passed: bool
    message: str = ""
    data: dict = field(default_factory=dict)


def _version_tuple(version) -> tuple[int, ...] | None:
    """(major, minor, patch) from a firmware version string such as 'v1.8.2'
    or '1.8.2-rc.5'; None if it has no such triple."""
    m = re.search(r"(\d+)\.(\d+)\.(\d+)", str(version or ""))
    return tuple(int(x) for x in m.groups()) if m else None


@dataclass
class Check:
    module: str
    id: str
    name: str
    group: str
    fn: Callable[[], Outcome]


def _now() -> str:
    return _dt.datetime.now().astimezone().isoformat(timespec="seconds")


def _finite(value):
    """float(value), or None for NaN/inf (keeps the JSON document valid)."""
    value = float(value)
    return value if math.isfinite(value) else None


def _hexlist(addrs) -> list[str]:
    return [f"0x{a:02X}" for a in addrs]


def _base58(raw: bytes) -> str:
    """Bitcoin-alphabet base58 - the encoding the test app shows as Device ID."""
    n = int.from_bytes(raw, "big")
    out = ""
    while n:
        n, r = divmod(n, 58)
        out = _B58_ALPHABET[r] + out
    return "1" * (len(raw) - len(raw.lstrip(b"\0"))) + out


def _temp_outcome(celsius, args, source: str) -> Outcome:
    lo, hi = args.temp_limits[source]
    value = _finite(celsius)
    ok = value is not None and lo <= value <= hi
    data = {"celsius": value, "min_c": lo, "max_c": hi, "source": source}
    if value is None:
        return Outcome(False, f"invalid reading {celsius!r}", data)
    limits = "" if ok else f" (outside {lo:g}-{hi:g} C)"
    return Outcome(ok, f"{value:.1f} C{limits}", data)


# ---------------------------------------------------------------------------
# Checks shared by console and sensors
# ---------------------------------------------------------------------------

def _connect_check(handle, timeout: float):
    def fn():
        deadline = time.monotonic() + timeout
        while not handle.is_connected() and time.monotonic() < deadline:
            time.sleep(0.1)
        ok = handle.is_connected()
        data = {"state": handle.state.name, "reason": handle.state_reason}
        return Outcome(ok, "connected" if ok else f"not connected after {timeout:g} s", data)
    return fn


def _ping_check(dev):
    def fn():
        ok = bool(dev.ping())
        return Outcome(ok, "" if ok else "no ping response")
    return fn


def _echo_check(dev):
    def fn():
        data, length = dev.echo(echo_data=ECHO_PAYLOAD)
        echoed = bytes(data[:length]) if data is not None else b""
        ok = echoed == ECHO_PAYLOAD
        msg = f"{len(echoed)} bytes echoed" if ok else f"mismatch: got {echoed!r}"
        return Outcome(ok, msg, {"sent": ECHO_PAYLOAD.decode(), "received": echoed.hex()})
    return fn


def _version_check(dev, info: dict):
    def fn():
        version = dev.get_version()
        info["firmware_version"] = version
        ok = bool(version) and version != "v0.0.0"
        return Outcome(ok, version or "no version", {"version": version})
    return fn


def _serial_check(dev, info: dict):
    def fn():
        serial = dev.read_serial_number()
        info["serial_number"] = serial
        return Outcome(bool(serial), serial or "no serial (unprogrammed or read error)",
                       {"serial_number": serial})
    return fn


def _device_id_check(dev, info: dict):
    def fn():
        hw_id = dev.get_hardware_id()
        if not hw_id:
            return Outcome(False, "hardware id read failed")
        device_id = _base58(bytes.fromhex(hw_id))
        info["device_id"] = device_id
        info["hardware_id"] = hw_id
        return Outcome(True, device_id, {"device_id": device_id, "hardware_id": hw_id})
    return fn


def _comms_and_info(module, dev, info, timeout) -> list[Check]:
    return [
        Check(module, f"{module}.connect", "Connect", "connection", _connect_check(dev, timeout)),
        Check(module, f"{module}.ping", "Ping", "comms", _ping_check(dev)),
        Check(module, f"{module}.echo", "Echo", "comms", _echo_check(dev)),
        Check(module, f"{module}.version", "Firmware version", "info", _version_check(dev, info)),
        Check(module, f"{module}.serial", "Serial number", "info", _serial_check(dev, info)),
        Check(module, f"{module}.device_id", "Device ID", "info", _device_id_check(dev, info)),
    ]


# ---------------------------------------------------------------------------
# Console
# ---------------------------------------------------------------------------

def console_checks(console, info: dict, args) -> list[Check]:
    temps: dict = {}
    trigger: dict = {}

    def board_id():
        rev = console.read_board_id()
        info["board_id"] = rev
        return Outcome(isinstance(rev, int), f"rev {rev}", {"board_id": rev})

    def temperature(key: str, label: str):
        def fn():
            if not temps:
                mcu, safety, ta = console.get_temperatures()
                temps.update(mcu=mcu, safety=safety, ta=ta)
            return _temp_outcome(temps[key], args, label)
        return fn

    def pdu():
        found = console.scan_i2c_mux_channel(PDU_MUX, PDU_CHANNEL)
        missing = [a for a in PDU_ADDRS if a not in found]
        with contextlib.redirect_stdout(io.StringIO()):  # read_pdu_mon prints the raw packet
            mon = console.read_pdu_mon()
        data = {"mux": PDU_MUX, "channel": PDU_CHANNEL, "i2c_found": _hexlist(found),
                "i2c_expected": _hexlist(PDU_ADDRS), "pdu_mon_ok": mon is not None}
        if mon is not None:
            data["pdu_mon_volts"] = [round(v, 4) for v in mon.volts]
        problems = []
        if missing:
            problems.append("missing " + ", ".join(_hexlist(missing)))
        if mon is None:
            problems.append("PDU MON read failed")
        msg = "; ".join(problems) or f"{', '.join(_hexlist(PDU_ADDRS))} present, PDU MON read"
        return Outcome(not problems, msg, data)

    def fpga_bus(fpga: Fpga):
        def fn():
            found = console.scan_i2c_mux_channel(FPGA_MUX, fpga.channel)
            ok = FPGA_ADDR in found
            msg = f"0x{FPGA_ADDR:02X} {'present' if ok else 'missing'} on mux {FPGA_MUX} ch {fpga.channel}"
            return Outcome(ok, msg, {"mux": FPGA_MUX, "channel": fpga.channel,
                                     "i2c_found": _hexlist(found)})
        return fn

    def tec():
        setpoint = console.tec_voltage()
        vout, temp_set, current, voltage, good = console.tec_status()
        # Laser temperature: the TEC thermistor reading the trip compares, as
        # the SDK telemetry converts it (ScanWorkflow tec_temp_c).
        laser_c = _finite(tec_thermistor_voltage_to_celsius(float(vout)))
        data = {"dac_setpoint_v": setpoint, "vout_v": float(vout),
                "temp_set_v": float(temp_set), "tec_current_v": float(current),
                "tec_voltage_v": float(voltage), "tec_good": bool(good),
                "laser_temp_c": laser_c, "tec_trip_c": TEC_TRIP_C}
        if not isinstance(setpoint, (int, float)) or isinstance(setpoint, bool):
            return Outcome(False, f"TEC DAC read failed ({setpoint!r})", data)
        laser = "laser temp n/a (no R-T table)" if laser_c is None else f"laser {laser_c:.1f} C"
        msg = f"setpoint {setpoint:.3f} V, {laser}"
        if not good:
            # trigger.c Trigger_Start(): _safety_trigger_interlock refuses a
            # start until tec_trip_evaluate has seen 200 clean polls.
            return Outcome(False, msg + "; TEC trip latched (tec_good=False) - "
                           "trigger start is refused while it is set", data)
        if laser_c is not None and laser_c >= TEC_TRIP_C:
            return Outcome(False, msg + f"; at or above the {TEC_TRIP_C:g} C TEC trip", data)
        return Outcome(True, msg, data)

    def odometer():
        minutes = console.get_system_odometer_minutes()
        pulses = console.get_laser_odometer_pulses()
        ok = minutes is not None and pulses is not None
        msg = (f"{minutes} min, {pulses} laser pulses" if ok
               else "odometer read failed (firmware predates the feature?)")
        return Outcome(ok, msg, {"system_minutes": minutes, "laser_pulses": pulses})

    def indicator():
        original = console.get_rgb_led()
        if original not in (0, 1, 2, 3):
            return Outcome(False, f"indicator read failed ({original})")
        readback = {}
        try:
            for state in INDICATOR_STATES:
                console.set_rgb_led(state)
                time.sleep(0.3)  # long enough to see each state on the bench
                readback[state] = console.get_rgb_led()
        finally:
            console.set_rgb_led(original)
        wrong = [s for s, got in readback.items() if got != s]
        if wrong:
            # if_commands.c OW_CTRL_SET_IND: while a laser-safety fault is
            # latched the write is acknowledged but not applied - the fault
            # blink owns the LED until the latch clears.
            msg = ("read-back mismatch for state(s) " + ", ".join(map(str, wrong))
                   + " - the write was acknowledged but not applied; a latched "
                   "laser-safety fault owns the LED (fault blink)")
        else:
            msg = "all states read back"
        return Outcome(not wrong, msg, {"original": original,
                                        "readback": {str(k): v for k, v in readback.items()}})

    def indicator_reject():
        """Negative check: an out-of-range indicator state must be refused.

        The SDK's set_rgb_led() validates 0-3 itself, so the raw packet is
        sent here. On 1.8.2 the firmware's range check reads the wrong field
        (if_commands.c:238 tests uartResp->reserved, not cmd->reserved), so
        the state is accepted - this check documents that."""
        original = console.get_rgb_led()
        if original not in (0, 1, 2, 3):
            return Outcome(False, f"indicator read failed ({original})")
        try:
            r = console.uart.send_packet(id=None, packetType=OW_CONTROLLER,
                                         command=OW_CTRL_SET_IND,
                                         reserved=INDICATOR_INVALID)
            console.uart.clear_buffer()
            refused = r is not None and r.packetType == OW_ERROR
            after = console.get_rgb_led()
        finally:
            console.set_rgb_led(original)
        data = {"sent": INDICATOR_INVALID, "refused": refused, "read_back": after,
                "original": original}
        if refused:
            return Outcome(True, f"state {INDICATOR_INVALID} refused (OW_ERROR)", data)
        return Outcome(False, f"state {INDICATOR_INVALID} was ACCEPTED (read back {after}); "
                       "the firmware range check tests the wrong field", data)

    def fan():
        if console.set_fan_speed(args.fan_speed) != args.fan_speed:
            return Outcome(False, f"set_fan_speed({args.fan_speed}) rejected")
        time.sleep(FAN_SPINUP_S)
        rpms = [console.get_fan_rpm(i) for i in (1, 2, 3)]
        low = [i for i, rpm in enumerate(rpms, 1) if rpm is None or rpm < args.fan_min_rpm]
        msg = "RPM " + " / ".join("?" if r is None else str(r) for r in rpms)
        if low:
            msg += f"; fan {', '.join(map(str, low))} below {args.fan_min_rpm}"
        return Outcome(not low, msg, {"speed_pct": args.fan_speed, "rpm": rpms,
                                      "min_rpm": args.fan_min_rpm})

    def fpga_version(fpga: Fpga):
        def fn():
            data, length = console.read_i2c_packet(
                mux_index=FPGA_MUX, channel=fpga.channel, device_addr=FPGA_ADDR,
                reg_addr=fpga.version_reg, read_len=4)
            if data is None or length < 4:
                return Outcome(False, "version register read failed")
            rev, minor, major, fpga_id = data[:4]
            version = f"{major}.{minor}.{rev}"
            info.setdefault("fpga_versions", {})[fpga.key] = version
            return Outcome(True, f"v{version} (id 0x{fpga_id:02X})",
                           {"version": version, "fpga_id": fpga_id, "raw": bytes(data[:4]).hex()})
        return fn

    def trigger_running() -> bool:
        if "running" not in trigger:
            cfg = console.get_trigger_json() or {}
            trigger["running"] = cfg.get("TriggerStatus") == 2  # same test as the app
        return trigger["running"]

    def write_lock_skip_reason() -> str | None:
        """Why the refused-write checks must not run on this console, or None.
        Read once from OW_CMD_BOOT_INFO: before console 1.8.2 the host owns
        these registers and the write would land; a Factory image accepts
        writes to owned registers by design (bench provisioning)."""
        if "lock_skip" not in trigger:
            boot = console.get_boot_info()
            if boot is None or not boot.firmware_owns_safety_state:
                why = ("console does not report firmware-owned safety state "
                       "(boot-info v1, firmware before 1.8.2) - write not attempted")
            elif boot.factory_image:
                why = "Factory image - owned registers are writable by design"
            else:
                why = None
            trigger["lock_skip"] = why
            info["factory_image"] = None if boot is None else boot.factory_image
        return trigger["lock_skip"]

    def raw_i2c_write(bus: dict, payload: bytes) -> bool:
        """OW_CTRL_I2C_WR sent directly: True if the firmware ACCEPTED the
        write, False if it answered OW_ERROR. Any other outcome (no reply,
        transport error) raises, so a comm failure is a FAIL, never a
        'refused'. MotionConsole.write_i2c_packet returns False for both."""
        header = struct.pack("BBBBB", bus["mux_index"], bus["channel"],
                             bus["device_addr"], bus["reg_addr"], len(payload))
        r = console.uart.send_packet(id=None, packetType=OW_CONTROLLER,
                                     command=OW_CTRL_I2C_WR, data=header + payload)
        console.uart.clear_buffer()
        if r is None:
            raise RuntimeError("no reply to OW_CTRL_I2C_WR")
        if r.packetType == OW_ERROR:
            return False
        if r.packetType == OW_RESP:
            return True
        raise RuntimeError(f"unexpected reply type 0x{r.packetType:02X} to OW_CTRL_I2C_WR")

    def fpga_register(fpga: Fpga):
        def fn():
            if args.skip_fpga_write:
                raise Skip("--skip-fpga-write")
            reason = write_lock_skip_reason()
            if reason:
                raise Skip(reason)
            if trigger_running():
                raise Skip("trigger is running - FPGA registers left untouched")
            bus = dict(mux_index=FPGA_MUX, channel=fpga.channel,
                       device_addr=FPGA_ADDR, reg_addr=fpga.rw_reg)
            width = fpga.rw_width

            def read() -> bytes | None:
                data, length = console.read_i2c_packet(**bus, read_len=width)
                return bytes(data[:width]) if data is not None and length >= width else None

            original = read()
            if original is None:
                return Outcome(False, "initial register read failed")
            pattern = bytes([RW_PATTERN_BYTE]) * width
            # Console 1.8.2 Release firmware owns the safety state: a host
            # write to one of these registers is refused (OW_ERROR, nothing
            # written, "firmware-owned safety register"). The lock-down is
            # what this check verifies: a refused full-width write with an
            # unchanged read-back is the PASS, a successful write is the
            # failure. No restore is attempted - it would be refused as well.
            wrote = raw_i2c_write(bus, pattern)
            readback = read()
            data = {"register": fpga.rw_reg_name, "reg_addr": f"0x{fpga.rw_reg:02X}",
                    "width": width, "original": original.hex(), "written": pattern.hex(),
                    "read_back": readback.hex() if readback else None,
                    "write_refused": not wrote}
            if wrote:
                data["restored"] = raw_i2c_write(bus, original) and read() == original
                return Outcome(False, f"{fpga.rw_reg_name} write of {pattern.hex()} was "
                               "ACCEPTED - register is not locked down"
                               + ("" if data["restored"] else
                                  f"; original {original.hex()} NOT restored"), data)
            if readback is None:
                return Outcome(False, "write refused (OW_ERROR) but the read-back failed", data)
            if readback != original:
                return Outcome(False, f"write refused but register changed: "
                               f"{original.hex()} -> {data['read_back']}", data)
            return Outcome(True, f"{fpga.rw_reg_name} {width}-byte write refused, "
                           f"value {original.hex()} unchanged", data)
        return fn

    m = "console"
    checks = _comms_and_info(m, console, info, args.timeout)
    checks += [
        Check(m, "console.board_id", "Board rev ID", "info", board_id),
        Check(m, "console.temp.mcu", "MCU temperature", "temperature", temperature("mcu", "mcu")),
        Check(m, "console.temp.safety", "Safety temperature", "temperature",
              temperature("safety", "safety")),
        Check(m, "console.temp.ta", "TA temperature", "temperature", temperature("ta", "ta")),
        Check(m, "console.pdu", "PDU", "peripheral", pdu),
    ]
    checks += [Check(m, f"console.{f.key}", f.name, "peripheral", fpga_bus(f)) for f in FPGAS]
    checks += [
        Check(m, "console.tec", "TEC", "peripheral", tec),
        Check(m, "console.odometer", "Odometer", "odometer", odometer),
        Check(m, "console.indicator", "Indicator", "indicator", indicator),
        Check(m, "console.indicator.reject", "Indicator rejects state 4", "indicator",
              indicator_reject),
        Check(m, "console.fan", "Fan", "fan", fan),
    ]
    checks += [Check(m, f"console.fpga_version.{f.key}", f"{f.name} FPGA version",
                     "fpga_version", fpga_version(f)) for f in FPGAS]
    checks += [Check(m, f"console.fpga_rw.{f.key}", f"{f.name} FPGA write refused",
                     "fpga_register", fpga_register(f)) for f in FPGAS]
    return checks


# ---------------------------------------------------------------------------
# Sensor
# ---------------------------------------------------------------------------

def _nvcm_verdict_from_blob(blob: bytes) -> tuple[str, str]:
    """Verdict from the pin-drive boot byte sensor-fw #91 appends to the
    OW_FACTORY_NVCM_CHECK blob (always present on 1.8.3). Mirrors
    openmotion-test-app utils/nvcm_verdict.interpret_check_blob."""
    missing = "INCONCLUSIVE", "boot-verdict byte missing from the NVCM blob"
    if len(blob) < _NVCM_FIXED_LEN + 1:
        return missing
    verdict_off = _NVCM_FIXED_LEN + _NVCM_ROW_LEN * blob[26]
    if len(blob) <= verdict_off:
        return missing
    booted = blob[verdict_off]
    if booted == 1:
        return "PROGRAMMED", "NVCM design booted and drove the camera bus (pin probe)"
    if booted == 0xFF:
        return "NO RESPONSE", "firmware refused the boot probe - camera not powered?"
    if booted != 0:
        return "INCONCLUSIVE", f"unexpected boot-probe byte 0x{booted:02X}"
    if blob[5] & _NVCM_STEP_STATUS and blob[7] & 0x08:
        return "BLANK", ("no NVCM boot (pin probe) - Done fuse is burned but the image "
                         "does not boot; this part cannot be NVCM-flashed again")
    return "BLANK", "no NVCM boot (pin probe) - NVCM blank"


def sensor_checks(side: str, sensor, info: dict, args) -> list[Check]:
    def temperature():
        return _temp_outcome(sensor.imu_get_temperature(), args, "imu")

    def imu():
        accel = sensor.imu_get_accelerometer()
        if not any(accel):  # IMU not started by firmware - bring it up once
            sensor.imu_init()
            sensor.imu_on()
            accel = sensor.imu_get_accelerometer()
        gyro = sensor.imu_get_gyroscope()
        ok = any(accel)
        msg = f"accel {accel}, gyro {gyro}" + ("" if ok else " - accelerometer reads all zero")
        return Outcome(ok, msg, {"accel_raw": list(accel), "gyro_raw": list(gyro)})

    def nvcm(cam_idx: int):
        def fn():
            if args.skip_nvcm:
                raise Skip("--skip-nvcm")
            mask = 1 << cam_idx
            if not sensor.enable_camera_power(mask):
                return Outcome(False, "camera power-on failed", {"verdict": "NO RESPONSE"})
            time.sleep(0.3)
            # The probe runs on the firmware's active camera, so the mux switch
            # must be confirmed - a silent failure would re-probe the last camera.
            if getattr(sensor.switch_camera(cam_idx), "packetType", None) != OW_RESP:
                return Outcome(False, "camera select (switch_camera) failed",
                               {"verdict": "INCONCLUSIVE", "method": "pin_probe"})
            time.sleep(0.1)
            name, detail = _nvcm_verdict_from_blob(sensor.nvcm_check(boot_test=False))
            return Outcome(name == "PROGRAMMED", f"{name} - {detail}",
                           {"verdict": name, "detail": detail, "method": "pin_probe"})
        return fn

    def capture(cam_idx: int):
        def fn():
            if args.skip_capture:
                raise Skip("--skip-capture")
            mask = 1 << cam_idx
            if not sensor.get_camera_power_status()[cam_idx]:
                if not sensor.enable_camera_power(mask):
                    return Outcome(False, "camera power-on failed")
                time.sleep(0.3)
            status = (sensor.get_camera_status(mask) or {}).get(cam_idx)
            if status is None:
                return Outcome(False, "camera status read failed")
            data = {"status_before": f"0x{status:02X}"}
            if not status & 0x01:
                return Outcome(False, f"camera not READY (status 0x{status:02X})", data)
            if not (status & 0x02 and status & 0x04):
                # Same bring-up as the test app's configureCamera: the firmware
                # ACKs the program before the FPGA is usable, so settle first.
                if not sensor.program_fpga(camera_position=mask, manual_process=False):
                    return Outcome(False, "FPGA program failed", data)
                time.sleep(0.1)
                if not sensor.camera_configure_registers(camera_position=mask):
                    return Outcome(False, "camera register configure failed", data)
            # auto_upload=True only means program_fpga(manual_process=False),
            # i.e. OW_FPGA_PROG_SRAM reserved=1 -> firmware program_fpga(): an
            # NVCM boot detect, no host bitstream. The Debug-only host-upload
            # path (reserved=0 / OW_FPGA_BITSTREAM, sensor-fw #136) is never
            # taken, and the camera is already programmed above anyway.
            result = sensor.get_camera_histogram(
                camera_id=cam_idx, test_pattern_id=TEST_PATTERN_DISABLED, auto_upload=True)
            if not result:
                after = (sensor.get_camera_status(mask) or {}).get(cam_idx)
                data["status_after"] = None if after is None else f"0x{after:02X}"
                detail = "" if after is None else (
                    f" (status after 0x{after:02X}{'' if after & 0x01 else ', not READY'})")
                return Outcome(False, "histogram capture failed" + detail, data)
            bins = list(result[0][:1024])
            if bins:
                # UA-02 spurious counts land on the idle-bus bin 0; the test
                # app subtracts the same constant. An approximation.
                bins[0] = max(0, bins[0] - 6)
            total = sum(bins)
            mean = sum(i * b for i, b in enumerate(bins)) / total if total else 0.0
            data.update(bins=len(bins), total_counts=total, mean=round(mean, 2))
            if args.include_histograms:
                data["histogram"] = bins
            ok = len(bins) == 1024 and total > 0
            msg = (f"{total} counts, mean {mean:.1f}" if ok
                   else f"empty frame ({len(bins)} bins, {total} counts)")
            return Outcome(ok, msg, data)
        return fn

    checks = _comms_and_info(side, sensor, info, args.timeout)
    checks += [
        Check(side, f"{side}.temp", "Sensor temperature", "temperature", temperature),
        Check(side, f"{side}.imu", "IMU", "imu", imu),
    ]
    checks += [Check(side, f"{side}.nvcm.cam{i + 1}", f"Camera {i + 1} NVCM", "nvcm", nvcm(i))
               for i in range(CAMERAS)]
    checks += [Check(side, f"{side}.capture.cam{i + 1}", f"Camera {i + 1} frame capture",
                     "capture", capture(i)) for i in range(CAMERAS)]
    return checks


# ---------------------------------------------------------------------------
# Progress reporting
# ---------------------------------------------------------------------------

class TextProgress:
    def __init__(self, out):
        self.out = out
        self.width = 1

    def start(self, total: int, modules: list[str]) -> None:
        self.width = len(str(total))
        print(f"OpenMotion hardware unit test - {total} checks on {', '.join(modules)}",
              file=self.out, flush=True)

    def test_start(self, index: int, total: int, check: Check) -> None:
        self.out.write(f"[{index:>{self.width}}/{total}] {check.module:<7} {check.name:<32} ")
        self.out.flush()

    def test_end(self, index: int, total: int, result: dict) -> None:
        message = f"  {result['message']}" if result["message"] else ""
        self.out.write(f"{result['status']:<5} ({result['duration_s']:.2f} s){message}\n")
        self.out.flush()

    def done(self, report: dict, path: str) -> None:
        print("", file=self.out)
        for name, module in report["modules"].items():
            s = module["summary"]
            print(f"  {name:<7} {s['result']:<4}  {s['passed']} passed, {s['failed']} failed, "
                  f"{s['skipped']} skipped, {s['known']} known", file=self.out)
        s = report["summary"]
        print(f"\nOVERALL {s['result']} - {s['passed']}/{s['total']} passed, "
              f"{s['failed']} failed, {s['skipped']} skipped, {s['known']} known",
              file=self.out)
        print(f"Result written to {path}", file=self.out, flush=True)

    def error(self, message: str) -> None:
        print(f"ERROR: {message}", file=self.out, flush=True)


class JsonlProgress:
    def __init__(self, out):
        self.out = out

    def _emit(self, **event) -> None:
        self.out.write(json.dumps(event) + "\n")
        self.out.flush()

    def start(self, total: int, modules: list[str]) -> None:
        self._emit(event="start", total=total, modules=modules)

    def test_start(self, index: int, total: int, check: Check) -> None:
        self._emit(event="test_start", index=index, total=total, module=check.module,
                   id=check.id, name=check.name, group=check.group)

    def test_end(self, index: int, total: int, result: dict) -> None:
        self._emit(event="test_end", index=index, total=total,
                   **{k: v for k, v in result.items() if k != "data"})

    def done(self, report: dict, path: str) -> None:
        self._emit(event="done", summary=report["summary"], output=path)

    def error(self, message: str) -> None:
        self._emit(event="error", message=message)


# ---------------------------------------------------------------------------
# Runner / report
# ---------------------------------------------------------------------------

def _summarize(tests: list[dict]) -> dict:
    counts = {status: sum(t["status"] == status for t in tests)
              for status in (PASS, FAIL, SKIP, KNOWN)}
    return {"result": FAIL if counts[FAIL] else PASS, "total": len(tests),
            "passed": counts[PASS], "failed": counts[FAIL], "skipped": counts[SKIP],
            "known": counts[KNOWN]}


def _known_anomaly(check: Check, module: dict) -> str | None:
    """The KNOWN_ANOMALIES note if this failure is the recorded anomaly on an
    affected firmware version, else None."""
    entry = KNOWN_ANOMALIES.get(check.id)
    if entry is None:
        return None
    version = _version_tuple(module["info"].get("firmware_version"))
    if version is None or version > entry["through"]:
        return None
    return entry["note"]


def run_checks(checks: list[Check], modules: dict, progress) -> None:
    total = len(checks)
    progress.start(total, list(modules))
    for index, check in enumerate(checks, 1):
        module = modules[check.module]
        progress.test_start(index, total, check)
        t0 = time.perf_counter()
        data: dict = {}
        if module["connected"] is False:
            status, message = SKIP, "module not connected"
        else:
            try:
                outcome = check.fn()
                status = PASS if outcome.passed else FAIL
                message, data = outcome.message, outcome.data
                known = None if outcome.passed else _known_anomaly(check, module)
                if known:
                    status = KNOWN
                    data = {**data, "known_anomaly": known}
            except Skip as skip:
                status, message = SKIP, str(skip)
            except Exception as exc:  # a raising SDK call is a failed check, not a crash
                status, message = FAIL, f"{type(exc).__name__}: {exc}"
                data = {"traceback": traceback.format_exc()}
        if check.group == "connection":
            module["connected"] = status == PASS
        result = {"id": check.id, "module": check.module, "name": check.name,
                  "group": check.group, "status": status, "message": message,
                  "duration_s": round(time.perf_counter() - t0, 3), "data": data}
        module["tests"].append(result)
        progress.test_end(index, total, result)


def build_report(modules: dict, args, started_at: str, elapsed_s: float,
                 sdk_version: str) -> dict:
    for module in modules.values():
        module["summary"] = _summarize(module["tests"])
    all_tests = [t for module in modules.values() for t in module["tests"]]
    return {
        "schema": SCHEMA,
        "started_at": started_at,
        "finished_at": _now(),
        "duration_s": round(elapsed_s, 2),
        "sdk_version": sdk_version,
        "host": {"hostname": platform.node(), "platform": platform.platform(),
                 "python": platform.python_version()},
        "settings": {"temp_limits_c": {k: {"min": lo, "max": hi}
                                       for k, (lo, hi) in args.temp_limits.items()},
                     "fan_speed_pct": args.fan_speed, "fan_min_rpm": args.fan_min_rpm,
                     "fpga_write": not args.skip_fpga_write, "nvcm": not args.skip_nvcm,
                     "capture": not args.skip_capture},
        "left_as": {"cameras_powered": bool(modules.keys() - {"console"})
                    and not (args.skip_nvcm and args.skip_capture),
                    "fan_speed_pct": args.fan_speed if "console" in modules else None},
        "summary": _summarize(all_tests),
        "modules": modules,
    }


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-console", action="store_true", help="skip the console module")
    ap.add_argument("--sensors", nargs="*", choices=["left", "right"],
                    default=["left", "right"], help="sensor modules to test (default: both)")
    ap.add_argument("-o", "--output", default=None,
                    help="result JSON path (default: hardware_test_<timestamp>.json)")
    ap.add_argument("--progress", choices=["text", "jsonl"], default="text",
                    help="progress format on stdout (default: text)")
    ap.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for each module to connect (default: 15)")
    ap.add_argument("--temp-limit", action="append", default=[], metavar="SENSOR=MIN:MAX",
                    help="override one temperature window, C; SENSOR is mcu, safety, ta or imu "
                         "(defaults: " + ", ".join(f"{k}={lo:g}:{hi:g}"
                                                   for k, (lo, hi) in TEMP_LIMITS_C.items()) + ")")
    ap.add_argument("--fan-speed", type=int, default=100, choices=range(0, 101),
                    metavar="0-100", help="fan duty for the fan check (default: 100)")
    ap.add_argument("--fan-min-rpm", type=int, default=4000,
                    help="min passing RPM per fan (default: 4000, the test app's limit)")
    ap.add_argument("--skip-fpga-write", action="store_true",
                    help="skip the FPGA refused-write checks")
    ap.add_argument("--skip-nvcm", action="store_true", help="skip the camera NVCM checks")
    ap.add_argument("--skip-capture", action="store_true",
                    help="skip the camera frame-capture checks")
    ap.add_argument("--include-histograms", action="store_true",
                    help="store each captured 1024-bin histogram in the result")
    ap.add_argument("--verbose", action="store_true", help="show SDK debug logging")
    args = ap.parse_args(argv)
    args.temp_limits = dict(TEMP_LIMITS_C)
    for spec in args.temp_limit:
        m = re.fullmatch(r"(mcu|safety|ta|imu)=([-\d.]+):([-\d.]+)", spec.strip())
        if not m:
            ap.error(f"--temp-limit {spec!r}: expected SENSOR=MIN:MAX")
        args.temp_limits[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.CRITICAL,
                        format="%(levelname)s %(name)s: %(message)s")

    events = sys.stdout
    if args.progress == "jsonl":
        sys.stdout = sys.stderr  # keep stray SDK prints off the event stream
        progress = JsonlProgress(events)
    else:
        progress = TextProgress(events)

    sensors = list(dict.fromkeys(args.sensors))
    wanted = ([] if args.no_console else ["console"]) + sensors
    if not wanted:
        progress.error("nothing to test (--no-console with no --sensors)")
        return 2
    output = args.output or f"hardware_test_{_dt.datetime.now():%Y%m%d_%H%M%S}.json"

    started_at, t0 = _now(), time.perf_counter()
    try:
        from omotion import MotionInterface
        iface = MotionInterface()
        iface.start(wait=True, wait_timeout=args.timeout)
    except Exception as exc:
        progress.error(f"could not start MotionInterface: {exc}")
        return 2

    modules = {name: {"connected": None, "info": {}, "tests": []} for name in wanted}
    checks: list[Check] = []
    if "console" in modules:
        checks += console_checks(iface.console, modules["console"]["info"], args)
    for side in sensors:
        checks += sensor_checks(side, getattr(iface, side), modules[side]["info"], args)
    try:
        run_checks(checks, modules, progress)
    finally:
        iface.stop()

    report = build_report(modules, args, started_at, time.perf_counter() - t0,
                          MotionInterface.get_sdk_version())
    with open(output, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    progress.done(report, os.path.abspath(output))
    return 0 if report["summary"]["result"] == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
