"""Unit tests for the sensor reset-history record (openmotion-sensor-fw#137).

``parse_reset_history`` decodes the firmware's OW_CMD_RESET_HISTORY reply and
``MotionSensor.get_reset_history`` fetches it. No hardware: payloads are packed
here with the firmware's sysmon_reset_history_t layout.
"""
import struct
from types import SimpleNamespace
from unittest.mock import MagicMock

from omotion.MotionSensor import MotionSensor
from omotion.config import OW_CMD, OW_CMD_RESET_HISTORY, OW_RESP, OW_UNKNOWN
from omotion.reset_history import (
    describe_shutdown,
    format_reset_history,
    parse_reset_history,
)

# Mirrors sysmon_reset_history_t v1 in sensor-fw Core/Inc/system_monitor.h.
_FMT = "<BB2x3I8I7II4III"

POR, PIN, BOR, SFT, IWDG = 1 << 23, 1 << 22, 1 << 21, 1 << 24, 1 << 26


def _payload(
    last_shutdown=0, boot_count=1, prev_alive_ms=0, uptime_ms=5000,
    counts=(1, 0, 0, 0, 0, 0, 0, 0), rcc=(0,) * 7, rsr=0,
    ecc=(0, 0, 0, 0), dma=0, usb=0, version=1,
):
    return struct.pack(
        _FMT, version, last_shutdown, boot_count, prev_alive_ms, uptime_ms,
        *counts, *rcc, rsr, *ecc, dma, usb,
    )


def test_layout_is_104_bytes():
    # Must match sizeof(sysmon_reset_history_t) in the firmware.
    assert struct.calcsize(_FMT) == 104


def test_parses_every_field():
    h = parse_reset_history(_payload(
        last_shutdown=2, boot_count=3, prev_alive_ms=3_600_000, uptime_ms=12_345,
        counts=(1, 1, 1, 0, 0, 0, 0, 0), rcc=(1, 3, 1, 1, 0, 1, 0),
        rsr=PIN | SFT, ecc=(4, 0, 0x24001000, 9), dma=2, usb=1,
    ))
    assert h["version"] == 1
    assert h["last_shutdown"] == "host_reset"
    assert h["last_shutdown_code"] == 2
    assert h["boot_count"] == 3
    assert h["prev_alive_ms"] == 3_600_000
    assert h["uptime_ms"] == 12_345
    assert h["shutdown_counts"] == {
        "power_off": 1, "unexpected": 1, "host_reset": 1, "host_dfu": 0,
        "ecc": 0, "fault": 0, "error_handler": 0,
    }
    assert h["rcc_counts"] == {
        "por": 1, "pin": 3, "sft": 1, "iwdg": 1, "wwdg": 0, "bor": 1, "lpwr": 0,
    }
    assert h["last_rcc_rsr"] == PIN | SFT
    assert h["last_rcc_flags"] == ["PIN", "SFT"]
    assert (h["ecc_sbe_count"], h["ecc_dbe_count"]) == (4, 0)
    assert (h["ecc_last_addr"], h["ecc_last_monitor"]) == (0x24001000, 9)
    assert h["dma_err_count"] == 2
    assert h["usb_recover_count"] == 1


def test_old_firmware_and_garbage_parse_to_none():
    assert parse_reset_history(b"") is None
    assert parse_reset_history(None) is None
    assert parse_reset_history(_payload()[:50]) is None
    assert parse_reset_history(_payload(version=0)) is None


def test_appended_fields_still_parse():
    h = parse_reset_history(_payload(version=2) + b"\x00" * 12)
    assert h is not None and h["version"] == 2


def test_unknown_reason_code_is_kept_not_dropped():
    counts = (1, 0, 0, 0, 0, 0, 0, 1)  # spare slot 7 used by a newer firmware
    h = parse_reset_history(_payload(last_shutdown=7, boot_count=2, counts=counts))
    assert h["last_shutdown"] == "code_7"
    assert h["shutdown_counts"]["code_7"] == 1
    assert "code_7" in describe_shutdown(h)


def test_describe_power_off_says_nothing_about_the_previous_session():
    h = parse_reset_history(_payload(last_shutdown=0))
    assert describe_shutdown(h) == "power off (module was unpowered, or first boot)"


def test_describe_session_that_never_reached_the_main_loop():
    # The IWDG boot-loop signature: warm reset, no mark, no heartbeat.
    h = parse_reset_history(_payload(
        last_shutdown=1, boot_count=4, prev_alive_ms=0, counts=(1, 3, 0, 0, 0, 0, 0, 0),
    ))
    text = describe_shutdown(h)
    assert text.startswith("unexpected reset")
    assert "never reached the main loop" in text


def test_describe_reports_previous_session_length():
    h = parse_reset_history(_payload(last_shutdown=5, boot_count=2, prev_alive_ms=3_723_000))
    assert describe_shutdown(h) == (
        "CPU fault, then watchdog reset; the previous session ran 1h02m03s"
    )


def test_format_lines():
    h = parse_reset_history(_payload(
        last_shutdown=2, boot_count=2, prev_alive_ms=42_000, uptime_ms=8_500,
        counts=(1, 0, 1, 0, 0, 0, 0, 0),
    ))
    shutdown, boots, health = format_reset_history(h)
    assert shutdown == (
        "shutdown = host reset (OW_CMD_RESET); the previous session ran 42.0 s"
    )
    assert boots == "boots    = 2 since power-on (power_off 1, host_reset 1)"
    assert health.startswith("health   = uptime 8.5 s, usb recoveries 0, ")
    # RSR is 0 under the custom bootloader, which clears it before the app runs.
    assert health.endswith("rcc flags none (cleared by the bootloader)")


def test_format_shows_rcc_flags_when_present():
    h = parse_reset_history(_payload(rsr=POR | PIN | BOR))
    assert format_reset_history(h)[2].endswith("rcc flags PIN|POR|BOR")


# ---------------------------------------------------------------------------
# MotionSensor.get_reset_history
# ---------------------------------------------------------------------------

def _make_sensor(response=None, side_effect=None):
    s = MotionSensor.__new__(MotionSensor)
    s._send = MagicMock(return_value=response, side_effect=side_effect)
    s.demo_mode = False
    s.name = "left"
    return s


def _resp(data=b"", packet_type=OW_RESP):
    data = bytes(data)
    return SimpleNamespace(packetType=packet_type, data=data, data_len=len(data))


def test_get_reset_history_sends_the_command_and_parses():
    sensor = _make_sensor(_resp(_payload(last_shutdown=3, boot_count=2)))
    h = sensor.get_reset_history()
    assert h["last_shutdown"] == "host_dfu"
    kwargs = sensor._send.call_args.kwargs
    assert kwargs["packetType"] == OW_CMD
    assert kwargs["command"] == OW_CMD_RESET_HISTORY == 0x10


def test_get_reset_history_old_firmware_is_none():
    """Firmware without 0x10 replies OW_UNKNOWN: None, not an exception."""
    sensor = _make_sensor(_resp(b"", packet_type=OW_UNKNOWN))
    assert sensor.get_reset_history() is None


def test_get_reset_history_no_response_or_send_error_is_none():
    assert _make_sensor(None).get_reset_history() is None
    assert _make_sensor(side_effect=TimeoutError("no reply")).get_reset_history() is None


def test_get_reset_history_demo_mode_stays_off_the_wire():
    s = _make_sensor(side_effect=AssertionError("must not hit the wire in demo mode"))
    s.demo_mode = True
    assert s.get_reset_history() is None
