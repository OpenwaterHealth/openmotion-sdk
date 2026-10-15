"""Host-side mirror of the console trigger limits and refusal codes.

Console firmware (console-fw NEW-1, NEW-2, CFW-05, CFW-06) rejects an
oversized frame with OW_NAK, a trigger JSON over 511 bytes or outside the
operating limits with OW_ERROR, and a refused START_TRIG with OW_ERROR plus a
status byte. The SDK checks the same limits before sending and decodes the
status byte. No hardware needed.
"""

import json
from types import SimpleNamespace

import pytest

from omotion.MotionConsole import MotionConsole
from omotion.MotionUart import MotionUart
from omotion.CommandError import CommandError
from omotion.config import (
    CONSOLE_MAX_PAYLOAD,
    DEFAULT_TRIGGER_CONFIG,
    OW_ERROR,
    OW_RESP,
    TRIG_INTERLOCK_CONFIG,
    TRIG_INTERLOCK_LASER_SAFETY,
    TRIG_INTERLOCK_SAFETY_UNKNOWN,
    TRIG_INTERLOCK_TEC,
    TRIG_START_TIMER_FAILED,
    TRIGGER_JSON_MAX,
    describe_trigger_refusal,
    validate_trigger_config,
)

pytestmark = pytest.mark.unit


def _cfg(**over):
    d = dict(DEFAULT_TRIGGER_CONFIG)
    d.update(over)
    return d


# --------------------------------------------------------------------------
# validate_trigger_config: same verdicts as trigger_validate in the firmware
# --------------------------------------------------------------------------
@pytest.mark.parametrize(
    "over",
    [
        {},
        {"TriggerFrequencyHz": 1},
        {"TriggerFrequencyHz": 30},
        {"TriggerFrequencyHz": 40},
        {"TriggerFrequencyHz": 60},
        {"TriggerFrequencyHz": 100},
        {"LaserPulseWidthUsec": 20},
        {"LaserPulseWidthUsec": 2200},
        {"TriggerPulseWidthUsec": 24999},
        {"LaserPulseDelayUsec": 100, "LaserPulseWidthUsec": 2200, "LaserPulseSkipDelayUsec": 22699},
        {"TriggerFrequencyHz": 30, "LaserPulseSkipDelayUsec": 32700},
        {"TriggerFrequencyHz": 1, "LaserPulseSkipDelayUsec": 999000},
        {"TriggerFrequencyHz": 100, "LaserPulseDelayUsec": 100, "LaserPulseWidthUsec": 2200,
         "LaserPulseSkipDelayUsec": 7699},
        {"TriggerFrequencyHz": 60, "LaserPulseDelayUsec": 100, "LaserPulseWidthUsec": 2200,
         "LaserPulseSkipDelayUsec": 14366},
    ],
)
def test_valid_configs_pass(over):
    validate_trigger_config(_cfg(**over))


@pytest.mark.parametrize(
    ("over", "field"),
    [
        ({"TriggerFrequencyHz": 101}, "TriggerFrequencyHz"),
        ({"TriggerFrequencyHz": 100.01}, "TriggerFrequencyHz"),
        ({"TriggerFrequencyHz": 0.99}, "TriggerFrequencyHz"),
        ({"TriggerFrequencyHz": 0.5}, "TriggerFrequencyHz"),
        ({"TriggerFrequencyHz": 0}, "TriggerFrequencyHz"),
        ({"TriggerFrequencyHz": float("nan")}, "TriggerFrequencyHz"),
        ({"TriggerFrequencyHz": "40"}, "TriggerFrequencyHz"),
        ({"LaserPulseWidthUsec": 19}, "LaserPulseWidthUsec"),
        ({"LaserPulseWidthUsec": 2201}, "LaserPulseWidthUsec"),
        ({"LaserPulseWidthUsec": -1}, "LaserPulseWidthUsec"),
        ({"LaserPulseWidthUsec": 500.5}, "LaserPulseWidthUsec"),
        ({"TriggerPulseWidthUsec": 0}, "TriggerPulseWidthUsec"),
        ({"TriggerPulseWidthUsec": 25000}, "TriggerPulseWidthUsec"),
        (
            {"LaserPulseDelayUsec": 100, "LaserPulseWidthUsec": 2200, "LaserPulseSkipDelayUsec": 22700},
            "LaserPulseSkipDelayUsec",
        ),
        (
            {"TriggerFrequencyHz": 60, "LaserPulseDelayUsec": 100, "LaserPulseWidthUsec": 2200,
             "LaserPulseSkipDelayUsec": 14367},
            "LaserPulseSkipDelayUsec",
        ),
        ({"TriggerFrequencyHz": 60, "TriggerPulseWidthUsec": 16667}, "TriggerPulseWidthUsec"),
        (
            {"TriggerFrequencyHz": 100, "LaserPulseDelayUsec": 100, "LaserPulseWidthUsec": 2200,
             "LaserPulseSkipDelayUsec": 7700},
            "LaserPulseSkipDelayUsec",
        ),
        ({"LaserPulseDelayUsec": 0xFFFFFFFF}, "LaserPulseDelayUsec"),
        ({"LaserPulseSkipDelayUsec": 0x1_0000_0000}, "LaserPulseSkipDelayUsec"),
    ],
)
def test_invalid_configs_name_the_field(over, field):
    with pytest.raises(ValueError, match=field):
        validate_trigger_config(_cfg(**over))


def test_partial_config_checks_only_the_fields_present():
    # The console merges a partial dict onto its current config.
    validate_trigger_config({"TriggerStatus": 1})
    validate_trigger_config({"LaserPulseDelayUsec": 30000})  # no frequency: no frame check
    with pytest.raises(ValueError, match="LaserPulseWidthUsec"):
        validate_trigger_config({"LaserPulseWidthUsec": 5000})


# --------------------------------------------------------------------------
# START_TRIG refusal status byte
# --------------------------------------------------------------------------
def test_describe_trigger_refusal():
    assert "TEC" in describe_trigger_refusal(TRIG_INTERLOCK_TEC)
    both = describe_trigger_refusal(TRIG_INTERLOCK_TEC | TRIG_INTERLOCK_LASER_SAFETY)
    assert "TEC" in both and "laser-safety" in both
    assert "timer" in describe_trigger_refusal(TRIG_START_TIMER_FAILED)
    assert "config invalid or missing" in describe_trigger_refusal(TRIG_INTERLOCK_CONFIG)
    assert "unknown" not in describe_trigger_refusal(TRIG_INTERLOCK_CONFIG | TRIG_INTERLOCK_TEC)
    assert "unreadable" in describe_trigger_refusal(TRIG_INTERLOCK_SAFETY_UNKNOWN)
    assert "0x20" in describe_trigger_refusal(0x20)
    assert "no status byte" in describe_trigger_refusal(None)


# --------------------------------------------------------------------------
# MotionConsole with a stub transport
# --------------------------------------------------------------------------
class _StubUart:
    demo_mode = False

    def __init__(self, response):
        self.response = response
        self.sent = []

    def send_packet(self, **kwargs):
        self.sent.append(kwargs)
        return self.response

    def clear_buffer(self):
        pass


def _console(response):
    console = MotionConsole(vid=0x0483, pid=0xA53E)
    console.uart = _StubUart(response)
    console.is_connected = lambda: True
    return console


def _resp(packet_type, data=b""):
    return SimpleNamespace(packetType=packet_type, data=data, data_len=len(data))


def test_set_trigger_json_sends_compact_json():
    echo = json.dumps(DEFAULT_TRIGGER_CONFIG).encode()
    console = _console(_resp(OW_RESP, echo))
    assert console.set_trigger_json(DEFAULT_TRIGGER_CONFIG) == DEFAULT_TRIGGER_CONFIG
    payload = console.uart.sent[0]["data"]
    assert b", " not in payload and b": " not in payload
    assert json.loads(payload) == DEFAULT_TRIGGER_CONFIG


def test_set_trigger_json_refuses_out_of_range_before_sending():
    console = _console(_resp(OW_RESP))
    with pytest.raises(ValueError, match="TriggerFrequencyHz"):
        console.set_trigger_json(_cfg(TriggerFrequencyHz=101))
    assert console.uart.sent == []


def test_set_trigger_json_refuses_oversized_json_before_sending():
    console = _console(_resp(OW_RESP))
    big = _cfg(Comment="x" * TRIGGER_JSON_MAX)
    with pytest.raises(ValueError, match=str(TRIGGER_JSON_MAX)):
        console.set_trigger_json(big)
    assert console.uart.sent == []


def test_set_trigger_json_returns_none_when_the_console_rejects():
    console = _console(_resp(OW_ERROR))
    assert console.set_trigger_json(DEFAULT_TRIGGER_CONFIG) is None


@pytest.mark.parametrize(
    ("data", "status", "fragment"),
    [
        (bytes([TRIG_INTERLOCK_TEC]), TRIG_INTERLOCK_TEC, "TEC"),
        (bytes([TRIG_START_TIMER_FAILED]), TRIG_START_TIMER_FAILED, "timer"),
        (b"", None, "no status byte"),
    ],
)
def test_start_trigger_refusal_is_decoded(data, status, fragment):
    console = _console(_resp(OW_ERROR, data))
    assert console.start_trigger() is False
    assert console.last_start_refusal == status
    assert fragment in console.last_start_refusal_reason


def test_start_trigger_success_clears_the_refusal():
    console = _console(_resp(OW_ERROR, bytes([TRIG_INTERLOCK_TEC])))
    console.start_trigger()
    console.uart.response = _resp(OW_RESP)
    assert console.start_trigger() is True
    assert console.last_start_refusal is None
    assert console.last_start_refusal_reason is None


# --------------------------------------------------------------------------
# MotionUart payload limit
# --------------------------------------------------------------------------
def test_send_packet_refuses_payload_over_the_console_limit():
    uart = MotionUart(vid=0x0483, pid=0xA53E, demo_mode=True)
    sent = []
    uart._tx = sent.append
    with pytest.raises(CommandError, match=str(CONSOLE_MAX_PAYLOAD)):
        uart.send_packet(data=bytes(CONSOLE_MAX_PAYLOAD + 1))
    assert sent == []
