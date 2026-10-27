"""Integration tests for console calibration read/write — no hardware.

Uses unittest.mock to patch MotionConsole.read_config / write_config so
we don't need a connected console.
"""

from unittest.mock import MagicMock

import numpy as np
import pytest

from omotion.Calibration import (
    CALIBRATION_JSON_KEY,
    Calibration,
)
from omotion.MotionConfig import MotionConfig
from omotion.MotionConsole import MotionConsole


# A demo-mode console gives us a real instance without a serial port.
@pytest.fixture
def console():
    return MotionConsole(vid=0, pid=0, baudrate=921600, timeout=1, demo_mode=True)


def _valid_calibration_dict():
    return {
        CALIBRATION_JSON_KEY: {
            "C_min": [[0.0]*8, [0.0]*8],
            "C_max": [[0.5]*8, [0.5]*8],
            "I_min": [[0.0]*8, [0.0]*8],
            "I_max": [[300.0]*8, [300.0]*8],
        }
    }


def test_read_calibration_raises_when_read_config_returns_none(console):
    """A failed read is not "no calibration on device": returning defaults
    here let a partial calibration run write SDK defaults over the other
    module's stored values whenever the console momentarily failed to
    answer."""
    console.read_config = MagicMock(return_value=None)
    with pytest.raises(RuntimeError, match="could not be read"):
        console.read_calibration()


def test_read_calibration_propagates_transport_errors(console):
    console.read_config = MagicMock(side_effect=OSError("uart gone"))
    with pytest.raises(OSError, match="uart gone"):
        console.read_calibration()


def _wire_bytes(json_data: dict) -> bytes:
    return MotionConfig(json_data=json_data).to_wire_bytes()


def _console_answering(console, payload: bytes) -> MotionConsole:
    """Route read_config through a mocked UART that answers with payload."""
    from types import SimpleNamespace
    from omotion.config import OW_RESP
    console.uart.demo_mode = False
    console.is_connected = MagicMock(return_value=True)
    console.uart.send_packet = MagicMock(
        return_value=SimpleNamespace(packetType=OW_RESP, data=payload))
    console.uart.clear_buffer = MagicMock()
    return console


def test_from_wire_bytes_rejects_undecodable_json():
    """A truncated payload used to parse as an EMPTY config, which made
    every stored key — the calibration block included — look absent."""
    full = _wire_bytes(_valid_calibration_dict())
    truncated = full[: len(full) - 40]
    with pytest.raises(ValueError, match="not decodable"):
        MotionConfig.from_wire_bytes(truncated)


def test_from_wire_bytes_accepts_empty_payload():
    """json_len == 0 is a legitimately empty (never written) config."""
    cfg = MotionConfig.from_wire_bytes(_wire_bytes({}))
    assert cfg.json_data == {}


def test_read_config_returns_none_for_truncated_payload(console):
    full = _wire_bytes(_valid_calibration_dict())
    _console_answering(console, full[: len(full) - 40])
    assert console.read_config() is None


def test_read_calibration_raises_for_truncated_payload(console):
    full = _wire_bytes(_valid_calibration_dict())
    _console_answering(console, full[: len(full) - 40])
    with pytest.raises(RuntimeError, match="could not be read"):
        console.read_calibration()


def test_read_calibration_round_trips_through_wire_format(console):
    _console_answering(console, _wire_bytes(_valid_calibration_dict()))
    cal = console.read_calibration()
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.5))


def test_read_calibration_returns_defaults_when_block_absent(console):
    cfg = MotionConfig(json_data={"EE_THRESH": [1, 2, 3]})  # no calibration key
    console.read_config = MagicMock(return_value=cfg)
    cal = console.read_calibration()
    assert cal.source == "default"


def test_read_calibration_returns_console_when_valid(console):
    cfg = MotionConfig(json_data=_valid_calibration_dict())
    console.read_config = MagicMock(return_value=cfg)
    cal = console.read_calibration()
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.5))
    np.testing.assert_array_equal(cal.i_max, np.full((2, 8), 300.0))


def test_read_calibration_falls_back_when_block_malformed(console, caplog):
    bad = _valid_calibration_dict()
    bad[CALIBRATION_JSON_KEY]["C_max"] = [[0.0]*8, [0.0]*8]  # not > C_min
    cfg = MotionConfig(json_data=bad)
    console.read_config = MagicMock(return_value=cfg)
    with caplog.at_level("WARNING"):
        cal = console.read_calibration()
    assert cal.source == "default"
    assert any("monotonic" in rec.message.lower() or "greater" in rec.message.lower()
               for rec in caplog.records)
    # ... and the fallback is loud, unlike the "no block on device" case.
    assert any("invalid" in rec.message.lower() and rec.levelname == "WARNING"
               for rec in caplog.records)


# ----- write_calibration -----

def test_write_calibration_rejects_bad_shape_before_wire(console):
    console.read_config = MagicMock()
    console.write_config = MagicMock()
    with pytest.raises(ValueError, match="shape"):
        console.write_calibration(
            np.zeros((2, 7)), np.full((2, 7), 0.5),
            np.zeros((2, 7)), np.full((2, 7), 250.0),
        )
    console.read_config.assert_not_called()
    console.write_config.assert_not_called()


def test_write_calibration_preserves_other_keys(console):
    existing = MotionConfig(json_data={
        "EE_THRESH": [1, 2, 3],
        "OPT_GAIN": [4, 5, 6],
    })
    console.read_config = MagicMock(return_value=existing)
    captured = {}
    def _capture_write(cfg):
        captured["cfg"] = cfg
        return cfg
    console.write_config = MagicMock(side_effect=_capture_write)

    console.write_calibration(
        np.zeros((2, 8)), np.full((2, 8), 0.5),
        np.zeros((2, 8)), np.full((2, 8), 250.0),
    )

    written = captured["cfg"].json_data
    assert written["EE_THRESH"] == [1, 2, 3]
    assert written["OPT_GAIN"] == [4, 5, 6]
    assert CALIBRATION_JSON_KEY in written
    assert written[CALIBRATION_JSON_KEY]["C_max"][0][0] == 0.5


def test_write_calibration_returns_console_source(console):
    console.read_config = MagicMock(return_value=MotionConfig(json_data={}))
    console.write_config = MagicMock(side_effect=lambda cfg: cfg)
    cal = console.write_calibration(
        np.zeros((2, 8)), np.full((2, 8), 0.5),
        np.zeros((2, 8)), np.full((2, 8), 250.0),
    )
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.5))


def test_write_calibration_raises_when_read_config_returns_none(console):
    console.read_config = MagicMock(return_value=None)
    console.write_config = MagicMock()
    with pytest.raises(RuntimeError, match="read existing config"):
        console.write_calibration(
            np.zeros((2, 8)), np.full((2, 8), 0.5),
            np.zeros((2, 8)), np.full((2, 8), 250.0),
        )
    console.write_config.assert_not_called()


# ----- MotionInterface facade -----

from omotion.MotionInterface import MotionInterface


@pytest.fixture
def interface():
    return MotionInterface(demo_mode=True)


def test_get_calibration_returns_default_before_load(interface):
    cal = interface.get_calibration()
    assert cal.source == "default"
    assert cal.c_max.shape == (2, 8)


def test_refresh_calibration_pulls_from_console(interface):
    cfg = MotionConfig(json_data=_valid_calibration_dict())
    interface.console.read_config = MagicMock(return_value=cfg)
    cal = interface.refresh_calibration()
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.5))
    # And the cache reflects the same.
    assert interface.get_calibration().source == "console"


def test_log_console_info_loads_calibration(interface):
    cfg = MotionConfig(json_data=_valid_calibration_dict())
    interface.console.read_config = MagicMock(return_value=cfg)
    interface.console.is_connected = MagicMock(return_value=True)
    interface.console.log_device_info = MagicMock()
    interface.log_console_info()
    assert interface.get_calibration().source == "console"


def test_log_console_info_skips_load_when_console_disconnected(interface):
    interface.console.is_connected = MagicMock(return_value=False)
    interface.console.read_config = MagicMock()
    interface.log_console_info()
    interface.console.read_config.assert_not_called()
    assert interface.get_calibration().source == "default"


def test_write_calibration_refreshes_cache(interface):
    # Read returns empty initially; subsequent read returns the just-written
    # values so the read-back simulation matches reality.
    written_holder: dict = {}

    def _read_config():
        if "cfg" not in written_holder:
            return MotionConfig(json_data={"EE_THRESH": [9]})
        return written_holder["cfg"]

    def _write_config(cfg):
        written_holder["cfg"] = cfg
        return cfg

    interface.console.read_config = MagicMock(side_effect=_read_config)
    interface.console.write_config = MagicMock(side_effect=_write_config)

    cal = interface.write_calibration(
        np.zeros((2, 8)), np.full((2, 8), 0.42),
        np.zeros((2, 8)), np.full((2, 8), 275.0),
    )
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.42))
    assert interface.get_calibration().source == "console"
    np.testing.assert_array_equal(
        interface.get_calibration().c_max, np.full((2, 8), 0.42)
    )


def test_set_realtime_calibration_marks_override(interface):
    interface.scan_workflow.set_realtime_calibration(
        np.zeros((2, 8)), np.full((2, 8), 0.6),
        np.zeros((2, 8)), np.full((2, 8), 999.0),
    )
    cal = interface.get_calibration()
    assert cal.source == "override"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.6))


# ----- a failed read never downgrades a loaded cache -----

def _load_console_calibration(interface, json_data):
    interface.console.read_config = MagicMock(
        return_value=MotionConfig(json_data=json_data))
    interface.console.is_connected = MagicMock(return_value=True)
    interface.console.log_device_info = MagicMock()
    interface.log_console_info()
    assert interface.get_calibration().source == "console"


def test_refresh_calibration_raises_and_keeps_cache_on_read_failure(interface):
    _load_console_calibration(interface, _valid_calibration_dict())
    interface.console.read_config = MagicMock(return_value=None)
    with pytest.raises(RuntimeError):
        interface.refresh_calibration()
    cal = interface.get_calibration()
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.5))


def test_log_console_info_keeps_cache_on_read_failure(interface):
    """The best-effort connect-time load (the app calls this on every
    console CONNECTED transition) must not replace a loaded calibration
    with SDK defaults when one read fails — that reset was what a later
    right-only calibration run then wrote to the EEPROM."""
    _load_console_calibration(interface, _valid_calibration_dict())
    interface.console.read_config = MagicMock(return_value=None)
    interface.log_console_info()            # no raise: best-effort
    cal = interface.get_calibration()
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.5))


def test_log_console_info_keeps_cache_on_truncated_payload(interface):
    _load_console_calibration(interface, _valid_calibration_dict())
    full = MotionConfig(json_data=_valid_calibration_dict()).to_wire_bytes()
    _console_answering(interface.console, full[: len(full) - 40])
    interface.log_console_info()
    assert interface.get_calibration().source == "console"

