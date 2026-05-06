# Console-EEPROM-driven calibration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the SDK read `C_min`, `C_max`, `I_min`, `I_max` calibration arrays from the console's EEPROM JSON on connect, fall back to SDK-owned defaults when absent or invalid, expose read/refresh/write APIs on `MotionInterface`, and remove the bloodflow app's `set_realtime_calibration` bootstrap.

**Architecture:** A new `omotion/Calibration.py` module owns the four `(2, 8)` default arrays (verbatim copies of the values currently in `openmotion-bloodflow-app/processing/visualize_bloodflow.py`), a frozen `Calibration` dataclass `(c_min, c_max, i_min, i_max, source)`, and pure parser/serializer functions. `MotionConsole` gains `read_calibration()` and `write_calibration(...)`. `ScanWorkflow` replaces its four nullable BFI arrays with a single `Calibration` field, seeded with defaults. `MotionInterface.log_console_info()` triggers the connect-time load; `refresh_calibration()` / `get_calibration()` / `write_calibration()` are added to the facade. The bloodflow app's two-line bootstrap is removed.

**Tech Stack:** Python 3.12, numpy, pytest, pyserial (UART transport already exists). No new dependencies.

---

## Spec

`docs/superpowers/specs/2026-05-01-console-eeprom-calibration-design.md`

## File Structure

**Create:**
- `openmotion-sdk/omotion/Calibration.py` — defaults, `Calibration` dataclass, `parse_calibration`, `serialize_calibration`. ~150 lines.
- `openmotion-sdk/tests/test_calibration.py` — pure unit tests for the new module (parser, serializer, defaults). No hardware, no UART.
- `openmotion-sdk/tests/test_calibration_console.py` — integration tests using mocked `read_config`/`write_config`. No hardware required.

**Modify:**
- `openmotion-sdk/omotion/__init__.py` — export `Calibration`.
- `openmotion-sdk/omotion/MotionConsole.py` — add `read_calibration()` and `write_calibration(...)` methods near the existing `read_config` / `write_config`.
- `openmotion-sdk/omotion/ScanWorkflow.py` — replace four `_bfi_c_min/_bfi_c_max/_bfi_i_min/_bfi_i_max` fields with one `_calibration: Calibration`; add `_install_calibration`; tighten `set_realtime_calibration`; update `start_scan` consumption site.
- `openmotion-sdk/omotion/MotionInterface.py` — add `_load_calibration_from_console`, `refresh_calibration`, `get_calibration`, `write_calibration`; hook the loader into `log_console_info`.
- `openmotion-bloodflow-app/motion_connector.py` — delete `_BFI_CAL` instance and `_BFI_C_MIN/_BFI_C_MAX/_BFI_I_MIN/_BFI_I_MAX` module globals; delete the `self._scan_workflow.set_realtime_calibration(...)` call.

**Untouched:**
- `openmotion-bloodflow-app/processing/visualize_bloodflow.py` — its dataclass defaults remain for offline CSV analysis.

---

## Task 1: New `Calibration` module — defaults, dataclass, factory

**Files:**
- Create: `openmotion-sdk/omotion/Calibration.py`
- Test: `openmotion-sdk/tests/test_calibration.py`

- [ ] **Step 1: Write failing tests for defaults and `Calibration.default()`**

Create `openmotion-sdk/tests/test_calibration.py`:

```python
"""Unit tests for omotion.Calibration (no hardware required)."""

import numpy as np
import pytest

from omotion.Calibration import (
    Calibration,
    CALIBRATION_JSON_KEY,
    parse_calibration,
    serialize_calibration,
)


# Reference defaults — copied verbatim from
# openmotion-bloodflow-app/processing/visualize_bloodflow.py as of 2026-05-01.
# This is the golden test that pins the SDK defaults to the values the
# bloodflow app has been using.
_REF_C_MIN = np.zeros((2, 8), dtype=float)
_REF_C_MAX = np.array(
    [[0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4],
     [0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4]],
    dtype=float,
)
_REF_I_MIN = np.zeros((2, 8), dtype=float)
_REF_I_MAX = np.array(
    [[150, 300, 300, 300, 300, 300, 300, 150],
     [150, 300, 300, 300, 300, 300, 300, 150]],
    dtype=float,
)


def test_default_values_match_visualize_bloodflow_defaults():
    cal = Calibration.default()
    np.testing.assert_array_equal(cal.c_min, _REF_C_MIN)
    np.testing.assert_array_equal(cal.c_max, _REF_C_MAX)
    np.testing.assert_array_equal(cal.i_min, _REF_I_MIN)
    np.testing.assert_array_equal(cal.i_max, _REF_I_MAX)


def test_default_source_label():
    cal = Calibration.default()
    assert cal.source == "default"


def test_default_returns_independent_copies():
    a = Calibration.default()
    b = Calibration.default()
    a.c_max[0, 0] = 999.0
    # Mutating one default must not bleed into the next call.
    assert b.c_max[0, 0] == _REF_C_MAX[0, 0]


def test_default_arrays_have_correct_shape_and_dtype():
    cal = Calibration.default()
    for arr in (cal.c_min, cal.c_max, cal.i_min, cal.i_max):
        assert arr.shape == (2, 8)
        assert arr.dtype == np.float64
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py -v`
Expected: ImportError / ModuleNotFoundError on `omotion.Calibration`.

- [ ] **Step 3: Create the module skeleton with defaults and `Calibration`**

Create `openmotion-sdk/omotion/Calibration.py`:

```python
"""Calibration arrays for the BFI/BVI science pipeline.

The four arrays — ``C_min``, ``C_max``, ``I_min``, ``I_max`` — are stored
on the console's EEPROM JSON config under the ``"calibration"`` key. When
the SDK connects to a console it tries to load them; if they are missing
or fail validation it falls back to the defaults defined here.

Defaults are a verbatim copy of the values in
``openmotion-bloodflow-app/processing/visualize_bloodflow.py`` as of
2026-05-01 (the values the app has been shipping with).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Literal, Optional

import numpy as np

from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.Calibration" if _log_root else "Calibration"
)

# JSON keys
CALIBRATION_JSON_KEY = "calibration"
_C_MIN_KEY = "C_min"
_C_MAX_KEY = "C_max"
_I_MIN_KEY = "I_min"
_I_MAX_KEY = "I_max"
_ALL_ARRAY_KEYS = (_C_MIN_KEY, _C_MAX_KEY, _I_MIN_KEY, _I_MAX_KEY)

# Required shape — (modules, cams_per_module).
_EXPECTED_SHAPE = (2, 8)

# Defaults — copied verbatim from
# openmotion-bloodflow-app/processing/visualize_bloodflow.py.
_DEFAULT_C_MIN = np.zeros(_EXPECTED_SHAPE, dtype=float)
_DEFAULT_C_MAX = np.array(
    [[0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4],
     [0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4]],
    dtype=float,
)
_DEFAULT_I_MIN = np.zeros(_EXPECTED_SHAPE, dtype=float)
_DEFAULT_I_MAX = np.array(
    [[150, 300, 300, 300, 300, 300, 300, 150],
     [150, 300, 300, 300, 300, 300, 300, 150]],
    dtype=float,
)

CalibrationSource = Literal["console", "default", "override"]


@dataclass(frozen=True)
class Calibration:
    """Resolved BFI/BVI calibration with provenance.

    All four arrays are shape ``(2, 8)`` float64. ``source`` tells callers
    where the values came from:

    - ``"console"``: parsed from the device EEPROM JSON.
    - ``"default"``: SDK-owned defaults (no console JSON, or invalid).
    - ``"override"``: supplied directly via
      :meth:`omotion.ScanWorkflow.set_realtime_calibration`.
    """

    c_min: np.ndarray
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray
    source: CalibrationSource

    @classmethod
    def default(cls) -> "Calibration":
        """Return a fresh ``Calibration`` populated with SDK defaults.

        Each call returns independent array copies so mutating one
        instance never bleeds into another.
        """
        return cls(
            c_min=_DEFAULT_C_MIN.copy(),
            c_max=_DEFAULT_C_MAX.copy(),
            i_min=_DEFAULT_I_MIN.copy(),
            i_max=_DEFAULT_I_MAX.copy(),
            source="default",
        )


def parse_calibration(json_data: dict) -> Optional[Calibration]:
    """Parse a console JSON config dict into a Calibration or None.

    Returns ``None`` (not raises) when the calibration block is absent or
    invalid. Callers fall back to ``Calibration.default()``.
    """
    raise NotImplementedError  # filled in Task 2


def serialize_calibration(c_min, c_max, i_min, i_max) -> dict:
    """Return a ``{"calibration": {...}}`` dict ready to merge into the
    console JSON. Validates inputs and raises ``ValueError`` on bad data.
    """
    raise NotImplementedError  # filled in Task 3
```

- [ ] **Step 4: Run tests to verify defaults pass**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py -v -k default`
Expected: 4 passed (golden values, source label, copies, shape/dtype).

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/Calibration.py openmotion-sdk/tests/test_calibration.py
git commit -m "feat(sdk): add Calibration module with SDK-owned defaults"
```

---

## Task 2: `parse_calibration` — validation and parsing

**Files:**
- Modify: `openmotion-sdk/omotion/Calibration.py`
- Test: `openmotion-sdk/tests/test_calibration.py`

- [ ] **Step 1: Write failing tests for `parse_calibration`**

Append to `openmotion-sdk/tests/test_calibration.py`:

```python
# ----- parse_calibration -----

def _valid_block():
    return {
        CALIBRATION_JSON_KEY: {
            "C_min": [[0.0]*8, [0.0]*8],
            "C_max": [[0.4]*8, [0.4]*8],
            "I_min": [[0.0]*8, [0.0]*8],
            "I_max": [[200.0]*8, [200.0]*8],
        }
    }


def test_parse_valid_returns_console_source():
    cal = parse_calibration(_valid_block())
    assert cal is not None
    assert cal.source == "console"
    np.testing.assert_array_equal(cal.c_max, np.full((2, 8), 0.4))
    np.testing.assert_array_equal(cal.i_max, np.full((2, 8), 200.0))


def test_parse_missing_block_returns_none():
    assert parse_calibration({}) is None
    assert parse_calibration({"some_other_key": 1}) is None


def test_parse_block_not_a_dict_returns_none():
    assert parse_calibration({CALIBRATION_JSON_KEY: "not-a-dict"}) is None
    assert parse_calibration({CALIBRATION_JSON_KEY: [1, 2, 3]}) is None


def test_parse_missing_one_subkey_returns_none(caplog):
    blk = _valid_block()
    del blk[CALIBRATION_JSON_KEY]["I_max"]
    with caplog.at_level("WARNING"):
        assert parse_calibration(blk) is None
    assert any("I_max" in rec.message for rec in caplog.records)


@pytest.mark.parametrize("bad_shape", [
    [[0.0]*8],                                # (1, 8)
    [[0.0]*7, [0.0]*7],                       # (2, 7)
    [[0.0]*8, [0.0]*8, [0.0]*8],              # (3, 8)
    0.0,                                       # scalar
    [0.0, 0.0],                                # 1-D
])
def test_parse_wrong_shape_returns_none(bad_shape):
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_min"] = bad_shape
    assert parse_calibration(blk) is None


def test_parse_non_numeric_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_min"] = [["abc"]*8, ["abc"]*8]
    assert parse_calibration(blk) is None


def test_parse_nan_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_min"][0][0] = float("nan")
    assert parse_calibration(blk) is None


def test_parse_inf_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_max"][0][0] = float("inf")
    assert parse_calibration(blk) is None


def test_parse_non_monotonic_c_returns_none():
    # C_max must be > C_min element-wise
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["C_max"][1][3] = 0.0  # equal to C_min there
    assert parse_calibration(blk) is None


def test_parse_non_monotonic_i_returns_none():
    blk = _valid_block()
    blk[CALIBRATION_JSON_KEY]["I_max"][0][0] = -1.0  # less than I_min
    assert parse_calibration(blk) is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py -v -k parse`
Expected: NotImplementedError on every parser test.

- [ ] **Step 3: Implement `parse_calibration`**

Replace the `parse_calibration` placeholder in `openmotion-sdk/omotion/Calibration.py`:

```python
def parse_calibration(json_data: dict) -> Optional[Calibration]:
    if not isinstance(json_data, dict):
        return None

    block = json_data.get(CALIBRATION_JSON_KEY)
    if block is None:
        return None
    if not isinstance(block, dict):
        logger.warning(
            "Console calibration invalid (calibration block is %s, not a dict); "
            "falling back to SDK defaults.",
            type(block).__name__,
        )
        return None

    arrays: dict[str, np.ndarray] = {}
    for key in _ALL_ARRAY_KEYS:
        if key not in block:
            logger.warning(
                "Console calibration invalid (missing key %s); "
                "falling back to SDK defaults.",
                key,
            )
            return None
        try:
            arr = np.asarray(block[key], dtype=float)
        except (TypeError, ValueError):
            logger.warning(
                "Console calibration invalid (%s is non-numeric); "
                "falling back to SDK defaults.",
                key,
            )
            return None
        if arr.shape != _EXPECTED_SHAPE:
            logger.warning(
                "Console calibration invalid (%s has shape %s, expected %s); "
                "falling back to SDK defaults.",
                key, arr.shape, _EXPECTED_SHAPE,
            )
            return None
        if not np.all(np.isfinite(arr)):
            logger.warning(
                "Console calibration invalid (%s contains NaN or inf); "
                "falling back to SDK defaults.",
                key,
            )
            return None
        arrays[key] = arr

    if not np.all(arrays[_C_MAX_KEY] > arrays[_C_MIN_KEY]):
        logger.warning(
            "Console calibration invalid (C_max not strictly greater than "
            "C_min element-wise); falling back to SDK defaults."
        )
        return None
    if not np.all(arrays[_I_MAX_KEY] > arrays[_I_MIN_KEY]):
        logger.warning(
            "Console calibration invalid (I_max not strictly greater than "
            "I_min element-wise); falling back to SDK defaults."
        )
        return None

    return Calibration(
        c_min=arrays[_C_MIN_KEY],
        c_max=arrays[_C_MAX_KEY],
        i_min=arrays[_I_MIN_KEY],
        i_max=arrays[_I_MAX_KEY],
        source="console",
    )
```

- [ ] **Step 4: Run all calibration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py -v`
Expected: all parser tests + 4 default tests pass.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/Calibration.py openmotion-sdk/tests/test_calibration.py
git commit -m "feat(sdk): implement parse_calibration with all-or-nothing validation"
```

---

## Task 3: `serialize_calibration` — validation and JSON construction

**Files:**
- Modify: `openmotion-sdk/omotion/Calibration.py`
- Test: `openmotion-sdk/tests/test_calibration.py`

- [ ] **Step 1: Write failing tests for `serialize_calibration`**

Append to `openmotion-sdk/tests/test_calibration.py`:

```python
# ----- serialize_calibration -----

def test_serialize_round_trip():
    c_min = np.zeros((2, 8))
    c_max = np.full((2, 8), 0.5)
    i_min = np.zeros((2, 8))
    i_max = np.full((2, 8), 250.0)
    blob = serialize_calibration(c_min, c_max, i_min, i_max)
    assert CALIBRATION_JSON_KEY in blob
    cal = parse_calibration(blob)
    assert cal is not None
    np.testing.assert_array_equal(cal.c_min, c_min)
    np.testing.assert_array_equal(cal.c_max, c_max)
    np.testing.assert_array_equal(cal.i_min, i_min)
    np.testing.assert_array_equal(cal.i_max, i_max)


def test_serialize_emits_lists_not_ndarrays():
    blob = serialize_calibration(
        np.zeros((2, 8)), np.full((2, 8), 0.5),
        np.zeros((2, 8)), np.full((2, 8), 250.0),
    )
    inner = blob[CALIBRATION_JSON_KEY]
    for key in ("C_min", "C_max", "I_min", "I_max"):
        assert isinstance(inner[key], list)
        assert isinstance(inner[key][0], list)
        assert isinstance(inner[key][0][0], float)


def test_serialize_rejects_wrong_shape():
    with pytest.raises(ValueError, match="shape"):
        serialize_calibration(
            np.zeros((2, 7)), np.full((2, 7), 0.5),
            np.zeros((2, 7)), np.full((2, 7), 250.0),
        )


def test_serialize_rejects_nan():
    bad = np.zeros((2, 8))
    bad[0, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        serialize_calibration(bad, np.full((2, 8), 0.5),
                              np.zeros((2, 8)), np.full((2, 8), 250.0))


def test_serialize_rejects_non_monotonic():
    with pytest.raises(ValueError, match="monotonic|greater"):
        serialize_calibration(
            np.zeros((2, 8)), np.zeros((2, 8)),  # C_max == C_min
            np.zeros((2, 8)), np.full((2, 8), 250.0),
        )
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py -v -k serialize`
Expected: NotImplementedError on every serializer test.

- [ ] **Step 3: Implement `serialize_calibration`**

Replace the `serialize_calibration` placeholder in `openmotion-sdk/omotion/Calibration.py`:

```python
def serialize_calibration(c_min, c_max, i_min, i_max) -> dict:
    arrays = {
        _C_MIN_KEY: c_min,
        _C_MAX_KEY: c_max,
        _I_MIN_KEY: i_min,
        _I_MAX_KEY: i_max,
    }
    typed: dict[str, np.ndarray] = {}
    for key, val in arrays.items():
        try:
            arr = np.asarray(val, dtype=float)
        except (TypeError, ValueError) as e:
            raise ValueError(f"{key} is not numeric: {e}") from e
        if arr.shape != _EXPECTED_SHAPE:
            raise ValueError(
                f"{key} has shape {arr.shape}; expected {_EXPECTED_SHAPE}"
            )
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{key} contains non-finite values (NaN or inf)")
        typed[key] = arr

    if not np.all(typed[_C_MAX_KEY] > typed[_C_MIN_KEY]):
        raise ValueError(
            "C_max must be strictly greater than C_min element-wise (monotonic)"
        )
    if not np.all(typed[_I_MAX_KEY] > typed[_I_MIN_KEY]):
        raise ValueError(
            "I_max must be strictly greater than I_min element-wise (monotonic)"
        )

    return {
        CALIBRATION_JSON_KEY: {
            key: arr.tolist() for key, arr in typed.items()
        }
    }
```

- [ ] **Step 4: Run all calibration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py -v`
Expected: every test in the file passes (defaults + parser + serializer).

- [ ] **Step 5: Export `Calibration` from the package**

Modify `openmotion-sdk/omotion/__init__.py`:

Find the import block:
```python
from .MotionConfig import MotionConfig
from .connection_state import ConnectionState
```

Add after `MotionConfig`:
```python
from .Calibration import Calibration
```

Find the `__all__` list and add `"Calibration"`:
```python
__all__ = [
    "__version__",
    "set_log_root",
    "MotionInterface",
    "MotionConsole",
    "MotionSensor",
    "MotionUart",
    "MotionSignal",
    "MotionComposite",
    "USBInterfaceBase",
    "MotionConfig",
    "Calibration",
    "ConnectionState",
]
```

- [ ] **Step 6: Verify the export works**

Run: `cd openmotion-sdk && python -c "from omotion import Calibration; print(Calibration.default().c_max.shape)"`
Expected: `(2, 8)`

- [ ] **Step 7: Commit**

```bash
git add openmotion-sdk/omotion/Calibration.py openmotion-sdk/omotion/__init__.py openmotion-sdk/tests/test_calibration.py
git commit -m "feat(sdk): add serialize_calibration and export Calibration"
```

---

## Task 4: `MotionConsole.read_calibration()`

**Files:**
- Modify: `openmotion-sdk/omotion/MotionConsole.py`
- Test: `openmotion-sdk/tests/test_calibration_console.py`

- [ ] **Step 1: Write failing test for `read_calibration`**

Create `openmotion-sdk/tests/test_calibration_console.py`:

```python
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


def test_read_calibration_returns_defaults_when_read_config_returns_none(console):
    console.read_config = MagicMock(return_value=None)
    cal = console.read_calibration()
    assert cal.source == "default"
    assert cal.c_max.shape == (2, 8)


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_console.py -v -k read_calibration`
Expected: AttributeError — `MotionConsole` has no `read_calibration`.

- [ ] **Step 3: Add `read_calibration` to `MotionConsole`**

Modify `openmotion-sdk/omotion/MotionConsole.py`.

Find the import block near line 71:
```python
from omotion.MotionConfig import MotionConfig, MotionConfigHeader
from omotion.CommandError import CommandError
```

Add after `MotionConfig` import:
```python
from omotion.Calibration import (
    Calibration,
    parse_calibration,
    serialize_calibration,
    CALIBRATION_JSON_KEY,
)
```

Find the end of `write_config_json` (around line 1939, just before the `# Page-by-page direct FPGA programming commands` banner). Insert the two new methods just before that banner:

```python
    def read_calibration(self) -> Calibration:
        """Read the BFI/BVI calibration from the console EEPROM JSON.

        Returns a :class:`omotion.Calibration` with ``source="console"`` if
        the JSON contains a valid ``calibration`` block, otherwise a fresh
        :meth:`Calibration.default` (``source="default"``). Never raises on
        bad data — invalid blocks are logged and replaced with defaults.

        Raises:
            ValueError: if the UART is not connected (propagates from
                ``read_config``).
        """
        try:
            cfg = self.read_config()
        except ValueError:
            raise
        except Exception as e:
            logger.warning(
                "read_calibration: read_config raised %s; using SDK defaults.",
                e,
            )
            return Calibration.default()

        if cfg is None:
            logger.info(
                "read_calibration: no config returned from device; "
                "using SDK defaults."
            )
            return Calibration.default()

        parsed = parse_calibration(cfg.json_data or {})
        if parsed is None:
            logger.info(
                "read_calibration: no calibration on device or invalid; "
                "using SDK defaults."
            )
            return Calibration.default()

        logger.info("read_calibration: loaded calibration from console.")
        return parsed

    def write_calibration(
        self, c_min, c_max, i_min, i_max
    ) -> Calibration:
        """Write a new BFI/BVI calibration to the console EEPROM.

        Validates inputs (shape ``(2, 8)``, finite, monotonic) before
        touching the wire. Performs a read-modify-write so other JSON
        keys (``EE_THRESH``, etc.) are preserved.

        Returns the :class:`Calibration` (``source="console"``) that was
        written. The caller is responsible for refreshing any cached copy
        — :meth:`omotion.MotionInterface.write_calibration` chains a
        refresh automatically.

        Raises:
            ValueError: invalid input or UART not connected.
            RuntimeError: existing config could not be read for the
                read-modify-write.
        """
        # Validate (raises ValueError on bad input — before any wire activity).
        new_block = serialize_calibration(c_min, c_max, i_min, i_max)

        cfg = self.read_config()
        if cfg is None:
            raise RuntimeError(
                "write_calibration: could not read existing config for "
                "read-modify-write."
            )

        if cfg.json_data is None:
            cfg.json_data = {}
        cfg.json_data[CALIBRATION_JSON_KEY] = new_block[CALIBRATION_JSON_KEY]

        result = self.write_config(cfg)
        if result is None:
            raise RuntimeError("write_calibration: write_config returned None.")

        logger.info("write_calibration: calibration written to console.")
        # Re-read inputs into a Calibration so callers get the canonical
        # numpy form.
        return Calibration(
            c_min=np.asarray(c_min, dtype=float),
            c_max=np.asarray(c_max, dtype=float),
            i_min=np.asarray(i_min, dtype=float),
            i_max=np.asarray(i_max, dtype=float),
            source="console",
        )
```

`MotionConsole.py` does not currently import numpy. Add at the top of the file with the other stdlib/third-party imports (alongside `import logging`, `import json`, etc., near line 1-30 — find the existing imports block):

```python
import numpy as np
```

- [ ] **Step 4: Run read_calibration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_console.py -v -k read_calibration`
Expected: all 4 read_calibration tests pass.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/MotionConsole.py openmotion-sdk/tests/test_calibration_console.py
git commit -m "feat(sdk): add MotionConsole.read_calibration"
```

---

## Task 5: `MotionConsole.write_calibration()`

**Files:**
- Modify: `openmotion-sdk/omotion/MotionConsole.py` (already added in Task 4 — this task adds tests)
- Test: `openmotion-sdk/tests/test_calibration_console.py`

- [ ] **Step 1: Write tests for `write_calibration`**

Append to `openmotion-sdk/tests/test_calibration_console.py`:

```python
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
```

- [ ] **Step 2: Run all calibration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_console.py -v`
Expected: 8 tests pass (4 read + 4 write).

- [ ] **Step 3: Commit**

```bash
git add openmotion-sdk/tests/test_calibration_console.py
git commit -m "test(sdk): cover MotionConsole.write_calibration paths"
```

---

## Task 6: `ScanWorkflow` — replace four nullable arrays with `Calibration`

**Files:**
- Modify: `openmotion-sdk/omotion/ScanWorkflow.py`

This task changes internal state. There is no behavior change visible to callers yet (the loader is wired in Task 7). After this task, `set_realtime_calibration` still works the same way, but the underlying storage is a `Calibration` instance and the cache is pre-seeded with defaults.

- [ ] **Step 1: Add Calibration import to ScanWorkflow**

Modify `openmotion-sdk/omotion/ScanWorkflow.py`. Find the existing import block at lines 12-19:

```python
from omotion import _log_root
from omotion.connection_state import ConnectionState
from omotion.MotionProcessing import (
    CorrectedBatch,
    HISTO_SIZE_WORDS,
    create_science_pipeline,
    parse_histogram_stream,
)
```

Add after the `MotionProcessing` import block:

```python
from omotion.Calibration import Calibration
```

- [ ] **Step 2: Replace the four nullable fields with one `_calibration`**

In `ScanWorkflow.__init__` (around lines 137-140), replace:

```python
        self._bfi_c_min = None
        self._bfi_c_max = None
        self._bfi_i_min = None
        self._bfi_i_max = None
```

with:

```python
        self._calibration: Calibration = Calibration.default()
```

- [ ] **Step 3: Add `_install_calibration` and tighten `set_realtime_calibration`**

Replace the existing `set_realtime_calibration` method (lines 158-168):

```python
    def set_realtime_calibration(
        self,
        bfi_c_min,
        bfi_c_max,
        bfi_i_min,
        bfi_i_max,
    ) -> None:
        self._bfi_c_min = bfi_c_min
        self._bfi_c_max = bfi_c_max
        self._bfi_i_min = bfi_i_min
        self._bfi_i_max = bfi_i_max
```

with:

```python
    def set_realtime_calibration(
        self,
        bfi_c_min,
        bfi_c_max,
        bfi_i_min,
        bfi_i_max,
    ) -> None:
        """Override the cached calibration. Marks source as ``override``.

        Validates shapes — historically this method silently stored
        ``None``s if shapes were wrong; that is now a ``ValueError``.
        """
        import numpy as np
        arrs = []
        for name, val in (
            ("bfi_c_min", bfi_c_min), ("bfi_c_max", bfi_c_max),
            ("bfi_i_min", bfi_i_min), ("bfi_i_max", bfi_i_max),
        ):
            arr = np.asarray(val, dtype=float)
            if arr.shape != (2, 8):
                raise ValueError(
                    f"set_realtime_calibration: {name} has shape "
                    f"{arr.shape}; expected (2, 8)"
                )
            arrs.append(arr)
        self._install_calibration(
            Calibration(
                c_min=arrs[0],
                c_max=arrs[1],
                i_min=arrs[2],
                i_max=arrs[3],
                source="override",
            )
        )

    def _install_calibration(self, cal: Calibration) -> None:
        """Replace the cached calibration. Used by the connect-time loader
        and by ``set_realtime_calibration``.
        """
        self._calibration = cal
        logger.info("Calibration installed (source=%s).", cal.source)
```

- [ ] **Step 4: Update the consumption site in `start_scan`**

In `start_scan`, find the four-`is not None` gate around lines 380-385:

```python
                if (
                    self._bfi_c_min is not None
                    and self._bfi_c_max is not None
                    and self._bfi_i_min is not None
                    and self._bfi_i_max is not None
                ):
```

Replace with:

```python
                if self._calibration is not None:
```

Then find the `create_science_pipeline(...)` call around lines 627-633 with `bfi_c_min=self._bfi_c_min` and update the four lines:

```python
                        bfi_c_min=self._bfi_c_min,
                        bfi_c_max=self._bfi_c_max,
                        bfi_i_min=self._bfi_i_min,
                        bfi_i_max=self._bfi_i_max,
```

to:

```python
                        bfi_c_min=self._calibration.c_min,
                        bfi_c_max=self._calibration.c_max,
                        bfi_i_min=self._calibration.i_min,
                        bfi_i_max=self._calibration.i_max,
```

- [ ] **Step 5: Verify the SDK still imports cleanly and tests pass**

Run: `cd openmotion-sdk && python -c "from omotion import MotionInterface; mi = MotionInterface(demo_mode=True); print(mi.scan_workflow._calibration.source)"`
Expected: `default`

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py tests/test_calibration_console.py -v`
Expected: all calibration tests still pass.

- [ ] **Step 6: Commit**

```bash
git add openmotion-sdk/omotion/ScanWorkflow.py
git commit -m "refactor(sdk): ScanWorkflow stores Calibration instead of four nullable arrays"
```

---

## Task 7: `MotionInterface` — load on connect, refresh, get, write

**Files:**
- Modify: `openmotion-sdk/omotion/MotionInterface.py`
- Test: `openmotion-sdk/tests/test_calibration_console.py`

- [ ] **Step 1: Write failing tests for the facade**

Append to `openmotion-sdk/tests/test_calibration_console.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration_console.py -v -k "facade or refresh or log_console_info or write_calibration_refreshes or set_realtime"`
Expected: AttributeErrors on `get_calibration`, `refresh_calibration`, `write_calibration`.

- [ ] **Step 3: Add the facade methods to `MotionInterface`**

Modify `openmotion-sdk/omotion/MotionInterface.py`.

Find the import block at lines 20-25:
```python
from omotion.MotionConsole import MotionConsole
from omotion.MotionSensor import MotionSensor
from omotion.connection_monitor import ConnectionMonitor
from omotion.connection_state import ConnectionState
from omotion.config import CONSOLE_MODULE_PID, SENSOR_MODULE_PID
from omotion import __version__ as _SDK_VERSION, _log_root
```

Add after `connection_state` import:
```python
from omotion.Calibration import Calibration
```

Find `log_console_info` (around lines 325-327):
```python
    def log_console_info(self) -> None:
        if self.console.is_connected():
            self.console.log_device_info()
```

Replace with:
```python
    def log_console_info(self) -> None:
        if self.console.is_connected():
            self.console.log_device_info()
            self._load_calibration_from_console()

    def _load_calibration_from_console(self) -> None:
        """Read calibration from the console and install it into ScanWorkflow.

        Best-effort: any failure is logged and the existing cache is kept.
        Called automatically on console-connect via ``log_console_info``;
        also exposed publicly via ``refresh_calibration``.
        """
        try:
            cal = self.console.read_calibration()
        except Exception as e:
            logger.warning(
                "Could not load calibration from console: %s. "
                "Keeping existing cached calibration (source=%s).",
                e, self.scan_workflow._calibration.source,
            )
            return
        self.scan_workflow._install_calibration(cal)

    def refresh_calibration(self) -> Calibration:
        """Re-read calibration from the console and update the cache.

        Returns the resulting :class:`Calibration` (the same value
        accessible via :meth:`get_calibration`).
        """
        self._load_calibration_from_console()
        return self.scan_workflow._calibration

    def get_calibration(self) -> Calibration:
        """Return the currently cached calibration."""
        return self.scan_workflow._calibration

    def write_calibration(
        self, c_min, c_max, i_min, i_max
    ) -> Calibration:
        """Validate inputs, write the calibration to the console EEPROM,
        then read it back into the cache. Returns the cached value.
        """
        self.console.write_calibration(c_min, c_max, i_min, i_max)
        return self.refresh_calibration()
```

- [ ] **Step 4: Run all calibration tests**

Run: `cd openmotion-sdk && python -m pytest tests/test_calibration.py tests/test_calibration_console.py -v`
Expected: all tests pass.

- [ ] **Step 5: Commit**

```bash
git add openmotion-sdk/omotion/MotionInterface.py openmotion-sdk/tests/test_calibration_console.py
git commit -m "feat(sdk): MotionInterface refresh/get/write calibration; load on connect"
```

---

## Task 8: Remove the bloodflow app's `set_realtime_calibration` bootstrap

**Files:**
- Modify: `openmotion-bloodflow-app/motion_connector.py`

- [ ] **Step 1: Delete the module-level calibration globals**

In `openmotion-bloodflow-app/motion_connector.py`, find lines 67-71:

```python
_BFI_CAL = VisualizeBloodflow(left_csv="", right_csv="")
_BFI_C_MIN = _BFI_CAL.C_min
_BFI_C_MAX = _BFI_CAL.C_max
_BFI_I_MIN = _BFI_CAL.I_min
_BFI_I_MAX = _BFI_CAL.I_max
```

Delete those five lines.

- [ ] **Step 2: Delete the `set_realtime_calibration` call**

In the same file, find lines 348-350:

```python
        self._scan_workflow.set_realtime_calibration(
            _BFI_C_MIN, _BFI_C_MAX, _BFI_I_MIN, _BFI_I_MAX
        )
```

Delete those three lines. Do **not** remove the surrounding code or the
preceding/following statements — only this one call.

- [ ] **Step 3: Verify the app still imports**

Run: `cd openmotion-bloodflow-app && python -c "import motion_connector; print('ok')"`
Expected: `ok` (no NameError on `_BFI_C_MIN`).

If the import surfaces other usages of `_BFI_C_MIN` / `_BFI_C_MAX` / `_BFI_I_MIN` / `_BFI_I_MAX`, search and remove them — these symbols should have zero remaining references after this task.

Run: `cd openmotion-bloodflow-app && grep -n "_BFI_C_MIN\|_BFI_C_MAX\|_BFI_I_MIN\|_BFI_I_MAX\|_BFI_CAL" motion_connector.py || echo "clean"`
Expected: `clean`.

- [ ] **Step 4: Commit**

```bash
git add openmotion-bloodflow-app/motion_connector.py
git commit -m "refactor(bloodflow-app): drop manual calibration bootstrap; SDK loads from console"
```

---

## Task 9: End-to-end smoke test

**Files:** None (manual verification).

This is a hardware test — only run if a console is attached.

- [ ] **Step 1: Confirm fresh-console behavior (defaults)**

Connect to a console with no `calibration` block in its EEPROM JSON.

```bash
cd openmotion-sdk
python -c "
from omotion import MotionInterface
import time
mi = MotionInterface()
mi.start(wait=True, wait_timeout=5.0)
time.sleep(1)
mi.log_console_info()
cal = mi.get_calibration()
print('source =', cal.source)
print('c_max[0] =', cal.c_max[0])
mi.stop()
"
```

Expected output: `source = default` (or `console` if a previous run wrote one).

- [ ] **Step 2: Write a custom calibration and verify read-back**

```bash
cd openmotion-sdk
python -c "
import numpy as np
from omotion import MotionInterface
import time
mi = MotionInterface()
mi.start(wait=True, wait_timeout=5.0)
time.sleep(1)
c_max = np.full((2, 8), 0.42)
i_max = np.full((2, 8), 275.0)
zero = np.zeros((2, 8))
cal = mi.write_calibration(zero, c_max, zero, i_max)
print('post-write source =', cal.source, 'c_max[0,0] =', cal.c_max[0,0])
# Reconnect simulation: refresh should still show console source.
print('refresh source =', mi.refresh_calibration().source)
mi.stop()
"
```

Expected: `post-write source = console c_max[0,0] = 0.42` and `refresh source = console`.

- [ ] **Step 3: Reset back to defaults if desired**

If you want to clear the calibration block on the device, write the SDK
defaults back explicitly:

```bash
cd openmotion-sdk
python -c "
from omotion import MotionInterface, Calibration
import time
mi = MotionInterface()
mi.start(wait=True, wait_timeout=5.0)
time.sleep(1)
d = Calibration.default()
mi.write_calibration(d.c_min, d.c_max, d.i_min, d.i_max)
print('reset; source now =', mi.get_calibration().source)
mi.stop()
"
```

Expected: `reset; source now = console` (the values now stored on the
device happen to match the SDK defaults; the source label reflects
provenance, not equality).

- [ ] **Step 4: Run a full bloodflow scan**

Launch the bloodflow app, run a short scan, and confirm:

- The app starts without errors (no `NameError` on `_BFI_C_MIN` symbols).
- BFI/BVI numbers in the live plot look comparable to pre-change runs
  (within noise) when the device has the default calibration written.
- Logs contain a `Calibration loaded from console.` or `no calibration on
  device` message at console-connect time.

---

## Spec Coverage Self-Review

| Spec section | Implementing task |
|---|---|
| `omotion/Calibration.py` (defaults, dataclass, parser, serializer) | Tasks 1, 2, 3 |
| `MotionConsole.read_calibration` | Task 4 |
| `MotionConsole.write_calibration` | Tasks 4 (impl), 5 (tests) |
| `ScanWorkflow` switch to `Calibration` field | Task 6 |
| `_install_calibration` helper | Task 6 |
| `set_realtime_calibration` shape validation + override source | Task 6 |
| `start_scan` consumption site update | Task 6 |
| `MotionInterface._load_calibration_from_console` + `log_console_info` hook | Task 7 |
| `MotionInterface.refresh_calibration` | Task 7 |
| `MotionInterface.get_calibration` | Task 7 |
| `MotionInterface.write_calibration` (write + refresh) | Task 7 |
| Bloodflow app: drop `_BFI_CAL` globals + `set_realtime_calibration` call | Task 8 |
| All-or-nothing validation rules (shape, finite, monotonic) | Task 2 (parser) + Task 3 (serializer) |
| Unit tests | Tasks 1, 2, 3 |
| Integration tests (mocked UART) | Tasks 4, 5, 7 |
| Manual smoke test | Task 9 |
| `visualize_bloodflow.py` left untouched | (no task — explicitly preserved) |
