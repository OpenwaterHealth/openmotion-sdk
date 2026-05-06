# Console-EEPROM-driven calibration in the SDK

**Date:** 2026-05-01
**Status:** Draft

## Problem

The four calibration arrays used to compute BFI/BVI — `C_min`, `C_max`,
`I_min`, `I_max`, each shape `(modules=2, cams_per_module=8)` — currently live
only in `openmotion-bloodflow-app/processing/visualize_bloodflow.py`. The
bloodflow app instantiates `VisualizeBloodflow(left_csv="", right_csv="")` to
harvest its dataclass defaults and then feeds them into
`ScanWorkflow.set_realtime_calibration(...)` once during connector init. There
is no path for per-device calibration: every console uses the same hardcoded
values regardless of what was measured during manufacturing.

The console firmware already exposes a JSON config blob in its EEPROM
(`MotionConsole.read_config()` / `write_config()`), and the bloodflow app
already reads user overrides for laser parameters (`EE_THRESH`, `OPT_GAIN`,
etc.) from that blob. Calibration arrays are a natural extension of the same
pattern.

## Goal

Build a feature in the SDK so that:

1. On console connect, the SDK reads `C_min`, `C_max`, `I_min`, `I_max` from
   the console's EEPROM JSON.
2. If all four are present and valid, the SDK uses them as the active
   calibration.
3. If any are missing or invalid, the SDK falls back to the default
   calibration (a copy of the values currently in `visualize_bloodflow.py`).
4. The bloodflow app stops sourcing defaults from `VisualizeBloodflow` and
   stops calling `set_realtime_calibration` for routine setup.
5. `set_realtime_calibration` remains as an explicit override path.
6. The SDK exposes read, refresh, and write APIs on its facade.

## Non-goals

- Changing the BFI/BVI math itself.
- Changing the on-EEPROM storage format / `MotionConfig` wire protocol.
- Adding a UI in the bloodflow or test app to view/edit calibration values
  (that may come later; the new `get_calibration()` getter is sufficient
  groundwork).
- Migrating the offline-analysis path: `VisualizeBloodflow` keeps its own
  hardcoded dataclass defaults so CSV-only analysis remains independent.

## JSON schema

Calibration is stored under a single nested key in the console JSON config:

```json
{
  "calibration": {
    "C_min": [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]],
    "C_max": [[0.4,0.4,0.45,0.55,0.55,0.45,0.4,0.4],
              [0.4,0.4,0.45,0.55,0.55,0.45,0.4,0.4]],
    "I_min": [[0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0]],
    "I_max": [[150,300,300,300,300,300,300,150],
              [150,300,300,300,300,300,300,150]]
  }
}
```

- Outer rows index module (`0=left, 1=right`); inner columns index camera
  position 0..7 within the module.
- Other top-level keys (`EE_THRESH`, `OPT_GAIN`, …) are preserved on
  read-modify-write.

## Validation rules (all-or-nothing)

When parsing `json_data["calibration"]`:

1. The `"calibration"` block must be present and a JSON object. If absent →
   defaults, info log.
2. Each of `C_min`, `C_max`, `I_min`, `I_max` must be present.
3. Each must convert via `np.asarray(..., dtype=float)` without raising.
4. Each must have shape exactly `(2, 8)`.
5. All values must be finite (no NaN, no inf).
6. `C_max > C_min` element-wise. `I_max > I_min` element-wise. (BFI/BVI use
   these as denominators.)

Any failure → single `logger.warning("Console calibration invalid (<reason>);
falling back to SDK defaults.")` and the parser returns `None`. The caller
substitutes `Calibration.default()`.

## Components

### New module: `omotion/Calibration.py`

```python
from dataclasses import dataclass
from typing import Literal, Optional
import numpy as np

CALIBRATION_JSON_KEY = "calibration"
_C_MIN_KEY = "C_min"
_C_MAX_KEY = "C_max"
_I_MIN_KEY = "I_min"
_I_MAX_KEY = "I_max"

# Defaults — copied verbatim from
# openmotion-bloodflow-app/processing/visualize_bloodflow.py as of 2026-05-01.
# These are the canonical SDK-owned defaults; visualize_bloodflow.py keeps
# its own copy for offline CSV analysis.
_DEFAULT_C_MIN = np.array(
    [[0.0]*8, [0.0]*8], dtype=float)
_DEFAULT_C_MAX = np.array(
    [[0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4],
     [0.4, 0.4, 0.45, 0.55, 0.55, 0.45, 0.4, 0.4]], dtype=float)
_DEFAULT_I_MIN = np.array(
    [[0.0]*8, [0.0]*8], dtype=float)
_DEFAULT_I_MAX = np.array(
    [[150, 300, 300, 300, 300, 300, 300, 150],
     [150, 300, 300, 300, 300, 300, 300, 150]], dtype=float)

CalibrationSource = Literal["console", "default", "override"]

@dataclass(frozen=True)
class Calibration:
    c_min: np.ndarray  # (2, 8) float
    c_max: np.ndarray
    i_min: np.ndarray
    i_max: np.ndarray
    source: CalibrationSource

    @classmethod
    def default(cls) -> "Calibration":
        return cls(
            c_min=_DEFAULT_C_MIN.copy(),
            c_max=_DEFAULT_C_MAX.copy(),
            i_min=_DEFAULT_I_MIN.copy(),
            i_max=_DEFAULT_I_MAX.copy(),
            source="default",
        )

def parse_calibration(json_data: dict) -> Optional[Calibration]:
    """Return Calibration(source='console') if valid, else None.
    None means caller substitutes defaults; this function never raises
    on malformed input — it logs a warning and returns None."""

def serialize_calibration(c_min, c_max, i_min, i_max) -> dict:
    """Return {"calibration": {...}} with arrays converted to lists.
    Validates shape and finiteness, raises ValueError on bad input."""
```

### `MotionConsole.read_calibration() -> Calibration`

- Calls `self.read_config()`.
- If result is `None` (UART error, not connected) → `Calibration.default()`,
  info log.
- Else passes `result.json_data` to `parse_calibration()`.
  - Returns the parsed `Calibration(source="console")` on success.
  - Returns `Calibration.default()` on failure (parser already logged).

Pure read; does not touch the `ScanWorkflow` cache.

### `MotionConsole.write_calibration(c_min, c_max, i_min, i_max) -> Calibration`

1. Validate via `serialize_calibration` (raises `ValueError` for shape /
   finiteness / non-monotonic before any wire activity).
2. Read current config: `cfg = self.read_config()` (preserves other keys).
   If `cfg is None`, raise `RuntimeError`.
3. Merge: `cfg.json_data[CALIBRATION_JSON_KEY] = {C_min: …, C_max: …, …}`.
4. `self.write_config(cfg)` — re-raises on transport error.
5. Return `Calibration(c_min, c_max, i_min, i_max, source="console")`.

Cache refresh is the facade's responsibility (`MotionInterface.write_calibration`
chains a `refresh_calibration()` after).

### `ScanWorkflow` changes

- Replace these four fields:
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
- New internal helper:
  ```python
  def _install_calibration(self, cal: Calibration) -> None:
      self._calibration = cal
  ```
  Used by both the connect-time loader and `set_realtime_calibration`.
- `set_realtime_calibration(c_min, c_max, i_min, i_max)` — keep the
  existing signature. Internally wraps in
  `Calibration(c_min, c_max, i_min, i_max, source="override")` and calls
  `_install_calibration`. (Validates shape `(2, 8)`; raises `ValueError`
  otherwise.)
- In `start_scan(...)`:
  - The four-`is not None` gate at the science-pipeline construction site
    becomes a single sanity check on `self._calibration.c_min.shape == (2, 8)`
    (always true if invariants hold). Existing variable names
    `bfi_c_min=…` etc. are populated from the cached `Calibration`.

### `MotionInterface` changes

- New private `_load_calibration_from_console(self) -> None`:
  ```python
  cal = self.console.read_calibration()
  self.scan_workflow._install_calibration(cal)
  ```
- Hook into existing `log_console_info()` (right after the version/hw_id log
  line, same connect-time slot you specified):
  ```python
  def log_console_info(self) -> None:
      if self.console.is_connected():
          self.console.log_device_info()
          self._load_calibration_from_console()
  ```
- New public `refresh_calibration(self) -> Calibration`:
  ```python
  self._load_calibration_from_console()
  return self.scan_workflow._calibration
  ```
- New public `get_calibration(self) -> Calibration`:
  ```python
  return self.scan_workflow._calibration  # frozen dataclass; arrays exposed
  ```
- New public `write_calibration(self, c_min, c_max, i_min, i_max) -> Calibration`:
  ```python
  self.console.write_calibration(c_min, c_max, i_min, i_max)
  return self.refresh_calibration()  # read-back into cache
  ```

### Bloodflow app changes (`openmotion-bloodflow-app/motion_connector.py`)

- Delete the `_BFI_CAL = VisualizeBloodflow(left_csv="", right_csv="")`
  module-level instantiation (line ~67) and the four
  `_BFI_C_MIN / _BFI_C_MAX / _BFI_I_MIN / _BFI_I_MAX` module globals.
- Delete the call at line ~348:
  ```python
  self._scan_workflow.set_realtime_calibration(
      _BFI_C_MIN, _BFI_C_MAX, _BFI_I_MIN, _BFI_I_MAX
  )
  ```
- The existing `self._interface.log_console_info()` call at line ~838 inside
  the console-connect branch already triggers the new loader transitively —
  no replacement wiring needed.
- `from processing.visualize_bloodflow import VisualizeBloodflow` import
  stays (still used in offline analysis paths inside `motion_connector.py`).

`processing/visualize_bloodflow.py` is unchanged — its dataclass defaults
remain for the offline CSV analysis use case.

## Data flow

```
Console connect → ConnectionMonitor → motion_connector connection-state slot
  → interface.log_console_info()
      → console.log_device_info()                       (existing)
      → interface._load_calibration_from_console()      (NEW)
          → console.read_calibration() → Calibration
          → scan_workflow._install_calibration(cal)

scan_workflow.start_scan(...)
  → reads self._calibration.{c_min,c_max,i_min,i_max}
  → builds SciencePipeline                              (downstream unchanged)

interface.write_calibration(c, c, i, i)
  → console.write_calibration(...)                      (validates, RMW JSON, write)
  → interface.refresh_calibration()                     (read-back)
  → returns refreshed Calibration

interface.refresh_calibration()
  → _load_calibration_from_console()
  → return scan_workflow._calibration

scan_workflow.set_realtime_calibration(...)             (override, unchanged signature)
  → _install_calibration(Calibration(source="override"))
```

## Error handling

| Condition | Behavior |
|---|---|
| Console not connected at load time | `read_config()` returns `None` → defaults seeded; info log. Cache stays at defaults until next `log_console_info()` / `refresh_calibration()`. |
| `"calibration"` key absent from JSON | Defaults seeded; info log "no calibration on device, using SDK defaults." |
| Calibration block malformed (missing key, wrong shape, NaN, non-monotonic) | Defaults seeded; warning log naming the failed check. |
| `write_calibration` input shape / dtype wrong | `serialize_calibration` raises `ValueError` before any wire activity. |
| `write_calibration` flash write fails | `write_config` raises; cache not refreshed (stale but correct). Caller can retry. |
| Console disconnects between RMW read and write | `read_config` raises; propagates up; cache untouched. |
| `set_realtime_calibration` shape wrong | Raises `ValueError` (today the function silently stores `None`s; tightening is intentional). |

## Testing

### Unit tests — `openmotion-sdk/tests/test_calibration.py` (no hardware)

- Parser:
  - Valid JSON → `Calibration(source="console")` with arrays matching input.
  - `"calibration"` key absent → returns `None`.
  - Missing one sub-key → returns `None` + warning.
  - Wrong shape `(2, 7)` / `(3, 8)` / scalar → returns `None`.
  - NaN / inf → returns `None`.
  - `C_max <= C_min` somewhere → returns `None`.
  - Non-numeric value (string) → returns `None`.
- `Calibration.default()`:
  - Returns shapes `(2, 8)` for all arrays.
  - Returned arrays are independent copies (mutation does not bleed across
    calls).
  - Values match the literal arrays currently in `visualize_bloodflow.py`
    (golden test).
- `serialize_calibration`:
  - Round-trips through `parse_calibration` losslessly.
  - Raises on bad shape / non-finite / non-monotonic.

### Integration tests — `tests/test_calibration_console.py`

Uses the existing `demo_mode` / mock UART path so no hardware is required.

- `MotionConsole.read_calibration` with mocked `read_config`:
  - Empty JSON → defaults, source=`"default"`.
  - Valid calibration block → values match, source=`"console"`.
  - Malformed block → defaults.
- `MotionConsole.write_calibration`:
  - Preserves other JSON keys (`EE_THRESH`, `OPT_GAIN`).
  - Round-trip: write → read → matches.
  - `ValueError` raised before wire activity on bad input.
- `MotionInterface`:
  - `refresh_calibration` populates `scan_workflow._calibration`.
  - `write_calibration` followed by `get_calibration` returns the new values
    with `source="console"`.
  - `set_realtime_calibration` followed by `get_calibration` returns
    `source="override"`; subsequent `refresh_calibration` resets to
    `source="console"`/`"default"`.

## Implementation order

1. Create `omotion/Calibration.py` (defaults, dataclass, parser, serializer).
2. Wire it into `MotionConsole` (`read_calibration`, `write_calibration`).
3. Refactor `ScanWorkflow` to hold `Calibration` instead of four nullable
   fields; update `start_scan` and `set_realtime_calibration` accordingly.
4. Add the loader hook to `MotionInterface.log_console_info`; add
   `refresh_calibration` / `get_calibration` / `write_calibration` facade
   methods.
5. Add unit and integration tests.
6. Remove `_BFI_CAL` import / globals / `set_realtime_calibration` call from
   `motion_connector.py`.
7. Manual smoke test: connect to a console with no calibration written
   (defaults loaded), write a calibration, reconnect (console-loaded), run
   a scan and confirm BFI/BVI numbers shift as expected.

## Open questions

- `CalibrationSource` includes `"override"` for values supplied via
  `set_realtime_calibration`, distinct from `"default"` and `"console"`.
  This is one extra symbol beyond the two-value Literal originally agreed
  for the getter return; it's load-bearing because the bloodflow app
  may later want to display whether a scan is using overridden values vs.
  device-stored values vs. SDK defaults. Push back if you'd rather collapse
  override into default.
