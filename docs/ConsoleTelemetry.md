# ConsoleTelemetry

`omotion/ConsoleTelemetry.py` — SDK-level background poller for console health data.

## Overview

`ConsoleTelemetryPoller` runs as a daemon thread owned by `MotionConsole`. It polls the console at ~1 Hz and stores the result as an immutable `ConsoleTelemetry` snapshot. Clients can either pull the latest snapshot directly or register a push callback.

**Lifecycle** — wired automatically by `MotionInterface`:

| Event | Action |
|---|---|
| Console USB connects | `console.telemetry.start()` |
| Console USB disconnects | `console.telemetry.stop()` |
| Console already connected at init | `console.telemetry.start()` called immediately |

**Client API:**

```python
# Pull: read the last snapshot
snap = motion_interface.console_module.telemetry.get_snapshot()
if snap and snap.read_ok:
    print(snap.tcm, snap.tec_volt_raw)

# Push: register a callback (fires on the poller thread, ~1 Hz)
def on_update(snap: ConsoleTelemetry):
    ...

console.telemetry.add_listener(on_update)
console.telemetry.remove_listener(on_update)
```

---

## `ConsoleTelemetry` fields

Each poll produces one `ConsoleTelemetry` dataclass instance. All fields have defaults so a partial read (where an exception occurs mid-poll) still returns a usable object with `read_ok = False`.

### Analog telemetry

| Field | Type | Source | Description |
|---|---|---|---|
| `tcm` | `int` | `get_lsync_pulsecount()` | MCU LSYNC pulse count since last reset |
| `tcl` | `int` | I2C mux 1 ch 4 addr `0x41` reg `0x10` (4 B LE) | Laser trigger count |
| `pdc` | `float` (mA) | I2C mux 1 ch 7 addr `0x41` reg `0x1C` (2 B LE) × 1.9 | Photodiode current |

### TEC status

Raw ADC voltages returned by `MotionConsole.tec_status()` → `(vout, temp_set, tec_curr, tec_volt, tec_good)`. Conversion to engineering units (°C, A, V) lives in the SDK at `omotion/console_telemetry_conversions.py` (`tec_thermistor_voltage_to_celsius`, `tec_current_to_amps`, `tec_voltage_to_volts`), using the `10K3CG_R-T` thermistor lookup table shipped alongside it. (Bloodflow-app historically owned this math; it now lives in the SDK so every consumer applies the same formula.)

| Field | Type | ADC channel | Description |
|---|---|---|---|
| `tec_v_raw` | `float` (V) | OUT1 / VOUT1 | Thermistor bridge output voltage; converted to measured temperature (°C) in the app |
| `tec_set_raw` | `float` (V) | IN2P / TEMPSET | Setpoint bridge voltage; converted to target temperature (°C) in the app |
| `tec_curr_raw` | `float` (V) | V_itec | TEC current monitor voltage; converted to amps via `(v − 0.5·VREF) / (25·R_s)` |
| `tec_volt_raw` | `float` (V) | V_vtec | TEC voltage monitor; converted via `(v − 0.5·VREF) × 4` |
| `tec_good` | `bool` | TMPGD pin | `True` when `\|OUT1 − IN2P\| < 100 mV` (temperature settled to setpoint) |

### PDU monitor

16-channel ADC board with two 8-channel groups (ADC0 / ADC1). Raw values and pre-scaled voltages are both stored.

| Field | Type | Source | Description |
|---|---|---|---|
| `pdu_raws` | `List[int]` (len 16) | `read_pdu_mon()` → `PDUMon.raws` | Raw uint16 ADC counts, channels 0–15 |
| `pdu_volts` | `List[float]` (len 16) | `read_pdu_mon()` → `PDUMon.volts` | Pre-scaled float32 voltages, channels 0–15 |

ADC0 (indices 0–7) and ADC1 (indices 8–15) use different scaling: ADC1 channel 6 (index 14) is a voltage rail scaled by `SCALE_V`; all other ADC1 channels are current monitors scaled by `SCALE_I`.

### Safety interlock

Polled from two I2C channels on mux 1, device `0x41`, register `0x24`.

| Field | Type | I2C channel | Description |
|---|---|---|---|
| `safety_se` | `int` (raw byte) | ch 6 | SE interlock raw register byte |
| `safety_so` | `int` (raw byte) | ch 7 | SO interlock raw register byte |
| `safety_ok` | `bool` | derived | `True` when `(safety_se & 0x0F) == 0` and `(safety_so & 0x0F) == 0` — both low nibbles clear means the interlock is not tripped |

### Read health

| Field | Type | Description |
|---|---|---|
| `timestamp` | `float` | `time.time()` at the end of the poll (set twice: at start and end of `_read_all`, final value wins) |
| `read_ok` | `bool` | `False` if any sub-read raised an exception during this poll |
| `error` | `Optional[str]` | Exception message when `read_ok` is `False`, otherwise `None` |

---

## Poll sequence

Each ~1 Hz tick calls `_read_all()`, which runs these four sub-reads in order. If any raises, the remainder are skipped, `read_ok` is set to `False`, and the partial snapshot is still stored and delivered to listeners.

1. `_read_tec` — one round-trip to the console MCU via `tec_status()`
2. `_read_pdu` — one round-trip via `read_pdu_mon()`
3. `_read_safety` — two I2C reads (SE and SO channels)
4. `_read_analog` — one `get_lsync_pulsecount()` call + two I2C reads (tcl and pdc)

---

## Thread safety

`ConsoleTelemetryPoller` uses a single `threading.Lock` (`_lock`) to protect `_snapshot` and `_listeners`. Listeners are copied under the lock and then called outside it, so they cannot deadlock the poller. Listener callbacks run on the poller thread — they should be non-blocking.
