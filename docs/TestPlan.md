# Open-Motion SDK — Test Plan

## Overview

This document defines the **hardware-in-the-loop (HIL) test suite** for the Open-Motion SDK. Tests are grouped by system (console / sensor), then by subsystem, and finally by individual function. The plan also covers common command sequences, communication-path verification, and the recommended pytest infrastructure for a GitHub Actions CI workflow backed by physical hardware runners.

**Scope.** This plan covers HIL tests only — the ones gated behind the `console` / `sensor` / `sensor_left` / `sensor_right` / `sequence` / `destructive` markers and run on the self-hosted hardware runners. The pure-software tier (the stage-based pipeline tests under `tests/test_pipeline/`, the calibration-compute tests, contact-quality, scan-DB, and unit tests) runs anywhere without hardware and is intentionally **out of scope here** — those tests are their own authoritative spec. There is, by design, no software-only PR CI job; everything runs on the rig.

All tests are written against the public SDK API (`MotionConsole`, `MotionSensor`, `MotionInterface`). Lower-level transport classes (`MotionUart`, `CommInterface`, `StreamInterface`) are exercised indirectly; dedicated transport tests are called out where the transport behaviour itself is the thing under test.

**How to read this plan.** This document describes HIL coverage **by subsystem** — what each subsystem's hardware tests are meant to verify. It is **not** a line-by-line registry of every test function: the executable tests under `tests/test_*.py` are the authoritative, live registry. Individual test names below are illustrative of intent and may have drifted (renames, splits); when in doubt, the code wins. Coverage we intend but have not yet built is collected in **§8 Known coverage gaps / backlog**.

For the **whole-suite view** (all 504 tests grouped by SDK architecture layer, the software-vs-hardware split, and per-layer coverage ratings), see [`TestSuite.md`](TestSuite.md). This plan covers only the HIL tier.

---

## 1. Test Environment and Fixtures

### 1.1 Hardware requirements

| Role | Device | Notes |
|---|---|---|
| Console DUT | Console module (USB VCP) | Must be powered and enumerated before the test session starts |
| Sensor DUT (left) | Sensor module on USB port `…,2` | Optional; skip sensor tests if absent |
| Sensor DUT (right) | Sensor module on USB port `…,3` | Optional |

### 1.2 Pytest fixtures (conftest.py)

```python
# tests/conftest.py
import pytest
from omotion.MotionInterface import MotionInterface

@pytest.fixture(scope="session")
def motion():
    iface = MotionInterface()
    iface.connect()
    yield iface
    iface.disconnect()

@pytest.fixture(scope="session")
def console(motion):
    c = motion.console_module
    if not c.is_connected():
        pytest.skip("Console not connected")
    return c

@pytest.fixture(scope="session")
def sensor_left(motion):
    s = motion.sensors.left
    if s is None or not s.is_connected():
        pytest.skip("Left sensor not connected")
    return s

@pytest.fixture(scope="session")
def sensor_right(motion):
    s = motion.sensors.right
    if s is None or not s.is_connected():
        pytest.skip("Right sensor not connected")
    return s
```

Session-scoped fixtures keep the USB connection open for the entire test run. Individual test modules may use function-scoped fixtures that call a `teardown` sequence if their test mutates device state.

### 1.3 Markers

Markers are declared in `pyproject.toml` under `[tool.pytest.ini_options]` (there
is no `pytest.ini`). The full set:

```toml
markers = [
    "console:      requires a connected console module",
    "sensor:       requires at least one connected sensor module",
    "sensor_left:  tests specific to the left sensor",
    "sensor_right: tests specific to the right sensor",
    "slow:         takes more than 10 s (e.g. full scans, DFU)",
    "destructive:  modifies flash or firmware",
    "sequence:     multi-step round-trip tests",
    "fpga:         requires a loaded FPGA bitstream — excluded by default",
    "imu:          exercises the IMU subsystem — excluded by default",
]
```

Run only fast, non-destructive tests:
```
pytest -m "not slow and not destructive"
```

---

## 2. Console Module Tests

### 2.1 Basic connectivity

**`test_console_ping`**
Sends a `ping()` call and asserts `True` is returned.

**`test_console_version`**
Calls `get_version()`. Asserts the returned string is non-empty and follows the expected semver pattern (`\d+\.\d+\.\d+`).

**`test_console_hardware_id`**
Calls `get_hardware_id()`. Asserts the returned string is non-empty and has the expected length / prefix for the board variant.

**`test_console_echo`**
Calls `echo(b"hello")`. Asserts the echoed payload equals the sent bytes and the returned length equals `5`.

**`test_console_echo_empty`**
Calls `echo(b"")`. Asserts the returned payload is empty and length is `0`.

**`test_console_toggle_led`**
Calls `toggle_led()` twice. Asserts each call returns `True`.

**`test_console_board_id`**
Calls `read_board_id()`. Asserts the returned integer is within the set of known board IDs.

**`test_console_messages`**
Calls `get_messages()`. Asserts the returned string is a string (may be empty).

### 2.2 TEC subsystem

**`test_tec_status_types`**
Calls `tec_status()`. Asserts the returned tuple is `(float, float, float, float, bool)` and all float values are within physically plausible ADC ranges (0–3.3 V).

**`test_tec_adc_channels`**
Calls `tec_adc(ch)` for channels 0, 1, 2, 3. Asserts each returns a float in [0.0, 3.3].

**`test_tec_voltage_read`**
Calls `tec_voltage()` (no argument). Asserts the returned float is in [0.0, 3.3].

**`test_tec_voltage_set`**
Sets a known voltage with `tec_voltage(1.5)`. Reads back with `tec_voltage()`. Asserts the read-back value is within ±0.05 V of the set point.

**`test_temperatures`**
Calls `get_temperatures()`. Asserts a 3-tuple of floats is returned; each value is in the range -40 to 85 °C.

### 2.3 PDU monitor

**`test_pdu_mon_structure`**
Calls `read_pdu_mon()`. Asserts `PDUMon` is not `None`, `len(raws) == 16`, `len(volts) == 16`. Asserts all `raws` are `int` and all `volts` are `float`.

**`test_pdu_mon_ranges`**
Asserts every voltage in `volts` is in [-0.5, 60.0] V (gross sanity bounds). Reports a warning (not a failure) for channels that read zero, as unloaded rails may legitimately be zero.

### 2.4 I2C pass-through

**`test_i2c_scan`**
Calls `scan_i2c_mux_channel(mux_index=0, channel=0)`. Asserts the returned list is a list of integers, each in [0, 127].

**`test_i2c_read_write_roundtrip`**
Writes a known byte to a scratch register on a known I2C device. Reads it back. Asserts the read value equals the written value.

**`test_i2c_read_bad_address`**
Calls `read_i2c_packet` with a non-existent I2C address. Asserts that `CommandError` or `ValueError` is raised (graceful error propagation).

### 2.5 GPIO and ADC

**`test_read_gpio`**
Calls `read_gpio_value()`. Asserts the returned value is a float.

**`test_read_adc`**
Calls `read_adc_value()`. Asserts the returned float is in [0.0, 3.3].

### 2.6 Fan control

**`test_fan_set_and_get`**
Sets fan speed to 75 with `set_fan_speed(75)`. Reads back with `get_fan_speed()`. Asserts the read value equals 75.

**`test_fan_min_max`**
Sets speed to 0 and 100. Asserts both succeed. Restores a default value (50) after the test.

### 2.7 RGB indicator

**`test_rgb_set_and_get`**
Calls `set_rgb_led(0x01)`. Reads back with `get_rgb_led()`. Asserts the read value matches what was set.

### 2.8 Frame sync / trigger

**`test_fsync_pulsecount`**
Calls `get_fsync_pulsecount()`. Asserts the returned value is a non-negative integer.

**`test_lsync_pulsecount`**
Calls `get_lsync_pulsecount()`. Asserts the returned value is a non-negative integer.

**`test_trigger_set_get`**
Calls `set_trigger_json({"rate": 10})`. Calls `get_trigger_json()`. Asserts the returned dict contains the `"rate"` key with value 10.

**`test_trigger_start_stop`**
Calls `start_trigger()`. Asserts `True`. Waits 200 ms. Calls `stop_trigger()`. Asserts `True`. Asserts `get_lsync_pulsecount()` increased.

### 2.9 Configuration (MotionConfig)

**`test_read_config`**
Calls `read_config()`. Asserts the returned object is either `None` (no config stored) or a valid `MotionConfig` with a non-None `json` payload.

**`test_write_read_config_roundtrip`**
Constructs a `MotionConfig` with a known JSON string. Writes it with `write_config()`. Reads it back. Asserts the JSON content round-trips without modification.

**`test_write_config_json_roundtrip`**
Calls `write_config_json('{"key": "value"}')`. Reads back with `read_config()`. Asserts the parsed JSON matches.

### 2.10 FPGA programming (console-side)

These tests are marked `destructive` and `slow`. They are excluded from the standard CI run and are run manually or on a dedicated flash-validation runner.

**`test_fpga_prog_open_close`**
Calls `fpga_prog_open(MuxChannel.FPGA_A)` then `fpga_prog_close(MuxChannel.FPGA_A)`. Asserts no exception is raised.

**`test_fpga_prog_erase`** (destructive)
Opens, erases (mode 0), closes. Asserts no exception.

**`test_fpga_prog_read_status`**
Opens, calls `fpga_prog_read_status(MuxChannel.FPGA_A)`. Asserts the returned integer matches the expected idle status bitmask. Closes.

**`test_fpga_prog_cfg_reset`**
Opens, calls `fpga_prog_cfg_reset`. Asserts no exception. Closes.

**`test_fpga_prog_featrow_roundtrip`** (destructive)
Reads feature row, writes the same content back, reads again. Asserts identity.

**`test_fpga_prog_ufm_roundtrip`** (destructive)
Writes a known page to UFM, reads it back, asserts equality.

**`test_full_fpga_flash`** (destructive, slow)
Executes a full FPGA flash using `FPGAProgrammer`. Monitors progress callbacks. Asserts final status is success.

### 2.11 DFU

**`test_enter_dfu`** (destructive)
Calls `enter_dfu()`. Asserts `True` is returned. This causes the device to re-enumerate; the fixture marks the console as disconnected and the test runner must reconnect before continuing.

### 2.12 Console telemetry poller

**`test_telemetry_poller_starts_on_connect`**
Connects the console. Waits 1.5 s. Calls `console.telemetry.get_snapshot()`. Asserts the snapshot is not `None` and `read_ok is True`.

**`test_telemetry_fields_populated`**
Gets a snapshot. Asserts `tcm >= 0`, `tcl >= 0`, `pdu_raws` has 16 elements, `safety_ok` is a bool, `timestamp > 0`.

**`test_telemetry_listener_fires`**
Registers a callback via `add_listener`. Waits 2.5 s. Asserts the callback was called at least twice (confirming ~1 Hz rate). Removes the listener.

**`test_telemetry_poller_stops`**
Calls `console.telemetry.stop()`. Waits 2 s. Records the snapshot timestamp. Waits another 2 s. Asserts the snapshot timestamp has not changed (poller is idle).

**`test_safety_interlock_clear`**
Reads `safety_ok` from a snapshot on a powered-up, non-faulted system. Asserts `True`.

---

## 3. Sensor Module Tests

The tests in this section apply to both the left and right sensors unless otherwise noted. Parametrized tests using `@pytest.mark.parametrize("sensor", ["sensor_left", "sensor_right"])` reduce duplication.

### 3.1 Basic connectivity

**`test_sensor_ping`**
Calls `ping()`. Asserts `True`.

**`test_sensor_version`**
Calls `get_version()`. Asserts non-empty semver string.

**`test_sensor_hardware_id`**
Calls `get_hardware_id()`. Asserts non-empty string.

**`test_sensor_echo`**
Calls `echo(b"test")`. Asserts round-trip equality.

**`test_sensor_toggle_led`**
Calls `toggle_led()` twice. Asserts both return `True`.

### 3.2 IMU

**`test_imu_temperature`**
Calls `imu_get_temperature()`. Asserts a float in [-40, 85].

**`test_imu_accelerometer`**
Calls `imu_get_accelerometer()`. Asserts a list of 3 integers. Asserts the magnitude is in the range [0.5g, 2.0g] in raw units (i.e. device is sitting still on a bench).

**`test_imu_gyroscope`**
Calls `imu_get_gyroscope()`. Asserts a list of 3 integers. For a stationary device, asserts all three values are within ±100 raw LSB of zero.

### 3.3 Fan control (sensor)

**`test_sensor_fan_on`**
Calls `set_fan_control(True)`. Asserts `True`.

**`test_sensor_fan_off`**
Calls `set_fan_control(False)`. Asserts `True`.

**`test_sensor_fan_status`**
Calls `set_fan_control(True)`, then `get_fan_control_status()`. Asserts `True`. Calls `set_fan_control(False)`, then `get_fan_control_status()`. Asserts `False`.

### 3.4 Debug flags

**`test_debug_flags_roundtrip`**
Sets flags to `0x03` with `set_debug_flags(0x03)`. Reads back with `get_debug_flags()`. Asserts value is `0x03`. Restores to `0x00`.

### 3.5 Camera power

**`test_camera_power_on_off`**
Calls `enable_camera_power(0xFF)` (all cameras). Asserts `True`. Waits 100 ms. Calls `disable_camera_power(0xFF)`. Asserts `True`.

**`test_camera_power_status`**
Calls `enable_camera_power(0x01)`. Calls `get_camera_power_status()`. Asserts the returned list indicates camera 0 is powered. Calls `disable_camera_power(0x01)`.

**`test_camera_power_selective`**
Enables camera 0 only (`mask=0x01`). Asserts `get_camera_power_status()` shows camera 0 on and cameras 1–N off. Cleans up.

### 3.6 FPGA control

**`test_fpga_enable_disable`**
Calls `enable_camera_fpga(0)`. Asserts `True`. Calls `disable_camera_fpga(0)`. Asserts `True`.

**`test_fpga_check_after_enable`**
Calls `enable_camera_fpga(0)`. Calls `check_camera_fpga(0)`. Asserts `True`. Cleans up.

**`test_fpga_status`**
Calls `enable_camera_fpga(0)`. Calls `get_status_fpga(0)`. Asserts a value is returned. Cleans up.

**`test_fpga_usercode`**
Calls `enable_camera_fpga(0)`. Calls `get_usercode_fpga(0)`. Asserts a non-None value. Cleans up.

**`test_fpga_activate`**
Calls `activate_camera_fpga(0)`. Asserts `True`. Cleans up.

**`test_fpga_reset`**
Calls `reset_camera_sensor(0)`. Asserts `True`.

### 3.7 Camera configuration

**`test_camera_configure_registers`**
Calls `camera_configure_registers(0)`. Asserts `True`.

**`test_camera_configure_test_pattern`**
Calls `camera_configure_test_pattern(camera_position=0, pattern=1)`. Asserts `True`.

**`test_camera_status`**
Calls `get_camera_status(0)`. Asserts the returned dict/value is not `None`.

**`test_camera_security_uid`**
Calls `read_camera_security_uid(0)`. Asserts the returned bytes have the expected length for the camera variant.

**`test_cached_security_uid`**
Calls `refresh_id_cache()`. Calls `get_cached_camera_security_uid(0)`. Asserts the returned hex string is non-empty. Calls `clear_id_cache()`.

**`test_camera_switch`**
Calls `switch_camera(1)` then `switch_camera(0)`. Asserts no exception is raised.

### 3.8 Frame sync

**`test_fsin_enable_disable`**
Calls `enable_aggregator_fsin()`. Asserts `True`. Calls `disable_aggregator_fsin()`. Asserts `True`.

**`test_fsin_external_enable_disable`**
Calls `enable_camera_fsin_ext()`. Asserts `True`. Calls `disable_camera_fsin_ext()`. Asserts `True`.

**`test_camera_enable_disable`**
Calls `enable_camera(0)`. Asserts `True`. Calls `disable_camera(0)`. Asserts `True`.

### 3.9 Single-frame histogram capture

**`test_single_histogram_raw_bytes`**
Sets up camera (power on → FPGA enable → configure). Calls `camera_capture_histogram(0)`. Calls `camera_get_histogram(0)`. Asserts the returned `bytearray` has the expected histogram packet length (4100 bytes: 4096 histogram bytes + 4-byte temperature).

**`test_single_histogram_parsed`**
Calls `get_camera_histogram(camera_id=0)`. Asserts the returned list of `HistogramSample` is non-empty. Asserts each sample has `data_len == 1024` bins.

**`test_single_histogram_bin_sum`**
Gets a histogram from a camera illuminated by the laser at known intensity. Asserts the total photon count (sum of bins) is within an expected range. This test requires a stable optical target; it is gated by a fixture that checks for the `OPTICAL_TARGET` environment variable.

### 3.10 IMU streaming

**`test_imu_streaming_receives_data`**
The IMU `StreamInterface` (interface 2) is exercised by reading several IMU packets at low level. Asserts that at least one valid packet arrives within 500 ms.

### 3.11 Sensor DFU

**`test_sensor_enter_dfu`** (destructive)
Calls `enter_dfu()`. Asserts `True`. Marks the sensor fixture as requires reconnect.

---

## 4. Communication Path Tests

These tests verify that bytes take the correct path through the transport stack and that no packets are silently dropped.

### 4.1 UART framing (console)

**`test_uart_crc_corruption_rejected`**
Constructs a `UartPacket`, flips one byte in the CRC field, sends raw bytes over the serial port. Asserts the response is a `CommandError` with `BAD_CRC` (not a silent discard).

**`test_uart_response_arrives_on_correct_queue`**
Sends two commands with different packet IDs concurrently (two threads). Asserts each thread receives the response matching its own ID. (Tests the per-ID `queue.Queue` routing in `MotionUart` async mode.)

**`test_uart_sync_mode_blocking`**
Forces `MotionUart` into sync mode (if exposed). Sends a `ping`. Asserts the response arrives within the timeout and is correct.

**`test_uart_timeout_raises`**
Sends a command to a known-nonexistent address with a very short timeout. Asserts `TimeoutError` is raised.

### 4.2 USB bulk command path (sensor)

**`test_comm_interface_response_routing`**
Sends two `ping` commands back-to-back. Asserts both responses arrive and are `True`. (Exercises the `CommInterface` response queue under concurrent load.)

**`test_comm_interface_no_missed_acks`**
Sends 100 `echo` commands in a tight loop with a unique 2-byte payload each. Collects all responses. Asserts that 100 responses are received, they are all `True`, and the echoed payloads all match. This test is the primary "no missed comms" verification for the USB command endpoint.

**`test_comm_interface_timeout`**
Sends a known-unresponsive command pattern. Asserts `TimeoutError` within the expected window.

**`test_stream_interface_receives_on_correct_endpoint`**
Starts histogram streaming on the sensor. After 500 ms, stops it. Inspects the raw bytes queued on `StreamInterface` for interface 1 (histogram). Asserts all packet headers indicate they arrived on the histogram bulk endpoint (interface index 1, not 0 or 2). Asserts no data appeared on the IMU interface during this test.

**`test_stream_interface_no_data_loss`**
Streams histograms for 2 s. Counts frames drained from the `StreamInterface` IF1 packet queue (or the `LiveUsbSource` per-side reader). Queries the sensor for its internal frame counter. Asserts the two counts match (zero frame loss).

### 4.3 Dual-sensor USB topology

**`test_left_right_assignment`**
Enumerates USB devices. Asserts the device with `port_numbers[-1] == 2` is mapped to `sensors.left` and the device with `port_numbers[-1] == 3` to `sensors.right`.

**`test_dual_composite_auto_reconnect`**
Disconnects and reconnects a sensor module (simulated by calling `release()` and then plugging the device back in). Asserts the `signal_connect` fires with the correct side name within 5 s.

---

## 5. Sequence Tests

These are the round-trip end-to-end tests. Each test function stands up a complete subsystem, exercises it, then tears it back down.

### 5.1 Camera bring-up and single frame

```
test_camera_full_bringup_single_frame
```

Steps:
1. `enable_camera_power(mask)` — power on camera 0.
2. Wait 50 ms.
3. `enable_camera_fpga(0)` — assert `True`.
4. `camera_configure_registers(0)` — assert `True`.
5. `camera_capture_histogram(0)` — trigger one capture.
6. `camera_get_histogram(0)` — assert bytes, correct length.
7. Parse via `parse_histogram_packet()` — assert one `HistogramSample` returned.
8. Assert `HistogramSample.data_len == 1024`.
9. `disable_camera_fpga(0)` — assert `True`.
10. `disable_camera_power(mask)` — assert `True`.

Asserts at every step. If any step fails, the teardown (steps 9–10) is run in a `finally` block.

### 5.2 Camera power cycle

```
test_camera_power_cycle
```

Steps:
1. Power on.
2. Assert power status shows on.
3. Power off.
4. Assert power status shows off.
5. Power on again.
6. Assert power status shows on.
7. Power off. Restore.

### 5.3 FPGA enable → histogram → disable

```
test_fpga_enable_histogram_disable
```

Steps:
1. Power camera on.
2. Enable FPGA.
3. Check FPGA (`check_camera_fpga`) — assert `True`.
4. Configure registers.
5. Capture and read one histogram.
6. Parse histogram.
7. Disable FPGA.
8. Check FPGA — assert `False` (disabled).
9. Power off.

### 5.4 Streaming acquisition

> **Not yet implemented — see §8 backlog (HIGH).** The original spec here drove
> the now-removed `SciencePipeline` callback API; a real streaming-acquisition
> HIL test must instead run a short scan through `omotion/pipeline/` (the
> `LiveUsbSource` → stage chain → a collector sink) and assert on the corrected
> frames it emits. Re-spec'd in §8.

### 5.5 External FSIN enable → scan → disable

```
test_external_fsin_sequence
```

Steps:
1. Enable camera and FPGA.
2. `enable_camera_fsin_ext()`.
3. Assert `True`. Wait 200 ms.
4. Capture one frame.
5. `disable_camera_fsin_ext()`. Assert `True`.
6. Tear down camera.

### 5.6 Test pattern verification

```
test_test_pattern_histogram
```

Steps:
1. Bring camera up.
2. `camera_configure_test_pattern(0, pattern=1)`.
3. Capture and read histogram.
4. Assert all bins are equal (flat pattern) or follow the expected deterministic pattern for the configured mode.
5. Tear down.

### 5.7 Console trigger + LSYNC count

```
test_trigger_lsync_sequence
```

Steps:
1. Read baseline `get_lsync_pulsecount()`.
2. `set_trigger_json({"rate": 10})`.
3. `start_trigger()`.
4. Wait 1.1 s.
5. `stop_trigger()`.
6. Read final `get_lsync_pulsecount()`.
7. Assert delta is ≥ 9 and ≤ 12 (approximately 10 pulses per second).

### 5.8 Dual-sensor aligned frame acquisition

> **Not yet implemented — see §8 backlog (HIGH).** Original spec referenced the
> removed `ScienceFrame` type. Re-spec'd against the pipeline `FrameBatch` /
> per-side reader threads in §8.

### 5.9 Full scan workflow

```
test_scan_workflow_end_to_end   →   tests/test_sequences.py::test_scan_workflow_end_to_end
```

Verifies a short scan runs to completion through the current API (there is no
`ScanResult` or `on_complete` callback anymore — both were removed):

1. Build a `ScanRequest` with a ~5-second duration.
2. `motion.start_scan(request)` (returns a bool; the scan runs on a worker thread).
3. Poll `scan_workflow.running` / `await_complete()` until done.
4. Assert `scan_workflow.last_scan_error is None` and `not last_scan_canceled`.
5. Assert `scan_workflow.current_scan_label` is set, and the persisted output
   (CSV under `data_dir`, or `session_data` rows when a DB is configured) is
   non-empty.

> Robustness note: this test currently assumes a sensor is present and will error
> rather than skip if run with a console but no sensor — see §8.

---

## 6. Error and Edge Case Tests

Error-path coverage lives in `tests/test_errors.py`. The pure-code cases (CRC,
mutable-default isolation, `CommandError` API) run anywhere; the device cases are
hardware-gated. What's actually verified today:

- **Packet framing** — `test_value_error_on_crc_mismatch`: corrupt an incoming
  buffer, `UartPacket(buffer=...)` raises `ValueError`. `test_mutable_default_arg_isolation`:
  two packets built without explicit data don't share one list.
- **Bad command** — `test_command_error_or_timeout_on_bad_subtype`: a bad
  opcode raises `CommandError`/`TimeoutError`.
- **`CommandError` contract** — optional vs populated `response`, and that it
  subclasses `RuntimeError`.
- **Idempotent ping / connectivity** — `test_double_ping_after_connect`,
  `test_sensor_double_ping`, `test_sensor_ping_after_is_connected`.

**Intentionally NOT covered** (decided 2026-05-29): connection-robustness edge
tests — device-absent timeout, idempotent connect/disconnect, USB-error
propagation on `release()`, and any USB power-cycle / hotplug-reconnect path.
These are deliberately out of the current HIL scope (the transport-down
cancellation path *is* covered by the pure-software
`tests/test_comm_transport_down.py`).

---

## 7. HIL CI — how it runs today

> This section *describes the deployed setup*, not a proposal. The YAML and
> runner notes below (§7.2) are illustrative of the real workflows.

### 7.1 Layout and config (as built)

Tests live flat under `tests/` (HIL tests) with the pure-software pipeline tier
under `tests/test_pipeline/`. There is **no** `tests/hardware/` subfolder and
**no** `pytest.ini` — pytest is configured entirely in `pyproject.toml`:

```toml
[tool.pytest.ini_options]
testpaths = ["tests"]
addopts = "-v --tb=short -m 'not fpga and not imu'"   # fpga/imu excluded by default
timeout = 30
markers = [
    "console", "sensor", "sensor_left", "sensor_right",
    "slow", "destructive", "sequence", "fpga", "imu",
]
```

Note `fpga` and `imu` are **excluded by default** (temporarily skipped); opt in
explicitly with `-m fpga` / `-m imu`. Hardware tests gate on
`console` / `sensor` markers and skip gracefully when the device is absent
(§7.5).

### 7.2 GitHub Actions hardware runner

Two workflows are deployed: **`hardware-tests.yml`** runs the fast,
non-destructive subset (`-m "not destructive and not slow"`) on push/PR to
`testing`; **`hardware-tests-full.yml`** runs the full suite (including
destructive tests) on `release`. Both target `runs-on: [self-hosted, hardware,
openmotion]`. There is, by design, **no software-only `ubuntu-latest` job** — the
pure-software tier runs locally or when the rig picks it up.

Self-hosted runners are required for hardware-in-the-loop tests. Each runner machine must have a console module and at least one sensor module permanently attached. The runner must have `libusb` installed and the appropriate udev rules (Linux) or WinUSB driver (Windows) configured.

**Runner registration:**
```bash
# On the runner machine
./config.sh --url https://github.com/<org>/<repo> \
            --token <RUNNER_TOKEN> \
            --labels "hardware,openmotion"
```

**Workflow file** (`.github/workflows/hardware-tests.yml`):

```yaml
name: Hardware Tests

on:
  push:
    branches: [testing]
  pull_request:
    branches: [testing]

jobs:
  hardware-tests:
    runs-on: [self-hosted, hardware, openmotion]
    timeout-minutes: 30

    steps:
      - uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.11"

      - name: Install dependencies
        run: pip install -e ".[dev]"

      - name: Run fast non-destructive tests
        run: |
          pytest tests/ \
            -m "not destructive and not slow" \
            --junitxml=reports/junit.xml \
            --html=reports/report.html

      - name: Upload test report
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: test-report
          path: reports/

      - name: Publish test results
        if: always()
        uses: EnricoMi/publish-unit-test-result-action@v2
        with:
          files: reports/junit.xml
```

**Separate workflow for destructive/slow tests** — run only on explicit `workflow_dispatch` trigger or on the `release` branch:

```yaml
name: Full Hardware Validation

on:
  workflow_dispatch:
  push:
    branches: [release]

jobs:
  full-validation:
    runs-on: [self-hosted, hardware, openmotion]
    timeout-minutes: 120
    steps:
      - uses: actions/checkout@v4
      - name: Install dependencies
        run: pip install -e ".[dev]"
      - name: Run full suite including destructive tests
        run: pytest tests/ --junitxml=reports/junit_full.xml
      - name: Upload report
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: full-test-report
          path: reports/
```

### 7.3 Test output and GitHub integration

Use `--junitxml` to produce JUnit XML that GitHub Actions parses natively for the test summary panel. The `EnricoMi/publish-unit-test-result-action` action publishes per-test pass/fail results as PR check annotations and a summary comment.

For richer HTML reports, `pytest-html` generates a self-contained report artifact that can be browsed from the Actions run page.

### 7.4 Isolation and ordering

Mark any test that modifies persistent device state (fan speed, TEC setpoint, trigger config, config flash) with `@pytest.fixture(autouse=True)` teardown functions to restore defaults. Use `pytest-ordering` or alphabetical test naming to enforce a stable execution order when sequence tests depend on prior state.

### 7.5 Skipping gracefully when hardware is absent

The session-scoped fixtures (Section 1.2) call `pytest.skip()` when the targeted device is not present. This allows the same test suite to run in CI with only partial hardware attached — for example, a console-only runner will run all console tests and skip sensor tests without failing the build.

Set the environment variable `OPENMOTION_DEMO=1` to substitute `demo_mode=True` in the fixtures, enabling a fully offline dry-run of the test scaffolding without any physical hardware.

---

## 8. Known coverage gaps / backlog

HIL coverage we intend to build but have not yet. Reviewed and prioritised
2026-05-29. (Connection-robustness / USB power-cycle / hotplug-reconnect coverage
is intentionally **excluded** from this backlog — see §6.)

### HIGH

- **Streaming acquisition** (replaces the dead §5.4). Run a short scan through
  `omotion/pipeline/` end-to-end — `LiveUsbSource` → default stage chain → a
  collector sink — and assert: N corrected frames emitted, `abs_frame_id`
  monotonic per side, BFI/BVI in a physically plausible range. This is the actual
  product acquisition path and has no HIL test today.
- **EFT timestamp-repair: clean-scan false-positive + stim detection check.**
  Two-part verification of `TimestampRepairStage` against live hardware
  (background: the recorded "clean" fixture `owYWB8TN` turned out to have been
  captured with the stimulator running at 300 ms cadence, whose signature —
  one packet per ~12 frames stamped ~10 ms early, dt pattern `25,…,15,35,…,25`
  per camera — the detector correctly caught; we have never verified the
  no-stim baseline on hardware).
  - **(a) Clean baseline — no alerts for no reason.** Stimulator OFF, run a
    ≥5-minute scan with the scan DB enabled. Assert: zero `"ts_corrected"` /
    `"nan_filled"` rows in `session_data`, **zero WARNINGs** from
    `timestamp_repair` (the terminal stop frame — the firmware's laser-off
    frame fires ~151 ms off-grid at every scan stop — is reclassified at
    INFO and must not warn), and `session_meta` has **no** `"diagnostics"`
    key. Repeat across both sensor modules and with IMU streaming on/off,
    since periodic firmware tasks are the suspected jitter source if this
    ever trips.
  - **(b) Stim — detector and corrector behave.** Stimulator ON at 300 ms,
    same scan. Assert: misalignment windows are detected at the stim cadence
    (~3.3 Hz), `session_meta["diagnostics"]` records them, and the corrected
    timestamps land back on the 25 ms frame grid — per camera,
    `|t(fid) − t(anchor) − (fid − anchor)·25.02 ms| < tolerance` for the
    re-timestamped rows, against the raw CSV as the pre-repair reference.
- **Dual-sensor aligned-frame acquisition** (replaces the dead §5.8). Bring up
  left + right, run a short streamed scan, assert both sides produce frames with
  matching `abs_frame_id` within each `FrameBatch` (frame alignment across
  sensors). Core to any 2-sensor system.

### MEDIUM

- **I2C write→read roundtrip** (§2.4). Today only the bad-address path is tested;
  add a write-then-read on a safe scratch register via the console I2C
  pass-through.
- **UART CRC-corruption + timeout** (§4.1). Today only sync-mode blocking is
  tested; add: a corrupted response is rejected, and a command to a silent device
  raises `TimeoutError` within the configured window.
- **IMU streaming** (§3.10). Exercise the `StreamInterface` IF2 path: enable IMU
  streaming, receive N samples, assert plausible accel/gyro magnitudes.

### Hardware-safety fix (then re-enable)

- **DFU enter (console §2.11 + sensor §3.11)** — both are currently
  `@pytest.mark.skip(reason="DFU temporarily disabled")` because the test left
  the hardware in a bad state. Fix the test so it enters DFU and **returns the
  device cleanly to application mode** (or is safe to leave in DFU for the rest of
  the destructive run), then remove the skip. Belongs in the `destructive` lane.

> Maintenance: when a backlog item ships as a real test, delete it here — the
> tests are the registry (§Overview, "How to read this plan").
