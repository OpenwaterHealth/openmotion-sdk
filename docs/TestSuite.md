# Open-Motion SDK — Test Suite Overview

A whole-suite map of the `omotion` tests, grouped by **SDK architecture layer**.
This is the high-level "what do we test and how well" view; it spans both the
pure-software tier and the hardware-in-the-loop (HIL) tier.

- For the **HIL test plan + coverage backlog**, see [`TestPlan.md`](TestPlan.md).
- The **executable tests under `tests/` are the authoritative registry** — this
  doc is a navigational model, not a per-test list. Counts drift; regenerate with
  the commands at the bottom.

> Snapshot: 2026-05-29, branch `test-rehab`.

## Headline statistics

- **504** collected test items across **51** files (~**417** test *functions* —
  the gap is `@parametrize`, mostly `test_sensor.py` running each test on left
  **and** right).
- **337 pure-software (67%) / 165 hardware-gated (33%).**
- Markers: `console` 50, `sensor` 118 (×2 parametrized), `slow` 57,
  `sequence` 16, `fpga` 23, `imu` 6, `destructive` **5**. `fpga` + `imu` are
  excluded by default (`pyproject.toml` addopts).
- The **science pipeline is the largest single area: 167 tests (33%)**, all
  pure-software.

## Coverage by SDK layer

Counts are test-function definitions (parametrization expands the collected
total to 504). Ratings: **Strong** (near-complete), Adequate (core covered,
known holes), **Thin** (minimal), **Gap** (little/none).

| Layer | Key modules | ~Fns | Nature | Coverage | Notable holes |
|---|---|---:|---|---|---|
| **1. Facade** | `MotionInterface` | 6 | sw | **Thin** | Only construction + handle wiring (`test_motion_interface.py`). No facade-level scan/connect integration test off-hardware. |
| **2. Device wrappers** | `MotionConsole`, `MotionSensor` | 93 | hw (mostly) | **Strong** | ~1:1 with the console/sensor command surface (`test_console.py` 43, `test_sensor.py` 43 ×2 sides). Software units: `test_pedestal_height.py`, `test_get_pdc_buffer.py`. Holes: I2C write→read roundtrip; IMU streaming. |
| **3. Transport & protocol** | `MotionUart`, `CommInterface`, `StreamInterface`, `UartPacket`, `CommandError` | 18 | sw + hw | Adequate | `test_errors.py` (CRC, CommandError), `test_comm_paths.py` (path isolation, hw), `test_comm_transport_down.py` (cancellation, sw). Hole: UART CRC-corruption + timeout HIL. |
| **4. Workflows / orchestration** | `ScanWorkflow`, `CalibrationWorkflow`, `ContactQualityWorkflow` | 89 | sw + hw | **Strong** (compute/lifecycle) | Scan (`test_scan_workflow.py`, `test_run_collection_scan.py`, `test_reduced_mode.py`, `test_sequences.py` hw), contact-quality (14), calibration workflow/compute/console/procedure (45). Holes: streaming + dual-sensor **acquisition** HIL (backlog HIGH). |
| **5. Science pipeline** | `omotion/pipeline/` (+ `MotionProcessing` parse shim) | 170 | sw | **Strong** | Every stage, the dark model (estimators/history), and infra (sources, sinks, runner, tee, factory) have tests; golden-replay + determinism guard the math. Largest area. No major holes. |
| **6. Storage & playback** | `ScanDatabase`, `ScanDBSink`, `SessionPlayback` | 5 (+10 sink) | sw | Adequate | `test_scan_database.py` (DB roundtrips) + `test_pipeline/test_scan_db_sink.py` (sink, counted under pipeline). Hole: `SessionPlayback` has no dedicated test. |
| **7. Telemetry** | `ConsoleTelemetry`, `console_telemetry_conversions` | 6 (+hw) | sw + hw | Adequate | `test_console_telemetry_unit.py` (poller/PDC drain/conversions) + the telemetry subset of `test_console.py` (hw poller lifecycle). |
| **8. Programming / DFU** | `FPGAProgrammer`, `DFUProgrammer`, `jedecParser` | 2 | hw | **Thin / Gap** | `test_wheel_dfu.py` (path), `test_zz_dfu.py` (skipped). DFU enter is disabled (damages hw — backlog: fix safely + re-enable). FPGA-prog is exercised only via the console/sensor device tests; JED parser only via `scripts/`. |
| **9. Config / support** | `config`, `Calibration` (math), `laser` | 28 | sw | Adequate | `test_laser.py` (params, FPGA map, apply-power), `test_calibration.py` (Calibration parse/serialize/validate math). |
| **— Hotplug** | `connection_monitor`, `connection_state` | 0 | — | **Gap (by decision)** | No tests for USB topology assignment / hotplug reconnect. Deliberately out of scope for now (see `TestPlan.md` §6/§8). |

## Software vs hardware tiers

- **Pure-software (337)** — runs anywhere, no hardware: the entire
  `tests/test_pipeline/` suite (167), calibration compute/math, contact-quality,
  scan-DB, facade/config units, laser, telemetry unit, transport-down. Run with:
  `pytest -m "not console and not sensor and not destructive"`.
- **Hardware-gated (165)** — `console` / `sensor` markers; skip gracefully when
  the device is absent. Run on the self-hosted rig (see `TestPlan.md` §7).

## Regenerating the counts

```powershell
python -m pytest tests/ -m "" --collect-only -q | tail -1                       # total
python -m pytest tests/ -m "not console and not sensor and not destructive" --collect-only -q | tail -1   # software tier
python -m pytest tests/test_pipeline/ --collect-only -q | tail -1               # science pipeline
```
