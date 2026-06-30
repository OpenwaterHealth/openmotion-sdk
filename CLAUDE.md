# openmotion-sdk — Claude guide

Python package `omotion` (PyPI name `openmotion-pylib`, AGPL-3.0). The host-side library every other repo talks through. Apps import the wheel; firmware doesn't import anything here, it just speaks the same wire protocol.

Cross-repo context: [../CLAUDE.md](../CLAUDE.md).

## Run / install

```powershell
# Editable install for local dev (apps that pip install -e ../openmotion-sdk get changes live)
pip install -e ".[dev]"

# Build a wheel for the apps to consume
python -m build         # → dist/openmotion_sdk-X.Y.Z-py3-none-any.whl

# Test suite — defaults exclude fpga and imu markers (see pyproject.toml)
pytest tests/                                       # all (needs hardware)
pytest tests/ -m "not destructive and not slow"     # CI smoke subset
pytest tests/test_pipeline/                          # pure-software pipeline tests, no hw
```

- Python **3.12+**. Version is computed from git tags via `setuptools_scm` — never edit a version string by hand. To cut a release, tag and push. Tags use **semantic versioning** (`MAJOR.MINOR.PATCH`, e.g. `1.6.0`, `1.6.0-rc.1`).
- No Makefile, no pre-commit, no lint/format/type-check configured. If you reach for `black`/`ruff`/`mypy`, they aren't wired up here.
- `dfu-util` binaries are vendored under `omotion/dfu-util/` and shipped with the wheel (see `pyproject.toml` `[tool.setuptools.package-data]`).

## Layout

Three-tier API: facade → device wrapper → transport.

| Layer | Module | Lines | What lives here |
|---|---|---:|---|
| Facade | `omotion/MotionInterface.py` | 492 | `MotionInterface` — discover, connect, run scans. **Start here. The one and only front door.** |
| Device | `omotion/MotionConsole.py` | **2934** | UART-side device. Trigger, TEC, fan, FPGA programming, telemetry. Biggest file in the repo. |
| Device | `omotion/MotionSensor.py` | 1316 | USB-side device. Cameras, histograms, IMU, DFU. |
| Transport | `omotion/MotionUart.py` | 252 | UART framing, CRC-16. |
| Transport | `omotion/CommInterface.py` | 380 | USB bulk command/response (sensor IF 0). |
| Transport | `omotion/StreamInterface.py` | 382 | USB bulk streaming (sensor IF 1 = histo, IF 2 = IMU). Daemon reader thread per endpoint. |
| Workflow | `omotion/ScanWorkflow.py` | 1110 | Full acquisition orchestration. Owns hardware bring-up + lifecycle; feeds frames into the pipeline. |
| Workflow | `omotion/CalibrationWorkflow.py` | 1643 | Per-camera gain / I_max calibration. |
| **Science** | `omotion/pipeline/` | — | **Stage-based BFI/BVI pipeline** (`sources`, stages in `pipeline.py`, `sinks`, `runner`, `factory`, `pedestal`, `batch`, `tee`, `telemetry`). **The science lives here.** Full reference: `docs/SciencePipeline.md`. |
| Science | `omotion/MotionProcessing.py` | 730 | Wire-level histogram packet **parsing only** — a thin shim feeding the pipeline. (BFI/BVI moved to `omotion/pipeline/`; this module is slated to dissolve eventually.) |
| Config | `omotion/config.py` | 291 | VID/PID, baud, packet types, command opcodes, `DEBUG_FLAG_*` bits. Single source of truth. |
| Programming | `omotion/FPGAProgrammer.py` | 567 | Page-by-page Lattice XO2 flash. |
| Programming | `omotion/DFUProgrammer.py` | 346 | STM32 DFU over USB (uses vendored `dfu-util`). |
| Telemetry | `omotion/ConsoleTelemetry.py` | 478 | PDC + TEC poller. Daemon thread. Raw-ADC→units conversion lives in `console_telemetry_conversions.py`. |
| Hotplug | `omotion/connection_monitor.py`, `connection_state.py`, `hotplug/` | — | Daemon thread that watches Win32/libusb hotplug events and emits to the app thread. |
| Storage | `omotion/pipeline/sinks.py` (`CsvSink`, `ScanDBSink`), `omotion/ScanDatabase.py`, `SessionPlayback.py` | — | CSV + SQLite scan sinks + playback (issue #92). |

**Call graph for the common case:**

```
MotionInterface.start()
 ├── ConnectionMonitor (daemon thread — hotplug)
 ├── motion.console      = MotionConsole(MotionUart(pyserial))
 └── motion.left/right   = MotionComposite(CommInterface(pyusb), StreamInterface(pyusb))
```

Signals are `pyqtSignal` when PyQt is importable, otherwise a fallback `MotionSignal` — same API both ways, so headless scripts work identically to the apps.

## Working without hardware

- `MotionInterface(demo_mode=True)` **or** `OPENMOTION_DEMO=1` — skips device discovery, generates fake data. The first thing to reach for if a script is hanging on enumeration. (Note: the *new pipeline* scan path does not yet support demo mode end-to-end — demo sensors have no real `uart`, so a full `start_scan` in demo mode will not stream. Demo mode is for discovery/connection-level work.)
- Pure-software tests that run anywhere: the entire `tests/test_pipeline/` suite, plus `test_calibration_workflow_compute.py`, `test_contact_quality_workflow.py`, `test_scan_database.py`, `test_console_telemetry_unit.py`, `test_pedestal_height.py`. Run them with `pytest -m "not console and not sensor and not destructive"`.
- Pure-software scripts: `scripts/test_jed_parser.py`, `scripts/test_github_release.py`, `scripts/run_pipeline_csv_tests.py`, `scripts/plot_telemetry.py`, `scripts/view_corrected_scan.py`.

## Running a scan end-to-end (headless)

`MotionInterface.start_scan(ScanRequest(...))` is the front door, but it **only enables
streaming on already-powered-and-configured cameras** — it does *not* bring them up. The app
does that on connect (`motion_connector._run_sensor_init`). A bare script must replicate it or
you get `Failed to enable camera` / firmware "refuse stream enable on dead or unpowered camera"
and **0 USB chunks**. This bites hardest right after a power-cycle or firmware flash, which
cold-boots the cameras **OFF and unconfigured**. Close the app first — USB access is exclusive.

```python
from omotion import MotionInterface
from omotion.ScanWorkflow import ScanRequest, ConfigureRequest
import threading

iface = MotionInterface(data_dir="scan_out", scan_db_path="scan_out/scans.db")
iface.start()
iface.wait_for_ready(console=True, sensors=2, timeout=15)
assert all(iface.is_device_connected())            # (console, left, right)

MASK = 0x66
# 1) cold-boot bring-up — power THEN configure, else 0 frames flow:
iface.left.enable_camera_power(0xFF); iface.right.enable_camera_power(0xFF)
done = threading.Event()
iface.start_configure_camera_sensors(
    ConfigureRequest(left_camera_mask=MASK, right_camera_mask=MASK,
                     power_off_unused_cameras=False),
    on_complete_fn=lambda r: done.set())
done.wait(90)
iface.apply_laser_power()                           # laser-on only (cold-start: driver regs were cleared)
# iface.left.set_debug_flags(0x01)                  # optional: surface firmware printf over USB (USB_PRINTF)

# 2) run one scan, block until done:
req = ScanRequest(subject_id="TEST", duration_sec=15,
                  left_camera_mask=MASK, right_camera_mask=MASK,
                  disable_laser=False,              # False = external (console/laser) FSIN; True = internal
                  write_telemetry_csv=True,
                  raw_save_max_duration_s=None)     # None = full-scan raw CSV; 0 = no raw tee
iface.start_scan(req)
iface.scan_workflow.await_complete(timeout_sec=req.duration_sec + 15)
iface.stop()
```

- **Output** lands in `data_dir`: `{YYYYMMDD_HHMMSS}_{subject}_{side}_mask{MASK:02X}_raw.csv`
  (columns `cam_id,frame_id,timestamp_s,type,<1024 bins>,temperature,sum`), a telemetry CSV,
  and `scans.db`. `timestamp_s` is **scan-relative** — a *negative* value is a leftover frame
  from a previous scan re-shipped into this one.
- This path runs the full pipeline (classify + timestamp-repair + sinks), so it reproduces the
  app's warnings. `scripts/capture_data.py` is the *low-level* alternative — raw bytes to `.raw`,
  no pipeline, no CSV.
- Calibration / contact-quality sub-scans share the same engine via `run_collection_scan(...)`
  (ScanWorkflow.py) with a collector sink and `skip_default_storage=True`.

## Existing in-repo docs (read before re-explaining)

| Doc | Purpose |
|---|---|
| `docs/API.md` | **Public API / interface guide** — how a host app or script drives the SDK (`MotionInterface`, scans, reading the DB, custom sinks). Start here for consumer-facing usage. |
| `docs/Architecture.md` | Comprehensive — layer diagram, module reference, transport details. |
| `docs/scan-sequencing.md` | Frame ID unwrapping + histogram packet ordering. |
| `docs/SciencePipeline.md` | BFI/BVI computation. |
| `docs/ScanDatabase.md` | SQLite schema. |
| `docs/ScanDatabase-HardwareVerification.md` | DB sink test plan. |
| `docs/ConsoleTelemetry.md` | PDC (dark correction) + TEC telemetry. |
| `docs/CameraArrangement.md` | Camera orientation reference. |
| `docs/Releasing.md` | Release process — `next → main` PR enforced before tagging. |
| `docs/TestPlan.md` | Hardware-in-the-loop test plan + coverage backlog. |
| `docs/TestSuite.md` | Whole-suite overview — all tests grouped by SDK layer, sw/hw split, per-layer coverage. |

## Gotchas

- **`MotionConsole.py` is 2815 lines** — read its module-level docstring + method docstrings before scrolling. The class is the source-of-truth for the low-level console command set. For the consumer-facing public interface (facade, scans, DB read, pipeline sinks) see `docs/API.md`.
- **Three transport threads can run concurrently:** ConnectionMonitor + per-endpoint stream readers + telemetry poller. Anything touching shared state needs to assume cross-thread emission.
- **Histogram packets have two CRCs when `DEBUG_FLAG_HISTO_CMP` is on** — transport CRC + decompressed-payload CRC. See `StreamInterface.py` ~lines 51–100.
- **Debug flags live in firmware**, set via `MotionSensor.set_debug_flags()`. Bits defined at `config.py:116-125` (`USB_PRINTF`, `HISTO_THROTTLE`, `FAKE_DATA`, `HISTO_CMP`, etc.).
- **Dark correction (PDC) is an active design area** — the per-frame PDC buffer was added recently; see memory `pdc_correction_design_paused.md` for the latest reasoning (amplitude scaling dropped, existing shot-noise correction is the fix).
- **Windows USB:** sensors need WinUSB via Zadig (`pyusb` + `libusb1`). Console uses the OS VCP driver (no Zadig).

## Branching and releases

- Work on `next`, PR into `next`. Releases require a `next → main` PR **before** tagging (enforced by `docs/Releasing.md`).
- Current branches active in recent log: `feature/122-*`, `feature/calibration`, `feature/contact-quality-rehash`, `feature/compression`.
- CI: `.github/workflows/hardware-tests.yml` runs on the `testing` branch via self-hosted hardware runner, with `-m "not destructive and not slow"`. Full suite at `hardware-tests-full.yml`. Wheel build + upload via `publish-pypi.yml` / `release-build.yml`.

## "Start here" by task

| Task | First file |
|---|---|
| Add a host-side command for the console | `omotion/MotionConsole.py` — find a sibling method, copy its pattern; opcode lives in `omotion/config.py`. |
| Add a host-side command for a sensor | `omotion/MotionSensor.py` + `omotion/CommInterface.py`; opcode in `omotion/config.py`. |
| Change histogram parsing | `omotion/MotionProcessing.py` (parsing) + `omotion/StreamInterface.py` (framing). |
| Add a science-pipeline step | `omotion/pipeline/` (add a stage in `pipeline.py`, wire it in `factory.py`), then a unit test under `tests/test_pipeline/`. |
| Change connection/discovery behavior | `omotion/connection_monitor.py` + `omotion/MotionInterface.py`. |
| Flash sensor firmware from a script | `scripts/test_sensor_program.py` (CLI over `omotion/DFUProgrammer.py`); `scripts/enter_dfu.py` for DFU entry alone. |
| Flash a console FPGA | `omotion/FPGAProgrammer.py`; the JED parser is `scripts/test_jed_parser.py`. |
