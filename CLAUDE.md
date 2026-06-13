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
- **Many threads run concurrently:** the `ConnectionMonitor` (a *pure detector* — diffs USB presence and posts connect/disconnect intents, never blocks) + one `ConnectWorker` daemon per handle (does the blocking connect/teardown, so a slow device can't starve the others) + per-endpoint stream readers + the console telemetry poller. Anything touching shared state must assume cross-thread emission. Connect pings are bounded (2 s) and cancellable; the console force-safes the trigger on every connect.
- **HIL note — console firmware wedge on abrupt USB cut:** when the console's USB is yanked mid-activity, the firmware command interface can stop responding (`No data received from UART within timeout`) until a **mains power cycle** (Shelly), even though the SDK reconnect logic is healthy (sensors recover fine, the monitor keeps cleanly retrying). This is a firmware bug — tracked in `openmotion-console-fw` issue #26 (host-loss laser watchdog + USB-disconnect command-interface reset).
- **Histogram packets have two CRCs when `DEBUG_FLAG_HISTO_CMP` is on** — transport CRC + decompressed-payload CRC. See `StreamInterface.py` ~lines 51–100.
- **Debug flags live in firmware**, set via `MotionSensor.set_debug_flags()`. Bits defined at `config.py:116-125` (`USB_PRINTF`, `HISTO_THROTTLE`, `FAKE_DATA`, `HISTO_CMP`, etc.).
- **Dark correction (PDC) is an active design area** — the per-frame PDC buffer was added recently; see memory `pdc_correction_design_paused.md` for the latest reasoning (amplitude scaling dropped, existing shot-noise correction is the fix).
- **Windows USB:** sensors need WinUSB via Zadig (`pyusb` + `libusb1`). Console uses the OS VCP driver (no Zadig).

## Branching and releases

- Work on `next`, PR into `next`. Releases require a `next → main` PR **before** tagging (enforced by `docs/Releasing.md`).
- **`docs/superpowers/` is gitignored — do NOT commit it.** The brainstorming / writing-plans skills write spec & plan artifacts there; they are local-only process notes. The skill steps that say "commit the design doc" do not apply to this repo. Put durable design/decisions in the PR description, a GitHub issue, or a real doc under `docs/` instead.
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
