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
pytest tests/test_pipeline_csv.py                   # pure-software test, no hw
```

- Python **3.12+**. Version is computed from git tags via `setuptools_scm` — never edit a version string by hand. To cut a release, tag and push. Tags use **semantic versioning** (`MAJOR.MINOR.PATCH`, e.g. `1.6.0`, `1.6.0-rc.1`).
- No Makefile, no pre-commit, no lint/format/type-check configured. If you reach for `black`/`ruff`/`mypy`, they aren't wired up here.
- `dfu-util` binaries are vendored under `omotion/dfu-util/` and shipped with the wheel (see `pyproject.toml` `[tool.setuptools.package-data]`).

## Layout

Three-tier API: facade → device wrapper → transport.

| Layer | Module | Lines | What lives here |
|---|---|---:|---|
| Facade | `omotion/MotionInterface.py` | 616 | `MOTIONInterface` — discover, connect, run scans. **Start here.** |
| Device | `omotion/MotionConsole.py` | **2815** | UART-side device. Trigger, TEC, fan, FPGA programming, telemetry. Biggest file in the repo. |
| Device | `omotion/MotionSensor.py` | 1316 | USB-side device. Cameras, histograms, IMU, DFU. |
| Transport | `omotion/MotionUart.py` | 252 | UART framing, CRC-16. |
| Transport | `omotion/CommInterface.py` | 380 | USB bulk command/response (sensor IF 0). |
| Transport | `omotion/StreamInterface.py` | 382 | USB bulk streaming (sensor IF 1 = histo, IF 2 = IMU). Daemon reader thread per endpoint. |
| Workflow | `omotion/ScanWorkflow.py` | 1222 | Full acquisition orchestration. |
| Workflow | `omotion/CalibrationWorkflow.py` | 1222 | Per-camera gain / I_max calibration. Newer; not in old docs. |
| Science | `omotion/MotionProcessing.py` | **1941** | Histogram parsing + BFI/BVI pipeline. |
| Config | `omotion/config.py` | 291 | VID/PID, baud, packet types, command opcodes, `DEBUG_FLAG_*` bits. Single source of truth. |
| Programming | `omotion/FPGAProgrammer.py` | 567 | Page-by-page Lattice XO2 flash. |
| Programming | `omotion/DFUProgrammer.py` | 346 | STM32 DFU over USB (uses vendored `dfu-util`). |
| Telemetry | `omotion/ConsoleTelemetry.py` | 460 | PDC + TEC poller. Daemon thread, 10 Hz slow / 1 Hz fast. |
| Hotplug | `omotion/connection_monitor.py`, `connection_state.py`, `hotplug/` | — | Daemon thread that watches Win32/libusb hotplug events and emits to the app thread. |
| Storage | `omotion/ScanDatabase.py`, `ScanDBSink.py`, `SessionPlayback.py` | — | SQLite scan sink + playback (issue #92). |

**Call graph for the common case:**

```
MOTIONInterface.start()
 ├── ConnectionMonitor (daemon thread — hotplug)
 ├── motion.console      = MotionConsole(MotionUart(pyserial))
 └── motion.left/right   = MotionComposite(CommInterface(pyusb), StreamInterface(pyusb))
```

Signals are `pyqtSignal` when PyQt is importable, otherwise a fallback `MOTIONSignal` — same API both ways, so headless scripts work identically to the apps.

## Working without hardware

- `MOTIONInterface(demo_mode=True)` **or** `OPENMOTION_DEMO=1` — skips device discovery, generates fake data. The first thing to reach for if a script is hanging on enumeration.
- Pure-software tests that run anywhere: `test_pipeline_csv.py`, `test_rolling_average.py`, `test_frame_id_unwrapper.py`, `test_calibration_workflow_compute.py`, `test_realtime_dark_estimator.py`.
- Pure-software scripts: `scripts/test_jed_parser.py`, `scripts/test_github_release.py`, `scripts/run_pipeline_csv_tests.py`, `scripts/plot_telemetry.py`, `scripts/view_corrected_scan.py`.

## Existing in-repo docs (read before re-explaining)

| Doc | Purpose |
|---|---|
| `docs/Architecture.md` | Comprehensive — layer diagram, module reference, transport details. |
| `docs/scan-sequencing.md` | Frame ID unwrapping + histogram packet ordering. |
| `docs/SciencePipeline.md` | BFI/BVI computation. |
| `docs/PipelineComparison.md` | CSV vs DB output. |
| `docs/ScanDatabase.md` | SQLite schema. |
| `docs/ScanDatabase-HardwareVerification.md` | DB sink test plan. |
| `docs/ConsoleTelemetry.md` | PDC (dark correction) + TEC telemetry. |
| `docs/CameraArrangement.md` | Camera orientation reference. |
| `docs/Releasing.md` | Release process — `next → main` PR enforced before tagging. |
| `docs/TestPlan.md` | Hardware test strategy. |

## Gotchas

- **`MotionConsole.py` is 2815 lines** — read its module-level docstring + method docstrings before scrolling. The class is the source-of-truth for the console command set; there's no separate API reference doc.
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
| Add a science-pipeline step | `omotion/MotionProcessing.py`, then a unit test in `tests/test_pipeline_csv.py`. |
| Change connection/discovery behavior | `omotion/connection_monitor.py` + `omotion/MotionInterface.py`. |
| Flash sensor firmware from a script | `omotion/DFUProgrammer.py`; see `scripts/flash_sensors.py` for usage. |
| Flash a console FPGA | `omotion/FPGAProgrammer.py`; the JED parser is `scripts/test_jed_parser.py`. |
