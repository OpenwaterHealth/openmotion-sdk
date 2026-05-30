# AGENTS.md - openmotion-sdk

This repo is the Python SDK for Open-Motion hardware. It packages the `omotion`
library used by the bloodflow app and other tools.

## Architecture

- `omotion/MotionInterface.py`: high-level facade for connecting, scanning, and
  coordinating console and sensor modules.
- `omotion/MotionConsole.py`: console module wrapper; trigger, laser, TEC, fan,
  FPGA, telemetry, and controller commands.
- `omotion/MotionSensor.py`: sensor module wrapper; camera, histogram, IMU, DFU,
  and per-sensor operations.
- `omotion/MotionUart.py`: UART packet transport for the console.
- `omotion/CommInterface.py`: USB command/response transport for sensor
  interface 0.
- `omotion/StreamInterface.py`: USB streaming transport for histogram and IMU
  interfaces.
- `omotion/ScanWorkflow.py`: acquisition orchestration and scan lifecycle.
- `omotion/pipeline/`: the stage-based science pipeline (BFI/BVI, dark
  correction, sources, sinks, runner). This is where the science lives.
- `omotion/MotionProcessing.py`: wire-level histogram packet parsing only — a
  thin shim feeding the pipeline (the BFI/BVI science moved to `omotion/pipeline/`).
- `omotion/pipeline/sinks.py` (`CsvSink`, `ScanDBSink`) + `omotion/ScanDatabase.py`:
  scan persistence outputs.
- `omotion/CalibrationWorkflow.py`, `omotion/Calibration.py`: calibration flow
  and calibration math.
- `omotion/config.py`: protocol constants and USB identifiers.

The SDK is the right place for protocol behavior, hardware sequencing, stream
handling, science pipeline logic, and APIs shared by apps.

## Tests And Hardware

Pytest is configured in `pyproject.toml` with default markers excluding `fpga`
and `imu`:

```powershell
python -m pytest
python -m pytest tests/test_pipeline/          # pure-software pipeline tests, no hardware
python -m pytest -m "not fpga and not imu"
```

Many tests and scripts still require connected console/sensor hardware. If
hardware is unavailable, prefer focused unit tests and fixture-backed pipeline
tests, then document the hardware gap.

Useful fixture-heavy tests include pipeline, CSV, database, dark-frame,
telemetry, frame-id, and reduced-mode tests under `tests/`.

## Development Commands

```powershell
python -m pip install -e .[dev]
python -m build
python -m pip install --force-reinstall dist/openmotion_sdk-*.whl
python -c "import omotion; print(omotion.__file__)"
```

Windows USB behavior depends on WinUSB/libusb drivers. Driver files live under
`drivers/` and `winusb-driver/`.

## Packaging And Versioning

- Package metadata is in `pyproject.toml`.
- Versioning uses `setuptools_scm` from git tags such as `v1.2.3` or `1.2.3`.
- Release notes and PyPI guidance live in `PYPI_SETUP.md` and `docs/Releasing.md`.
- Keep package data rules in `pyproject.toml` in sync when adding bundled
  binaries, config, bitstreams, or dfu-util assets.

## Editing Guidance

- Preserve the layered API: app-facing workflows should sit above console/sensor
  wrappers, which sit above UART/USB transports.
- Avoid adding app-specific UI concepts here; expose generic SDK state, callbacks,
  events, or results.
- Be careful around stream lifecycle, disconnect handling, thread shutdown, and
  callback ownership. These paths affect the bloodflow app directly.
- Do not treat generated scan output, logs, build artifacts, or cache directories
  as source.
