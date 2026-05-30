# openmotion-sdk

Host-side Python SDK for **Open-Motion** — Openwater's optical speckle imaging
system for non-invasive blood-flow monitoring. The package `omotion` (PyPI:
`openmotion-sdk`) is the library every other host tool talks through: it
discovers and connects the hardware, drives scans, and turns the camera
histogram streams into Blood Flow Index (BFI) / Blood Volume Index (BVI) plus a
queryable scan database.

It talks to an **Open-Motion Console** over UART and to up to **two sensor
modules** (8 × OV2312 cameras each) over USB bulk. The console and sensor
firmware live in the `openmotion-console-fw` and `openmotion-sensor-fw` repos;
this library just speaks their wire protocol.

## The one front door: `MotionInterface`

Everything goes through the `MotionInterface` facade — discover, connect, scan,
configure, calibrate, read results back. Application code never touches the
transport or device classes directly.

```python
from omotion import MotionInterface
from omotion.ScanWorkflow import ScanRequest

iface = MotionInterface(
    data_dir="C:/scans",      # output root (optional)
    operator_id="alice",
)
iface.start(wait=True, wait_timeout=3.0)        # discover + connect (spawns daemons)

console_ok, left_ok, right_ok = iface.is_device_connected()

iface.start_scan(ScanRequest(
    subject_id="subj-001",
    duration_sec=60,
    left_camera_mask=0xFF, right_camera_mask=0xFF,
))
iface.scan_workflow.await_complete()            # scans run on a worker thread
if iface.scan_workflow.last_scan_error:
    print("scan failed:", iface.scan_workflow.last_scan_error)

iface.stop()
```

### Where the scan data goes

- **No database configured (the common dev case):** the SDK runs in a convenient
  CSV mode — a corrected CSV is written under `data_dir` so a scan is never
  silently unrecorded.
- **`scan_db_path` set:** the SQLite scan database is the system of record
  (per-camera BFI/BVI + raw frames + metadata), read back via `ScanDatabase` /
  `SessionPlayback`. The corrected CSV becomes opt-in.

See [`docs/API.md`](docs/API.md) §"Where the data goes" for the full model.

## Without hardware

Pass `demo_mode=True` (or set `OPENMOTION_DEMO=1`) to skip USB/serial discovery
and generate synthetic data. The same API works headless — signals fall back
from `pyqtSignal` to `MotionSignal` automatically.

```python
iface = MotionInterface(demo_mode=True)
iface.start()
```

## Runnable examples

[`scripts/sdk_examples.py`](scripts/sdk_examples.py) drives each operation
against **connected hardware** and prints the result:

```
python scripts/sdk_examples.py connect          # connect + version
python scripts/sdk_examples.py configure        # configure cameras
python scripts/sdk_examples.py contact-quality  # per-camera contact-quality verdicts
python scripts/sdk_examples.py scan             # run a short scan (laser on)
python scripts/sdk_examples.py read-scan        # summarize the scan DB (read-only, no hw)
python scripts/sdk_examples.py                  # all of the above on one connection
```

Plot a finished scan with
[`scripts/visualize_scan.py`](scripts/visualize_scan.py):

```
python scripts/visualize_scan.py --csv <scan_id>_<subject>.csv   # -> <stem>_viz.png
```

## Install

```powershell
# Editable install for local dev
pip install -e ".[dev]"

# Or build a wheel for an app to consume
python -m build                                  # -> dist/openmotion_sdk-*.whl
pip install --force-reinstall dist/openmotion_sdk-*.whl
```

- **Python 3.12+.** Version is computed from git tags via `setuptools_scm` —
  never edit a version string by hand; tag and push to release.
- **Windows USB:** sensors need WinUSB via Zadig (`pyusb` + `libusb`); the
  console uses the OS VCP driver. `dfu-util` is vendored under `omotion/dfu-util/`.

```powershell
# quick runtime check (device bound to WinUSB/libusbK)
python -c "import usb, omotion.usb_backend as ub; print(ub.get_libusb1_backend())"
```

## Documentation

| Doc | What it covers |
|---|---|
| [`docs/API.md`](docs/API.md) | **Public API guide** — start here for consumer usage. |
| [`docs/Architecture.md`](docs/Architecture.md) | Layer diagram, module reference, transport details. |
| [`docs/SciencePipeline.md`](docs/SciencePipeline.md) | BFI/BVI computation — the `omotion/pipeline/` stage chain. |
| [`docs/ScanDatabase.md`](docs/ScanDatabase.md) | SQLite scan-database schema. |
| [`docs/scan-sequencing.md`](docs/scan-sequencing.md) | Frame-ID unwrapping + histogram packet ordering. |
| [`docs/ConsoleTelemetry.md`](docs/ConsoleTelemetry.md) | PDC (dark correction) + TEC telemetry. |
| [`docs/TestPlan.md`](docs/TestPlan.md) | Hardware-in-the-loop test plan. |
| [`docs/Releasing.md`](docs/Releasing.md) | Release process (`next → main` gate before tagging). |

## License

AGPL-3.0.
