# Agent brief: capture full-frame image sequences on an Open-Motion rig

You are setting up an Open-Motion rig to record **sequences of full-frame
camera images** (1920×1280, raw 10-bit) with the stride-composite method. The
main use is a **thermal soak**: hours of images while logging camera die
temperatures, with the sensors running as close to a normal scan as possible.

Everything you need is on one SDK branch. This brief is the operating recipe;
the background lives in two guides on the same branch:
- `docs/ThermalSoak.md`: the soak tool in depth.
- `docs/FullFrameImages.md`: how the composite works, plus traps.

Tracking issue: OpenwaterHealth/openmotion-sdk#296 (draft PR #297).

## Ground rules: read first

1. **Laser.** The capture scripts fire the laser at production settings.
   - Before the first capture, have the human confirm the sensor modules sit on
     the phantom and the rig is set up for laser operation.
   - Never change the console's laser or safety configuration.
2. **Ask the human before:**
   - flashing firmware;
   - the first laser-on capture;
   - starting a run longer than 30 minutes;
   - power-cycling anything.
3. **USB is exclusive.**
   - Close the Open-Motion app before any script.
   - If an app instance is running that you did not start, ask; don't kill it.
4. **Never hard-kill a capture.**
   - Stop with Ctrl-C: the scripts tear down cleanly.
   - A killed process leaves the trigger and laser running. If one dies
     uncleanly, make sure its python process is gone, then run
     `python scripts\quiesce_rig.py`.
   - When you run a capture in the background, launch `python` itself (not a
     shell wrapper), so stopping the task stops python.
5. **Heat.** On the reference bench (2026-09-29):
   - all 8 cameras of a module at 40 Hz reached **108 °C** die temperature in
     ~4 min, and one camera dropped out;
   - the normal 4-camera scan mask settled at 60-90 °C.

   The soak stops itself above `--max-die-c` (105 °C). Watch
   `camera_telemetry.csv` in the first minutes of every run.
6. **Tests.** Don't run the whole SDK `pytest` suite with hardware plugged in:
   it drives the hardware. Only run these:
   `pytest tests/test_thermal_soak.py tests/test_image_composite.py tests/test_image_capture.py`
7. **No data in git.** Image data never goes into git.

## 0. What the rig needs

- A console (any production firmware) and one or two sensor modules, on USB.
  - Sensor USB needs the WinUSB driver: the Open-Motion driver package, or
    Zadig on VID `0x0483` / PID `0x5A5A`.
- **Sensor firmware:** `openmotion-sensor-fw` branch
  `feature/99-drip-scan-image-mode`, built **bare-metal** with the map-v3
  camera bitstream (step 3).
  - Sensors with the secure bootloader cannot take this build. Stop and tell
    the human if `rig_info.py` reports `BOOTLOADER`.
- **Host:** Windows, Python 3.12+, git. For building firmware (step 3):
  arm-none-eabi-gcc, CMake 3.22+, Ninja, and STM32_Programmer_CLI or dfu-util.
  Docker image `ghcr.io/openwaterhealth/stm32-build-env:latest` also builds it.

## 1. Get the code

```powershell
git clone https://github.com/OpenwaterHealth/openmotion-sdk.git
git -C openmotion-sdk checkout feature/296-composite-thermal-soak
git clone https://github.com/OpenwaterHealth/openmotion-sensor-fw.git
git -C openmotion-sensor-fw checkout feature/99-drip-scan-image-mode
git clone https://github.com/OpenwaterHealth/openmotion-camera-fpga.git
git -C openmotion-camera-fpga checkout feature/8-drip-scan-single-frame
```

The map-v3 bitstream is committed at
`openmotion-camera-fpga\tools\full_frame_capture\validated_bitstream\HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit`
(163,489 B, SHA256 `799c2837...a782d`). No FPGA tools are needed.

## 2. Install the SDK and check the rig

```powershell
cd openmotion-sdk
pip install -e .
pip install matplotlib pillow
python scripts\rig_info.py
```

`rig_info.py` is read-only. It reports:
- console firmware;
- per sensor: **boot mode**, **"image mode supported"**, camera power and die
  temperatures.

Die temperatures of powered-off cameras are stale: the firmware keeps their
last reading.

- **`BARE_METAL` and image mode supported `True`:** skip to step 4.
- **`BARE_METAL` but image mode `False`:** flash the firmware (step 3).
- **`BOOTLOADER`:** stop and tell the human.

## 3. Build and flash the sensor firmware (only if needed; ask first)

Order matters:
- The bare-metal CMake configure **always downloads the latest release
  bitstream** over `fpga\openmotion-camera-fpga.bin`, so copy map v3 in
  *after* configuring.
- The merge into the flash image happens at link time, so build clean.

```powershell
cd openmotion-sensor-fw
cmake --preset Debug -DBARE_METAL=ON
copy ..\openmotion-camera-fpga\tools\full_frame_capture\validated_bitstream\HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit fpga\openmotion-camera-fpga.bin
cmake --build build\Debug --clean-first
python ..\openmotion-sdk\scripts\check_fw_bitstream.py build\Debug\motion-sensor-fw.bin ..\openmotion-camera-fpga\tools\full_frame_capture\validated_bitstream\HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit
python scripts\deploy.py --device left --no-build --no-confirm
python scripts\deploy.py --device right --no-build --no-confirm
```

- `check_fw_bitstream.py` must print `OK` before you flash.
- Use the `Debug` preset. Release builds do not boot, and `deploy.py` only
  flashes `build\Debug`.
- Flash the full image (not `--fw-only`).
- The sensor needs a **power cycle** after flashing. `deploy.py
  --power-cycle-cmd "<cmd>"` runs one automatically if the rig has a
  switchable supply; otherwise ask the human to power-cycle.
- Re-run `rig_info.py` afterwards: image mode supported must be `True`.

## 4. Sanity capture (~1 min, laser on; ask first)

```powershell
cd openmotion-sdk
python scripts\full_frame_1hz.py --side left --cam 1 --frames 3 --out check_left
```

Expect lines like `cam1 frame 0: 1280 lines from 40 exposures, mean ...` and
`period 1.000 s`. The output folder holds:
- `left_cam1_000_raw16.png`: lossless 16-bit, raw values;
- `.npy`;
- `_preview8.png`: for viewing only;
- `meta.json`.

Open a preview: you should see laser speckle, not a flat grey frame. Dark rig
at ~128 DN means the laser isn't reaching the camera.

This quick check uses the 1 Hz composite timing. The soak (step 5) defaults to
production timing: 1 image per 2 s per camera.

## 5. The thermal soak

Always start with a 5-minute validation:

```powershell
python scripts\thermal_soak.py --sides left --duration-h 0.085 --dwell-s 60 --out soak_validation
python scripts\thermal_soak_plot.py soak_validation --png-every 20
```

Check before going long:
- `events.csv` has `segment_end` rows with `link_err` 0 and `stage_full` a
  few dozen at most per minute;
- `composites.csv` has a composite every ~2 s per imaged camera, with
  `n_exposures` mostly 80;
- `soak_left.png` looks sane: temperatures levelling off, and lit means well
  above 128.

Then the long run (ask the human for duration, sides and mask):

```powershell
python scripts\thermal_soak.py --sides left right --duration-h 3 --out soak_<date>
python scripts\thermal_soak_plot.py soak_<date> --png-every 60
```

Defaults are the normal-operation settings:
- the clinical scan mask `0xC3` (cameras 0, 1, 6, 7 powered and imaged,
  0-based indices);
- production sensor timing and the production laser schedule;
- one camera imaged at a time, rotating every 120 s.

Common changes:
- `--scan-mask 0x66`: the research mask.
- `--save-every N`: less disk; ~5.5 GB/h per sensor at N=1.
- `--power-cycle-cmd "<cmd>"`: lets stall recovery power-cycle the rig.

Full option list: `python scripts\thermal_soak.py -h`, and `docs\ThermalSoak.md`.

The run ends on duration, Ctrl-C, low disk (`--min-free-gb`), the die-temperature
limit, or too many recoveries. `events.csv` says why (`end` row).

## 6. Outputs

| File | |
|---|---|
| `run.json` | Arguments, versions, trigger config, STRIDE |
| `composites.csv` | One row per composite: time, camera, frame counters, lit mean/std, speckle K, dark-row stats, saturation |
| `camera_telemetry.csv` | Die temperatures etc. per powered camera every 10 s |
| `console_telemetry.csv` | Console temperatures and TEC every 10 s |
| `events.csv` | Segments with firmware loss counts, errors, recoveries, dropouts |
| `images\<side>\cam<N>\*.npz` | The images (see below) |
| `soak_<side>.png` | Plots |
| `soak.log` | Full log |

Each `.npz` holds:

```python
z = np.load(path)
z["image"]      # uint16 1280x1920, raw 10-bit values
z["row_fc"]     # per row: frame counter of the exposure it came from
z["row_t_s"]    # per row: arrival time, seconds since start
z["dark_rows"]  # per row: from a scheduled laser-off frame
meta = json.loads(str(z["meta"]))
```

A composite image is built from 80 consecutive 40 Hz exposures, each
contributing every 80th row. On a static target it is equivalent to one
exposure.

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `FPGA register map < v3 (no STRIDE)` | Firmware carries the release bitstream: redo step 3 in order, and check with `check_fw_bitstream.py` |
| `sensor not connected` / timeouts at start | App or another script holds USB; or the sensor needs a power cycle after flashing |
| All frames ~128 DN | Laser not reaching the cameras (fiber, TA, rig): ask the human. Don't touch console laser config |
| Many `stage_full` in `events.csv`, composites with `n_exposures` >> 80 | USB overload: keep `--concurrent 1` at production timing (2 max) |
| `camera_dropout` events, a camera's rows stop | Likely camera-board regulator thermal shutdown. The run recovers after `--stall-s`; report temperatures |
| `thermal_limit` end | Die temperature above `--max-die-c`: report to the human; don't just raise the limit |
| Laser still firing after a crash | Run `python scripts\quiesce_rig.py` (add `--power-off` to cool cameras) |

## 8. Report back

Post a comment on OpenwaterHealth/openmotion-sdk#296 (or give the human)
with:
- rig description;
- firmware / `rig_info.py` output;
- the command line;
- duration and composite count;
- the `end` reason from `events.csv`;
- any errors, recoveries, dropouts, and the peak die temperature per camera;
- `soak_<side>.png` and where the data folder is.

Keep the data folder on the rig machine or a share; don't upload it to git.
