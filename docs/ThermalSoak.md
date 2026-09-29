# Thermal soak: hours of full-frame images under near-normal operation

`scripts/thermal_soak.py` captures full-frame (1920×1280, raw 10-bit) composite
images from the cameras of one or both sensor modules for hours. It logs camera
die temperatures and console temperatures alongside, to show how the OX02C1B
sensors change as they heat. The sensors run as close to a normal scan as the
image path allows.

- Tracking: openmotion-sdk#296.
- Builds on the full-frame work parked under
  [bloodflow-app#480](https://github.com/OpenwaterHealth/openmotion-bloodflow-app/issues/480)
  (see `docs/FullFrameImages.md` for how the composite works).

## What is normal, what is not

| | Thermal soak | Normal scan |
|---|---|---|
| Sensor registers | **Unchanged** (production: HTS 432 × VTS 2768, 72-row exposure) | same |
| Trigger / laser | **Production:** 40 Hz, laser every frame except the scheduled dark frame every 600 | same |
| Cameras powered and triggered | **The scan mask** (`--scan-mask`, default `0xC3` = the app's clinical mask; the app powers the others off) | same |
| Camera FPGA, cameras not imaged | Map-v3 bitstream computing histograms (`--idle-fpga histogram`), not forwarded | the flash bitstream computing histograms, forwarded |
| Camera FPGA, imaged camera | Map-v3 image mode: pushes every STRIDE-th row | histograms |
| Sensor firmware / USB | Image mode for the imaged camera(s); rows instead of histograms | histograms |
| Rotation pause | Trigger stopped ~1-2 s every `--dwell-s` to switch the imaged camera | none |

On the bench-test firmware (`feature/99`) the flash-resident bitstream **is** the
map-v3 image, so a normal scan on that firmware also runs map v3 in histogram
mode.

Why production timing (the default) rather than the 1 Hz composite timing
(`--timing composite`: 18 µs rows, 1 image/s):
- **Readout wait.** It doubles how long each row waits in the pixel's storage
  node before readout, and that wait is what drives the thermal dark pattern
  (storage-node dark signal ∝ wait time; see `docs/FullFrameImages.md`).
- **Laser capture.** It captured only ~87% of the laser pulse
  (bench 2026-09-29).

The price is 2 s per image instead of 1 s.

## Image rate: USB burst budget

One sensor's USB path sustains ~1,850 rows/s. At production timing a frame's
1,280 rows are read out in the first 11.6 ms of each 25 ms frame, so what
matters is the rate during that burst. `composite_stride()` picks the STRIDE:

| Timing | Cameras streaming at once (`--concurrent`) | STRIDE | Image per camera every |
|---|---|---|---|
| production | 1 | 80 | 2.0 s |
| production | 2 | 159 | 4.0 s |
| production | 3 | 238 | 6.0 s |
| composite | 1 | 40 | 1.0 s |

With `--concurrent 1` and the 4-camera clinical mask, the script images one
camera for `--dwell-s` (default 120 s, i.e. ~58 images), then the next.

## Setting up the test rig

1. **Sensor firmware:** `openmotion-sensor-fw` `feature/99-drip-scan-image-mode`
   @ `252042d`, built bare-metal Debug with the map-v3 bitstream merged. Steps
   are in `docs/FullFrameImages.md` → "Resuming on a bench":
   - copy the bitstream to `fpga/openmotion-camera-fpga.bin`;
   - `cmake --preset Debug -DBARE_METAL=ON ; cmake --build build\Debug`;
   - `deploy.py`.

   **Secure-bootloader sensors cannot take this build** (anti-rollback floor).
   Check first with `MotionSensor.get_boot_mode()`.
2. **Console:** any production console firmware (only the trigger and laser are
   used).
3. **Host:** Python 3.12+, this SDK branch installed or on `PYTHONPATH`, plus
   `numpy`, and `matplotlib` and `pillow` for the plots/PNG export. Close the
   app: USB is exclusive.
4. **Sanity check** (one image, ~30 s):
   `python scripts/full_frame_1hz.py --side left --cam 1 --frames 2 --out check`.

## Running

```powershell
# 3 hours, both modules, clinical cameras 0/1/6/7 rotated one at a time
python scripts\thermal_soak.py --sides left right --duration-h 3 --out D:\soak_2026-10-01

# research mask, two cameras at a time, image every other composite
python scripts\thermal_soak.py --sides left --scan-mask 0x66 --concurrent 2 --save-every 2 --duration-h 4
```

Useful options:

| Option | Default | |
|---|---|---|
| `--scan-mask` | `0xC3` | cameras powered and triggered (normal scan mask); research is `0x66` |
| `--cams` | the scan mask | cameras imaged (must be in the mask) |
| `--concurrent` | 1 | cameras streaming at once per sensor |
| `--dwell-s` | 120 | seconds per camera group before rotating |
| `--timing` | production | `composite` = 1 image/s, less normal |
| `--laser-schedule` | production | `all-lit` removes the scheduled dark frame |
| `--no-laser` | off | dark soak |
| `--save-every` | 1 | save every Nth composite; stats are logged for all |
| `--telemetry-s` | 10 | camera and console telemetry period |
| `--max-die-c` | 105 | stop when any die temperature exceeds this; 0 = no limit |
| `--min-free-gb` | 10 | stop when free disk falls below this |
| `--stall-s` | 30 | recover when an imaged camera sends no rows this long |
| `--power-cycle-cmd` | none | run before a recovery reconnect (e.g. a Shelly relay command) |

Ctrl-C stops cleanly and keeps everything written so far.

## Outputs

| File | Content |
|---|---|
| `run.json` | Arguments, host, SDK `git describe`, sensor and console firmware, trigger config, STRIDE |
| `composites.csv` | One row per composite (see columns below) |
| `camera_telemetry.csv` | Every `--telemetry-s`, each camera of the scan mask: reading age (`age_ms`), die temps (`tpm_avg_c`, `tpm0_c`, `tpm1_c`), on-chip mean, gain, exposure, trigger errors. Cameras outside the mask are skipped: the firmware's telemetry cache keeps a powered-off camera's LAST reading and still marks it valid. |
| `console_telemetry.csv` | Console temperature sensors t1-t3 and TEC readings |
| `events.csv` | Segment starts/ends with the firmware's per-camera line accounting, errors, recoveries, camera dropouts, thermal limit |
| `images/<side>/cam<N>/*.npz` | Lossless composites (see below) |
| `soak.log` | Everything, including SDK logging |

`composites.csv` columns:
- **Time:** wall time, seconds since start.
- **Identity:** side, camera, gain, sequence number, segment.
- **Provenance:** exposures used, first/last frame counter.
- **Lit rows:** mean, std, speckle K (central ROI, minus `--pedestal`),
  per-quarter-height means.
- **Pixels:** saturated and zero fractions.
- **Dark rows:** count, mean and std of rows from scheduled dark exposures.
  Roughly one composite in 7 carries one, so this gives a dark-level reference
  over time.

Each `.npz` holds:

```python
z = np.load(path)
z["image"]      # uint16 1280x1920, raw 10-bit values
z["row_fc"]     # int16 per row: FPGA frame counter of the exposure that supplied it
z["row_t_s"]    # float64 per row: arrival time, seconds since the run started
z["dark_rows"]  # bool per row: from a scheduled dark (laser-skipped) exposure
json.loads(str(z["meta"]))   # side, cam, gain, frame counters, wall times, STRIDE, ...
```

Disk: ~3 MB per composite. That is ~5.4 GB/h per sensor at one image per 2 s;
the script prints its estimate at start and stops at `--min-free-gb`.

Plots:
`python scripts\thermal_soak_plot.py <run_dir> [--png-every 30]` writes
`soak_<side>.png`:
- die temperatures;
- lit mean, K and dark level per camera;
- console temperatures.

`--png-every N` also exports every Nth image as a 16-bit PNG plus a viewing
preview.

## Heat: read this before a long run

**Bench 2026-09-29 (left module, static phantom, open air):**

| Configuration | Die temperatures | Note |
|---|---|---|
| All 8 cameras in histogram mode at 40 Hz | 31-39 °C → **58-108 °C in ~4 min**, still rising | Camera 7 dropped from 84 °C to 45 °C, consistent with the camera-board regulator's thermal shutdown |
| 2 cameras powered | 80-87 °C in ~2.5 min, still rising | |

That is well above what the July camera-drift campaign logged (45-73 °C), and
not yet explained. A baseline on production firmware is owed.

The script therefore:
- powers only the scan mask;
- stops at `--max-die-c`;
- logs a `camera_dropout` event when a powered camera's telemetry goes invalid
  or stale (>5 s old), or its die temperature falls >15 °C between reads.

Watch `camera_telemetry.csv` / the plot early in the first run.

## Recovery

On a stall (no rows for `--stall-s`) or a transport error the script:
1. tears the sessions down;
2. optionally runs `--power-cycle-cmd`;
3. reconnects;
4. reloads the camera FPGAs (the map-v3 image is lost when a camera powers off)
   and carries on in the same output directory, up to `--max-recoveries`.

## Known limits

- Imaged cameras run image mode, not histograms (table above). A camera's image
  segment and its histogram-mode segments can be compared in
  `camera_telemetry.csv` (column `imaged`).
- The `--dwell-s` rotation stops the trigger for ~1-2 s; the first three
  exposures after each restart are skipped (laser warm-up).
- Speckle patterns decorrelate over minutes (laser drift); compare statistics
  over time, not pixels across hours.
