# OX02C1B full-frame raw images through the cold-start warm-up dip

Openwater Health, captured 30 September 2026.

Full-resolution raw frames (1920 × 1280, 10-bit) from three OX02C1B cameras of
one OpenMOTION sensor module, recorded from camera power-up through the
warm-up sensitivity dip and into the thermal plateau.

## Summary

| Camera | Analog gain | Plateau signal (DN above black level) | Dip | Minimum (s after power-up) | Loss across the array, 5th–95th percentile of 32 × 32-pixel blocks |
|---|---|---|---|---|---|
| 2 | 4× | 68 | 7.0 % | 155 | 6.3–7.2 % |
| 7 | 4× | 77 | 14.6 % | 201 | 13.3–15.0 %, smooth left-to-right gradient |
| 8 | 16× | 368 | 8.2 % | 160 | 7.7–8.4 % |

The loss is spread across the whole pixel array; it is not concentrated in any
region. The black level stays at 127–128 DN throughout.

The plots in `figures/` show each camera's dip curve and die temperature,
the mean plateau and trough images, and the loss map.

## How the data was taken

**Module and cameras.** One sensor module with eight OX02C1B cameras. Cameras
1, 2, 7 and 8 were powered, configured and triggered at 40 Hz, as in our
standard scan; cameras 3–6 were unpowered. Two runs: run FFDIP_01 imaged
cameras 1 and 2 (camera 1 barely dips and is not included), run FFDIP_02
imaged cameras 7 and 8.

**Cold start.** Before each run the whole module was unpowered for 2 hours
with a cooling fan on. Die temperatures were 26–29 °C when streaming began.

**Illumination.** A stable external light source, monitored by a photodiode
(`run_logs/*_photodiode.csv`). The source was switched off for 1 s every 60 s,
and for 2 s at the start and end of imaging, to give a black-level reference.
No laser was used.

**Sensor settings.** Our production initialization, unchanged:

| Setting | Value |
|---|---|
| Row time | 9.032 µs (HTS 432, VTS 2768) |
| Exposure | 72 rows |
| Frame trigger | external FSIN, 40 Hz |
| Analog gain | camera 2: 4×, camera 7: 4×, camera 8: 16× (digital gain 1×) |
| 0x4001 / 0x0104 / 0x3400 | 0x23 / 0x00 / 0x0c |
| BLC | on, black level target 128 |

**Time base.** All times are seconds since camera power-up. After power-up,
our camera FPGA images are loaded (about 10.5 s per camera), then the sensors
are configured and streaming starts, 44.7 s after power-up in both runs. The
first part of the descent is therefore not captured; every minimum is.

**How each image is assembled.** Our USB link cannot carry every full frame
at 40 Hz, so each image is built from consecutive exposures. With STRIDE 159,
exposure *k* sends only rows `(k mod 159) + j·159`, and the row phase advances
by one row per exposure; 159 consecutive exposures (about 4 s) cover every row
once. Rows lost on the link are filled from a later exposure, so some images
span up to about 7 s. One image per camera arrives every 4–7 s. The sensor
itself runs its normal production timing; only the readout to the host differs
from a normal scan.

**Imaging firmware.** The camera FPGA image used for this readout heats the
cameras more than a normal scan (camera 7 reached 87 °C instead of 78 °C), and
the dips are correspondingly somewhat deeper than in our standard scans.

## Files

```
index.csv                       one row per exported image (see columns below)
summary.csv                     the summary table above
cam2/, cam7/, cam8/
  frames/camN_KKK_tTTTTT.Ts.png 16-bit grayscale PNG, raw 10-bit values (0-1023), unscaled
  camN_rows.npz                 per-row metadata for every exported image (see below)
  camN_all_frames_curve.csv     dark-corrected mean of EVERY image in the run (not only exported ones)
figures/camN_dip_and_loss_map.png
run_logs/
  FFDIP_0x_camera_telemetry.csv  die temperatures (tpm0, tpm1), on-chip mean, gain, exposure
                                 for all four powered cameras, every 10 s (run 2) or 1 s (run 1)
  FFDIP_0x_source_off_windows.csv when the light source was switched off and on
  FFDIP_0x_photodiode.csv         light-source power
```

Exported images per camera: every image from the first 7 minutes after
power-up (75–80 images) plus 20 images spread over the plateau (10–26 minutes).

### index.csv columns

| Column | Meaning |
|---|---|
| `camera`, `run`, `file` | Camera number (1–8), run, PNG path |
| `set` | `early` (first 7 minutes) or `plateau` |
| `t_since_power_on_s` | Centre of the image's lit exposures, s after power-up |
| `first_row_s`, `last_row_s` | Earliest and latest row exposure in the image |
| `n_exposures` | Number of distinct exposures the image was assembled from |
| `die_temp_tpm0_c`, `die_temp_tpm1_c`, `die_temp_avg_c` | On-die temperatures, interpolated to the image time |
| `pedestal_dn` | Black level measured from source-off rows at that time |
| `lit_mean_minus_pedestal_dn` | Mean of lit rows, central columns 200–1719, minus black level |
| `pct_of_plateau` | The same as a percentage of this camera's plateau |
| `source_correction_factor` | Light-source step correction applied to the two columns above (1 = none) |
| `n_lit_rows`, `n_dark_rows`, `n_partial_rows` | Rows exposed with the source on, off, and switching |

Camera 2: die-temperature readings stopped 195 s after power-up in run
FFDIP_01 (a readout issue on our side), so its later images have no
temperature. Cameras 7 and 8 have temperatures throughout.

### camN_rows.npz

Arrays of shape (images, 1280), one row per image row, in the order of
`frame_file`:

- `row_time_s_since_power_on`: exposure time of each row, s after power-up.
  It is the row's arrival time corrected for the run's median USB delay:
  accurate to about 0.1 s in run FFDIP_02 (cameras 7, 8); in run FFDIP_01
  (camera 2) the delay varied, so times there are good to about 2 s
- `row_exposure_counter`: FPGA frame counter of the exposure that supplied the
  row (8-bit, wraps at 256); exact relative order
- `row_class`: 0 = lit, 1 = source off (black level only), 2 = partly lit
  (the source switched during the exposure)

Rows with class 1 appear as dark lines in the PNGs. Exclude rows with class
other than 0 when measuring the illuminated signal.

### Light-source steps

The source occasionally comes back from an off window about 5 % brighter or
dimmer and stays at that level until a later off window. The photodiode
records these steps and every camera follows them. The PNGs are raw and
uncorrected; `lit_mean_minus_pedestal_dn` and `pct_of_plateau` are corrected
using the photodiode (`source_correction_factor`).

## Reading the data (Python)

```python
import numpy as np, pandas as pd
from PIL import Image

idx = pd.read_csv("index.csv")
rows = np.load("cam7/cam7_rows.npz")
i = 40                                              # 41st exported cam 7 image
img = np.array(Image.open("cam7/frames/" + rows["frame_file"][i]))   # uint16, 1280 x 1920
lit = rows["row_class"][i] == 0
signal = img[lit, 200:1720].mean() - idx[idx.camera == 7].pedestal_dn.iloc[i]
```
