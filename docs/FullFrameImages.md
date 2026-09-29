# Full-frame camera images (parked 2026-09-29)

**Status: working, parked.** Complete 1920×1280 RAW10 images from the
OX02C1B cameras, laser-lit, at 1 Hz per camera. Everything below is in pushed
branches (nothing merged). Hub for history and evidence: epic
[openmotion-bloodflow-app#480](https://github.com/OpenwaterHealth/openmotion-bloodflow-app/issues/480)
(the checkpoint comments dated 2026-09-29 have the full measurements).

## What works (bench, 2026-09-29)

| Mode | Command-line knobs | Result |
|---|---|---|
| 1 camera | `--cam N` (STRIDE 40) | Complete frame every **1.000 s**, every frame from exactly 40 production exposures, zero overruns |
| 2 cameras | `--cam A B` | Mostly 1 Hz; occasional extra second when USB saturates |
| 8 cameras, one module | `--cam 0 1 2 3 4 5 6 7 --stride 255` | A frame per camera every ~6.4 s; cameras 1-3 sometimes need an extra cycle |
| All 16 cameras | one module at a time | Verified laser-lit on both modules: mean 183-355 DN (dark 128), speckle K 0.36-0.58 |

Images are saved losslessly: `*_raw16.png` (16-bit PNG, raw 10-bit values
0..1023, unscaled) and `*.npy` (uint16 1280×1920), bit-exact. `*_preview8.png`
is an 8-bit contrast-stretched copy for viewing only.

## How it works: stride composite

A single-exposure readout (the original "drip-scan") was set aside, not ruled
out. The FPGA→MCU link drains one 2408-B line in ~0.69 ms, so a
single-exposure readout needs ≥0.7 ms rows. At long row times the dark frame's
whole-frame std grows. Measured at a constant 650 µs exposure on camera 0
(16× analog gain):

| Row time (µs) | 9 | 18 | 27 | 42 | 84 | 167 | 335 | 836 |
|---|---|---|---|---|---|---|---|---|
| Pixel std (DN) | 17.0 | 17.3 | 17.8 | 19.0 | 23.3 | 50 | 86 | ~140, 20% clipped |

**Re-analysis (2026-09-29): that excess is almost entirely a fixed per-pixel
pattern.** Two dark frames at identical 836 µs timing correlate r = 0.99 and
differ by only ~16-18 DN, which is production-level temporal noise. So
subtracting a matched dark frame might bring a single exposure back to
production quality. This is untested, and three obstacles are known:
- Pixels clipped at 0 can't be recovered, so the black level would need
  raising.
- The pattern shifted between two runs 15 s apart at different exposure
  settings, so the dark frame must match the image's settings and be taken
  beside it.
- Only the 16× camera was measured.

See "Open work".

The stride composite instead runs production-quality 40 Hz, laser-synced frames with
18 µs rows (HTS 866 × VTS 1380, 36-row exposure). The camera FPGA's **STRIDE**
register makes frame *k* send only lines `(k mod STRIDE) + j·STRIDE`, and the
phase advances by one line per frame. So STRIDE consecutive frames cover every
line once: STRIDE 40 gives a complete image per second, assembled on the host
from 40 exposures 25 ms apart. On a static phantom this is indistinguishable
from a single exposure: speckle autocorrelation is isotropic, with no row
artifacts. On a moving target, adjacent rows come from different exposures.

Pieces:

- **camera-fpga** `feature/8-drip-scan-single-frame` @ `64f9216` (RTL
  `68c9972`, no PR): register map v3.
  - Adds the STRIDE register `0x0A`.
  - Sweep mode pushes each selected line as a 2408-B packet: header + RAW10 +
    CRC-16.
  - Prebuilt bitstream:
    `tools/full_frame_capture/validated_bitstream/HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit`
    (SHA256 `799c2837…a782d`).
- **sensor-fw** `feature/99-drip-scan-image-mode` @ `252042d`: image receive
  mode (`OW_CAMERA_IMAGE_MODE` 0x30).
  - Each camera receives into a circular 2-line DMA ring.
  - Lines are batched into zero-copy USB transfers on the HISTO endpoint.
  - Includes the #96 DIEPEMPMSK race fix.
  - The exit reply carries a per-camera loss breakdown.
  - Draft PR #100.
- **SDK** `feature/167-drip-scan-capture` @ `ee6ae7d` (this branch):
  - `omotion.ImageCapture.capture_composite_frames()`, `CompositeAssembler`.
  - Composite timing profiles in `config.py`.
  - CLI `scripts/full_frame_1hz.py`.
  - Bench tools in `scripts/full_frame_bench/`.
  - Draft PR #168.

## Resuming on a bench

1. **Sensors must be bare-metal** (`MotionSensor.get_boot_mode()` →
   `BARE_METAL`). Secure-bootloader units only accept images signed in
   sensor-fw CI, and CI signs branch builds as FwVersion 1, below the units'
   anti-rollback floor. A bootloader bench needs a signed build at the current
   FwVersion first.
2. **Build the sensor firmware with the map-v3 bitstream:**
   ```powershell
   cd openmotion-sensor-fw   # on feature/99-drip-scan-image-mode
   copy ..\openmotion-camera-fpga\tools\full_frame_capture\validated_bitstream\HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit fpga\openmotion-camera-fpga.bin
   cmake --preset Debug -DBARE_METAL=ON ; cmake --build build\Debug   # bare-metal Debug: deploy.py reads build\Debug; Release does not boot
   python scripts\deploy.py --device left --no-confirm --power-cycle-cmd "python ..\openmotion-bloodflow-app\tests\shelly.py cycle"
   ```
   Flash the full image, not `--fw-only`, the first time: the capture
   force-loads the bitstream from sensor flash (`0x081A0000`) into each
   camera's FPGA SRAM (~10 s per camera, volatile).
3. **Run** (close the app first; USB is exclusive):
   ```powershell
   cd openmotion-sdk   # on feature/167-drip-scan-capture, with this checkout on PYTHONPATH
   python scripts\full_frame_1hz.py --side left --cam 0 --frames 10 --out ff_out --live
   python scripts\full_frame_1hz.py --side right --cam 0 1 2 3 4 5 6 7 --stride 255 --frames 2 --out ff_all8
   ```
   Options:
   - `--no-laser` gives dark frames.
   - `--no-fpga-load` reuses the SRAM image loaded earlier in this power
     cycle.
   - `--laser-delay` overrides `LaserPulseDelayUsec`.

## Traps learned the hard way (don't relearn)

- **Laser dark-slot footgun.** With `LaserPulseSkipInterval = 0`, console
  firmware never leaves its initial dark laser slot. That slot fires at
  `delay + LaserPulseSkipDelayUsec`, outside the exposure, and is a 25.7 ms
  one-shot that swallows every other FSIN. The capture therefore also zeroes
  `LaserPulseSkipDelayUsec`. Symptom: every camera black with the laser
  visibly firing.
- **Laser warm-up.** The first exposure after the trigger starts is black and
  the next two are ~5% dim. The assembler skips 3.
- **Minimum VTS is 1378.** Below it the sensor stops framing in every mode.
  This is why the old 1312-row sweep profile "stopped framing".
- **Write order in trigger mode.** Shrink VTS before stretching HTS; restore
  HTS first. An intermediate frame longer than the FSIN period wedges trigger
  mode.
- **Arm with FSIN stopped.** Retime, set STRIDE and arm all sweeps before
  `start_trigger()`, and stop the trigger before any teardown command. COMM
  commands against a live line stream can wedge the COMM endpoint.
- **Sweep arm order** (FPGA): target 4095 → sweep bit → start line. Otherwise
  single-line mode pushes a 4100-B legacy packet and auto-increments the
  target, and row 0 never arrives.
- **Image mode before FSIN.** Enter firmware image mode before starting FSIN.
  In histogram mode the stall detector power-cycles a camera for 10 s after 3
  missed frames.
- **No USB printf during image streaming.** Leave `DEBUG_FLAG_USB_PRINTF` off:
  it wedges COMM (reproduced). Diagnose through the firmware's image-mode exit
  reply instead (`MotionSensor.image_mode_exit_status()`).
- **Fast CRC.** Line CRC uses `binascii.crc_hqx(data, 0xFFFF)`, identical to
  `util_crc16` and 80× faster. The pure-Python CRC starved the USB reader.
- **0x5000.** Its power-up value is 0x34 (defect-pixel correction and white
  balance off), not the datasheet's 0x3E. Writing 0x3E turns on DPC.
- **Diamond license.** A silent ~49 s `pnmainc` exit means the Diamond license
  has expired. It was renewed 2026-09-29.

## Open work (when this is picked back up)

1. **8 cameras at 1 Hz.**
   - Needs ~26 MB/s of USB per sensor. The current per-packet HISTO chaining
     moves ~1,850 lines/s (~4.5 MB/s), and every 8-camera loss is USB staging
     overflow (zero link errors).
   - Options: fix multi-packet IN transfers for image batches, or enable OTG
     USB DMA. A first multi-packet attempt stalled until the 500 ms stuck-TX
     watchdog fired and was reverted, root cause unknown.
   - Also: all cameras push the same rows at the same instant. A per-camera
     phase offset (RTL: seed the stride phase from the start-line register)
     would spread that load.
2. **Rebase.** SDK branch is ~190 commits behind `next`, sensor-fw ~13. The
   camera-fpga branch sits on unmerged `feature/5` (PR #7).
3. **Console firmware.** Make `LaserPulseSkipInterval = 0` mean "no dark
   frames", and reject laser one-shots longer than the FSIN period.
4. **App integration** (none yet): a viewer or export in bloodflow-app.
5. **Single exposure with matched dark subtraction.** The test: on a 1× gain
   camera at ≥0.7 ms rows, capture dark and laser-lit single exposures back to
   back at identical settings, then compare the dark-subtracted noise and
   speckle contrast against the stride composite. If it holds up, every image
   comes from one exposure (still ~1 s per frame and the same USB ceiling).

Bench data and the experiment scripts from 2026-09-29 are archived in
`Projects/investigations/full_frame_1hz_2026-09-29/` on the bench PC.
