# Full-frame camera images (parked 2026-09-29)

**Status: working, parked.** Complete 1920×1280 RAW10 images from the
OX02C1B cameras, laser-lit, at 1 Hz per camera, two ways:
- **Stride composite** (the SDK API): 40 production exposures interleaved row
  by row.
- **Single exposure** (bench script, tested 2026-09-29): the whole image from
  one laser pulse, with a matched dark frame subtracted. Everything below is in pushed
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
| Single exposure, 1 camera | `scripts/full_frame_bench/single_exposure.py --cam N` | One laser pulse per image at 1 Hz. After matched-dark subtraction, noise and speckle contrast match the composite (see below) |
| Single exposure, all 16 cameras | `single_exposure.py --cam 0 7 1 6 2 5 3 4 --warmup-s 30`, one module at a time | All 16 laser-lit. **12 of 16 match the composite; idx 6 and 7 on both modules saturate** (see below) |

Images are saved losslessly: `*_raw16.png` (16-bit PNG, raw 10-bit values
0..1023, unscaled) and `*.npy` (uint16 1280×1920), bit-exact. `*_preview8.png`
is an 8-bit contrast-stretched copy for viewing only.

## How it works: stride composite

Why not simply read one exposure out slowly? The FPGA→MCU link drains one
2408-B line in ~0.69 ms, so a single-exposure readout needs ≥0.7 ms rows,
and at those row times the raw frame carries a large dark pattern (next
section). The stride composite avoids that; the single-exposure mode
subtracts it.

The stride composite runs production-quality 40 Hz, laser-synced frames with
18 µs rows (HTS 866 × VTS 1380, 36-row exposure). The camera FPGA's **STRIDE**
register makes frame *k* send only lines `(k mod STRIDE) + j·STRIDE`, and the
phase advances by one line per frame. So STRIDE consecutive frames cover every
line once: STRIDE 40 gives a complete image per second, assembled on the host
from 40 exposures 25 ms apart. On a static phantom this is indistinguishable
from a single exposure: speckle autocorrelation is isotropic, with no row
artifacts. On a moving target, adjacent rows come from different exposures.

## Single exposure with matched dark subtraction (tested 2026-09-29)

Every image comes from one laser pulse. Bench script:
`scripts/full_frame_bench/single_exposure.py`. Analysis:
`single_exposure_analyze.py`.

**Recipe:**
- **Trigger mode** at the console's 1.0 Hz floor.
- **Timing:** HTS 34300 × VTS 1378 (the sensor minimum) → 717 µs rows (4% over
  the line drain) and a 0.988 s frame, inside the 1 s FSIN period.
- **Exposure:** 8 rows.
- **Laser:** `LaserPulseDelayUsec` 8300. The exposure opens ~10 rows after
  FSIN, which is ~90 µs at production row time but ~7 ms here. The fully lit
  plateau is a 7.5-9.0 ms delay; outside ~7.0-9.5 ms the frame is dark.
- **BLC off** (`0x4001 = 0x00`): a fixed pedestal (255 DN at 1×, 489 at 16×).
  With BLC on, the servo shifts the whole frame by up to 40 DN between frames.
- **Dark reference:** in the same stream, laser toggled on the console only
  (between frames), dark phases before and after the lit phase, interpolated
  per pixel in time.

**What the raw frame carries:** a storage-node dark signal. Charge waits in
each pixel's storage node until its row is read, up to ~1 s for the last
rows:
- It scales with row index: pattern std ~21 → 90 DN (1×) and 61 → 211 DN (16×)
  from the top rows to the bottom rows.
- It grows as the sensor warms after timing starts: ~4× over the first
  ~2 minutes, then levels off.
- It is the same pixels throughout (r 0.91 between early and late darks), so it
  subtracts cleanly.

(Checkpoint 1's "independent of row index" was wrong.)

**Results** (left module, static phantom):

| | Camera 3 (1×) single | Camera 3 composite | Camera 0 (16×) single | Camera 0 composite |
|---|---|---|---|---|
| Frame-to-frame noise | 1.8 DN | 1.2 DN | 15.8 DN | 15.2 DN |
| Residual after dark subtraction (held-out darks) | 2.0-2.1 DN | n/a | 16.4 DN | n/a |
| Laser signal | 210 DN | 185 DN | 125 DN | 113 DN |
| Speckle K | 0.55 | 0.57 | 0.410 | 0.405-0.410 |
| Saturated pixels | 0.17% | 0% | 2.1% (5% in bottom rows) | 0% |

**All 16 cameras** (single_exposure.py, cameras in turn, one camera streaming
at a time since USB carries ~1,850 rows/s per sensor; about 27 s per camera
after a shared warm-up):

| Module | idx (gain) | 0 (16×) | 1 (4×) | 2 (2×) | 3 (1×) | 4 (1×) | 5 (2×) | 6 (4×) | 7 (16×) |
|---|---|---|---|---|---|---|---|---|---|
| Left | K single / composite | 0.44 / 0.41 | 0.37 / 0.37 | 0.50 / 0.50 | 0.57 / 0.58 | 0.55 / 0.58 | 0.53 / 0.50 | **0.71 / 0.38** | **1.26 / 0.42** |
| Left | saturated | 3.3% | 3.1% | 1.2% | 0.5% | 0.9% | 3.0% | **18.5%** | **28.3%** |
| Right | K single / composite | 0.43 / 0.45 | 0.35 / 0.36 | 0.47 / 0.50 | 0.53 / 0.57 | 0.55 / 0.57 | 0.49 / 0.50 | **0.51 / 0.37** | **0.62 / 0.50** |
| Right | saturated | 1.0% | 0.2% | 0.0% | 0.2% | 0.1% | 0.2% | **4.5%** | **6.9%** |

On every camera the residual after dark subtraction sits within ~10% of that
camera's frame-to-frame noise.

**idx 6 and 7 fail on saturation.** At the BLC-off pedestal (~500-550 DN at
4×/16×), their storage dark pattern overflows 10 bits, and a pixel saturated
in both lit and dark frames subtracts to 0. Two things drive it:
- **Position:** idx 6/7 carry a 1.5-2× larger storage pattern than idx 0/1
  at the same gain and similar time into the run (on both modules). That
  looks like those positions running warmer; not confirmed.
- **Warm-up time:** the pattern grows as the module warms.
  - Left run: 90 s warm-up, idx 6/7 captured last (~6 min in): 18-28%
    saturated.
  - Right run: 30 s warm-up, high-gain cameras first, idx 7 second: 5-7%.
  - Capture order matters, but reordering alone doesn't fix it.

Single-exposure K runs ~2-8% below the composite on the unsaturated cameras,
consistent with the 1 Hz pulse difference below.

Caveats:
- **The laser differs at 1 Hz.** Pulses at a 1 Hz rep rate are ~13% brighter
  and the speckle pattern differs from 40 Hz: captures on either side of a
  rep-rate switch don't correlate, and even two composites 2.5 min apart gave
  r 0.02. Within a 1 Hz run, consecutive frames correlate r 0.98 (1×).
- **16× cameras lose ~2% of pixels to saturation** (pedestal + storage pattern
  > 1023).
- **Free-run mode doesn't work for lit frames.** The exposure register doesn't
  behave as rows there: 25 ms and 250 ms exposures caught no 40 Hz laser pulse,
  1 s saturated.
- **Tested so far:** all 16 cameras, one camera streaming at a time. Frames
  within one camera are 1 s apart; cameras are ~27 s apart.

## Pieces

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
- **SDK** `feature/167-drip-scan-capture` (this branch):
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
2. **Build the sensor firmware with the map-v3 bitstream.** Order matters:
   - A bare-metal configure always downloads the latest *release* bitstream
     over `fpga/openmotion-camera-fpga.bin`, so copy map v3 in AFTER
     configuring.
   - The merge into the flash image is a post-link step, so build from
     clean.
   ```powershell
   cd openmotion-sensor-fw   # on feature/99-drip-scan-image-mode
   cmake --preset Debug -DBARE_METAL=ON     # configures build\Debug; downloads the release bitstream
   copy ..\openmotion-camera-fpga\tools\full_frame_capture\validated_bitstream\HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit fpga\openmotion-camera-fpga.bin
   cmake --build build\Debug --clean-first
   python ..\openmotion-sdk\scripts\check_fw_bitstream.py build\Debug\motion-sensor-fw.bin ..\openmotion-camera-fpga\tools\full_frame_capture\validated_bitstream\HistoFPGAFw_impl1_2026-09-29_map-v3-stride.bit
   python scripts\deploy.py --device left --no-build --no-confirm --power-cycle-cmd "python ..\openmotion-bloodflow-app\tests\shelly.py cycle"
   ```
   - Use the `Debug` preset with `-DBARE_METAL=ON`. `deploy.py` only knows
     `build\Debug` / `build\Release`, and Release does not boot.
   - `check_fw_bitstream.py` (SDK `feature/296-composite-thermal-soak`)
     compares the bytes at flash offset 0x1A0000 with the map-v3 file.
   - A wrong bitstream shows up at capture time as
     `FPGA register map < v3 (no STRIDE)`.

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
- **Single exposure: laser delay.** The exposure opens ~10 rows after FSIN.
  At 717 µs rows that is ~7 ms, so the production 100 µs laser delay lands
  before the exposure and every frame is dark. Use ~8.3 ms. An exposure of 1
  row has almost no window at all (the length looks like ~N−4 rows).
- **Single exposure: teardown order.** Stop the trigger, let the frame in
  flight drain (~1 s), then send sensor commands. Restoring registers while
  lines stream wedged COMM (reproduced 2026-09-29; Shelly recovered it).
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
5. **Single exposure: from bench script to product.**
   - An SDK API that keeps a rolling dark reference: periodic dark frames,
     after a warm-up of ~2 min.
   - Fix saturation on idx 6/7 (and the few % on idx 0/1/5). Options:
     - A lower pedestal: BLC on with a low target, plus per-frame offset
       correction or frozen BLC offsets.
     - Reduced gain for those positions in this mode.
     - Shorter storage time, i.e. a faster link.
     - Checking whether those positions really run warmer (camera
       temperature telemetry).
   - Deciding whether the 1 Hz laser rep rate is acceptable for the images'
     purpose.

Bench data and the experiment scripts from 2026-09-29 are archived in
`Projects/investigations/full_frame_1hz_2026-09-29/` on the bench PC (single-exposure
test data and the comparison figure under `single_exposure_test/`).
