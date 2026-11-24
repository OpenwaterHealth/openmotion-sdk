# Seedless C_min measurement

Engineering measurement, requested by Brad (bahartl) 2026-11-24. Not part of
WI-00015. Static phantom only. **Reports only: C_min is never written to the
device.**

```
python -m omotion.scripts.seedless_cmin --side right --output-dir cmin_out
```

The test-app runs the same module from its Procedures pane
("Seedless C_min (engineering)").

## What it measures

With the seed laser off the TA emits low-coherence light, so the speckle
contrast it gives on the static phantom is the floor the system can see:
C_min. Contrast estimates depend on intensity, so C_min is taken at the image
mean the seeded system produces. The TA pulse is lengthened until the
seedless mean matches; when the longest pulse that fits the exposure is not
enough, the TA drive current is stepped up, 100 mA at a time, to at most 5 A.

| Phase | Seed | Exposure | TA pulse | Frames |
|---|---|---|---|---|
| Warm-up | on | 648 us | console's (500 us) | 800 light frames (20 s) at a time, until the total output (average of the per-camera ratios) holds within 0.2 % (at most 5 min) |
| Nominal | on | 648 us | console's (500 us) | 80 light frames (2 s) |
| Search bursts | **off** | 2295 us | 500 us, then rounds of 3 widths per drive current | 10 light frames per burst |
| C_min captures | **off** | 2295 us | matched width and current | 3 x 40 light frames (3 x 1 s) |
| Nominal again | on | 648 us | console's, restored | 40 light frames (1 s) |

* All analysed frames are **light** frames from the science pipeline's
  corrected stream: dark-subtracted mean, shot-noise-corrected std,
  contrast = std/mean. Each capture is its own short scan, so its dark
  frames (and dark correction) are at the same exposure as its light frames.
* Brightness ratio = mean over cameras of (seedless mean / nominal mean) for
  the same camera. Matched when within 3 % of 1 (`--tolerance`).
* Search (Brad / Ethan, 2026-11-24): after the 500 us seed-off check, each
  round runs 3 bursts at 50 %, 75 % and 100 % of the 2150 us ceiling
  (`omotion.seedless.MAX_TA_PULSE_WIDTH_US`: the pulse starts 100 us into
  the frame and must end inside the 2295 us exposure) at one TA drive
  current. A least-squares line through the round says where 1.0x falls:
  * within 2150 us: refine there (the fitted width, then secant steps, up
    to 10 bursts) and take the C_min capture at that width and current;
  * beyond 2150 us: raise `TA_CURRENT_DRV` by 100 mA (`current_step_ma`)
    and run the round again, up to 4999.68 mA (the 5 A EE drive rating as
    the safety FPGA holds it; `--max-current-ma` can lower it). Out of reach
    at the ceiling: FAIL.
  `--no-current-stepping` keeps the unit's own current (the earlier,
  pulse-width-only search).
* **The final match tunes the current, not the width** (bench 2026-11-24,
  `--match-by current`, the default). Three back-to-back runs matched at
  4300 mA / 2150 us (C_min 0.0923, 0.0934) and 4400 mA / 2059 us (0.0998):
  seedless contrast depends on the pulse width, so a width chosen by
  where the search happened to land moved C_min ~8 % while the repeats
  agreed within 1 %. Once a round's line reaches 1.0x within 2150 us, the
  pulse stays at 2150 us and the current is set by a secant on (current,
  brightness) between that round and the one before, to 1.0x +- 1 %
  (down to 1000 mA if the unit's own current is already too bright). C_min
  then always refers to the full 2150 us pulse. `--match-by width` is the
  earlier width refinement.
* **Warm-up** (default; `--no-warmup` skips it): the cold run's seeded
  image rose 0.6-2.0 % during the run and failed the drift check; seeded
  20 s chunks run until the laser's total output, the average of the
  per-camera chunk-to-chunk ratios, changes by no more than 0.2 % (at least
  2 chunks, at most 15, else FAIL before the seed is touched). The largest
  single-camera change is logged alongside. It is not the gate: on
  `--side both` the left/right split wandered 0.3-0.8 % per chunk,
  anti-correlated (corr -0.89), while the L+R average held within 0.11 %
  (bench 2026-11-24), so a per-camera rule never settled. That split is the
  fixture, not the laser; the per-camera gates below still see it.
* **C_min, per camera** (Ethan, 2026-11-24: "confidently ascertain a per
  camera C_min"):
  1. Three 1 s captures at the matched width and current
     (`cmin_repeats`). Each must stay within 6 % of the nominal brightness
     overall.
  2. Raw C_min = the average of the three captures' contrast (each an
     average of per-frame ratios, sdk#148).
  3. Each camera's brightness over the captures must be within 5 % of its
     **own** nominal (`camera_tolerance`): the search matches the average, a
     camera far from it is not measured at its own brightness.
  4. The three captures must agree within 2 % per camera
     (`repeat_spread_max`, (max - min) / mean).
  5. Seedless contrast falls as the image brightens, so a camera at 0.98x
     reads a little high. A least-squares slope of contrast against
     brightness, per camera, from the bursts at the matched current (from
     0.6x up) and the three captures corrects each camera's C_min to exactly
     1.0x. Raw and corrected are both reported; a correction over 1 % is
     noted. Without a spread of brightness there (< 5 %), no correction.
  6. The laser settings are restored and the seeded image is measured again:
     each camera's nominal must be within 2 % of the start
     (`nominal_drift_max`), so neither the phantom nor the laser moved
     during the run. `nominal_recheck=False` skips it.
  The overall C_min is the average of the per-camera values. A run that
  fails any of these still reports the numbers, marked not confirmed, and
  still re-measures the seeded image after the restore.
* The nominal is checked against the unit's calibrated drive current
  before the laser starts: the TA_CURRENT_DRV the TA FPGA holds after the
  bring-up must match the console config's. A console with no
  TA_CURRENT_DRV in its config (1.8.1 straight after a flash) refuses.

### Plateau check (opt-in, `--plateau-check`)

On the bench (2026-11-24, console 1.8.1) the longest pulse that fits the
exposure, 2150 us, reached only 0.75x the seeded brightness; a linear fit
puts 1.0x at about 2.9 ms. Both the seeded and the seedless frames were at
a 4999.68 mA drive current: flashing 1.8.1 empties the console config, so
the laser bring-up used the bundled baseline instead of the unit's 3900 mA
(found on the third run, which recorded the current). Seedless contrast barely
moved across the sweep (0.103, 0.099, 0.100, 0.098 overall). Brad's first
review proposed checking that contrast is flat **per camera** instead of
failing; the drive-current search replaced it as the default, and it stays
available for an out-of-reach result at the highest current:

* Pulse widths: `plateau_points` (3) evenly spaced, from the width that
  gives `plateau_min_ratio` (0.5x, `--plateau-min-ratio`) of nominal
  brightness by a line through the search bursts, up to the longest pulse.
  10 light frames at each, and the 1 s C_min capture at the longest.
* PASS when every camera's contrast spread, (max - min) / mean over those
  widths, is at most 5 %. C_min is then the capture at the longest pulse,
  reported with the brightness it reached. Otherwise FAIL, with each
  camera's spread in the report.

On the bench (2026-11-24 day run, 1387 to 2150 us at 4999.68 mA) it failed on
R8 alone: R1 3.3 %, R2 2.6 %, R3 4.3 %, R4 3.5 %, R5 5.0 %, R6 4.3 %,
R7 2.4 %, R8 6.0 %. Across all widths from 500 us the corner cameras R1 and
R8 move 28 % and 40 % (their contrast falls as the image brightens), while
R3 to R6 rise 6 to 9 %.

## When it stops by itself ("bonk out")

| Check | Where | Limit |
|---|---|---|
| Console accepts widened safety limits | before the laser | firmware that owns the safety state refuses them on **every** image (see below) |
| Nominal looks like the phantom | after the nominal scan, before the seed is touched | factory floors: mean >= 40 (cams 1, 8) / 80, contrast >= 0.25 |
| Seed really off | first burst (normal 500 us pulse) | brightness <= 0.75x nominal (expected ~0.26x) |
| TA not shut down / phantom in place | first burst | brightness >= 0.05x nominal |
| Unseeded light | first burst | contrast <= 0.75x nominal contrast (bench: 0.09 vs 0.40) |
| Pulse width reaches the cameras | every lengthening | brightness rises by >= 2 % |
| Current reaches the cameras | every current step | brightness at 2150 us rises by >= 0.5 % |
| No safety fault | after every seedless burst | EE/OPT STATUS must read 0 (a latched power, pulse-width or rate fault stops the run) |
| Target reachable | search | at 5 A and 2150 us the mean is still short: FAIL (or the plateau check, if asked for) |
| Contrast flat per camera | plateau check (opt-in) | spread <= 5 % per camera from the 0.5x width up |
| Every camera delivers | every capture | >= 80 % of the requested frames |
| Calibrated drive current | after the laser bring-up, before any light | live TA_CURRENT_DRV = the console config's (config must have it) |
| Each camera matched | C_min captures | every camera within 5 % of its own nominal |
| C_min repeats | C_min captures | per camera, 3 captures within 2 % |
| Nominal unchanged | seeded re-check after the restore | per camera within 2 % of the start |

The bench numbers behind the first-burst limits are from the 2026-07-17
validation on the seedless-frames branch (sdk#146): 2 ms unseeded gave
0.975x the seeded intensity, and 500 us -> 2 ms gave 3.70x.

## Register changes and restore

`omotion.seedless.SeedlessController` (ported from the sdk#146 branch, which
was never merged): clear any latched EE/OPT fault, snapshot the live values,
then widen EE/OPT pulse-width UL to 2.5 ms, lower EE/OPT rate LL to 19 ms
(the seedless dark-frame spacing is 22.5 ms, under the stock 23.125 ms
limit; sdk#146 disabled the limit, this keeps a guard on the pulse rate),
seed DDS/CW gain to 0, TA pulse width, camera exposure 2295 us. The TA
drive current (`TA_CURRENT_DRV`, ch4 reg 0x06, 0.16 mA ticks) is snapshotted
too and changed only between bursts, by the search, up to 4999.68 mA. Every
write is verified by read-back. Seedless scans use `LaserPulseSkipDelayUsec=2500`
so dark frames stay dark.

On every exit path, pass or fail, the adapter stops the trigger and sends
the standard trigger config back (1800 us dark skip, read back), *then* the
controller restores the snapshot (TA width first, then the drive current, then the limits, then the seed,
then the rate LL, then exposure) and the adapter reads every register back.
Without the trigger config, the console would keep the 2500 us skip while
the rate LL returns to 23.125 ms, and anything that starts the trigger with
the config the console holds would latch a rate fault (E-202) on its first
dark frame. A restore problem fails the run
and says to power-cycle the console (the laser registers are volatile).

## Console firmware (bench 2026-11-24)

Console firmware that owns the laser-safety state (boot-info v2,
PTR-2026-1-5; 1.8.2) range-checks every host write to the EE/OPT limits
against its WI-00015 envelope (`safety_envelope.h`): pulse-width upper limit
at most **660 us**, rate lower limit at least **23125 us**. A Factory image
lifts the firmware's ownership of those registers but not the envelope, so
`EE_PULSE_WIDTH_UL = 2.5 ms` is refused on 1.8.2-factory too (confirmed on
the bench: the nominal phase completed, then the first seedless write was
refused and rolled back). The unseeded TA cannot be pulsed long enough
inside that envelope (TA_PULSE_WIDTH itself is also capped at 600 us there),
so seedless mode needs console firmware that allows these limits, or
firmware that leaves them to the host (pre-PTR-2026-1-5, e.g. 1.8.1).

A Factory build whose I2C pass-through skips the range check (console-fw
branch `feature/factory-passthrough-unbounded`, in progress) is meant to
run it. The script therefore refuses Release/Debug images before any
camera or laser activity, and lets a Factory image through; a Factory
build that still refuses fails the first seedless write, which is read back
and rolled back before the seed is turned off.

## Report

In `<output-dir>/seedless-cmin-<run id>/`, prefixed with the console serial:

* `-frames.csv`: every light frame of every phase (phase, burst, seed, TA
  pulse width, TA drive current, side, camera, frame id, mean, std, contrast).
* `.json`: identity, settings, nominal stats, search history, C_min
  captures, per-camera results (`cameras`: nominal mean and contrast,
  brightness, raw C_min, the 3 repeats and their spread, slope, C_min at
  1.0x, correction, nominal drift, C_min / seeded contrast), the seeded
  re-check, overall C_min, matched TA pulse width and current, verdict.
* `.txt`: the same summary as text.
