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
mean the seeded system produces. The TA pulse is lengthened (never driven
harder) until the seedless mean matches.

| Phase | Seed | Exposure | TA pulse | Frames |
|---|---|---|---|---|
| Nominal | on | 648 us | console's (500 us) | 80 light frames (2 s) |
| Search bursts | **off** | 2295 us | 500 us, then lengthened | 10 light frames per burst |
| C_min capture | **off** | 2295 us | matched width | 40 light frames (1 s) |

* All analysed frames are **light** frames from the science pipeline's
  corrected stream: dark-subtracted mean, shot-noise-corrected std,
  contrast = std/mean. Each capture is its own short scan, so its dark
  frames (and dark correction) are at the same exposure as its light frames.
* Brightness ratio = mean over cameras of (seedless mean / nominal mean) for
  the same camera. Matched when within 3 % of 1 (`--tolerance`).
* Search: proportional from the first burst, secant afterwards; the pulse
  never more than doubles in one step and never exceeds 2150 us
  (`omotion.seedless.MAX_TA_PULSE_WIDTH_US`: the pulse starts 100 us into
  the frame and must end inside the 2295 us exposure). Up to 10 bursts.
* **C_min** = average contrast over the 1 s capture (average of per-frame
  ratios, sdk#148), per camera and over all cameras. The capture must stay
  within 6 % of the nominal brightness.

### When the nominal brightness is out of reach: plateau check

On the bench (2026-11-24, console 1.8.1) the longest pulse that fits the
exposure, 2150 us, reached only 0.75x the seeded brightness; a linear fit
puts 1.0x at about 2.9 ms. Seedless contrast barely moved across the sweep
(0.103, 0.099, 0.100, 0.098 overall). Following Brad's review, the run then
checks that contrast is flat **per camera** instead of failing outright:

* Pulse widths: `plateau_points` (3) evenly spaced, from the width that
  gives `plateau_min_ratio` (0.5x, `--plateau-min-ratio`) of nominal
  brightness by a line through the search bursts, up to the longest pulse.
  10 light frames at each, and the 1 s C_min capture at the longest.
* PASS when every camera's contrast spread, (max - min) / mean over those
  widths, is at most 5 %. C_min is then the capture at the longest pulse,
  reported with the brightness it reached. Otherwise FAIL, with each
  camera's spread in the report.

The bench data say where this lands. Per camera, across all four bursts
(0.20x to 0.75x), the corner cameras R1 and R8 moved 27 % and 36 %; their
contrast still falls as the image gets brighter. Across the two longest
pulses (0.71x to 0.75x) every camera stayed within 2.1 %. So the floor
decides the verdict, and on this unit 0.5x would likely fail R1 and R8.

## When it stops by itself ("bonk out")

| Check | Where | Limit |
|---|---|---|
| Console accepts widened safety limits | before the laser | firmware that owns the safety state refuses them on **every** image (see below) |
| Nominal looks like the phantom | after the nominal scan, before the seed is touched | factory floors: mean >= 40 (cams 1, 8) / 80, contrast >= 0.25 |
| Seed really off | first burst (normal 500 us pulse) | brightness <= 0.75x nominal (expected ~0.26x) |
| TA not shut down / phantom in place | first burst | brightness >= 0.05x nominal |
| Unseeded light | first burst | contrast <= 0.75x nominal contrast (bench: 0.09 vs 0.40) |
| Pulse width reaches the cameras | every lengthening | brightness rises by >= 2 % |
| Target reachable | search | at 2150 us the mean is still short: plateau check |
| Contrast flat per camera | plateau check | spread <= 5 % per camera from the 0.5x width up |
| Every camera delivers | every capture | >= 80 % of the requested frames |

The bench numbers behind the first-burst limits are from the 2026-07-17
validation on the seedless-frames branch (sdk#146): 2 ms unseeded gave
0.975x the seeded intensity, and 500 us -> 2 ms gave 3.70x.

## Register changes and restore

`omotion.seedless.SeedlessController` (ported from the sdk#146 branch, which
was never merged): clear any latched EE/OPT fault, snapshot the live values,
then widen EE/OPT pulse-width UL to 2.5 ms, lower EE/OPT rate LL to 19 ms
(the seedless dark-frame spacing is 22.5 ms, under the stock 23.125 ms
limit; sdk#146 disabled the limit, this keeps a guard on the pulse rate),
seed DDS/CW gain to 0, TA pulse width, camera exposure 2295 us. Every write
is verified by read-back. Seedless scans use `LaserPulseSkipDelayUsec=2500`
so dark frames stay dark.

On every exit path, pass or fail, the adapter stops the trigger and sends
the standard trigger config back (1800 us dark skip, read back), *then* the
controller restores the snapshot (TA first, then the limits, then the seed,
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
inside that envelope, so seedless mode needs console firmware that allows
these limits, or firmware that leaves them to the host (pre-PTR-2026-1-5).
The script checks this before any camera or laser activity.

## Report

In `<output-dir>/seedless-cmin-<run id>/`, prefixed with the console serial:

* `-frames.csv`: every light frame of every phase (phase, burst, seed, TA
  pulse width, side, camera, frame id, mean, std, contrast).
* `.json`: identity, settings, nominal stats, search history, C_min capture,
  per-camera and overall C_min, matched TA pulse width, verdict.
* `.txt`: the same summary as text.
