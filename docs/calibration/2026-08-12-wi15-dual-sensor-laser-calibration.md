# WI-00015 Dual-Sensor Laser Calibration Specification

**Date:** 2026-08-12

**Status:** Implemented and verified in software and on dual-sensor hardware.
Amended 2026-08-14 to restate the WI-00015 Ophir meter setup values and 0 cm
fixture orientation requirements.

**Procedure:** Dual-Sensor Laser Calibration

## 1. Objective

Tune the shared console laser for a unit that will ship with both left and
right sensor modules. The procedure rejects an initial differential greater
than 100 microjoules, adjusts the shared laser so the two-sensor midpoint is
as close as practical to 350 microjoules, and passes only when both valid
final readings are within 300-400 microjoules inclusive.

It permits at most three complete post-adjustment cross-checks.

## 2. Authority and related specifications

This specification implements the Dual-Sensor Laser Calibration requirements
in:

- the approved WI-00015 automated process addendum (filed separately); and
- WI-00015 revision 2, as modified by the team-approved automated process.

The process addendum controls if the sources conflict.

## 3. Scope

### Included

- Exact two-sensor topology validation.
- Console/sensor identity and serial validation.
- Ophir connection, identity, configuration, and readback.
- Pre-existing configuration capture and exact default configuration write.
- Valid left and right 0 cm measurements.
- Initial 100-microjoule differential gate.
- Direction-specific tuning toward a 350-microjoule midpoint.
- Up to three complete left/right cross-checks.
- Final dual-energy and register-readback acceptance.
- Structured state and report evidence.

### Excluded

- Processing either side as a standalone one-sensor unit.
- Safety ADC calibration.
- Static-phantom Measurement Calibration.
- Final TestApp UI.

## 4. Entry point and reusable API boundary

The operator entry point is
`omotion/scripts/wi15_dual_sensor_laser_calibration.py`. It guides each required
change from the left module to the right module or from the right module to
the left module, but does not own the tuning calculations. Consecutive
measurements of the same seated module do not repeat the placement prompt.

Shared SDK code receives one bench adapter (owning the Motion session,
Ophir meter, and configuration I/O - the dual topology is implicit in the
dual bench protocol), a run recorder, an injected placement-change
acknowledgement callback, and the target/acceptance energies. It must not
call `input()`. There are no progress or cancellation callbacks: a decline,
non-boolean answer, or interrupt raised from the placement callback cancels
the run, and interactive callers (the test-app Procedures pane) run the
operator script as a subprocess.

It returns a structured outcome containing the initial pair, differential,
tuning choices, cross-checks, final settings, terminal disposition, and
artifact references. Failure maps to a nonzero script exit code.

Every operator-facing step and report event uses descriptive audit language
that states the phase, sensor side, reason, requested action, and result when
applicable. Internal method names and terse event codes may appear in JSON
field names, but they are not sufficient operator or report labels.

The following label style is normative (recovered from the 2026-08-13
implementation design):

- `Initial paired measurement - left sensor`
- `Initial differential gate - accepted at 100 uJ or below`
- `Midpoint adjustment round 1 - selected right sensor because it had the
  lower energy reading`
- `Adjustment step 2 - increased pulse width from 410 us to 420 us`
- `Cross-check 1 - paired result: both sensors within the approved
  300-400 uJ range`
- `Final configuration verification - active current within +/-2% of the
  requested value`

Labels must not rely on method names such as `_measure_once`, terse codes
such as `adj_2`, or unexplained register names; register names remain present
as technical evidence alongside a plain-language explanation. Placement
acknowledgements use simple language for factory operators while still
naming the side and serial, for example: "Put the left sensor (serial
12345) into the 0 cm fixture. Is it in place? (yes/no)". Operator-facing
terminal text throughout the procedures uses short plain sentences (ruling
2026-08-14: factory operators may have limited English); the descriptive
audit labels above remain the evidence-record style.

Every placement acknowledgement attests to the WI-00015 Figure F seating
requirements: the module's strap covers and straps are removed before
insertion, the module sits in the 0 cm fixture with its optics facing up,
and the sensor cable exits as perpendicular to the module as possible with
minimal bending within 40 cm of the module.

## 5. Preconditions and fail-closed preflight

Before configuration mutation or laser action:

1. the console must be connected and responsive (command echo round-trip);
   the left and right sensors must be connected with readable serials;
2. console, left, and right serial numbers must be non-`None` and non-empty;
3. the TA, Seed, Safety EE, and Safety OPT FPGA major/minor/revision values
   must all be read (fail-closed) and are stored in the console identity;
   console firmware and hardware ID are read best-effort and may record as
   unavailable; sensor-camera FPGA revision fields are omitted from the
   human report;
4. Ophir COM instantiation, scan, open, energy-sensor presence, identity, and
   calibration-due reads must pass; and
5. all Ophir settings and readbacks in the process addendum must pass.

When only one sensor is connected (or none), the topology failure reason
directs the operator to Single-Sensor Laser Calibration instead (and the
reverse: Single-Sensor Laser Calibration's own preflight directs the
operator to this procedure when both sensors are connected).

The required meter configuration restates the WI-00015 step-4 setup values
(the process addendum controls):

| Setting | Required value |
|---|---:|
| Measurement mode | Energy |
| Range | 2.0 mJ |
| Wavelength | 795 nm |
| Pulse length | 1.0 ms |
| Threshold | Minimum available |
| Display/statistics averaging | 3 seconds, when applicable |
| Graph/display mode | Statistics, when applicable |

Each applicable value is written and read back over the meter's COM
interface. The two display rows describe an attended StarLab-style display;
they do not alter the direct-stream acquisition defined in section 7.

Any failure stops before default configuration or firing. A missing side is
not reinterpreted as a one-sensor unit.

## 6. Default configuration setup

1. Read and preserve the complete existing User Configuration.
2. Write exactly the ten-key default object in the process addendum.
3. Require successful SDK write and exact complete readback.
4. Bring up the laser configuration and verify five active registers within
   the 2 percent tolerance - `TA_CURRENT_DRV`, `TA_PULSE_WIDTH`,
   `SEED_CW_GAIN`, `EE_PULSE_WIDTH_UL`, and `OPT_PULSE_WIDTH_UL` - plus 40
   Hz trigger frequency.
5. Correct only trigger frequency if needed; fail on another required
   operating mismatch.

Retain pre-existing, requested-default, and actual-default objects.

## 7. Valid measurement definition

Every left/right observation used for a calculation or decision must have:

- more than 25 valid samples after Ophir sentinel filtering;
- standard deviation below 40 microjoules;
- repetition rate from 39 through 41 Hz inclusive; and
- finite mean, standard deviation, rate, minimum, and maximum.

Record invalid observations but stop rather than using their means.

## 8. Initial pair and differential gate

1. Prompt the operator to seat the left module and acquire a valid
   measurement.
2. Prompt the operator to seat the right module and acquire a valid
   measurement.
3. Calculate `difference = abs(left - right)` and
   `midpoint = (left + right) / 2`.
4. If `difference > 100`, record the pair and fail/NCR immediately.
5. At exactly 100, continue; final acceptance still requires both readings
   within 300-400.

The initial pair is a baseline, not one of the three post-adjustment
cross-checks.

The workflow tracks the side most recently acknowledged as seated. The first
measurement requires acknowledgement of the left module. The second requires
acknowledgement of the switch to the right module. The acknowledgement must
identify both the side and serial number. If the operator declines or cancels,
the run fails before the associated firing or measurement.

## 9. Approved midpoint-tuning algorithm

For every tuning round, use the latest valid left/right pair.

### 9.1 Midpoint above 350

1. Select the higher-reading module.
2. Calculate its expected target as `350 + difference / 2`.
3. Keep TA pulse width fixed.
4. Reduce `TA_CURRENT_DRV` by 50 mA per step.
5. After each checked write/readback, acquire a valid measurement of the
   selected module.
6. Continue until the selected measurement reaches/crosses its expected
   target or the conservative current floor is reached.
7. Select the recorded setting (including the starting one) whose valid
   measured mean is closest to the target - ties break toward the lower
   setting - then reapply/read back it if necessary.
8. Reaching the current floor without an acceptable reachable setting is a
   terminal NCR.

### 9.2 Midpoint below 350

1. Select the lower-reading module.
2. Calculate its expected target as `350 - difference / 2`.
3. Temporarily set both pulse-width upper limits to 660 microseconds and
   verify readback.
4. Keep TA current fixed.
5. Increase `TA_PULSE_WIDTH` by 10 microseconds per step.
6. After each checked write/readback, acquire a valid measurement of the
   selected module.
7. Continue until the selected measurement reaches/crosses its expected
   target or TA pulse width reaches 600 microseconds.
8. Select the recorded setting (including the starting one) whose valid
   measured mean is closest to the target - ties break toward the lower
   setting - and reapply/read back it if necessary.
9. At 600 microseconds, a selected measurement below the acceptance
   minimum (300 in production; the comparison follows an injected window)
   is an immediate terminal NCR.

### 9.3 Midpoint at 350

Do not alter the setting when the midpoint is exactly 350; continue directly
to a complete cross-check. In every other case the sweep takes at least one
approved step; if the sweep then shows the starting setting was closest to
the target, the closest-setting selection re-applies it, and the recorded
evidence shows both the step and the re-selection.

Before the first tuning measurement of the selected module, request a
placement change only when that selected side differs from the side currently
seated. Leave the selected module seated across consecutive steps of the same
tuning sweep. Record the sensor side on every measurement even when no new
placement acknowledgement is required.

## 10. Complete cross-check loop

After every adjustment decision, including no adjustment:

1. apply and verify the selected final setting for that round;
2. prompt for and measure left;
3. prompt for and measure right;
4. increment `crosscheck_count` only after both valid measurements exist;
5. calculate difference, midpoint, midpoint distance from 350, and individual
   signed offsets from 350; and
6. pass immediately if both readings are within 300-400 inclusive.

If either side is outside range and fewer than three complete cross-checks
have run, recompute the next adjustment from the latest pair. If either side
is outside range after cross-check three, fail/NCR immediately.

An invalid measurement does not consume a complete cross-check, but it fails
the current execution rather than silently retrying or tuning from partial
data.

The placement-change rule applies to every cross-check: prompt only when the
next required side differs from the currently seated side. A complete
cross-check still always measures left first and right second.

## 11. Final readback and acceptance

After a passing cross-check:

1. require requested-versus-active `TA_CURRENT_DRV` within plus or minus 2
   percent;
2. require requested-versus-active `TA_PULSE_WIDTH` within plus or minus 2
   percent;
3. retain the passing left/right pair and all midpoint metrics; and
4. construct the complete passing User Configuration with the final TA
   current/pulse values, the other approved defaults, and provisional
   pulse-width limits of 660 when upward pulse tuning was used (otherwise
   550);
5. require successful write and exact immediate complete readback; and
6. that recorded readback (`final_config_readback`) is the authoritative
   input to Safety Calibration; no separate flag marks it.

Both energy bounds and the readback-tolerance bounds are inclusive.
Dual-Sensor Laser Calibration does not power-cycle; Safety Calibration owns
final safety-limit calculation and persistence verification.

## 12. Failure behavior

Topology, identity, Ophir, default configuration, measurement-quality,
initial differential, adjustment bound, third cross-check, or readback
failure returns nonzero with an exact reason. The initial differential,
adjustment-bound, and third-cross-check failures are NCR dispositions;
measurement-quality failures are procedural failures (an out-of-range final
energy is only reachable through the third-cross-check NCR).

After terminal NCR, no final tuned/safety configuration write or later guided
phase may execute. The required earlier default write remains recorded. The
guided runner has no continue-anyway path.

## 13. Report evidence

Record (the request-metadata table carries the common operator, build
revision, fixture, and procedure-revision evidence; build revision and
fixture calibration status are CLI-only per the 2026-08-13 ruling and may
record as `unspecified`/unset):

- exact declared and actual dual topology;
- all four console-board FPGA firmware revisions;
- initial left/right observations, differential, and midpoint;
- selected tuning side and why;
- every acknowledged physical placement change, including side, serial
  number, procedure phase, and operator response;
- target calculation for every round;
- every current/pulse step and quantized readback;
- each complete cross-check number and pair;
- each pair's differential, midpoint, distance from 350, and asymmetry;
- final 300/400 and 2 percent results;
- passing tuned configuration and exact immediate readback;
- highlighted default-versus-final changes;
- resource-cleanup and report-artifact finalization evidence; and
- terminal outcome and NCR reason.

The durable artifacts are named `<console-serial>-dual-laser-cal-run.json`
and `<console-serial>-dual-laser-cal-report.html`, and at finalization the
per-run directory itself (created as `WI-00015-<timestamp>`) is renamed to
`<console-serial>-dual-laser-cal-<timestamp>` - serial first throughout, so
listings sort by unit (serial prefix omitted when preflight never read one;
the JSON lives under `run.json` until finalization renames it atomically).

## 14. Automated tests

Unit tests cover:

- valid dual topology and either-side-missing rejection;
- all three required serial checks;
- Ophir preflight failures and proof of no mutation/firing;
- default configuration write/readback failures;
- measurement-quality boundaries;
- differential 100 accepted and greater than 100 NCR;
- above-midpoint higher-side selection and current stepping;
- below-midpoint lower-side selection and pulse stepping;
- target-straddling closest-setting selection;
- exact-350 midpoint proceeding straight to cross-check (pinned by the
  four-measurement pass-shape tests);
- first-, second-, and third-cross-check pass;
- failure immediately after cross-check three;
- invalid partial cross-check behavior;
- 300/400 inclusive energy bounds and plus/minus 2 percent readback bounds;
- tuned-configuration write failure and complete-readback mismatch;
- proof NCR prevents final writes;
- required report contents;
- side-change-only prompt behavior, including no duplicate prompt during a
  same-side tuning sweep;
- declined/canceled placement acknowledgement preventing the associated
  measurement; and
- descriptive operator/report labels for initial measurements, tuning
  rationale, adjustment steps, cross-check results, and final verification.

## 15. TestApp integration

The test-app Procedures pane runs this procedure today by executing the
operator script as a subprocess (uniform terminal/stdin contract; the swap
prompts render as answer buttons). The pane supplies no calibration logic of
its own and may not duplicate or relax topology, differential, tuning,
cross-check, or failure logic.

## 16. Implementation mapping and verification

Design rationale (recovered from the 2026-08-13 implementation design): the
dual procedure is a separate typed workflow rather than a generalization of
the single-sensor workflow. Generalizing would have rewritten the
live-verified single-sensor path and forced paired state into a
single-observation design; keeping the procedures separate preserves that
verified path, makes dual-only rules visible in types and tests, and limits
shared code to hardware behavior that is genuinely common (the Motion,
Ophir, configuration, recording, and cleanup seams).

The software implementation is divided at the intended reusable boundaries:

- shared constants, validation, topology, paired metrics, and selection rules:
  `omotion/calibration/laser.py`;
- shared laser-workflow primitives (preflight validation, default
  establishment, checked register writes, active-default restoration,
  passing-configuration write, immutable evidence records):
  `omotion/calibration/_procedure.py`;
- UI-neutral dual procedure and dual-specific evidence model:
  `omotion/calibration/dual_sensor_laser.py`;
- Motion console bench base and FPGA revision readback:
  `omotion/calibration/motion_bench.py`;
- exact-dual Motion preflight, pre-fire topology guards, and shared Ophir
  acquisition: `omotion/calibration/laser_hardware.py`;
- auditor-readable HTML evidence:
  `omotion/calibration/dual_sensor_laser_report.py`;
- shared script scaffolding (argument parsing, operator prompts,
  artifact/resource finalization): `omotion/calibration/script_support.py`;
- operator CLI: `omotion/scripts/wi15_dual_sensor_laser_calibration.py`.

Focused automated coverage is provided by:

- `tests/test_wi15_laser_calibration.py`;
- `tests/test_wi15_dual_sensor_laser_calibration.py`;
- `tests/test_wi15_laser_calibration_hardware.py`;
- `tests/test_wi15_dual_sensor_laser_calibration_report.py`; and
- `tests/test_wi15_dual_sensor_laser_script.py`.

Software verification on 2026-08-13 completed with 322 passing tests in the
exact single/dual WI-00015 matrix and 1,125 passing tests with 207
hardware-marked tests deselected in the repository hardware-independent
suite (counts at commit `bf55d72`; the suite has since been consolidated
and extended). Static compilation, Ruff, forbidden-dependency, and diff
checks also passed.

A live dual-sensor execution passed on 2026-08-13 using run
`WI-00015-20260813T190000Z` and commit `f2af676`. The exact topology contained
console and left serial `ZZZ99Z99999` plus right serial `WWWA4Q40005`. The
initial valid means were 316.107 uJ left and 299.867 uJ right, giving a
16.240 uJ differential and 307.987 uJ midpoint. Approved upward tuning of the
right sensor selected a requested 570 us pulse width, with 569.92 us active
readback. Cross-check 1 passed at 363.962 uJ left and 345.429 uJ right, with a
354.695 uJ midpoint and 18.533 uJ differential. The final complete User
Configuration read back exactly, both final active-setting checks passed,
JSON and HTML artifacts finalized, and no trigger, active-restoration, or
resource-cleanup failure was recorded.

Post-refactor live regression testing passed on 2026-08-13 using commit
`bf55d72` and run `WI-00015-20260813T221750Z`. The exact dual topology and all
three required non-null identities remained stable. The initial accepted
means were 321.333 microjoules left and 306.333 microjoules right, giving a
15.000-microjoule differential. The approved right-side upward sweep selected
a requested 560-microsecond pulse width. Cross-check 1 passed at 362.321
microjoules left and 338.519 microjoules right. All ten observations met the
acquisition-quality criteria, both final active-setting checks passed, the
complete final configuration read back, and finalized JSON and HTML artifacts
recorded no trigger, restoration, or resource-cleanup failure.

The same build exercised two fail-closed paths before that pass. Run
`WI-00015-20260813T221643Z` rejected a missing right sensor before measurement.
Run `WI-00015-20260813T221251Z` used an exact dual topology but returned
`failed_ncr` when the selected sensor remained below 300 microjoules at the
600-microsecond ceiling. Both runs finalized their evidence; the bound NCR
restored active defaults and did not write the passing tuned configuration.

A deliberately non-production bench validation exercised the downward-current
path on 2026-08-13 in run `WI-00015-20260813T223817Z`. The injected midpoint
target was 300 microjoules and the injected final acceptance window was
250-350 microjoules; both values were recorded explicitly in JSON and HTML.
The production defaults remained 350 and 300-400 microjoules. Initial means
were 312.346 microjoules left and 295.815 microjoules right. One approved
50-mA downward step on the higher left side produced a selected 306.077-
microjoule observation at requested current 4950 mA. Cross-check 1 then passed
at 305.423 microjoules left and 295.429 microjoules right, a 300.426-
microjoule midpoint and 9.995-microjoule differential. The procedure stopped
after that first cross-check with exactly four placement acknowledgements,
persisted and read back the artificial 4950-mA configuration, passed both
final active-setting checks, finalized both artifacts, and recorded no
trigger, restoration, or resource-cleanup failure. This validation
configuration is not an approved production calibration and requires a normal
350-microjoule execution before unit release. At that date the
600-microsecond ceiling NCR compared against a hardcoded 300; since
2026-08-14 the ceiling comparison uses the injected acceptance minimum, so
an injected window governs every energy bound.

The normal production rerun after the artificial validation passed on
2026-08-13 using run `WI-00015-20260813T225916Z` and commit `fc20627`. Its
console identity recorded TA `1.1.0`, Seed `0.1.1`, Safety EE `0.1.4`, and
Safety OPT `0.1.4` FPGA firmware revisions before configuration mutation or
firing. Initial means were 302.241 microjoules left and 293.630 microjoules
right. Approved upward tuning selected 590 microseconds on the lower right
side. Cross-check 1 passed at 362.074 microjoules left and 351.963
microjoules right, with a 357.019-microjoule midpoint and 10.111-microjoule
differential. The production 350-microjoule configuration read back exactly,
both active-setting checks passed, and no trigger, restoration, resource,
or report-artifact failure was recorded. The human report included all four
console-board FPGA revisions while omitting sensor-camera FPGA revision
fields, request metadata, and the redundant pre-mutation topology table.

## 17. Operator override mode (engineering only, 2026-08-21)

Epic OpenwaterHealth/openmotion-bloodflow-app#482 adds the same
password-gated operator override as the single-sensor procedure
(`omotion/calibration/override.py`, `--allow-override`; the password and then
the minimum/maximum/target energies - 10-1000 microjoules, Enter keeps the
factory value - are asked before any hardware is touched; the optional
`--min-energy-uj` / `--max-energy-uj` / `--target-energy-uj` flags pre-supply
those answers). Nothing changes unless it is switched on for a run:

- The operator's band and target replace the injected window of section 4
  for that run and steer sections 9 and 10: the current floor without an
  acceptable setting, and energy still below the minimum at the
  600-microsecond ceiling, no longer end the run as NCR; the closest
  candidate (or the ceiling setting) is kept and the miss is recorded as an
  `override` event.
- A cross-check inside the operator's band but with at least one side
  outside the factory 300-400 window asks one question before the section
  11 write. After three complete cross-checks outside the band, the third
  pair is offered the same way. The question shows both side means, the
  midpoint, both bands and the configuration about to be written, then
  requires a free-text reason. A decline is exactly the section 12 NCR.
- An accepted override writes through the same exact-readback path as a
  pass and ends in `ProcedureStatus.OVERRIDDEN` (`Final result: OVERRIDE`,
  exit code 3, settings and decision in `run.json` and the amber HTML
  status). A pair inside the factory window stays PASSED even in override
  mode.
