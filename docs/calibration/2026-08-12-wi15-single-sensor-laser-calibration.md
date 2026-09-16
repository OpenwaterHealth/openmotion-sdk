# WI-00015 Single-Sensor Laser Calibration Specification

**Date:** 2026-08-12

**Status:** Implemented, software-verified, and successfully executed on the
left-only Motion/Ophir bench. Amended 2026-08-14 to restate the WI-00015
Ophir meter setup values and 0 cm fixture orientation requirements.

**Procedure:** Single-Sensor Laser Calibration

## 1. Objective

Tune the shared console laser for a unit that will ship with exactly one
sensor module. The operator declares the installed side, and the procedure
tunes that module's 0 cm per-pulse energy as close as practical to 350
microjoules. It passes only with a valid final energy from 300 through 400
microjoules, inclusive.

This procedure is distinct from Dual-Sensor Laser Calibration. A two-sensor
unit may not be processed by running this procedure twice.

## 2. Authority and related specifications

This specification implements the Laser Calibration requirements in:

- the approved WI-00015 automated process addendum (filed separately); and
- WI-00015 revision 2, as modified by the team-approved automated process.

The process addendum controls if this file is inadvertently interpreted in a
way that conflicts with the approved process.

## 3. Scope

### Included

- Operator declaration and confirmation of `left` or `right`.
- Exact one-sensor topology validation.
- Console/sensor identity and serial validation.
- Ophir connection, identity, configuration, and readback.
- Pre-existing configuration capture and exact default configuration write.
- Valid 0 cm energy measurement.
- Direction-specific tuning toward 350 microjoules.
- Final energy and register-readback acceptance.
- Structured state and report evidence.

### Excluded

- A second sensor or inter-sensor comparison.
- Safety ADC calibration.
- Static-phantom Measurement Calibration.
- Final TestApp UI.

## 4. Entry points and reusable API boundary

The operator entry point is
`omotion/scripts/wi15_single_sensor_laser_calibration.py`. Before any mutation it asks
the operator to select `left` or `right`, displays the selection, and requires
confirmation. It then prints a reminder to disconnect the sensor not being
calibrated and requires a separate confirmation that only the declared side
is connected. This operator confirmation is advisory only - the authoritative
gate is the fail-closed preflight topology check in section 5, which reads
the actual Motion discovery topology and fails the procedure before any
configuration mutation if two sensors or the wrong sensor are connected,
regardless of how the operator answered. When both sensors are connected,
the failure reason directs the operator to Dual-Sensor Laser Calibration
instead (and the reverse: Dual-Sensor Laser Calibration's own preflight
directs the operator to this procedure when only one sensor is connected).

The shared implementation resides outside the script and receives explicit
inputs. It must not call `input()` or format terminal prompts. It accepts:

- the request (declared side, operator metadata, fixture confirmation);
- a bench adapter owning the Motion session, Ophir meter, and configuration
  I/O (acquisition duration/policy lives on the meter adapter);
- a run recorder / report sink; and
- the target energy.

It returns a structured outcome whose single `status` field
(`ProcedureStatus`) carries the terminal disposition, together with the
reason, selected side, measurements, settings, and artifact references. A
non-passing status maps to a nonzero script exit code. There are no
cancellation or progress callbacks; callers that need interactivity (the
test-app Procedures pane) run the operator script as a subprocess.

## 5. Preconditions and fail-closed preflight

Preflight runs before step-9 configuration or laser action.

1. The console must be connected and responsive.
2. Exactly the declared sensor side must be connected.
3. The opposite side must not be connected.
4. Console and selected-sensor serial numbers must be non-`None` and
   non-empty after trimming.
5. Console firmware and hardware ID are read. The TA, Seed, Safety EE, and
   Safety OPT FPGA major/minor/revision registers must all be readable and are
   recorded as four semantic firmware revisions in the console identity.
   Sensor-camera FPGA revision fields are not shown in the human report.
6. The Ophir COM object must instantiate.
7. USB scan must find a meter, the meter must open, and a sensor head must
   be present on the configured channel. Energy capability is enforced by the
   subsequent `Energy` measurement-mode set and readback rather than by a
   separate head-type query.
8. Meter/sensor identity and calibration-due fields must be readable.
9. Every Ophir setting and readback in the process addendum must pass.

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

After Ophir setup, the Motion discovery topology must remain unchanged for an
injectable quiet period before the preflight snapshot is accepted. The same
stable exact topology is revalidated immediately before the first User
Configuration mutation and immediately before each firing authorization. A
late opposite-side sensor fails setup without configuration mutation or
firing.

Any failure stops the procedure before configuration mutation or firing.

## 6. Default configuration setup

1. Read and preserve the complete existing User Configuration.
2. Obtain a fresh run-local copy of the canonical ten-key defaults (the
   public immutable mapping `DEFAULT_USER_CONFIG`), then write exactly that
   object from the process addendum.
3. Require a successful SDK write result.
4. Treat the adapter-returned complete write readback as authoritative and
   require exact keys and values. A later corroborating read may not erase an
   immediate mismatch.
5. Bring up the laser configuration and verify five active registers within
   the 2 percent tolerance - `TA_CURRENT_DRV`, `TA_PULSE_WIDTH`,
   `SEED_CW_GAIN`, `EE_PULSE_WIDTH_UL`, and `OPT_PULSE_WIDTH_UL` - plus 40 Hz
   trigger frequency.
6. Correct only trigger frequency when necessary. A mismatch in another
   required operating value fails the procedure.

The pre-existing, requested-default, and actual-default objects are retained
as deeply immutable, run-local report evidence. Later mutation of a
caller/adapter-owned mapping cannot redefine a run or change its recorded
result.

## 7. Valid measurement definition

Every energy observation used for adjustment or acceptance must contain:

- more than 25 valid Ophir samples;
- standard deviation below 40 microjoules;
- repetition rate from 39 through 41 Hz inclusive; and
- finite mean, standard deviation, rate, minimum, maximum, valid count,
  discarded count, and acquisition duration (each a recorded criterion).

Samples with nonzero Ophir status are discarded and counted. If an
observation fails, record it and stop the procedure; do not tune from its
mean.

Direct-stream acquisition continues until a returned batch brings the total
to at least 26 valid status-zero samples, or until the bounded 2.0-second
maximum acquisition duration expires. Stop immediately after processing the
batch that reaches the target and use every accepted sample in that batch for
the reported statistics and repetition rate. Always stop the stream. At
timeout, retain and report the below-target observation so the existing
measurement-quality criteria fail closed.

The first nonempty, complete `GetData` batch after each `StartStream` is a
session-priming drain and is excluded in full. Its values, timestamps, and
statuses do not contribute to statistics, repetition rate, valid count, or
discarded count. Collection of the fresh 26-valid observation begins only
after that drain, within the same 2.0-second overall bound. A missing fresh
batch or a fresh below-target observation at timeout fails closed. Misaligned
priming or fresh arrays fail immediately, and `StopStream` is always attempted
for a stream that this adapter successfully started.

Within fresh collection, exclude and count a non-increasing status-zero
timestamp as discarded. If a positive timestamp jump is longer than the
current stream's host-observed lifetime plus 50 milliseconds, exclude and
count the preceding status-zero prefix as buffered data from an older epoch,
then continue fresh collection inside the unchanged 2.0-second bound. Retain
smaller positive gaps in the rate calculation so a genuine firing interruption
still fails the 39-41 Hz criterion.

## 8. Tuning algorithm

### 8.1 Initial measurement

The 0 cm fixture placement confirmation is collected by the operator script
up front, before any hardware object is constructed, and passed into the
workflow, which validates it during preflight - an unconfirmed fixture fails
before mutation or firing. The initial step then acquires a valid measurement
at the default operating point.

The confirmation attests to the WI-00015 Figure F seating requirements: the
module's strap covers and straps are removed before insertion, the module
sits in the 0 cm fixture with its optics facing up, and the sensor cable
exits as perpendicular to the module as possible with minimal bending within
40 cm of the module.

### 8.2 Downward adjustment

If the mean is above 350 microjoules:

1. leave TA pulse width unchanged;
2. reduce `TA_CURRENT_DRV` by 50 mA;
3. check requested-versus-active readback;
4. acquire a valid measurement; and
5. continue until the mean reaches or crosses 350, or the conservative
   current floor is reached.

When two adjacent permitted settings straddle 350, select the setting whose
valid measured mean is closest to 350. Reapply and verify that setting if it
is not the last one tested. Reaching the current floor without an acceptable
result is a terminal NCR.

### 8.3 Upward adjustment

If the mean is below 350 microjoules:

1. set `EE_PULSE_WIDTH_UL` and `OPT_PULSE_WIDTH_UL` to 660 microseconds and
   check readback;
2. leave TA current unchanged;
3. increase `TA_PULSE_WIDTH` by 10 microseconds;
4. check requested-versus-active readback;
5. acquire a valid measurement; and
6. continue until the mean reaches or crosses 350, or TA pulse width reaches
   600 microseconds.

When two adjacent permitted settings straddle 350, select the setting whose
valid measured mean is closest to 350 and reapply/verify it if necessary. If
600 microseconds is reached while energy remains below 300 microjoules, fail
NCR immediately. If the closest reachable setting is outside 300-400, fail
NCR.

### 8.4 No adjustment

If the initial mean is exactly 350, no setting adjustment is required. The
procedure still performs the final verification measurement and readback.

## 9. Final verification and acceptance

With the selected final settings active:

1. acquire a new valid measurement of the declared module;
2. require energy from 300 through 400 microjoules inclusive;
3. compare requested `TA_CURRENT_DRV` with active readback and require plus or
   minus 2 percent;
4. compare requested `TA_PULSE_WIDTH` with active readback and require plus or
   minus 2 percent; and
5. record workflow-owned typed checks containing requested value, actual
   value, absolute difference, percent difference, 2 percent tolerance, and
   explicit pass/fail for each attempted final TA check.

Only after all five checks pass, construct and write the complete passing
User Configuration:

- set `TA_CURRENT_DRV` and `TA_PULSE_WIDTH` to the final requested values;
- preserve the other approved default values;
- retain 660 for both pulse-width limits when upward pulse-width tuning was
  used, otherwise retain the default 550 limits; and
- require SDK write success and an exact immediate complete readback.

Do not power-cycle in this procedure. The passing readback becomes the
authoritative input to the separate Safety Calibration procedure, which owns
final safety-limit calculation and persistence verification.

## 10. Failure behavior

Topology, identity, Ophir, configuration, measurement-quality, adjustment
bound, final-energy, or readback failure returns nonzero and records a
terminal reason. Energy/bound failure is identified as NCR.

After terminal NCR, the procedure must not invoke Safety Calibration or write
the passing tuned-configuration handoff or final safety configuration. The
earlier required default configuration write remains part of the record.
There is no continue-anyway prompt.

After any failure once defaults were established and measurement began, the
workflow best-effort restores the five active default registers and records
the restoration (`active_default_restore`) or its failure
(`active_default_restore_failure`) as evidence; a trigger-stop failure during
cleanup is folded into the terminal result rather than hidden.

## 11. Report evidence

The report includes all common evidence in section 6 of the automated process
addendum and, specifically:

- declared and actual topology;
- selected side and operator confirmation;
- identities (serial validation is a preflight gate; a failure records
  its reason, a pass is implied by the recorded identities);
- all four console-board FPGA firmware revisions;
- Ophir setup and readbacks;
- pre-existing, default, and final configuration;
- every energy observation and validity criterion;
- every requested setting, quantized readback, and adjustment;
- closest-to-350 selection rationale;
- final energy and 2 percent checks;
- passing tuned configuration and exact immediate readback;
- highlighted default-versus-final changes; and
- terminal outcome/NCR reason.

Structured evidence also records the runtime `omotion.__version__` (using the
explicit value `unavailable` only when no runtime version is available), the
build revision (CLI `--build-revision` only, recorded as `unspecified` when
not supplied - the operator is not prompted, per the 2026-08-13 ruling; the
same ruling makes fixture calibration status CLI-only and unset by default),
and run start/end timestamps. The workflow
atomically checkpoints a current `in_progress` structured result after every
acquired observation and checked mutation, including immediate adapter write
readbacks, measurement criteria, candidates, selections, and cleanup
readbacks. JSON serialization consumes this workflow-owned evidence without
recomputing acceptance.

The durable artifacts are named `<console-serial>-single-laser-cal-run.json`
and `<console-serial>-single-laser-cal-report.html`, and at finalization the
per-run directory itself (created as `WI-00015-<timestamp>`) is renamed to
`<console-serial>-single-laser-cal-<timestamp>` - serial first throughout,
so listings sort by unit. The serial prefix is omitted when preflight never
read one; the JSON lives under `run.json` until finalization renames it
atomically, and any rename failure keeps the old name.

The HTML report is not listed as finalized in durable JSON until its atomic
write has produced a file. Report construction or rendering failure records a
failed report-artifact state, replaces a prior workflow pass with a structured
report failure, and exits nonzero. Live UI/report consumers use the same
workflow checkpoints and do not own duplicate acceptance logic.

## 12. Automated tests

Unit tests cover:

- left and right success;
- neither, both, and wrong-side topology rejection;
- late opposite-side discovery, pre-mutation topology revalidation, and
  pre-firing topology revalidation;
- `None`, empty, and valid serials;
- every Ophir preflight failure and proof of no mutation/firing;
- default write failure, extra/missing key, and value mismatch;
- immediate default, adjustment, trigger-correction, cleanup, and final
  handoff readback mismatches that cannot be erased by later matching reads;
- adapter-side mutation isolation of retained evidence;
- exactly 25 versus 26 valid pulses;
- bounded acquisition to 26 valid pulses, stale-session priming-batch
  isolation, and idempotent stream cleanup;
- standard-deviation, rate, and non-finite boundaries;
- initial 350 no-adjustment;
- downward steps, crossing selection, and current-floor NCR;
- upward steps, crossing selection, 600-microsecond success, and ceiling NCR;
- final 300/400 inclusive acceptance and outside failure;
- plus/minus 2 percent readback boundaries;
- exact workflow-owned final-check fields in JSON and HTML;
- tuned-configuration write failure and complete-readback mismatch;
- proof terminal NCR cannot reach final persistence; and
- interruption-safe in-progress observation/mutation evidence; report
  construction/rendering failure with real JSON/HTML components; runtime SDK
  metadata; and
- required report fields and change highlighting.

## 13. TestApp integration

The test-app Procedures pane runs this procedure today by executing the
operator script as a subprocess (uniform terminal/stdin contract; choice
prompts render as answer buttons). The pane supplies no calibration logic of
its own and may not reimplement topology, tuning, or acceptance rules.

## 14. Implementation mapping

- Domain values and validation: `omotion/calibration/laser.py`
- Shared laser-workflow primitives (preflight validation, default
  establishment, checked register writes, active-default restoration,
  passing-configuration write): `omotion/calibration/_procedure.py`
- UI-neutral procedure workflow: `omotion/calibration/single_sensor_laser.py`
- Motion console bench base and FPGA revision readback:
  `omotion/calibration/motion_bench.py`
- Motion and Ophir adapters: `omotion/calibration/laser_hardware.py`
- Incremental JSON and HTML evidence: `omotion/calibration/reporting.py`
- Shared script scaffolding (argument parsing, operator prompts, artifact
  finalization): `omotion/calibration/script_support.py`
- Operator entry point: `omotion/scripts/wi15_single_sensor_laser_calibration.py`
- Software verification: `tests/test_wi15_laser_calibration.py`,
  `tests/test_wi15_single_sensor_laser_calibration.py`,
  `tests/test_wi15_laser_calibration_hardware.py`,
  `tests/test_wi15_laser_calibration_report.py`, and
  `tests/test_wi15_single_sensor_laser_script.py`

## 15. Hardware verification

The production entry point passed on 2026-08-12 Pacific time using commit
`f42294aaf3b4628f4f4be6c746e0b1ad56cb2f74` and run identifier
`WI-00015-20260813T045443Z`:

- topology: console connected, left sensor connected, right sensor absent;
- Motion firmware: console and left sensor `1.8.0`;
- Ophir: Centauri `3199176`, PE10BF-C `3200878`, with meter calibration due
  2027-10-16 and sensor calibration due 2027-08-22;
- initial observation: 29 valid samples, 345.414 microjoules mean, 2.472
  microjoules standard deviation, and 40.105 Hz;
- one approved upward step: requested `TA_PULSE_WIDTH` 510 microseconds,
  active readback 510.08 microseconds, and 354.207 microjoules mean;
- distinct final observation: 29 valid samples, 353.759 microjoules mean,
  2.474 microjoules standard deviation, and 40.105 Hz;
- final readback checks: `TA_CURRENT_DRV` 5000 mA exactly and
  `TA_PULSE_WIDTH` within 0.016 percent of the 510-microsecond request;
- persisted passing configuration read back exactly, including provisional
  660-microsecond EE/OPT upper limits for standalone Safety Calibration; and
- terminal trigger status `1` (laser off), with no trigger, stream, or
  cleanup failure (since 2026-08-14 a bench-close failure is captured as
  `resource_cleanup_failure` evidence and downgrades a pass, matching the
  dual and safety procedures).

The finalized evidence is stored outside the repository under
`C:\Users\ethan\WI15_runs\WI-00015-20260813T045443Z` as `run.json` and
`report.html`.

Two earlier live executions failed closed before tuning/handoff and informed
the production adapter fixes. Run `WI-00015-20260813T043222Z` exposed a fixed
0.65-second window returning only 23 valid pulses. Run
`WI-00015-20260813T044254Z` exposed stale cross-session Ophir timestamps in the
first nonempty batch. Both runs retained complete failure evidence, restored
active defaults, stopped the trigger, and wrote no tuned configuration.

Post-refactor live regression testing passed on 2026-08-13 using commit
`bf55d72` and run `WI-00015-20260813T220514Z`. The exact single-left topology
remained stable, seven accepted observations stayed at approximately 40 Hz,
and the approved upward sweep selected 540 microseconds at 348.100
microjoules. The distinct final observation was 347.630 microjoules. Both
active-setting checks passed, the complete final configuration read back, the
JSON and HTML artifacts finalized, and no trigger or active-restoration
cleanup failure was recorded.

The same build also exercised the terminal upward-bound NCR on a second
console in run `WI-00015-20260813T220752Z`. Energy rose monotonically from
154.667 microjoules at the 500-microsecond default to only 183.767
microjoules at the 600-microsecond ceiling. Every acquisition-quality check
passed. The procedure returned `failed_ncr`, restored the active defaults,
finalized both evidence artifacts, and did not perform the passing tuned-
configuration handoff.

Post-restructure re-validation completed on 2026-08-14 through the test-app
Procedures pane: passing runs on both sides (left at 570 microseconds, right
at 590 microseconds) and 600-microsecond ceiling NCRs on the dim unit (run
identifiers recorded on openmotion-sdk#214). Run
`WI-00015-20260814T165720Z` additionally motivated carrying the underlying
exception identity into energy-measurement failure reasons.

## 16. Operator override mode (engineering only, 2026-08-21)

Epic OpenwaterHealth/openmotion-bloodflow-app#482 adds a password-gated
operator override for the test-app Procedures pane and headless bench runs
(`omotion/calibration/override.py`, `--allow-override` on the script). It
changes nothing unless it is switched on for a run:

- The script must be started with `--allow-override`; the operator then
  types the override password (three attempts) before any hardware is
  touched, and is asked for the minimum, maximum and target energy for the
  run (10-1000 microjoules each, Enter keeps the factory 300/400/350 value,
  asked again until minimum <= target <= maximum). `--min-energy-uj`,
  `--max-energy-uj` and `--target-energy-uj` pre-supply those answers for
  headless runs and are accepted only together with `--allow-override`.
- The operator's band steers sections 8.2 and 8.3: a closest candidate
  outside it, or energy still below the minimum at the 600-microsecond
  ceiling, no longer ends the run as NCR. The candidate is kept and the miss
  is recorded as an `override` event.
- Before the section 9 write, a final energy outside the factory 300-400
  window, or outside the operator's own band, asks one question with the
  measured energy, both bands and the configuration about to be written,
  then requires a free-text reason. A decline (including EOF, an empty
  reason, or a failing callback) is exactly the section 10 NCR, including
  the active-default restore.
- An accepted override writes through the same exact-readback path as a
  pass and ends in `ProcedureStatus.OVERRIDDEN`: `Final result: OVERRIDE`,
  exit code 3 (`EXIT_OVERRIDE`), and the settings plus the decision
  (operator, justification, timestamp, request) in `run.json` and in the
  HTML report, which renders the status amber and never as passing. A final
  energy inside the factory window stays PASSED even in override mode; the
  mode is still recorded in the evidence.
- Safety Calibration has no override mode, and the clinical bloodflow-app
  never enables override mode for any procedure.
