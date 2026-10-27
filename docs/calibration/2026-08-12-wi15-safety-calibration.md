# WI-00015 Safety Calibration Specification

**Date:** 2026-08-12

**Status:** Implemented and live-verified (console-only operator flow,
2026-08-14; run IDs on openmotion-sdk#214). Amended 2026-08-14 per Ethan's
rulings: console-only operator flow, 1-second minimum dwell, observed
(confirmation-free) power cycle.

**Procedure:** Safety Calibration

## 1. Objective

Derive and persist laser safety current and pulse-width limits from the
console's internal safety ADC measurements and prove those values survive a
power cycle.

The procedure is console-side only: the ADC calculation is independent of
sensor modules and does not require an Ophir meter. The supported operator
flow always runs with the console-only topology - sensor modules may be
attached or absent, they are not used, and the normal-scan stage is recorded
as not applicable. The workflow API still accepts a declared one- or
two-sensor topology, in which case a normal 30-second scan runs with the
persisted values active and no overrides.

## 2. Authority and related specifications

This specification implements Safety Calibration in:

- the approved WI-00015 automated process addendum (filed separately); and
- WI-00015 revision 2, as modified by the team-approved automated process.

The process addendum controls if the sources conflict.

## 3. Scope

### Included

- Console connection and identity validation.
- Mandatory TA, Seed, Safety EE, and Safety OPT console-board FPGA firmware
  revision readback before configuration mutation or firing.
- Current tuned configuration capture.
- TA current and pulse-width plus/minus 2 percent readback check.
- At least ten scaled-mA ADC samples per safety controller while firing.
- Mean, multiplier, and nearest-integer current-limit calculations.
- Pulse-width-limit calculation.
- Complete configuration write and immediate readback.
- Minimum 1-second measured power-off dwell and persistence verification.
- Normal 30-second sensor-data scan with no configuration overrides
  (declared-topology API runs only; recorded not applicable for the
  console-only operator flow).
- Laser-safety-warning failure detection and report evidence.

### Excluded

- Ophir connection or energy measurement.
- Laser midpoint tuning.
- Static-phantom calibration calculations.
- A hard stored-state requirement proving Laser Calibration previously ran.
- Final TestApp UI.

## 4. Entry point and reusable API boundary

The operator entry point is a dedicated Safety Calibration script. Existing
low-level runner commands may remain available, but the dedicated procedure
is the supported operator flow.

Shared SDK code receives one bench adapter (owning the Motion session,
configuration I/O, ADC reads, power-cycle coordination, normal-scan
execution, and warning capture), plus the run recorder, ADC sampling
policy, and clock; the declared topology arrives on the request. It does
not call `input()`.

It returns a structured outcome containing input configuration, raw ADC
samples, calculations, requested and readback configurations, restart proof,
scan/warning result, and terminal disposition.

## 5. Preconditions and local validation

Safety Calibration does not require stored evidence from Laser Calibration.
It independently validates the state it consumes:

1. Console must be connected, responsive, and have a non-`None`, non-empty
   serial number.
2. Read and preserve the complete current User Configuration.
3. Require finite, positive `TA_CURRENT_DRV` and `TA_PULSE_WIDTH` values and
   the required rate/limit keys needed to construct the final configuration.
4. Bring up the persisted operating point without temporary laser or safety
   overrides.
5. Compare requested TA current and pulse width with active hardware/UI
   readbacks and require each to be within plus or minus 2 percent.
6. Verify the trigger rate reads 39-41 Hz inclusive, first correcting it
   to 40 Hz when the initial reading is not exactly 40.

Failure stops before ADC-derived configuration is written.

No sensor module or containment fixture is required for the ADC acquisition.

## 6. ADC sampling

### 6.1 Firing lifecycle

1. Guard firing in the bench adapter: the trigger rate must read 39-41 Hz
   and `TA_PULSE_WIDTH` must be strictly below both active pulse-width
   upper limits.
2. Start the laser once.
3. Collect OPT and EE ADC samples while the laser is actively firing.
4. Stop the laser in a guaranteed cleanup path on success, cancellation, or
   exception.
5. Capture laser safety warnings throughout the firing window.
6. Require at least one known (non-unknown) safety-telemetry observation
   during the firing window; zero known observations fail the run even when
   every ADC read succeeded.

### 6.2 Sample validity

For each of `SAFETY_OPT ADC_DATA` and `SAFETY_EE ADC_DATA`:

- collect at least ten successful values;
- use scaled engineering-unit mA returned/displayed by the SDK/TestApp
  conversion, never raw ADC counts;
- reject exceptions, missing values, booleans, NaN, infinity, and
  negative values;
- record rejected reads and their reasons; and
- fail rather than calculating from fewer than ten valid readings.

Sampling stops for a controller once ten values are accepted (each mean is
over exactly ten), and at most 30 read attempts are made per controller.

Sampling may be sequential or interleaved, provided every accepted value was
read during the same active firing period and both controllers meet the
minimum count.

## 7. Safety calculations

Use the arithmetic mean of the accepted samples (exactly ten per
controller) for each controller.

1. `opt_mean_mA = sum(opt_samples) / len(opt_samples)`
2. `ee_mean_mA = sum(ee_samples) / len(ee_samples)`
3. `OPT_DRIVE_CL = nearest_integer(opt_mean_mA * 1.3)`
4. `EE_DRIVE_CL = nearest_integer(ee_mean_mA * 1.1)`

Record the sample list, count, mean, multiplier, unrounded product, rounding
operation, and result for both controllers.

Use one shared explicit nearest-integer helper so Python's implicit/banker's
rounding cannot make report and configuration disagree at an exact half.
The helper and its tie behavior must be unit tested and stated in the report.

## 8. Pulse-width limit calculation

Read the requested integer `TA_PULSE_WIDTH` from User Configuration.

- When below 600 microseconds, calculate both `EE_PULSE_WIDTH_UL` and
  `OPT_PULSE_WIDTH_UL` as `nearest_integer(TA_PULSE_WIDTH * 1.1)`.
- At exactly 600 microseconds, require and retain 660 for both limits.
- A pulse width above 600 or a 600-microsecond operating point without the
  required 660 limits is a configuration failure.

Do not change the rate lower limits, tuned TA values, seed value, `TEC_TRIP`,
or unrelated permitted configuration data.

## 9. Configuration write and immediate verification

1. Construct the complete intended configuration by updating the current
   object with the four derived safety limits.
2. Preserve all other permitted current keys and values.
3. Write the complete object and require successful SDK operation.
4. Read it back immediately.
5. Compare every intended key/value, not only the four updated fields.
6. On write failure or mismatch, fail and do not claim persistence.

Record current, intended, and immediate-readback configurations with a
highlighted comparison.

## 10. Power-cycle persistence verification

1. Stop all laser/scan activity.
2. Instruct the operator once to power the console off and back on. There
   are no confirmation gates: the coordinator observes the disconnect and the
   reconnect itself and never prompts for input during the cycle.
3. Console liveness is judged by monitor state AND a command echo round-trip,
   because Windows can keep a surprise-removed COM port "present" under an
   open handle - a state check alone can miss the power-off.
4. The measured off dwell must be at least 1 second. A faster flip is
   measured, recorded, and fails the run (the dwell cannot be proven).
5. Restart proof for the observed manual cycle is the observed disconnect
   and reconnect of the same long-lived console handle, recorded as the
   `restart_proof` text; firmware uptime is not read.
6. Read the complete User Configuration.
7. Require every intended key/value to be unchanged.

An unproven cycle, insufficient dwell, reconnect failure, read failure, or
mismatch fails the procedure.

## 11. Normal 30-second scan verification

The console-only operator flow records this stage as not applicable
("Normal 30-second scan not applicable") and passes without it. When the
workflow API is invoked with a declared one- or two-sensor topology, this
final end-to-end check requires that topology connected:

- one-sensor: exactly the declared left or right module;
- two-sensor: both left and right modules.

Required sensor serial numbers must be non-`None` and non-empty. Run the
ordinary production/SDK sensor-data scan path for 30 seconds using the
persisted configuration. Do not pass temporary trigger, laser, safety, or
calibration overrides.

Pass only if:

- the scan starts;
- it runs for the required duration and completes normally;
- expected topology remains connected;
- no laser safety warning is raised; and
- no safety interlock prevents normal scanning.

Any failure returns nonzero. Always stop scan/trigger activity in cleanup.

## 12. Acceptance and failure behavior

Safety Calibration passes only when local preflight, readback tolerance, both
ADC sample sets, all calculations, immediate configuration verification,
power-cycle persistence, and the normal-scan stage pass (for the console-only
operator flow the scan stage passes by being recorded not applicable).

The procedure records the exact failed gate. It does not require Ophir or
stored proof of a prior procedure, and it does not offer a continue-anyway
path after a failed write, persistence check, or safety scan.

Two further terminal gates run in the operator script: HTML-report
generation failure and bench-close/resource-cleanup failure each downgrade
a workflow pass to a failed terminal result, so a stale passing artifact
cannot survive either.

## 13. Report evidence

Record:

- console identity and topology (final-scan sensor identities for
  declared-topology runs only);
- all four console-board FPGA firmware revisions, without sensor-camera FPGA
  revision fields in the human report;
- complete input configuration;
- TA requested/readback values and 2 percent calculations;
- trigger frequency result;
- every accepted/rejected OPT and EE ADC read;
- mean, multiplier, unrounded result, rounding/tie rule, and final limit;
- pulse-width-limit calculation;
- complete intended and immediate-readback configurations;
- power-off timestamps/dwell and restart proof;
- post-restart complete configuration and its equality criterion (the
  highlighted diff is rendered for the pre-cycle handoff);
- for declared-topology runs, the scan configuration showing no overrides
  and scan start/end/duration/outcome with every safety warning (the
  console-only report has no scan section - the stage appears as not
  applicable in the event timeline);
- resource-cleanup diagnostics and report-artifact finalization evidence;
  and
- terminal disposition and reason.

The durable artifacts are named `<console-serial>-safety-cal-run.json` and
`<console-serial>-safety-cal-report.html`, and at finalization the per-run
directory itself (created as `WI-00015-<timestamp>`) is renamed to
`<console-serial>-safety-cal-<timestamp>` - serial first throughout, so
listings sort by unit (serial prefix omitted when preflight never read one;
the JSON lives under `run.json` until finalization renames it atomically).

## 14. Automated tests

Unit tests cover:

- no Ophir dependency;
- console connection and serial failures;
- missing/invalid current configuration fields;
- plus/minus 2 percent boundaries;
- trigger correction;
- fewer than ten versus ten ADC samples;
- rejected exceptions, NaN, infinity, and booleans;
- scaled-mA usage and exact means;
- both multipliers and nearest-integer tie behavior;
- pulse widths below 600, exactly 600, and above 600;
- write failure and complete-readback mismatch;
- measured minimum-dwell and restart-proof failures (including a
  too-fast cycle);
- console-only, one-, and two-sensor normal-scan topology handling;
- proof that no scan overrides were supplied;
- early scan failure, disconnect, warning, and successful 30-second scan;
- guaranteed laser/scan cleanup; and
- required report fields.

## 15. Future TestApp integration

This procedure maps to one future TestApp button. The TestApp presents
connection, power-cycle, and scan progress while calling the same shared SDK
implementation. It may not recalculate limits or weaken warning/persistence
gates in UI code.

## 16. Implementation mapping and verification status

- Safety rules and immutable evidence records:
  `omotion/calibration/safety.py`.
- Shared evidence primitives and report-artifact states:
  `omotion/calibration/_procedure.py`.
- Console bench base, scaled-mA register I/O, echo liveness probe, and FPGA
  revision readback: `omotion/calibration/motion_bench.py`.
- Shared script scaffolding (report/cleanup terminal gates, artifact
  finalization): `omotion/calibration/script_support.py`.
- UI-neutral procedure orchestration:
  `omotion/calibration/safety_workflow.py`.
- Motion console, FPGA, power-cycle, and normal-scan adapter:
  `omotion/calibration/safety_hardware.py`.
- Auditor-readable HTML evidence:
  `omotion/calibration/safety_report.py`.
- Current script-only operator entry point:
  `omotion/scripts/wi15_safety_calibration.py`.

The automated domain, workflow, adapter, report, and operator-script tests pass
against simulated hardware. A live dual-sensor execution passed on 2026-08-13
using run `WI-00015-20260813T204811Z` and commit `712199f`. Ten accepted samples
per controller produced an OPT mean of 1991.874 mA and rounded 1.3x limit of
2589 mA, plus an EE mean of 4163.610 mA and rounded 1.1x limit of 4580 mA. The
complete immediate readback matched the intended configuration. The procedure
then observed console disconnection, measured 15.000 seconds off, reconnected
to the same `ZZZ99Z99999` console, and verified an identical complete persisted
configuration. The ordinary dual-sensor scan requested 30 seconds with no
overrides, completed without cancellation or error after normal pipeline
drain, recorded 34 known-clear safety observations, and finalized both JSON and
HTML artifacts with a passing disposition.

The console-only operator flow was live-verified end-to-end on 2026-08-14
(observed power cycle 7.6 seconds, persistence proven, scan stage recorded
not applicable; run IDs on openmotion-sdk#214). The declared-topology
normal-scan variants remain reachable only through the workflow API; any
future use of them requires re-verification against the current code - the
2026-08-13 dual live run predates both the console-only amendment and the
2026-08-14 fix that re-applies the persisted laser drive point after the
power cycle, before the scan.
