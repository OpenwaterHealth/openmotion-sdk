# WI-00015 Measurement Calibration Specification

**Date:** 2026-08-12

**Status:** Approved requirements. Currently implemented as a thin
runner over the SDK calibration engine - section 18 records the
section-by-section implementation status (2026-08-14).

**Procedure:** Measurement Calibration

## 1. Objective

Calculate, write, validate, and persist per-camera measurement calibration for
the unit's declared shipping sensor topology. Each module is calibrated
sequentially on a static phantom. A dual-sensor unit runs left and right one
at a time; a one-sensor unit runs only its declared side.

Measurement Calibration does not require an Ophir meter and is never
parallelized across sides.

## 2. Authority and related specifications

This specification implements Measurement Calibration in:

- the approved WI-00015 automated process addendum (filed separately); and
- WI-00015 revision 2, as modified by the team-approved automated process.

The process addendum controls if the sources conflict.

## 3. Scope

### Included

- Declared one- or two-sensor topology.
- Sequential per-side static-phantom operator flow.
- 15-second calibration scan.
- Per-camera image mean, average contrast, and dark mean.
- Factory threshold gate before writing.
- Per-camera calibration-array calculation.
- Target-side-only configuration update preserving the other side.
- 2-second BFI/BVI validation scan.
- Final minimum 15-second power cycle and complete persistence verification.
- CSV/JSON and human-readable report evidence.

### Excluded

- Parallel left/right calibration.
- Ophir operation or laser-energy tuning.
- Safety ADC calculations.
- A hard stored-state requirement proving earlier procedures passed.
- Final TestApp UI.

## 4. Entry point and reusable API boundary

The operator entry point is a dedicated Measurement Calibration script. It
collects the declared topology and guides static-phantom placement one side
at a time.

Shared SDK code receives explicit expected sides, motion interface, scan
adapter/workflow, thresholds, configuration store, power-cycle adapter,
state/report sink, and progress/cancellation callbacks. It must not call
`input()`.

It returns a structured procedure outcome plus one structured side outcome
per expected side, including raw artifact paths, per-camera rows, calibration
arrays, write/readback evidence, validation results, and terminal reason.

## 5. Topology and identity preflight

1. Operator declares one-sensor `left`, one-sensor `right`, or two-sensor
   topology.
2. The console and all sides required for the declared topology must be
   connected and responsive at procedure start.
3. Console and required sensor serial numbers must be non-`None` and
   non-empty.
4. A missing side on a two-sensor unit is a topology failure.
5. A two-sensor unit may not be calibrated as two one-sensor executions.
6. Record firmware, FPGA, hardware IDs, and available identities.
7. Read and preserve the complete starting User Configuration.

The procedure does not enforce stored proof that Laser or Safety Calibration
previously passed.

## 6. Sequential physical workflow

Use a deterministic side order: left then right for a two-sensor unit, or the
declared side for a one-sensor unit.

Before each side:

1. prompt the operator to place only the target module on the static phantom
   in the required orientation with the specified weight;
2. require an explicit placement/no-touch confirmation;
3. verify the target side remains connected; and
4. ensure the non-target side is not included in that side's calibration
   camera masks or acquisition.

Do not start the next side until the current side has a terminal side
outcome. Do not use a `both` calibration request.

## 7. Calibration scan and statistics

Run a 15-second calibration scan for the target side using the active
persisted laser/safety configuration. Capture raw scan artifact paths.

For every active camera independently calculate:

- `mean_average`: arithmetic average of per-frame image means;
- `contrast_average`: arithmetic average of each valid frame's
  `standard_deviation / mean`; and
- `dark_mean`: arithmetic average of applicable laser-off dark-frame image
  means.

Do not calculate contrast as `average standard deviation / average mean`.
Require sufficient valid samples for every active camera and finite
statistics. Missing-camera or non-finite data fails the side before writing.

## 8. Versioned pre-write threshold gate

Use the versioned factory threshold set:

| Camera | Minimum mean | Minimum average contrast | Maximum dark mean |
|---:|---:|---:|---:|
| 1 | 40 | 0.25 | 3.0 |
| 2 | 80 | 0.25 | 3.0 |
| 3 | 80 | 0.25 | 3.0 |
| 4 | 80 | 0.25 | 3.0 |
| 5 | 80 | 0.25 | 3.0 |
| 6 | 80 | 0.25 | 3.0 |
| 7 | 80 | 0.25 | 3.0 |
| 8 | 40 | 0.25 | 3.0 |

Bounds are inclusive. Every active camera must have:

- `mean_average >= minimum_mean`;
- `contrast_average >= 0.25`; and
- `dark_mean <= 3.0`.

If any criterion fails, mark the side and procedure failed, record all rows,
and stop before a calibration configuration write. There is no `allow-dim`,
bench-threshold, or continue-to-write override in the supported procedure -
except the password-gated engineering override mode of section 19, which is
off unless the script is started with `--allow-override`.

## 9. Calibration calculation

For each passing active camera calculate:

- `I_min = 0.0`;
- `I_max = 2.0 * mean_average`;
- `C_min = 0.0`; and
- `C_max = contrast_average`.

Arrays retain the console's required two-side, eight-camera shape. Only the
targeted side's active entries are replaced. Values for the non-target side
are copied from the complete configuration read immediately before the side
write.

For a two-sensor procedure, the second-side update must therefore preserve
the first side's newly passing calibration values. For a one-sensor
procedure, do not manufacture measurements for the absent side.

## 10. Configuration write and immediate verification

1. Construct the complete intended User Configuration with the updated
   calibration block.
2. Preserve all non-calibration keys and the non-target side.
3. Write the complete configuration and require SDK success.
4. Read back immediately.
5. Require all intended non-calibration values and the complete calibration
   arrays to match.
6. Store intended and actual objects plus a targeted-side diff.

Write or readback failure fails the procedure. The calibration block is not
considered accepted until the validation scan passes.

## 11. Two-second validation scan

Run a 2-second scan for only the target side with the newly written
calibration active. For every active camera, calculate time-averaged BFI and
BVI using the SDK's normal calibrated processing path.

Every active camera must pass both inclusive ranges:

- `-0.5 <= BFI <= 0.5`; and
- `4.5 <= BVI <= 5.5`.

Do not average values across cameras for acceptance. Missing, non-finite, or
out-of-range BFI/BVI fails the side and procedure. Record the written
calibration and failure; do not claim the side passed.

## 12. Multi-side behavior

For a two-sensor unit:

1. calibrate and validate left;
2. only after left passes, prompt for right;
3. calibrate and validate right while preserving left; and
4. proceed to final persistence only after both pass.

For a one-sensor unit, complete only the declared side. Neither mode uses a
parallel/both request.

If a later side fails, retain and report every earlier side outcome but mark
the overall procedure failed. Do not perform the final passing persistence
claim.

## 13. Final post-calibration persistence

After every required side passes:

1. retain the complete intended laser, safety, and calibration configuration;
2. stop scan/laser activity;
3. power off for at least 15 measured seconds;
4. restart and prove reboot using firmware uptime or equivalent evidence;
5. reconnect the declared topology;
6. read the complete User Configuration; and
7. require every intended key and calibration value to be unchanged.

Insufficient dwell, unproven restart, reconnect/read failure, or any mismatch
fails the procedure.

## 14. Acceptance and failure behavior

Measurement Calibration passes only when every expected side passes its
pre-write gates, checked configuration write/readback, and per-camera BFI/BVI
validation, followed by complete post-restart persistence.

Every failure returns nonzero with side, camera when applicable, criterion,
observed value, threshold, and reason. Cleanup always stops scan/trigger
activity. No threshold override or continue-anyway prompt is permitted in the
supported procedure outside the engineering override mode of section 19.

## 15. Report and artifacts

Record:

- declared topology, ordered target sides, identities, and serial checks;
- static-phantom confirmations and timestamps;
- threshold-set name/version and every numerical threshold;
- calibration and validation scan settings/durations/artifact paths;
- per side/camera mean, contrast, dark, validity, threshold, and result;
- per side/camera calculated `I_min`, `I_max`, `C_min`, and `C_max`;
- complete before/intended/immediate-readback configuration comparisons;
- evidence that a second-side write preserved the first side;
- per side/camera BFI, BVI, ranges, and results;
- calibration CSV and JSON paths using the current application naming;
- final 15-second dwell, restart proof, and complete persistence comparison;
  and
- terminal outcome with exact side/camera failure context.

## 16. Automated tests

Unit tests cover:

- single-left, single-right, and dual topology;
- missing/wrong topology and invalid serials;
- deterministic left-then-right order and no `both` request;
- 15-second and 2-second requested durations;
- average-of-ratios contrast, not ratio-of-averages;
- missing/non-finite camera data;
- every mean, contrast, and dark threshold at/below/above boundary;
- proof pre-write failure performs no write;
- exact calibration-array formulas and shapes;
- target-only update preserving non-target side;
- second-side update preserving first-side results;
- write failure and immediate full-readback mismatch;
- every BFI/BVI inclusive boundary and per-camera failure;
- multi-side failure preventing final pass claim;
- 15-second dwell, restart proof, and complete persistence mismatch;
- guaranteed scan cleanup; and
- required report/CSV/JSON evidence.

## 17. Future TestApp integration

This procedure maps to one future TestApp button. The UI guides side-by-side
phantom placement and renders per-camera outcomes while calling the same
shared SDK implementation. It may not use `both`, parallelize sides, alter
thresholds, or reproduce calibration math in UI code.

## 18. Implementation status (2026-08-14)

The supported operator flow today is `omotion/scripts/wi15_measurement_calibration.py`,
a thin runner over the SDK calibration engine (`omotion/CalibrationWorkflow.py`
via `MotionInterface.start_calibration`) - the same engine behind the
bloodflow-app's Calibrate button. Against the sections above:

**Implemented:**

- One side per run on the static phantom with a mandatory placement
  attestation (section 6's placement/no-touch confirmation). The
  attestation is phrased positively - ensure the module has been moved
  from the 0 cm fixture to the static phantom - per Ethan's 2026-08-24
  direction.
- Operator and fixture-ID collection matching the other WI-15
  procedures (`--operator`/`--fixture-id` or a required prompt); the
  fixture ID is recorded in the engine request notes, which land in the
  engine's JSON evidence.
- 15-second calibration scan and 2-second validation scan (sections 7
  and 11; the engine gained a first-class `validation_duration_sec`
  on 2026-08-14 - previously the validation scan reused the calibration
  duration).
- Average-of-ratios contrast, per-camera statistics, and the
  `I_min = 0 / I_max = 2 x mean / C_min = 0 / C_max = contrast` arrays
  (sections 7 and 9; `CALIBRATION_I_MAX_MULTIPLIER = 2.0`).
- The versioned factory mean/contrast thresholds and SPEC-69 BFI/BVI
  validation bounds (sections 8 and 11), with cameras displayed 1-8.
- The absolute below-threshold gate: a below-threshold calibration is
  never written - the gate prints the failing rows and always refuses;
  there is no consent path (section 8).
- Target-side-only update preserving the other side: the engine copies
  the non-targeted side from the live console calibration baseline
  (section 9; bloodflow-app #117 semantics).
- Engine CSV/JSON evidence artifacts (part of section 15), named
  serial-first (`<console-serial>-calibration-<ts>.csv/.json` via the
  engine's `artifact_prefix`) inside a run folder that is renamed
  `<console-serial>-measurement-cal-<runid>` once the interface has
  released its file handles - so listings sort by unit, matching the
  other WI-15 procedures. Prefix and rename are skipped when the console
  serial is unreadable.

**Divergences and future work:**

- **One side per invocation** (per Ethan's 2026-08-13 direction): a
  dual unit runs the script once per side. The single-procedure
  sequential dual flow of sections 5 and 12 - including the rule that a
  two-sensor unit may not be calibrated as two one-sensor executions -
  applies to the future full workflow, not the current thin runner. The
  runner reminds the operator on every pass that a shipping dual unit
  needs the other side calibrated in its own run.
- **Dark-mean gate placement** (section 8): the engine evaluates the
  3.0 dark maximum against the validation scan's dark frames, after the
  write - a dark failure fails the procedure but does not prevent the
  write the way the mean/contrast gates do.
- **Final power-cycle persistence** (section 13) is not performed by the
  thin runner.
- **Dedicated evidence workflow** - durable run recorder, HTML report,
  and the full section 15/16 evidence and test matrix - remains future
  work; current evidence is the engine's CSV/JSON plus the terminal
  transcript.
- The CLI keeps a loud `--bench-thresholds` development flag (disables
  the absolute brightness gates for dim dev benches and prints that a
  pass does not certify signal level). The supported operator flow (the
  test-app Procedures pane) never passes it.

## 19. Operator override mode (engineering only, 2026-08-21)

Epic OpenwaterHealth/openmotion-bloodflow-app#482 adds a password-gated
operator override to the thin runner (`--allow-override`); the engine hook
behind it (`MotionInterface.start_calibration(on_override_fn=...)`) is
passed only by this runner, never by the bloodflow-app, so the section 8
never-write rule is unchanged everywhere else:

- The operator types the override password (three attempts) before any
  hardware is touched. The factory limits stay in force; this is not the
  `--bench-thresholds` flag.
- When the section 8 gate fails, the engine asks once with the failing
  cameras' mean and contrast, then requires a free-text reason. A decline
  is the usual FAILED run with nothing written.
- An accepted override continues to the section 11 validation scan and
  then writes the console once, unless any camera fails the ambient-dark
  check - that is a data-integrity fault and is never overridable, so the
  run FAILS with nothing written.
- The run ends with outcome `overridden` (`Final result: OVERRIDE`, exit
  code 3); the JSON manifest records `override_granted` and
  `override_justification` alongside `calibration_written`.
