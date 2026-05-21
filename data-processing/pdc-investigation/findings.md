# PDC investigation — why we're not building a per-frame PDC amplitude correction

**Status:** investigation closed. Per-frame PDC plumbing on
``feature/per-frame-pdc`` retained as a diagnostic stream. The
correction for laser-intensity-driven BFI bias is the shot-noise
correction already in the science pipeline (Soren's code).

## TL;DR

1. **The mPD (PDC) and the fiber output sample orthogonal
   polarizations** of the TA's output, separated by a PBS inside the
   laser package. The TA's polarization ratio drifts in time, so PDC
   and fiber-output power are **negatively correlated** under polarization
   shift and **positively correlated** under total-power change — the
   two effects are not separable from PDC alone.
2. **PDC is therefore an unreliable proxy** for the optical power
   actually reaching the tissue, and amplitude-scaling BFI/BVI by a
   ``pdc_ref / pdc_frame`` ratio can actively *worsen* the result
   when polarization drift is the dominant variation.
3. **The correction we actually want is the shot-noise correction,**
   subtracting the Poisson-variance contribution from the speckle
   contrast: ``contrast_true² = (σ² − k_e · μ) / μ²``. This is
   already implemented in ``SciencePipeline._emit_corrected_for_camera``
   and (post #data-pipeline-tweaks) in ``_emit_realtime_corrected``.
   The constants ``ADC_GAIN`` and ``CAMERA_GAIN_MAP`` are the values
   Omnivision specified — no change required.
4. **Per-device variance in PDC is real and significant** (different
   PBS performance, fiber-coupling tolerances, polarization-mode
   distribution out of the TA), but it's an optomechanical problem the
   science pipeline can't paper over.

## The polarization-split mechanism

Inside the laser package, the TA's output passes through a polarizing
beamsplitter (PBS). One polarization is reflected to the mPD that
produces our PDC signal. The orthogonal polarization is transmitted
through the PBS into the output fiber:

```
   ┌────┐    ┌────┐    ┌──────────┐
   │ TA ├───►│PBS ├───►│  Fiber   ├──► to tissue
   └────┘    └─┬──┘    └──────────┘
               │ (other polarization)
               ▼
            ┌────┐
            │ PD │  ← this is what PDC measures
            └────┘
```

The TA's output polarization ratio is **not stable in time** —
internal birefringence, thermal gradients in the TA chip, and seed-current
sensitivity all shift the split between the two orthogonal modes
even when total output power is constant. When that happens,
power on the PD goes one way and power into the fiber goes the other.

**Evidence (PDC Investigation deck, slide 15):**

> "Internal and external measurements have different polarization
> and inverted oscillations."

The plot on that slide overlays in-package PDC and fiber-output
energy-meter readings over a 1000 s window. The two traces visibly
move in opposite directions across the slow features. This is
direct proof that PDC and fiber-output power are not the same signal.

## Evidence from the Thorlabs/Ophir/mPD comparison study (20260113)

Setup: Console EVT2-015, Laser SN0031, with three meters running in
parallel — the laser mPD (PDC), an Ophir pyroelectric energy meter at
40 Hz, and a Thorlabs integrating-sphere peak power meter at ~10 Hz.

### With fiber splitter (trials 1–3)

The three meters disagree in different ways trial to trial:
- Trial 1: all three discrepant on the temperature step.
- Trial 2: Ophir is the outlier on the 120 mA seed step.
- Trial 3: Thorlabs is the outlier on the same step.

Read: **the fiber splitter itself is a time-variable optical path.**
Power at each splitter output is not a fixed fraction of the laser
output — the split ratio drifts.

### Without fiber splitter (slides 8–10)

> "Both Thorlabs and Ophir setups correlate well with the mPD value
> overall ... No indication that the mPD is not a good representation
> of the laser's overall power."

When the splitter is removed, PDC tracks the external power meters
under perturbations of TA current (5.0 → 4.5 → 4.0 A) and seed
current (140 → 120 → 100 mA). One caveat: at the seed 120 mA step,
Thorlabs shows a **reverse trend** vs PDC, which is the polarization
signature again — that seed current happens to shift the
polarization ratio inverse to total output power.

**What this tells us about the laser package:** PDC is a reasonable
proxy for the power-leaving-the-PBS-into-the-fiber **when polarization
is stable** (e.g. across small TA-current steps). It breaks down when
polarization itself shifts (e.g. cold-start thermal soak, certain seed
current setpoints).

## Evidence from the multi-device DVT1A scans (20260316)

Five DVT1A units (QWW04Q10005 / 10012 / 10013 / 10014 / 10017) plus
one scan on Console 005. Each scan: 1200 s on a static phantom, with
PDC and pyroelectric energy meter co-recorded.

### Cross-device qualitative observations

- **PDC vs camera mean correlation is inconsistent across devices.**
  Some units show the mean tracking PDC well (e.g. QWW04Q10005 — both
  drift down together late in the scan). Others show the mean drifting
  the opposite direction (QWW04Q10013 — mean drifts up while PDC drifts
  down across the late half of the scan).
- The deck's conclusion was: "Not clear how to use PDC data to
  calibrate mean."
- **Sensor-module-side asymmetry:** cameras on positions 4–7 are
  consistently noisier than positions 0–3 in dark variance. This is
  a hardware geometry effect inside the sensor module, not PDC-related.

### Dark intensity / variance reproduces our dark-drift study

The "Dark Intensity" and "Dark Variance" plots in each device's
section show exactly the same shape we characterized independently
in ``data-processing/dark-drift-study/`` — u1 drifts by a fraction
of a bin index over the scan, variance drifts substantially in a
thermal-settling curve. This is the camera, not the laser.

### Camera 0 oscillation

The right-maskFF scan (scan_owFD016P_20260305) shows camera 0 (a
gain-16 outer-row camera) with a clearly periodic oscillation in
contrast / std / mean (~250 s period). PDC and the energy meter
both show similar slow oscillations. This is consistent with the
package-internal polarization beating with whatever thermal cycle
the system is in.

## Why amplitude scaling by `pdc_ref / pdc_frame` is the wrong correction

The proposed correction would do:

```
corrected_mean_scaled = corrected_mean × pdc_ref / pdc_frame
```

This is exactly right **iff** `pdc_frame / pdc_ref` is the same as
`(actual fiber-output power) / (fiber-output power at the reference
condition)`. The polarization-split data shows that's not the case.

Two specific failure modes:

1. **Polarization drift at constant total power.** PDC moves up, the
   fiber output (and therefore the photon flux on the sensor) moves
   down. The correction scales the mean down further, which is the
   exact opposite of what we want.

2. **Per-device variance.** Even if we calibrate ``pdc_ref`` per unit
   at the bench, the relationship `pdc / fiber_output` is itself
   time-varying within a single scan because polarization drifts.
   No static `pdc_ref` (per-unit or system-wide) handles this.

The "build-to-build not reproducible" point from the scientist's
review applies at *two* levels:
- The PBS / fiber-coupling assembly varies unit-to-unit (per-device
  variance, optomechanical).
- The polarization itself varies *within a scan* (within-device
  variance, polarization-mode drift).

A SW correction can't fix either. The fix is upstream in the
optical assembly.

## The shot-noise correction — what we actually want, and what we
already have

The laser-intensity-driven bias on BFI comes from a different
mechanism than amplitude scaling addresses. As laser intensity drops:

- Mean photon flux μ drops linearly.
- Speckle-pattern variance ``σ²_speckle`` drops as μ² (it's a relative
  fluctuation around the mean).
- **Shot-noise variance** drops only as μ (Poisson statistics).

So the *relative* shot-noise contribution to total measured variance
grows as μ shrinks. The measured contrast:

```
contrast_measured² = σ_total² / μ²
                  = (σ_speckle² + σ_shot²) / μ²
                  = contrast_speckle² + k_e · μ / μ²
                  = contrast_speckle² + k_e / μ
```

To recover the true speckle contrast we subtract the shot-noise
variance term before computing contrast:

```
contrast_true² ≈ (σ² − k_e · μ) / μ²
```

with `k_e` the total camera gain in DN per electron. This is the
textbook DCS / SCOS correction and is what Soren landed in the
science pipeline.

### Confirming the implementation in the SDK

`omotion/MotionProcessing.py`:

```python
ADC_GAIN: float = (1024 - 64) / 11_000          # DN per electron ≈ 0.0873
CAMERA_GAIN_MAP: np.ndarray = np.array(
    [16, 4, 2, 1, 1, 2, 4, 16], dtype=np.float64
)
```

And in `_emit_corrected_for_camera` (batched path) plus
`_emit_realtime_corrected` (real-time path on
``feature/data-pipeline-tweaks``):

```python
corrected_mean = fm.u1 - dark_u1
raw_var        = fm.u2 - fm.u1 * fm.u1
corrected_var  = raw_var - dark_var

# Shot noise variance in DN = ADC_GAIN · analog_gain · mean_DN.
cam_pos        = int(key[1]) % 8
shot_noise_var = ADC_GAIN * max(0.0, corrected_mean) * CAMERA_GAIN_MAP[cam_pos]
corrected_var -= shot_noise_var

corrected_std      = sqrt(max(0.0, corrected_var))
corrected_contrast = corrected_std / corrected_mean    # → BFI/BVI
```

Substituting the constants:
`k_e = ADC_GAIN × CAMERA_GAIN_MAP[cam_pos] = 0.0873 × {16, 4, 2, 1, 1, 2, 4, 16}`

The 11,000 e⁻ full-well figure in `ADC_GAIN` and the per-position
analog-gain map are the Omnivision-supplied values for the OV2312 at
our operating conditions. They are correct.

## What per-frame PDC is still good for

Drop PDC from the BFI/BVI correction math. Keep the per-frame PDC
stream as a diagnostic channel:

- **Laser-health surveillance.** PDC drops to zero → laser off /
  failed. PDC plateaus at an unusual value → warmup anomaly. PDC
  steps mid-scan → switching transient. None of these need to feed
  the BFI math, but they all need to be visible in saved data and
  the live UI.
- **Cross-device characterization.** The unit-to-unit variance we
  saw in the DVT1A scans is useful when triaging hardware issues
  (which units have which kind of PBS/fiber-coupling behavior).
- **Triggering the upstream optical fix.** If a unit shows
  pathological PDC ↔ fiber-output decoupling, that's a build issue
  to send back, not something to compensate for in SW.

## Decisions resulting from this investigation

1. **Drop** the PDC-amplitude-scaling correction design. Do not pursue
   per-device `pdc_ref` calibration or any variant of
   `mean *= pdc_ref / pdc_frame`.
2. **Keep** the existing shot-noise correction in the science
   pipeline. Constants are correct; no code change needed.
3. **Keep** the per-frame PDC plumbing on ``feature/per-frame-pdc``
   as a diagnostic stream. Make sure it's stored in `session_data`
   and visible in the live UI when relevant, but do *not* wire it
   into the corrected BFI/BVI math.
4. **Defer** to the optomechanical team on per-device PDC variance.
   Henry's polarization investigation + a unit teardown are the
   next steps, not a SW change.

---

## Appendix A — 2026-05-21 hardware verification of the per-frame PDC stream

After the investigation closed, the per-frame PDC plumbing was finished
and brought up on hardware. This appendix documents what we now know
about the stream's behaviour end-to-end. It does **not** change any of
the decisions above — it just substantiates point 3 (PDC kept as a
diagnostic) with measured data.

### Pipeline summary as deployed

- **Console FW** samples the safety FPGA's `peak_power_value`
  (mux 1 ch 7 addr 0x41 reg 0x1C, 2 B LE × 1.9 mA/LSB) once per camera
  frame in `LSYNC_DelayElapsedCallback` (CC1 / rising edge of the laser
  pulse). Frame index and `dark_slot` are tagged in-ISR and enqueued to
  a 32-slot SPSC ring; the main loop's `pdc_poll_tick` waits ~2 ms for
  the FPGA's averaging window and pushes a `pdc_sample_t` (7 bytes
  packed) into a 256-entry SRAM ring buffer.
- **SDK** drains the ring buffer at 10 Hz via the new
  `OW_CTRL_GET_PDC_BUFFER` opcode (`0x25`) inside
  `ConsoleTelemetryPoller._tick_once`. Samples fire through
  `add_pdc_listener(PdcSample)`. The slow telemetry refresh continues
  at 1 Hz on every 10th tick; the dedicated PDC I²C read in the slow
  path was removed (`snap.pdc` is now sourced from the most recent
  `PdcSample`).
- **`ScanWorkflow`** writes one telemetry-CSV row per drained
  `PdcSample` — 40 Hz steady state — with slow columns carry-forwarded
  from the last `ConsoleTelemetry` snapshot. New appended columns:
  `frame_idx, dark_slot, pdc_flags, pdc_dropped_delta, slow_age_ms`.

### Verification scan — 600 s on bench

`scripts/test_pdc_vs_mean.py --duration 600 --subject pdc_10min`

Both sensors connected, masks `0xFF`/`0xFF`. Console firmware at
``feature/per-frame-pdc`` (commit `b9f4d50`); SDK on the same branch.

**Stream health:**

| Metric | Value |
|---|---|
| PDC samples received | 24 005 (~ 40.0 Hz over 600 s) |
| Firmware-side drops (`pdc_dropped_delta` sum) | 0 |
| `frame_idx` monotonic gaps | 0 |
| `frame_idx` backsteps | 0 |
| `dark_slot` agreement with science pipeline (post-warmup) | 100 % |

The dark-slot off-by-one flagged in the spec is fixed (`dark_slot`
locally computed from `lsync_counter` in the LSYNC ISR — see
`Core/Src/trigger.c` in the FW change). Periodic darks now land on
`frame_idx ∈ {10, 601, 1201, …}`, matching the science pipeline's
predicted dark-frame schedule exactly. Warmup-window mismatches at
`frame_idx ∈ [1, discard_count]` are cosmetic: the FW marks them dark
because the laser physically fires in long-slot during warmup, but the
science pipeline discards those frames regardless.

### Pearson r — per-frame PDC vs pedestal-subtracted image mean

Image mean from `on_uncorrected_fn` `Sample.mean` (= `max(0, μ₁ − 64)`).
PDC interpolated onto each camera's mean timestamps. n ≈ 24 000 unless
noted.

| Side | min r | median r | max r |
|---|---|---|---|
| Left  (8 cams) | 0.864 | 0.892 | 0.899 |
| Right (8 cams) | 0.738 | 0.823 | 0.841 |

Full table in `2026-05-21_10min_corr.csv`. Plots:

- `2026-05-21_10min_timeseries.png` — top: PDC over 600 s. Bottom: all
  16 image-mean traces. The PDC has a clear ~30–60 s oscillation
  (drive-loop / TEC cycling) plus a slow downward drift across the
  scan; the image-mean traces visibly lock to the same oscillation.
- `2026-05-21_10min_scatter.png` — 4×4 grid of PDC-vs-mean scatter,
  one panel per (side, cam_id), with r and n annotated.

### How to reconcile this with the polarization-split mechanism

The strong PDC ↔ mean correlation observed here does **not**
contradict the investigation's central finding. There are two
mechanisms that move PDC and fiber-output power, and they operate on
different time scales:

1. **Total-output variation** (TA seed-current, temperature) moves PDC
   and fiber output **in the same direction.** This dominates fast
   variation within a scan — the ~30–60 s oscillation seen in the
   time-series plot is this mode. The 0.74–0.90 Pearson r we measure
   is essentially this single-mode coupling.

2. **Polarization-split variation** moves PDC and fiber output in
   **opposite directions.** From the PDC Investigation deck (slide 15)
   this manifests on a much slower time scale (~minutes to hours of TA
   thermal soak) and shows up as anti-correlated oscillations between
   the in-package PD and an external energy meter over a 1000 s window.

A 10-minute scan on a single bench unit doesn't span enough TA-thermal
drift to make mechanism (2) dominate, and we have no external energy
meter to separate the two components from PDC alone. So **the high r
here measures mechanism (1), not mechanism (1) ⊕ (2).** It does not
license amplitude scaling — under polarization drift the residual
between PDC and true fiber output is still arbitrary and a `pdc_ref /
pdc_frame` correction can still actively worsen the result.

What this measurement *does* establish: when polarization is stable
(or drift is small relative to total-output variation, as in
controlled bench conditions), PDC closely tracks the laser intensity
illuminating the camera. That is exactly the regime where PDC is
useful as a **laser-health diagnostic**: a sudden change in PDC almost
certainly reflects a real intensity change worth investigating, not
just a polarization-mode reshuffle.

### Where this leaves the per-frame PDC stream

The stream is operational and reliable. It is now safe to:

- Persist `pdc_mA`, `dark_slot`, `frame_idx`, and the drop counter to
  the telemetry CSV (already done on `feature/per-frame-pdc`) and to
  the SQLite session sink whenever that gets enabled.
- Surface live PDC in the engineering test app for laser-health
  surveillance (sudden drop, plateau, mid-scan steps) — see the
  bullets under "What per-frame PDC is still good for" above.
- Use the stream as the input signal for any **anomaly-detection**
  feature (e.g., flag scans where PDC drifts more than X mA from its
  warmup steady state). Anomaly detection ≠ correction; this is fine.

The stream is still **not** safe to feed into the BFI/BVI math. The
investigation's decision #3 stands.
