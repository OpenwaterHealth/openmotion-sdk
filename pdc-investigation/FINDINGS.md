# PDC investigation — findings

## Setup

- Source: 5 telemetry CSVs from `C:\Users\ethan\Projects\scan_data\` collected 2026-05-20 (subject `owEENEJ6`/`owDYFXQ6`/`owKZOSBT`)
- Two long scans drive most conclusions:
  - `20260520_163109_owEENEJ6_telemetry.csv` — 3603 s (~1 h), mask 0x66, laser cold-start
  - `20260520_191204_owEENEJ6_telemetry.csv` — 9108 s (~2.5 h), mask 0xF0, started 31 s after a prior short scan
- `pdc` = photodiode current in mA (raw ADC × 1.9 mA/LSB; `omotion/ConsoleTelemetry.py`)
- Polling cadence: ~1 Hz (median dt 1.003 s, jitter 0.77–2.21 s)
- Scripts in this folder:
  - `plot_pdc.py` — single- or multi-CSV PDC overlay with optional rolling mean
  - `plot_pdc_tec.py` — PDC + TEC channels + tcl rate on a shared time axis
  - `plot_pdc_fft.py` — single-sided FFT of steady-state PDC + PDC↔TEC correlations

## Summary stats (2.5h scan)

| metric | value |
| --- | --- |
| n | 8906 |
| mean | 2270.45 mA |
| std | 21.15 mA |
| min | 2209.70 mA |
| max | 2390.20 mA |
| range | 180.5 mA (~8% of mean) |

## Original hypothesis 1 (overshoot = TEC pulling thermistor to setpoint, ends when `tec_good` first asserts): **FALSIFIED**

`tec_good == 1` from the very first sample of every scan, and `|tec_v_raw - tec_set_raw|` stays under ±3 mV the entire time (TMPGD threshold is ±100 mV). The TEC PID loop is already locked at t=0. So whatever drives the PDC overshoot, it is **not** the TEC catching up to setpoint.

## Revised hypothesis (well supported by the data): **laser self-heating, TEC compensates**

The TEC loop pins the thermistor voltage rock-solid, but it has to **work harder over time** to do so — meaning the heat load on the cold side is rising. The most plausible source is the laser diode's own dissipation soaking through the package, and as the diode's internal junction temperature drifts (even while the thermistor reads constant), its electro-optical efficiency drops. PDC, which monitors laser output, falls in lockstep.

### Evidence

**Correlations (PDC vs each TEC channel, Pearson r over the full scan):**

| channel | 1h scan (163109) | 2.5h scan (191204) | what it represents |
| --- | ---: | ---: | --- |
| `tec_set_raw` | −0.05 | −0.05 | setpoint — should not move (✓ doesn't) |
| `tec_v_raw` | weak | −0.15 | measured thermistor voltage — held at setpoint by PID |
| `tec_curr_raw` | **−0.88** | **−0.72** | TEC current monitor — rises as cold-side load increases |
| `tec_volt_raw` | **−0.88** | **−0.73** | TEC voltage monitor — same |

**Endpoint shifts:**

| scan | PDC start → end | drop | tec_curr_raw start → end |
| --- | --- | ---: | --- |
| 163109 (1 h, cold start) | 2340 → 2073 mA | **−267 mA (−11.4 %)** | 1.506 → 1.543 V |
| 191204 (2.5 h, warm start) | 2330 → 2272 mA | −57 mA (−2.5 %) | 1.498 → 1.526 V |

Cold-start scan (163109) shows a large monotonic PDC drop with TEC drive rising in step. Warm-start scan (191204) — which began 31 s after the laser had been running — drops far less because the assembly was already near thermal equilibrium.

**Per-window variance also collapses:** across the first 1000 s of the 2.5h scan, sample-window std drops 21.6 → 14.6 → 13.6 → 8.8 → 6.1 mA. PDC noise itself shrinks as the system settles — consistent with a thermal mode dying out, not a fixed measurement noise floor.

### Additional supporting signal: `pdu_volt_9`

The PDU monitor channel 9 also moves with PDC (r = −0.68 over the 1 h scan), rising from 62 mV → 76 mV. None of the other 15 PDU voltage channels show meaningful drift or PDC correlation. Whatever this channel physically monitors (laser-package thermistor? laser drive current?), it is responding to the same underlying thermal trend and would be worth identifying from the PDU schematic.

## Multi-scan comparison

`multi_scan_pdc.png` overlays PDC from 4 scans on a common time axis. Notes:

- **First scan after device idle has a one-sample spike to 0 mA** at t≈0 (visible only in `20260520_121817_owDYFXQ6`). Likely the laser was still off when the first telemetry tick fired. Not a bug in PDC computation per se — but if the analysis pipeline assumes PDC > 0, that first sample needs filtering or guarding.
- All scans reach the same ~2350–2390 mA peak within the first ~200 s.
- Settling time is consistently ~600–800 s.
- **Steady-state PDC differs scan-to-scan** (1 h scan settled near 2080 mA, 2.5 h scan steady at 2270 mA). This is the dominant scan-to-scan variability and is explained by the thermal-history-dependent equilibrium — what matters is how long the laser had been off prior, not the scan duration.

## Spectral analysis (steady-state region of 2.5 h scan)

Detrended PDC for t ≥ 1000 s, Hann-windowed, single-sided FFT on a uniform 1 Hz grid (Nyquist = 0.5 Hz).

- No dominant peaks. Top peaks are all at very low frequencies — periods 200–450 s, amplitudes 1.4–2.2 mA — and look like residual slow drift bleeding through the windowing, not a real periodicity.
- No fan PWM beat, no mains-hum alias, no TEC PID limit cycle.
- Implies the in-band PDC noise is broadband / thermal-fluctuation-class, not a discrete tone. No "fix this oscillator" lever here.

## What we now believe vs what's still uncertain

**Believed (well supported):**

- TEC loop is healthy and locked from t=0 — no PID issue.
- The startup PDC transient and the slow long-tail drift are both manifestations of the laser/package thermal soak; same physical phenomenon, different timescales.
- Scan-to-scan variation in steady-state PDC reflects how long the laser had been off, not anything wrong with the device.

**Uncertain / could be either:**

- Whether the residual ~21 mA std at steady state is real laser ripple, ADC noise (~1 LSB ≈ 1.9 mA, so this is ~10 LSB), or photodiode/transimpedance noise. Need a known-stable bench source to separate.
- What `pdu_volt_9` physically measures.
- Whether the 200–450 s "peaks" in the FFT have a thermal origin (room HVAC cycle?) or are just spectral leakage from non-stationary drift.

**Not yet investigated:**

- Long-term scan-over-scan trend (does the steady-state value drift over weeks?).
- Whether PDC drift correlates with corrected-stream output (BFI/BVI) — if PDC drops 11 % over an hour, does that change the science?

## Caveat

Two long scans on one device on one day. Patterns are internally consistent and the cold-vs-warm-start comparison is strong, but reproducing on a second device would harden the conclusion before this becomes design-of-experiment input.
