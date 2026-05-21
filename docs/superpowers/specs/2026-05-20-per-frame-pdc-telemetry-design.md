# Per-Frame PDC Telemetry

**Status:** Design approved; plan not yet written.
**Date:** 2026-05-20
**Repos:** openmotion-sdk (primary), openmotion-console-fw (firmware support)

## Summary

Capture one photodiode-current (PDC) value per camera frame at 40 Hz, tagged with the console MCU's frame index, and write it to the telemetry CSV alongside the existing slow telemetry. PDC currently sampled at ~1 Hz from a generic I2C passthrough; this change drives sampling from the safety FPGA's laser-pulse semantics so each value is the genuine peak measured during a single laser pulse and is mapped to a specific camera frame. Every frame — bright-slot and dark-slot alike — gets a fresh PDC measurement, because OpenMOTION's dark-frame scheme retimes the laser pulse rather than disabling it; a single per-row flag distinguishes the two.

The console firmware grows a small ring buffer of `(frame_idx, pdc_raw, flags)` tuples populated on every laser pulse, plus a drain opcode. The SDK's existing `ConsoleTelemetryPoller` runs at 10 Hz, drains that buffer each tick (~4 samples per call), and re-reads the slow telemetry once per second. The telemetry CSV becomes one row per drained PDC sample, with slow-telemetry columns carrying their last-known values forward.

## Decisions

- One PDC value per **camera frame** (40 Hz steady state). Exact alignment via a firmware-side ring buffer populated by the laser-pulse interrupt; the host does not have to time the I2C read itself.
- Cross-repo change: console firmware adds the buffer + drain opcode; SDK adds the drain client and per-frame CSV writer. No sensor-firmware or FPGA changes.
- PDC sampling logic is integrated into the existing `ConsoleTelemetryPoller`, not a new module. The poller's tick rate increases from 1 Hz to 10 Hz; slow telemetry re-reads on every 10th tick.
- Telemetry CSV becomes per-frame (always — no flag). Existing columns keep their names and positions; new columns are appended.
- Slow telemetry columns (TEC, PDU, safety, tcl, lsync) carry forward their last-known value between 1 Hz refreshes. A new `slow_age_ms` column makes the carry-forward visible to analysts.
- Backwards compatibility for SDK consumers: existing `ConsoleTelemetry` snapshot listeners keep working unchanged. New per-frame `PdcSample` listeners are opt-in.
- The 1 Hz `ConsoleTelemetry.pdc` field is derived from the most recent `PdcSample` (carry-forward), not from a separate I2C read. The dedicated I2C read for PDC in `_read_analog` is removed.

## Background — Safety FPGA semantics

The PDC register lives on a Lattice MachXO2 safety FPGA at I2C address `0x41` on console mux 1, channel 7. The relevant HDL is in `openmotion-safety-fpga/src/`.

- **Registers `0x1C` / `0x1D`** = `peak_power_value`, 16-bit little-endian. Scale: raw × 1.9 mA/LSB (matches the existing SDK constant in `ConsoleTelemetry.py:_PDC_MA_PER_LSB`).
- **Sampling is gated by `laser_pulse`** (`adc_control.v:226-275`). On the rising edge of `laser_pulse`, the FPGA waits `CONVERT_DELAY ≈ 3000` cycles, then samples the on-board ADC ~160 times and stores an exponentially-weighted average (`final_voltage_data <= (voltage_data + final_voltage_data) >> 1`).
- **Read does not clear or advance the value.** The register holds the last laser-pulse average until the next laser pulse produces a new one. Asserting `peak_power_read` (the FPGA's internal signal when `0x1C/0x1D` is being read) only freezes the value during the read so it cannot be torn.
- **Dark frames are not laser-off in this system.** OpenMOTION's dark-frame mechanism (`trigger.c:343-358`) does not disable the laser. Instead it switches `LASER_TIMER` to a "long-slot" period where the laser pulse fires *after* the camera's exposure window has closed. The safety FPGA still sees a real `laser_pulse` on every frame — bright or dark — and produces a fresh `peak_power_value` for each one. Every per-frame PDC sample is a live measurement of laser output; the dark/bright distinction only tells analysts whether that pulse was integrated by the camera.
- Companion registers (`peak_power_min/max` at `0x29-0x2C`, `peak_power_value_capture` at `0x2D-0x2E`) are not per-pulse and are out of scope.

**Timing requirement for the firmware read:** read `0x1C/0x1D` *after* the laser pulse falling edge plus the FPGA's sample window has completed (conservatively ~1 ms after laser fall, well within the FPGA's ~hundreds-of-µs settling), and *before* the next laser pulse rising edge (~25 ms later at 40 Hz). Reading too early returns the previous pulse's value tagged with the new frame index. Reading too late is safe as long as it's before the next pulse.

## Architecture

### Firmware: openmotion-console-fw

**New opcode in `Core/Inc/common.h`:**

```c
OW_CTRL_GET_PDC_BUFFER = 0x25,   /* next free slot after OW_CTRL_PDUMON = 0x24 */
```

**Sample tuple (7 bytes, packed LE):**

```c
typedef struct __attribute__((packed)) {
    uint32_t frame_idx;   /* value of lsync_counter at the I2C read */
    uint16_t pdc_raw;     /* raw 16-bit safety FPGA reg 0x1C/0x1D */
    uint8_t  flags;       /* bit 0 = long_slot (dark-frame timing) */
} pdc_sample_t;
```

**Ring buffer in `Core/Src/trigger.c` (or a new `pdc_buffer.c`):**

- 256 entries, drop-oldest on overflow, monotonic `dropped_count` counter.
- **Slot-tracking flag:** add a small `static volatile bool current_slot_is_long;` updated inside `FSYNC_PeriodElapsedCallback` at the same point the long/short ARR/CCR1 are loaded (`trigger.c:348-356`). This captures the slot type of the laser pulse that is about to fire and is read by the PDC sample handler.
- **Producer ISR:** hook `LASER_TIMER`'s period-elapsed (`TIM_UPDATE`) callback — which fires on the **laser-pulse falling edge** — set a `pdc_sample_pending` flag and stash the current `lsync_counter` plus `current_slot_is_long`. (The existing `LSYNC_DelayElapsedCallback` fires on the *rising* edge, too early to read peak_power_value; we add a new callback for TIM_UPDATE on LASER_TIMER, leaving the rising-edge callback unchanged.)
- **Main-loop task:** consumes the flag, waits ~1 ms for the FPGA's averaging window to settle, performs the I2C read of `0x41` reg `0x1C` for 2 bytes via the existing TCA9548 mux path, and pushes the tuple.
- **Consumer:** handler for `OW_CTRL_GET_PDC_BUFFER`. Payload-in: `uint8 max_samples` (clamp to 64). Payload-out: `uint16 dropped_count_delta` + `uint8 sample_count` + `sample_count × 7-byte tuples`. `dropped_count_delta` is the count of drops since the previous drain (zero on first drain after boot).
- All buffer mutations under a critical section (one short `__disable_irq()` window per push/pop).

**Why main-loop and not ISR for the I2C read:** the I2C transaction through the TCA9548 mux + safety FPGA chip takes ~1-2 ms at 100 kHz. Doing that inside the LASER_TIMER ISR would block the FSYNC ISR (which is higher-priority timing). The 1 ms post-pulse delay is naturally absorbed by main-loop scheduling latency, with the deadline of "before next laser pulse" comfortably met.

**Why `lsync_counter` as the frame index:** it is incremented in `LSYNC_DelayElapsedCallback` on every laser-pulse rising edge — bright or dark slot (`trigger.c:359-369`). It is already exposed via `OW_CTRL_GET_LSYNC` (the `tcm` field in `ConsoleTelemetry`) and resets to 1 on every `Trigger_Start`. Hosts can correlate against the existing `tcm` column without any new identifier. By the time the PDC sample is captured (LASER_TIMER period-elapsed = falling edge of the same pulse), `lsync_counter` already reflects the pulse that produced this PDC value.

### SDK: openmotion-sdk

**Changes confined to `omotion/ConsoleTelemetry.py` and `omotion/ScanWorkflow.py`.** No new module file.

**New dataclass in `ConsoleTelemetry.py`:**

```python
@dataclass
class PdcSample:
    frame_idx: int        # console MCU lsync_counter at FW I2C read time
    pdc_mA: float         # raw_u16 * 1.9
    long_slot: bool       # flags bit 0: True = laser pulse fired in the long-slot
                          # (dark-frame timing — pulse outside camera exposure window)
    host_recv_timestamp: float  # time.time() when SDK received the drain response
    dropped_delta: int    # firmware-reported drops since last drain (attached to first sample of a drain batch; 0 otherwise)
```

**Modified `ConsoleTelemetryPoller`:**

- `_POLL_INTERVAL_S` becomes `0.1` (10 Hz).
- New internal counter `_slow_tick_phase: int = 0`. Each tick:
  1. Call `OW_CTRL_GET_PDC_BUFFER` (`max=64`). Parse response into a list of `PdcSample`. Emit each via `_pdc_listeners`.
  2. If `_slow_tick_phase == 0`: re-read TEC, PDU, safety, tcl, lsync and build a `ConsoleTelemetry` snapshot. Emit via `_listeners`. Increment phase; wrap at 10.
- New API:
  - `add_pdc_listener(fn: Callable[[PdcSample], None]) -> None`
  - `remove_pdc_listener(fn) -> None`
  - `get_last_pdc_sample() -> Optional[PdcSample]`
- The 1 Hz snapshot's `pdc` field is set to `last_pdc_sample.pdc_mA` (or `0.0` if none yet). The dedicated `_read_analog` I2C call for PDC (reg `0x1C`) is removed; the `tcl` and `lsync` reads stay.
- The `read_ok` / `error` fields on the snapshot reflect only the slow re-read; drain failures are logged independently and do not flip `read_ok` on a snapshot that is otherwise healthy.
- Same single thread, same single lock as today. Listener lists are copied under the lock and invoked outside it, as in the current implementation.

**Lifecycle:** unchanged. `MOTIONInterface` starts/stops the poller on console USB connect/disconnect, same as today.

### CSV writer: `omotion/ScanWorkflow.py`

The telemetry CSV is now one row per `PdcSample` (~40 Hz, file size ~70-100 MB for a 12-hour scan at full populated columns).

**Implementation:**

- A new `add_pdc_listener` callback in `ScanWorkflow` writes one row per sample. Slow columns come from `console.telemetry.get_snapshot()`; if `None`, slow columns are empty strings.
- A `slow_age_ms` column captures `(host_recv_timestamp - snapshot.timestamp) * 1000`, rounded to int.
- The existing `_TELEMETRY_HEADERS` list at `ScanWorkflow.py:39-47` is extended; the existing `_snap_to_row` becomes `_pdc_row(pdc_sample, snap)`.
- File header order: existing columns first (unchanged), new columns appended.

**Final header order (existing columns first, unchanged; new columns appended):**

```
timestamp, tcm, tcl, pdc,
tec_v_raw, tec_set_raw, tec_curr_raw, tec_volt_raw, tec_good,
pdu_raw_0..15, pdu_volt_0..15,
safety_se, safety_so, safety_ok,
read_ok, error,
frame_idx, pdc_flags, pdc_dropped_delta, slow_age_ms
```

Column notes:

- `timestamp` — `PdcSample.host_recv_timestamp`. Same column name and semantic as today (host wall time when the row was generated); now ticks at 40 Hz instead of 1 Hz.
- `tcm` — equals `frame_idx` for rows generated by a PDC sample. Kept in its existing position so the column order is unchanged.
- `tcl` — last known value from the slow snapshot (carry-forward). Empty string before the first slow tick lands.
- `pdc` — `PdcSample.pdc_mA`, this row's per-frame value.
- TEC, PDU, safety — last known values from the slow snapshot (carry-forward). Empty string before the first slow tick lands.
- `read_ok`, `error` — last known from the slow snapshot.
- `frame_idx` — new column, same value as `tcm` (kept separate so the per-frame ID is unambiguously named for new consumers).
- `pdc_flags` — int, bit 0 = `long_slot` (1 = dark-frame timing, laser pulse outside camera exposure; 0 = bright-frame timing, laser pulse inside camera exposure).
- `pdc_dropped_delta` — firmware-reported drops since last drain. Non-zero only on the first row of a batch that follows a backlog.
- `slow_age_ms` — milliseconds since the last successful slow refresh.

### Data flow

```
Console MCU
  FSYNC ISR (40 Hz): set current_slot_is_long for the upcoming pulse
        │
  LASER_TIMER period-elapsed ISR (= laser-pulse falling edge, 40 Hz):
    └── set pdc_sample_pending; snapshot lsync_counter + current_slot_is_long
        │
Main-loop task
  └── if pending: wait ~1 ms → I2C read safety FPGA 0x41 reg 0x1C (2 B)
        │
        └── push {frame_idx, pdc_raw, flags} into ring buffer (256, drop-oldest)

Console MCU command handler
  └── on OW_CTRL_GET_PDC_BUFFER: drain up to max_samples, return
      [dropped_count_delta:u16][count:u8][tuples:count*7]

SDK ConsoleTelemetryPoller (10 Hz)
  ├── every tick: call drain → parse → emit PdcSample × N to pdc_listeners
  └── every 10th tick: read slow telemetry → emit ConsoleTelemetry to listeners

ScanWorkflow
  ├── pdc_listener: write one CSV row, slow cols from last snapshot
  └── (existing snapshot listener removed for CSV purposes; snapshot still drives UI/diagnostics)
```

## Error handling

| Condition | Behaviour |
|---|---|
| FW ring overflow | Drop-oldest; `dropped_count_delta` on next drain reports the count. Logged at INFO in SDK if non-zero. |
| FW I2C read fails (e.g., mux contention) | Skip pushing for that frame. FW logs and increments an internal `pdc_i2c_fail_count` (printf only — not exposed in this iteration). |
| SDK drain call fails | Caught in poller, logged at WARN. Snapshot's `read_ok` is **not** flipped; PDC stream just gaps until the next successful drain. |
| Console disconnect mid-poll | Existing `_read_all` exception path stops the loop cleanly — same logic applies to the new drain step. |
| Scan idle (`Trigger_Stop` called) | LASER_TIMER is disabled (`trigger.c:268`), so the LASER_TIMER period-elapsed ISR no longer fires and no PDC samples are produced. CSV row rate naturally drops to zero until the next scan starts. |
| Laser hardware-disabled while trigger running | This is not a documented operating mode in `trigger.c`. If it were to occur (e.g. safety-FPGA interlock cuts the diode while LASER_TIMER keeps running), the FPGA's `peak_power_value` would stop updating but the MCU would keep pushing samples tagged with stale-but-prior PDC values. The slow-cadence `safety_se` / `safety_so` columns would flag the interlock condition on the same CSV rows. |
| Slow re-read fails on a tick | `read_ok=0`, `error` populated on that snapshot. CSV rows continue to carry the previous successful snapshot's values; `slow_age_ms` grows. |

## Backward compatibility

- **Apps (`bloodflow-app`, `test-app`):** consume `ConsoleTelemetry` snapshots via `add_listener`. Unchanged signature, unchanged 1 Hz cadence. They opt into the new `add_pdc_listener` later if/when they want per-frame PDC.
- **Telemetry CSV consumers:** anything that reads by column name (the existing `scripts/plot_telemetry.py` and the `stream-db/` importer) keeps working — every existing column is still present. Consumers that assumed ~1 Hz row cadence will see 40× more rows; if any such consumer assumes a fixed timestamp grid, it will need updating, but none currently does.
- **Existing 1 Hz `pdc` column:** still present, now sourced from the most recent per-frame sample (carry-forward). Values seen by older consumers are at least as fresh as before — usually fresher.

## Testing

### Firmware (console-fw)

- Unit-style: simulate LASER_TIMER ISR ticks at 40 Hz against a mocked I2C read, verify ring buffer contents and FIFO ordering.
- Bench: capture a UART trace of `OW_CTRL_GET_PDC_BUFFER` responses with a logic analyser tee'd on I2C; confirm `frame_idx` matches the `lsync_counter` returned by `OW_CTRL_GET_LSYNC` on the same tick.
- Stress: deliberately delay host drain to ~30 s; verify `dropped_count_delta` matches `(elapsed_s × 40) − 256` after the delay, and that subsequent normal operation resumes cleanly.

### SDK (openmotion-sdk)

- Unit: parse `OW_CTRL_GET_PDC_BUFFER` response — 0, 1, 64 samples; oversized payload; truncated payload; explicit `dropped_count_delta`.
- Unit: drive the poller from a mocked console; verify 10 Hz drain cadence and 1 Hz slow re-read cadence, that `add_pdc_listener` fires once per parsed sample, and that `add_listener` fires once per second.
- Unit: confirm `_read_analog` no longer issues the dedicated PDC I2C call (one fewer `read_i2c_packet` invocation per snapshot).
- Hardware integration: 60-second scan, verify telemetry CSV row count ≈ 2400, monotonic `frame_idx`, slow columns populated from row 1 (or empty on first row when slow tick has not yet landed), `pdc_dropped_delta == 0` in steady state, `slow_age_ms` < 1200 for every row after row 40.

## Out of scope

- Per-frame TCL (laser trigger counter from the external chip on ch 4). Stays at 1 Hz carry-forward.
- Exposing `peak_power_min/max` or `peak_power_value_capture` from the safety FPGA. Not asked for; can be added in a follow-up via the same drain opcode.
- Sensor-side per-frame telemetry (IMU, camera temps). Independent stream, untouched.
- Live UI display of per-frame PDC. The plumbing (`add_pdc_listener`) is added; whether/how the app uses it is a separate change.

## Open questions (to resolve in the plan)

- Whether to delay the firmware I2C read inside the main-loop task with a hardware timer or by polling SysTick. Either works; the plan picks one.
- Exact `OW_CTRL_GET_PDC_BUFFER` opcode value (`0x25` assumed free; verify against the latest `common.h`).
- Where to maintain `current_slot_is_long` exactly: the natural spot is alongside the `__HAL_TIM_SET_AUTORELOAD` calls in `FSYNC_PeriodElapsedCallback` (`trigger.c:348-356`). Need to confirm the slot decided at FSYNC for frame N is the one that actually fires before LASER_TIMER's period-elapsed for frame N — given preload (`trigger.c:223-224`) and the FSYNC → LASER_TIMER trigger relationship. If there's an off-by-one between the slot decision and the pulse it gates, the firmware reading needs to lag by one frame.
