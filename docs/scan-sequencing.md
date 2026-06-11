# Scan Sequencing Reference

Full system sequence from MCU power-on through scan teardown, scoped to a single camera sensor. Each step is tagged with where it occurs: **[MCU]**, **[SDK]**, or **[APP]**.

---

## Part 1 — As the Code Currently Stands

### Phase 0 — MCU Boot

| # | Location | Action | Detail |
|---|----------|--------|--------|
| 0.1 | **[MCU]** | HAL and peripheral init | Clocks, GPIOs, USART, SPI, I2C, DMA, timers, USB all initialised by CubeMX-generated `MX_*_Init()` calls. |
| 0.2 | **[MCU]** | FSIN line forced low | `X02C1B_FSIN_EXT_disable()` and `GPIO_SetOutput(FSIN_EN_Pin, RESET)` run before the main loop. The fsin output is off at boot. |
| 0.3 | **[MCU]** | Enter main loop | Loop body calls `comms_host_check_received()` then `check_streaming()` on every iteration. |

---

### Phase 1 — FPGA Flash and Sensor Register Configuration (`start_configure_camera_sensors`)

This phase is a one-time prerequisite. It must complete before any scan.

| # | Location | Action | Detail |
|---|----------|--------|--------|
| 1.1 | **[APP]** | Call `start_configure_camera_sensors(ConfigureRequest)` | Launches a background thread. |
| 1.2 | **[SDK]** | `enable_camera_power` → `OW_CAMERA_POWER_ON` | Host sends `OW_CAMERA_POWER_ON` with camera bitmask over USB-UART. |
| 1.3 | **[MCU]** | `OW_CAMERA_POWER_ON` handler | Reads and saves current fsin_ext state. If fsin_ext was enabled, calls `X02C1B_FSIN_EXT_disable()` and waits 10 ms. Then asserts camera power-rail GPIO via `enable_camera_power(i)`. Directly sets `cam->isPresent = true` (I2C presence scan is bypassed). Waits 10 ms, then re-enables fsin_ext if it was previously on. |
| 1.4 | **[SDK]** | `get_camera_status` → `OW_CAMERA_STATUS` | Reads status byte. SDK verifies bit 0 (READY) is set before proceeding. If bits 1 (programmed) and 2 (configured) are both already set, steps 1.5–1.9 are skipped for that camera unless `ConfigureRequest.force_program=True`. |
| 1.5 | **[SDK]** | `program_fpga` → `OW_FPGA_PROG_SRAM reserved=1` | 60-second timeout. |
| 1.6 | **[MCU]** | `program_fpga(cam_id, force_update=false)` | Checks `camera_request_is_valid` (range + `isPresent`). If `cam->isProgrammed` and `force_update=false`, returns early. Otherwise: selects I2C mux channel via `TCA9548A_SelectChannel`, calls `fpga_configure()` which loads the bitstream from internal ROM into the iCE40 FPGA SRAM over I2C (`device_address = 0x40`). On success sets `cam->isProgrammed = true`. If camera uses USART, **immediately toggles `USART_CR1_UE` off → on** as a post-flash reset. |
| 1.7 | **[SDK]** | `sleep(100 ms)` | Host-side settle before register config. |
| 1.8 | **[SDK]** | `camera_configure_registers` → `OW_CAMERA_SET_CONFIG` | 60-second timeout. |
| 1.9 | **[MCU]** | `configure_camera_sensor(cam_id)` | Checks `camera_request_is_valid`. Returns early if `cam->isConfigured && cam->isPowered`. Fails if not powered. Selects I2C mux channel, calls `X02C1B_configure_sensor()` to write default register set to the image sensor over I2C. Sets `cam->isConfigured = true`. |
| 1.10 | **[APP]** | `on_complete_fn(ConfigureResult)` | Workflow complete. App may now start scans. |

---

### Phase 2 — Scan Startup (`start_scan`)

| # | Location | Action | Detail |
|---|----------|--------|--------|
| 2.1 | **[APP]** | Call `start_scan(ScanRequest)` | Launches `_worker()` background thread. Telemetry CSV is opened and listener registered on `ConsoleTelemetryPoller` if `write_telemetry_csv=True`. |
| 2.2 | **[SDK]** | `enable_camera_fsin_ext` → `OW_CAMERA_FSIN_EXTERNAL reserved=1` | **fsin line is armed here, before cameras are enabled.** (See Issue 1.) |
| 2.3 | **[MCU]** | `OW_CAMERA_FSIN_EXTERNAL` handler | Calls `X02C1B_FSIN_EXT_enable()`. The fsin output GPIO is now active. Any pulse from the console will now trigger EXTI. |
| 2.4 | **[SDK]** | `sleep(100 ms)` | Host-side delay. |
| 2.5 | **[SDK]** | `reset_camera_uart` → `OW_CAMERA_RESET_UART` | Force-resets each camera's USART/SPI peripheral before streaming is enabled. Failure is non-fatal (warning only). |
| 2.6 | **[MCU]** | `reset_camera_usart(cam_id)` per bit in mask | Calls `HAL_USART_Abort()` / `HAL_SPI_Abort()` (soft abort of any in-flight DMA), then toggles `USART_CR1_UE` / `SPI_CR1_SPE` off → on at the hardware register level, then forces `pUart->State = HAL_USART_STATE_READY` / `pSpi->State = HAL_SPI_STATE_READY`. Guarantees peripheral and HAL state machine are both READY regardless of what the previous scan left behind. |
| 2.7 | **[SDK]** | `enable_camera` → `OW_CAMERA_STREAM reserved=1` | Requests streaming enable for the camera bitmask. |
| 2.8 | **[MCU]** | `enable_camera_stream(cam_id)` | Full sequence: |
| | | → 1. `camera_request_is_valid()` | Range check + `isPresent` gate. |
| | | → 2. `event_bits_enabled` guard | If bit already set, logs "already enabled" and returns true (no-op). |
| | | → 3. Auto-configure if needed | If `!cam->isConfigured`: calls `configure_camera_sensor(cam_id)` inline. |
| | | → 4. `delay_us(10)` | Short settle. |
| | | → 5. `TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target)` | Select I2C mux channel. Returns false on failure. |
| | | → 6. `reset_camera_usart(cam_id)` | **Peripheral reset inside enable path** — HAL abort + hardware toggle + HAL state force-READY. Guards against cases where teardown left the peripheral stuck or where a delayed DMA-complete interrupt re-armed it. |
| | | → 7. Atomically clears `event_bits &= ~(1u << cam_id)` | Discards any stale event bit from the previous scan's DMA completion (e.g., a callback that fired after abort). Prevents `send_histogram_data()` from acting on garbage data on the first frame. |
| | | → 8. `start_data_reception(cam_id)` | Arms `HAL_USART_Receive_DMA()` / `HAL_SPI_Receive_DMA()`. DMA is now waiting for data. **Must happen before `stream_on`** — arming after the sensor starts sending causes immediate SPI overrun on fast cameras. |
| | | → 9. `X02C1B_stream_on(cam)` | Image sensor begins outputting histogram data over USART/SPI. DMA is already armed so the first bytes are caught. |
| | | → 10. `event_bits_enabled \|= (1 << cam_id)` | Mark this camera as enabled. Only reached if `start_data_reception()` returned true. |
| | | → 11. `cam->streaming_enabled = true` | |
| | | → 12. `HAL_GPIO_WritePin(gpio1_port, gpio1_pin, GPIO_PIN_SET)` | GPIO1 HIGH — required for operation (note: schematic has FPGA GPIO0/GPIO1 swapped). |
| | | → 13. `camera_failure_counters[cam_id] = 0` | Reset failure counter. |
| 2.9 | **[SDK]** | Host streaming setup | Per-side USART histogram queue and writer thread started. Science pipeline created if calibration coefficients are present. |
| 2.10 | **[SDK]** | `start_trigger` (console module) | Console begins generating trigger pulses on the fsin line. |
| 2.11 | **[MCU]** | First FSIN pulse arrives on GPIO_PIN_13 → EXTI15_10_IRQHandler → `HAL_GPIO_EXTI_Callback` | Because `streaming_active == false`, `send_data()` sets `streaming_start_time`, sets `streaming_active = true`, sets `streaming_first_frame = true`. Increments `pulse_count`. |

---

### Phase 3 — Per-Frame Loop (repeats for every trigger pulse)

| # | Location | Action | Detail |
|---|----------|--------|--------|
| 3.1 | **[MCU]** | FSIN pulse → EXTI interrupt → `send_data()` | Debounce check: if `(now - most_recent_frame_time) < 15 ms`, the frame is silently dropped. Otherwise updates `most_recent_frame_time`. |
| 3.2 | **[MCU]** | `send_data()` → `send_histogram_data()` | Atomically snapshots and clears `event_bits` (`__disable_irq()` / `__enable_irq()`). Builds a single USB packet: SOF + 4-byte total size + 4-byte timestamp + per-camera payload (SOH + cam_id + histogram + 4-byte temperature float + EOH) + CRC16 + EOF. |
| 3.3 | **[MCU]** | DMA re-arm inside `send_histogram_data()` | Immediately after copying each camera's histogram buffer (only cameras with `ready_bits` set), calls `start_data_reception(cam_id)` to re-arm `HAL_USART_Receive_DMA`. This happens from within the EXTI interrupt context. |
| 3.4 | **[MCU]** | `USBD_HISTO_SendData(...)` | Queues the histogram packet for USB transmission to the host. Increments `frame_id` and `total_frames_sent`. |
| 3.5 | **[MCU]** | USART DMA RX complete → `HAL_USART_RxCpltCallback` | When the next frame's USART data fills `pRecieveHistoBuffer`, the DMA transfer completes and sets the appropriate bit in `event_bits` atomically via `set_event_bit_atomic(BIT_n)`. |
| 3.6 | **[SDK]** | USB HID read → histogram queue | Host receives USB packet, pushes raw bytes into the per-side queue. |
| 3.7 | **[SDK]** | `stream_queue_to_csv_file` / `parse_stream_to_csv` | Writer thread drains queue, parses framing, writes rows to raw CSV (or to `_NullCsvWriter` if `write_raw_csv=False`). Calls `on_row_fn` to feed science pipeline. |
| 3.8 | **[MCU]** | Main loop: `check_streaming()` | Every iteration: if `streaming_active` and `(now - most_recent_frame_time) > STREAMING_TIMEOUT_MS` (150 ms), calls `send_data()` once more (flushes final partial frame), resets `streaming_active = false`, resets frame/failure counters. |

---

### Phase 4 — Scan Teardown

| # | Location | Action | Detail |
|---|----------|--------|--------|
| 4.1 | **[APP]** | Duration elapsed (or cancel) | `_worker()` exits wait loop. |
| 4.2 | **[SDK]** | `stop_trigger` (console module) | Console stops generating FSIN pulses. Always attempted even on error paths. |
| 4.3 | **[MCU]** | FSIN pulses cease | No more EXTI callbacks. `most_recent_frame_time` stops updating. |
| 4.4 | **[MCU]** | `check_streaming()` timeout fires | After `STREAMING_TIMEOUT_MS` (150 ms) with no new FSIN pulse, calls `send_data()` for a final flush, resets `streaming_active = false`, resets `total_frames_sent`, `total_frames_failed`, `pulse_count`. Note: at this moment DMA is re-armed by `send_histogram_data()` and left in BUSY_RX waiting for data that will never arrive. |
| 4.5 | **[SDK]** | `disable_camera` → `OW_CAMERA_STREAM reserved=0` | Requests streaming disable for the camera bitmask. |
| 4.6 | **[MCU]** | `disable_camera_stream(cam_id)` | Selects I2C mux channel. Calls `X02C1B_stream_off(cam)` → image sensor stops outputting pixel data. Calls `abort_data_reception(cam_id)`. On failure (abort did not cleanly complete): returns false without clearing `event_bits_enabled`. |
| 4.7 | **[MCU]** | `abort_data_reception(cam_id)` | Calls `HAL_USART_Abort(pUart)` / `HAL_SPI_Abort(pSpi)`. Waits 10 ms. Re-checks busy state — if still BUSY_RX, prints "USART/SPI still busy aborting on camera N" and returns false. On success: returns true, caller (`disable_camera_stream`) then clears `event_bits_enabled` bit, sets `cam->streaming_enabled = false`, sets GPIO1 LOW. |
| 4.8 | **[SDK]** | Stop host histogram queues and writer threads | Sets stop events; joins writer threads with 5 s timeout. |
| 4.9 | **[SDK]** | `science_pipeline.stop()` | Shuts down science pipeline if created. |
| 4.10 | **[SDK]** | Drain telemetry CSV listener | Sets `_telem_stop`, acquires `_telem_lock`, removes listener, closes file handle. |
| 4.11 | **[SDK]** | Write merged corrected CSV | Flushes `corrected_by_frame` to disk sorted by `absolute_frame_id`. Skipped if `write_corrected_csv=False`. |
| 4.12 | **[APP]** | `on_complete_fn(ScanResult)` | Scan complete. |

---

## Part 2 — Correctness Analysis

The following issues are present in the code. Issues that have been fixed are marked accordingly.

---

### Issue 1 — `enable_camera_fsin_ext` is called before cameras are enabled or DMA is armed (CRITICAL)

**Where:** `ScanWorkflow._worker()`, steps 2.2–2.3 vs. 2.7–2.8.

**What happens:** The fsin line is armed (`X02C1B_FSIN_EXT_enable()`) before `enable_camera` is called. If the console is already producing any pulse-like glitch, or if a stale pulse arrives from a previous scan, the MCU receives a FSIN edge while `event_bits_enabled == 0x00`. `send_data()` fires, `streaming_active` becomes `true`, but there is no DMA transfer in progress so `event_bits` is zero and `send_histogram_data()` logs "No cameras have data to send". More critically, when DMA is finally armed, `streaming_active` is already `true` — the MCU is in an inconsistent state before the scan properly begins.

**Correct fix:** Arm fsin_ext only after all cameras have had `start_data_reception()` called, immediately before `start_trigger`.

---

### Issue 2 — No `disable_camera_fsin_ext` at scan startup (CRITICAL)

**Where:** `ScanWorkflow._worker()`, beginning of scan.

**What happens:** If a previous scan ended without disabling fsin_ext (e.g., due to an exception in teardown), the line may already be armed when the next scan starts. Combined with Issue 1, fsin pulses can arrive at any point during the new startup sequence.

**Correct fix:** Unconditionally call `disable_camera_fsin_ext` as the very first step of scan startup, before touching cameras or streaming.

---

### Issue 3 — `enable_camera_fpga` / `disable_camera_fpga` were removed intentionally (RESOLVED)

**Where:** `Sensor.py`, `ScanWorkflow._worker()`.

**Background:** `OW_FPGA_ON` asserts CRESET_B HIGH (releases iCE40 from reset); `OW_FPGA_OFF` asserts CRESET_B LOW. These were initially part of the scan workflow, but were removed after it was discovered that the CRESET_B GPIO transition was acting as a spurious edge on the fsin line when fsin_ext was already armed. This caused the MCU to process a phantom first frame at the moment of FPGA release.

**Resolution:** `enable_camera_fpga()` and `disable_camera_fpga()` have been removed from `Sensor.py`. The FPGA is left in whatever state the configure phase (`program_fpga`) left it. `OW_FPGA_ON` and `OW_FPGA_OFF` are retained in `config.py` for wire-protocol documentation but are not callable from the SDK.

---

### Issue 4 — USART/SPI peripheral left BUSY_RX between scans causes first-frame failure (FIXED)

**Where:** `disable_camera_stream()` / `abort_data_reception()` at teardown; `enable_camera_stream()` / `start_data_reception()` at next scan startup.

**Root cause:** At the end of every scan, `send_histogram_data()` re-arms DMA for each camera (so it is ready for the "next" frame that never comes). The sensor stops sending because FSIN stops, but the DMA transfer is still open and waiting for `USART_PACKET_LENGTH` bytes. When teardown calls `abort_data_reception()`, `HAL_USART_Abort()` / `HAL_SPI_Abort()` runs. There is a known STM32 HAL race: a pending DMA-complete interrupt in the NVIC can fire after the abort returns, placing the peripheral back in BUSY_RX or leaving a stale bit in `event_bits` before the 10 ms re-check completes. When that happens `abort_data_reception()` returns false, `disable_camera_stream()` returns false without clearing `event_bits_enabled`, and the second scan's `start_data_reception()` sees BUSY_RX → prints "USART busy" / "SPI busy" → returns false. The camera self-heals after one frame because the old DMA catches the first new packet, but the first frame is lost and the error message is printed.

**Fix applied (firmware, `camera_manager.c`):**

1. `reset_camera_usart()` is extended to handle both USART and SPI cameras, and now performs three-step cleanup: `HAL_USART_Abort()` / `HAL_SPI_Abort()` (soft abort), hardware peripheral disable/re-enable via `__HAL_USART_DISABLE/ENABLE` / `__HAL_SPI_DISABLE/ENABLE` (clears any stuck state in silicon), and explicit `->State = HAL_*_STATE_READY` (re-syncs the HAL state machine regardless of interrupt races).

2. `enable_camera_stream()` now calls `reset_camera_usart(cam_id)` immediately before `start_data_reception()` (after `X02C1B_stream_on` succeeds). This guarantees the peripheral is READY and the HAL state machine agrees, regardless of teardown outcome. It also atomically clears `event_bits &= ~(1u << cam_id)` to discard any stale callback bit from the previous scan.

**Fix applied (SDK, `ScanWorkflow.py`):** `reset_camera_uart` is called for each active camera mask before `enable_camera` in the scan startup sequence. This provides defense-in-depth for cases where older firmware is running, and emits a warning (non-fatal) on failure.

---

### Issue 5 — `send_data()` called from both EXTI interrupt and main loop without mutual exclusion (MEDIUM)

**Where:** `HAL_GPIO_EXTI_Callback` (ISR context) and `check_streaming()` (main loop context).

**What happens:** `send_data()` modifies `streaming_active`, `total_frames_sent`, `total_frames_failed`, and `pulse_count` without disabling interrupts. If a FSIN pulse arrives at the exact moment `check_streaming()` is calling `send_data()` for the timeout flush, both contexts run `send_data()` concurrently. The `event_bits` snapshot is protected by `__disable_irq()`, but the statistics variables and `streaming_active` are not.

**Correct fix:** Protect `streaming_active` and the counter updates in `send_data()` with `__disable_irq()` / `__enable_irq()`, or better, set a `volatile` flag in the EXTI callback and process in the main loop only.

---

### Issue 6 — DMA re-arm (`start_data_reception`) runs in EXTI interrupt context (MEDIUM)

**Where:** `send_histogram_data()`, called from `send_data()`, called from `HAL_GPIO_EXTI_Callback`.

**What happens:** `start_data_reception()` calls `camera_request_is_valid()`, reads `cam_array`, checks USART state, and calls `HAL_USART_Receive_DMA()` — all while inside the EXTI interrupt handler. Running complex HAL calls inside a GPIO EXTI ISR creates stack depth and re-entrancy risk, and holds off lower-priority interrupts for a non-trivial duration.

**Correct fix:** Set a "needs re-arm" flag in the ISR and perform `start_data_reception()` in the main loop on the next iteration.

---

### Issue 7 — `streaming_active` on MCU is never explicitly reset from the host (MEDIUM)

**Where:** `check_streaming()` / `send_data()`.

**What happens:** The MCU has no command to force-reset the streaming state machine. `streaming_active` is only cleared by the `STREAMING_TIMEOUT_MS` watchdog in `check_streaming()`. If the SDK starts a second scan within 150 ms of the last FSIN pulse of the first scan, `streaming_active` is still `true` on the MCU and the elapsed-time counter will be wrong for the new scan.

**Correct fix:** Either enforce a host-side delay of at least `STREAMING_TIMEOUT_MS` (150 ms) between scans, or add a firmware command to explicitly reset the streaming state.

---

### Issue 8 — `OW_CAMERA_POWER_ON` silently manages fsin_ext internally (LOW)

**Where:** `if_commands.c`, `OW_CAMERA_POWER_ON` handler.

**What happens:** The power-on handler reads the fsin_ext state, disables it, changes power, then re-enables it. This creates a hidden coupling: the MCU autonomously controls the fsin line without the SDK knowing. If the I2C read or timing fails, the fsin line may be left in an unexpected state.

**Correct fix:** Move fsin_ext management to the host. The SDK should explicitly call `disable_camera_fsin_ext` before `power_on_camera`. The firmware handler should just power the camera.

---

### Issue 9 — USART reset is duplicated in two places with different semantics (LOW)

**Where:** `program_fpga()` (end of function) and `reset_camera_usart()` (standalone).

**What happens:** `program_fpga()` contains an inline `USART_CR1_UE` toggle after bitstream load. `reset_camera_usart()` does the same (and now more) in response to `OW_CAMERA_RESET_UART`. Having the reset in both places means the intent is split across firmware-internal and host-controlled paths.

**Correct fix:** Remove the inline reset from `program_fpga` and rely solely on the explicit `OW_CAMERA_RESET_UART` call from the host at the correct point in the scan startup sequence.

---

### Issue 10 — `enable_camera_stream` auto-configures sensor registers if not configured (LOW)

**Where:** `enable_camera_stream()`, lines checking `cam->isConfigured`.

**What happens:** If the sensor was never explicitly configured, `enable_camera_stream` quietly calls `configure_camera_sensor()` inline. This creates a hidden dependency that makes the system appear to work without Phase 1 having run.

**Correct fix:** Remove the implicit configure from `enable_camera_stream`. If the sensor is not configured, fail explicitly and require the caller to run the configure phase first.

---

## Recommended Corrected Startup Sequence

Based on the above analysis (Issues 1, 2, and 4 addressed, enable/disable_camera_fpga removed):

| # | Location | Action |
|---|----------|--------|
| 1 | **[SDK]** | `disable_camera_fsin_ext` — unconditional, best-effort (guards against leftover state) |
| 2 | **[SDK]** | `reset_camera_uart` — SDK-level peripheral reset before streaming |
| 3 | **[SDK]** | `enable_camera` — arms sensor streaming + DMA receiver (internally also calls `reset_camera_usart` + clears stale `event_bits`) |
| 4 | **[SDK]** | Set up host streaming queues and writer threads |
| 5 | **[SDK]** | `enable_camera_fsin_ext` — arm fsin only after DMA is ready |
| 6 | **[SDK]** | `start_trigger` — console begins generating pulses |

And the correct teardown:

| # | Location | Action |
|---|----------|--------|
| 1 | **[SDK]** | `stop_trigger` |
| 2 | **[SDK]** | `disable_camera` — stops sensor streaming + aborts DMA |
| 3 | **[SDK]** | Stop host queues, writer threads, pipeline, CSVs |

---

## Firmware Data Path Reference

### FSIN reception (external)
`GPIO_PIN_13 pulse` → `EXTI15_10_IRQHandler` → `HAL_GPIO_EXTI_Callback(GPIO_PIN_13)` → `send_data()`

### FSIN reception (internal, TIM4)
`TIM4 PWM period` → `HAL_TIM_PWM_PulseFinishedCallback` → `send_data()`

### Per-frame data path
`send_data()` → debounce check (< 15 ms → drop) → `send_histogram_data()` → snapshot `event_bits` → pack USB frame → `start_data_reception()` per ready camera → `USBD_HISTO_SendData()`

### DMA completion (per-camera re-arm trigger)
`HAL_USART_RxCpltCallback` → `set_event_bit_atomic(BIT_n)` (sets `event_bits`) — picked up on next FSIN pulse in `send_histogram_data()`

### Streaming timeout (`check_streaming`, main loop)
If `streaming_active && (now - most_recent_frame_time) > STREAMING_TIMEOUT_MS` (150 ms): final `send_data()` flush → reset `streaming_active`, counters

---

## `ScanRequest` CSV Feature Flags

| Flag | Default | Effect when `False` |
|------|---------|---------------------|
| `write_raw_csv` | `True` | `_NullCsvWriter` used — science pipeline still receives data; no raw histogram CSV written. |
| `write_corrected_csv` | `True` | Merged corrected CSV not written; `corrected_path = ""`. |
| `write_telemetry_csv` | `True` | Telemetry CSV not opened; no `ConsoleTelemetryPoller` listener registered. |

---

## Firmware Opcodes Reference

| Opcode | Value | Description |
|--------|-------|-------------|
| `OW_CAMERA_POWER_ON` | — | Assert camera power rail GPIO. Internally saves/disables/restores fsin_ext. |
| `OW_CAMERA_POWER_OFF` | — | De-assert camera power rail GPIO. |
| `OW_CAMERA_STREAM reserved=1` | — | `enable_camera_stream()`: optionally auto-configures, `X02C1B_stream_on()`, peripheral reset, stale `event_bits` clear, arm `HAL_USART_Receive_DMA()`. |
| `OW_CAMERA_STREAM reserved=0` | — | `disable_camera_stream()`: `X02C1B_stream_off()`, `HAL_USART_Abort()`, 10 ms wait. |
| `OW_CAMERA_SET_CONFIG` | — | Write default register set to image sensor via I2C. |
| `OW_CAMERA_RESET_UART` | `0x27` | `reset_camera_usart()`: HAL abort + peripheral disable/re-enable + HAL state force-READY. Handles both USART and SPI cameras. Range-checked only, not gated on `isPresent`. |
| `OW_CAMERA_FSIN_EXTERNAL reserved=1` | `0x26` | `X02C1B_FSIN_EXT_enable()` — arm fsin output GPIO. |
| `OW_CAMERA_FSIN_EXTERNAL reserved=0` | `0x26` | `X02C1B_FSIN_EXT_disable()` — disarm fsin output GPIO. |
| `OW_FPGA_ON` | `0x11` | `enable_fpga()`: assert CRESET_B HIGH, 2 ms delay. **Wire protocol only — not exposed in SDK** (toggling CRESET_B was found to cause spurious FSIN edges). |
| `OW_FPGA_OFF` | `0x12` | `disable_fpga()`: assert CRESET_B LOW, 2 ms delay. **Wire protocol only — not exposed in SDK.** |
| `OW_FPGA_PROG_SRAM reserved=1` | — | `program_fpga()`: load bitstream from ROM via I2C, USART reset on success. |
| `OW_FPGA_PROG_SRAM reserved=0` | — | `program_sram_fpga()`: load bitstream from host buffer via I2C. |
| `OW_FPGA_BITSTREAM` | — | Transfer bitstream block to MCU buffer; `reserved=1` on last block validates CRC. |
