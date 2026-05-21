# Per-Frame PDC Telemetry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Capture one photodiode-current (PDC) value per camera frame at 40 Hz on the console firmware, drain it from the host at 10 Hz, and write per-frame rows into the telemetry CSV — with an explicit `dark_slot` label so downstream analysis can distinguish bright vs. dark-slot pulses.

**Architecture:**
- Console FW samples the safety FPGA's `peak_power_value` register (`0x1C/0x1D`) once per laser pulse, on the LASER_TIMER period-elapsed (falling-edge) ISR. Samples are tagged with `lsync_counter` and a `current_slot_is_dark` bit and pushed into a 256-entry ring buffer in SRAM. A new opcode `OW_CTRL_GET_PDC_BUFFER` drains the buffer.
- SDK `ConsoleTelemetryPoller` becomes a 10 Hz poller. Every tick drains the FW buffer (~4 samples per call); every 10th tick re-reads the existing slow telemetry. Per-frame samples are delivered via a new `add_pdc_listener` channel.
- `ScanWorkflow` writes one CSV row per drained PDC sample, with slow columns carry-forwarded from the last 1 Hz snapshot and an explicit `dark_slot` column.

**Tech Stack:** C (STM32 HAL, STM32H743), Python 3.12 (pyserial + crcmod + dataclasses), pytest.

**Spec:** `docs/superpowers/specs/2026-05-20-per-frame-pdc-telemetry-design.md`

**Repos touched:**
- `openmotion-console-fw` — new ring buffer module, ISR hook, opcode handler.
- `openmotion-sdk` — `ConsoleTelemetry.py`, `MotionConsole.py`, `ScanWorkflow.py`, tests, diagnostic script.

---

## File Structure

### openmotion-console-fw (new + modified)

| Path | Action | Responsibility |
|---|---|---|
| `Core/Inc/pdc_buffer.h` | **Create** | Pure ring-buffer API for `pdc_sample_t` tuples. Host-testable. |
| `Core/Src/pdc_buffer.c` | **Create** | Ring-buffer impl with drop-oldest + monotonic `dropped_count`. |
| `Core/Inc/pdc_poll.h` | **Create** | Main-loop poller API (`pdc_poll_init`, `pdc_poll_tick`, `pdc_poll_request_sample`). |
| `Core/Src/pdc_poll.c` | **Create** | Owns the pending flag, the post-pulse delay, the I2C read, and the push. |
| `Core/Inc/trigger.h` | Modify | Declare `LSYNC_PeriodElapsedCallback`. Expose `get_current_slot_is_dark()`. |
| `Core/Src/trigger.c` | Modify | Maintain `current_slot_is_dark` in `FSYNC_PeriodElapsedCallback`. Add `LSYNC_PeriodElapsedCallback` that sets the pending flag with snapshotted `lsync_counter` + slot bit. Enable / disable `TIM_IT_UPDATE` on LASER_TIMER in `Trigger_Start` / `Trigger_Stop`. |
| `Core/Src/main.c` | Modify | Dispatch LASER_TIMER (`TIM3`) period-elapsed to `LSYNC_PeriodElapsedCallback`. Call `pdc_poll_tick()` in the main while loop. |
| `Core/Inc/common.h` | Modify | Add `OW_CTRL_GET_PDC_BUFFER = 0x25` to `MotionControllerCommands`. |
| `Core/Src/if_commands.c` | Modify | Handler for `OW_CTRL_GET_PDC_BUFFER` that drains up to `max_samples`. |
| `CommandHandling.md` | Modify | Document the new opcode + payload format. |
| `host_tests/test_pdc_buffer.c` | **Create** | Standalone GCC-built test of the ring buffer. |
| `host_tests/Makefile` | **Create** | One-command host build. |

### openmotion-sdk (modified + new)

| Path | Action | Responsibility |
|---|---|---|
| `omotion/config.py` | Modify | Add `OW_CTRL_GET_PDC_BUFFER = 0x25`. |
| `omotion/MotionConsole.py` | Modify | Add `get_pdc_buffer(max_samples)` raw-bytes call. Wire the constant into the imports. |
| `omotion/ConsoleTelemetry.py` | Modify | New `PdcSample` dataclass. Change `_POLL_INTERVAL_S` to 0.1. Add `_pdc_listeners`, `add_pdc_listener`, `get_last_pdc_sample`. Add drain step on every tick; do slow re-reads on every 10th tick. Remove the dedicated PDC I2C read in `_read_analog`. Derive `snap.pdc` from the last `PdcSample`. |
| `omotion/ScanWorkflow.py` | Modify | Extend `_TELEMETRY_HEADERS` with the new appended columns (`frame_idx`, `dark_slot`, `pdc_flags`, `pdc_dropped_delta`, `slow_age_ms`). Replace `_snap_to_row` with `_pdc_row(pdc_sample, snap)`. Switch the telemetry-CSV listener from `add_listener` to `add_pdc_listener`. |
| `tests/test_console_telemetry_unit.py` | Modify | Add tests for `PdcSample` parsing, drain-response parsing, 10 Hz tick / 1 Hz slow phasing. |
| `tests/test_telemetry_csv_per_frame.py` | **Create** | Unit tests for `_pdc_row` schema and carry-forward behavior. |
| `scripts/check_dark_slot_consistency.py` | **Create** | Joins a telemetry CSV's `dark_slot` against the science-pipeline-predicted dark-frame mask. |

---

## Branching

- [ ] **Step 1: Create a feature branch in `openmotion-console-fw`**

```
cd C:/Users/ethan/Projects/openmotion-console-fw
git checkout -b feature/per-frame-pdc
```

- [ ] **Step 2: Create a feature branch in `openmotion-sdk`**

```
cd C:/Users/ethan/Projects/openmotion-sdk
git checkout feature/data-pipeline-tweaks
git checkout -b feature/per-frame-pdc
```

(Branch off the existing `feature/data-pipeline-tweaks` so the spec commits travel with the implementation.)

---

## Firmware tasks (openmotion-console-fw)

### Task FW-1: Ring buffer module (host-testable)

**Files:**
- Create: `Core/Inc/pdc_buffer.h`
- Create: `Core/Src/pdc_buffer.c`

- [ ] **Step 1: Write the header**

```c
/* Core/Inc/pdc_buffer.h */
#ifndef INC_PDC_BUFFER_H_
#define INC_PDC_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define PDC_BUFFER_CAPACITY 256

typedef struct __attribute__((packed)) {
    uint32_t frame_idx;
    uint16_t pdc_raw;
    uint8_t  flags;   /* bit 0 = dark_slot */
} pdc_sample_t;

#define PDC_FLAG_DARK_SLOT (1u << 0)

void     pdc_buffer_reset(void);
bool     pdc_buffer_push(const pdc_sample_t *sample);  /* drop-oldest on overflow, returns true if a drop occurred */
size_t   pdc_buffer_drain(pdc_sample_t *out, size_t max_samples);
uint16_t pdc_buffer_dropped_since_last_drain(void);
size_t   pdc_buffer_count(void);

#endif
```

- [ ] **Step 2: Write the implementation**

```c
/* Core/Src/pdc_buffer.c */
#include "pdc_buffer.h"
#include <string.h>

static pdc_sample_t s_buf[PDC_BUFFER_CAPACITY];
static volatile uint16_t s_head;        /* write idx */
static volatile uint16_t s_tail;        /* read idx */
static volatile uint16_t s_count;
static volatile uint16_t s_dropped_pending;  /* drops since last drain */

void pdc_buffer_reset(void) {
    s_head = 0; s_tail = 0; s_count = 0; s_dropped_pending = 0;
    memset(s_buf, 0, sizeof(s_buf));
}

bool pdc_buffer_push(const pdc_sample_t *sample) {
    bool dropped = false;
    if (s_count == PDC_BUFFER_CAPACITY) {
        /* drop oldest */
        s_tail = (uint16_t)((s_tail + 1) % PDC_BUFFER_CAPACITY);
        s_count--;
        s_dropped_pending++;
        dropped = true;
    }
    s_buf[s_head] = *sample;
    s_head = (uint16_t)((s_head + 1) % PDC_BUFFER_CAPACITY);
    s_count++;
    return dropped;
}

size_t pdc_buffer_drain(pdc_sample_t *out, size_t max_samples) {
    size_t n = 0;
    while (n < max_samples && s_count > 0) {
        out[n++] = s_buf[s_tail];
        s_tail = (uint16_t)((s_tail + 1) % PDC_BUFFER_CAPACITY);
        s_count--;
    }
    return n;
}

uint16_t pdc_buffer_dropped_since_last_drain(void) {
    uint16_t d = s_dropped_pending;
    s_dropped_pending = 0;
    return d;
}

size_t pdc_buffer_count(void) {
    return s_count;
}
```

- [ ] **Step 3: Write the host test**

Create `host_tests/test_pdc_buffer.c`:

```c
#include <assert.h>
#include <stdio.h>
#include "../Core/Inc/pdc_buffer.h"
/* pull in the source directly for the host build */
#include "../Core/Src/pdc_buffer.c"

static pdc_sample_t mk(uint32_t f) { pdc_sample_t s = { f, (uint16_t)(f & 0xFFFF), 0 }; return s; }

int main(void) {
    pdc_buffer_reset();
    assert(pdc_buffer_count() == 0);

    /* push 3, drain 3 */
    pdc_sample_t s1 = mk(1), s2 = mk(2), s3 = mk(3);
    assert(!pdc_buffer_push(&s1));
    assert(!pdc_buffer_push(&s2));
    assert(!pdc_buffer_push(&s3));
    assert(pdc_buffer_count() == 3);

    pdc_sample_t out[8];
    size_t n = pdc_buffer_drain(out, 8);
    assert(n == 3);
    assert(out[0].frame_idx == 1 && out[1].frame_idx == 2 && out[2].frame_idx == 3);
    assert(pdc_buffer_dropped_since_last_drain() == 0);

    /* overflow: push 260, capacity 256, drain should see frames 5..260 (256 samples), drop count 4 */
    pdc_buffer_reset();
    for (uint32_t i = 1; i <= 260; i++) {
        pdc_sample_t s = mk(i);
        pdc_buffer_push(&s);
    }
    assert(pdc_buffer_count() == PDC_BUFFER_CAPACITY);
    pdc_sample_t big[PDC_BUFFER_CAPACITY];
    n = pdc_buffer_drain(big, PDC_BUFFER_CAPACITY);
    assert(n == PDC_BUFFER_CAPACITY);
    assert(big[0].frame_idx == 5);    /* oldest dropped: 1..4 */
    assert(big[255].frame_idx == 260);
    assert(pdc_buffer_dropped_since_last_drain() == 4);

    /* second drain after read: dropped counter resets to 0 */
    assert(pdc_buffer_dropped_since_last_drain() == 0);

    /* partial drain */
    pdc_buffer_reset();
    for (uint32_t i = 1; i <= 10; i++) { pdc_sample_t s = mk(i); pdc_buffer_push(&s); }
    n = pdc_buffer_drain(out, 4);
    assert(n == 4);
    assert(out[0].frame_idx == 1 && out[3].frame_idx == 4);
    assert(pdc_buffer_count() == 6);

    printf("pdc_buffer host tests OK\n");
    return 0;
}
```

- [ ] **Step 4: Write the host Makefile**

Create `host_tests/Makefile`:

```
CC ?= gcc
CFLAGS ?= -Wall -Wextra -Wno-unused-parameter -O0 -g -I../Core/Inc

test_pdc_buffer: test_pdc_buffer.c ../Core/Src/pdc_buffer.c
	$(CC) $(CFLAGS) -o $@ $<

run: test_pdc_buffer
	./test_pdc_buffer

clean:
	rm -f test_pdc_buffer
```

- [ ] **Step 5: Build and run**

```
cd host_tests
make run
```

Expected: `pdc_buffer host tests OK`

- [ ] **Step 6: Commit**

```
git add Core/Inc/pdc_buffer.h Core/Src/pdc_buffer.c host_tests/
git commit -m "feat(fw): pdc_buffer ring buffer with drop-oldest + host tests"
```

---

### Task FW-2: Slot-tracking and falling-edge ISR in `trigger.c`

**Files:**
- Modify: `Core/Inc/trigger.h`
- Modify: `Core/Src/trigger.c`

- [ ] **Step 1: Add the slot accessor + new ISR declaration**

In `Core/Inc/trigger.h`, after `void FSYNC_PeriodElapsedCallback(...)` (line 47):

```c
void LSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
bool get_current_slot_is_dark(void);
uint32_t get_pending_pdc_frame_idx(void);   /* lsync_counter snapshot at last LSYNC period-elapsed */
bool consume_pdc_sample_pending(bool *out_dark_slot, uint32_t *out_frame_idx);
```

- [ ] **Step 2: Add slot tracking in `trigger.c`**

Add file-scope statics near the existing volatile counters (around line 24):

```c
static volatile bool s_current_slot_is_dark = false;

/* Set by LSYNC_PeriodElapsedCallback; consumed by pdc_poll_tick() from main loop. */
static volatile bool     s_pdc_sample_pending = false;
static volatile bool     s_pdc_sample_dark    = false;
static volatile uint32_t s_pdc_sample_frame   = 0;
```

- [ ] **Step 3: Update `FSYNC_PeriodElapsedCallback` to track slot**

Modify the existing callback (around `trigger.c:343-358`):

```c
void FSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    fsync_counter++;
    if (trigger_config.LaserPulseSkipInterval > 0) {
        bool dark = ((fsync_counter % trigger_config.LaserPulseSkipInterval) == 0)
                    || (fsync_counter < NUM_DARK_FRAMES_AT_START);
        if (dark) {
            __HAL_TIM_SET_AUTORELOAD(&LASER_TIMER, long_lsync_arr);
            __HAL_TIM_SET_COMPARE  (&LASER_TIMER, TIM_CHANNEL_1, long_lsync_ccr1);
        } else {
            __HAL_TIM_SET_AUTORELOAD(&LASER_TIMER, short_lsync_arr);
            __HAL_TIM_SET_COMPARE  (&LASER_TIMER, TIM_CHANNEL_1, short_lsync_ccr1);
        }
        s_current_slot_is_dark = dark;
    }
}
```

- [ ] **Step 4: Add the LASER_TIMER period-elapsed callback**

Append to `trigger.c` after `LSYNC_DelayElapsedCallback`:

```c
void LSYNC_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Fires on laser-pulse falling edge.  Snapshot lsync_counter + slot bit
     * for pdc_poll to consume from the main loop. */
    s_pdc_sample_frame  = lsync_counter;
    s_pdc_sample_dark   = s_current_slot_is_dark;
    s_pdc_sample_pending = true;
}

bool get_current_slot_is_dark(void) { return s_current_slot_is_dark; }

bool consume_pdc_sample_pending(bool *out_dark_slot, uint32_t *out_frame_idx)
{
    __disable_irq();
    bool pending = s_pdc_sample_pending;
    if (pending) {
        *out_dark_slot = s_pdc_sample_dark;
        *out_frame_idx = s_pdc_sample_frame;
        s_pdc_sample_pending = false;
    }
    __enable_irq();
    return pending;
}
```

- [ ] **Step 5: Enable TIM_IT_UPDATE on LASER_TIMER in `Trigger_Start`**

Add immediately after line 250 (`__HAL_TIM_ENABLE_IT(&LASER_TIMER, TIM_IT_CC1);`):

```c
__HAL_TIM_CLEAR_FLAG(&LASER_TIMER, TIM_FLAG_UPDATE);
__HAL_TIM_ENABLE_IT(&LASER_TIMER, TIM_IT_UPDATE);
```

And reset the slot/pending state alongside the lsync/fsync reset (immediately above, around line 247):

```c
s_current_slot_is_dark = false;
s_pdc_sample_pending = false;
```

- [ ] **Step 6: Disable TIM_IT_UPDATE in `Trigger_Stop`**

In `Trigger_Stop` after `__HAL_TIM_DISABLE(&LASER_TIMER);` (line 268):

```c
__HAL_TIM_DISABLE_IT(&LASER_TIMER, TIM_IT_UPDATE);
```

- [ ] **Step 7: Commit**

```
git add Core/Inc/trigger.h Core/Src/trigger.c
git commit -m "feat(fw): track slot + snapshot frame_idx on LASER_TIMER falling edge"
```

---

### Task FW-3: Main-loop PDC poll module

**Files:**
- Create: `Core/Inc/pdc_poll.h`
- Create: `Core/Src/pdc_poll.c`

- [ ] **Step 1: Header**

```c
/* Core/Inc/pdc_poll.h */
#ifndef INC_PDC_POLL_H_
#define INC_PDC_POLL_H_

#include <stdint.h>

void pdc_poll_init(void);
/* Call from the main loop. Performs the I2C read of the safety FPGA peak-power
 * register and pushes a pdc_sample_t to the buffer when one is pending and the
 * post-pulse settling time has elapsed. Non-blocking otherwise. */
void pdc_poll_tick(void);

#endif
```

- [ ] **Step 2: Implementation**

```c
/* Core/Src/pdc_poll.c */
#include "pdc_poll.h"
#include "pdc_buffer.h"
#include "trigger.h"
#include "tca9548.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>

#define PDC_MUX_INDEX   1
#define PDC_CHANNEL     7
#define PDC_I2C_ADDR    0x41
#define PDC_REG         0x1C
#define PDC_BYTES       2
#define PDC_SETTLE_MS   1   /* FPGA averaging window after laser pulse falls */

static uint32_t s_pending_since_tick = 0;
static bool     s_have_pending = false;
static bool     s_dark_slot = false;
static uint32_t s_frame_idx = 0;
static uint32_t s_fail_count = 0;

void pdc_poll_init(void) {
    s_pending_since_tick = 0;
    s_have_pending = false;
    s_fail_count = 0;
}

void pdc_poll_tick(void) {
    /* Pick up any new pending sample from the LSYNC ISR. */
    if (!s_have_pending) {
        bool d; uint32_t f;
        if (consume_pdc_sample_pending(&d, &f)) {
            s_have_pending = true;
            s_dark_slot = d;
            s_frame_idx = f;
            s_pending_since_tick = HAL_GetTick();
        }
    }

    if (!s_have_pending) return;

    /* Wait for the FPGA peak-power averaging window to complete. */
    if ((HAL_GetTick() - s_pending_since_tick) < PDC_SETTLE_MS) return;

    uint8_t bytes[PDC_BYTES] = {0};
    int8_t rc = TCA9548A_Read_Data(PDC_MUX_INDEX, PDC_CHANNEL, PDC_I2C_ADDR,
                                   PDC_REG, PDC_BYTES, bytes);
    s_have_pending = false;   /* always consume — don't get stuck on a failing read */

    if (rc != TCA9548A_OK) {
        s_fail_count++;
        /* Log sparsely so we don't flood the UART. */
        if ((s_fail_count & 0x3F) == 1) {
            printf("pdc_poll: I2C read failed (count=%lu, rc=%d)\r\n",
                   (unsigned long)s_fail_count, (int)rc);
        }
        return;
    }

    pdc_sample_t sample = {
        .frame_idx = s_frame_idx,
        .pdc_raw   = (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8)),
        .flags     = (uint8_t)(s_dark_slot ? PDC_FLAG_DARK_SLOT : 0u),
    };
    (void)pdc_buffer_push(&sample);
}
```

- [ ] **Step 3: Commit**

```
git add Core/Inc/pdc_poll.h Core/Src/pdc_poll.c
git commit -m "feat(fw): pdc_poll main-loop driver with FPGA settle delay"
```

---

### Task FW-4: Wire callbacks + main-loop call

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Include the new headers**

Near the other `#include`s in `main.c` (search for `#include "trigger.h"`):

```c
#include "pdc_poll.h"
```

- [ ] **Step 2: Dispatch LASER_TIMER period-elapsed**

Modify `HAL_TIM_PeriodElapsedCallback` (around `main.c:1726-1739`):

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    comms_telemetry_tick();
  }
  if (htim->Instance == FSYNC_TIMER.Instance) {
    FSYNC_PeriodElapsedCallback(htim);
  }
  if (htim->Instance == LASER_TIMER.Instance) {
    LSYNC_PeriodElapsedCallback(htim);
  }
  if (htim->Instance == TIM12) {
    CDC_Idle_Timer_Handler();
  }
}
```

- [ ] **Step 3: Init the poller before main loop**

In `main()`, immediately after `comms_init();` (around line 562):

```c
pdc_poll_init();
```

- [ ] **Step 4: Call the tick from main loop**

Modify the while(1) (around lines 570-578):

```c
while (1)
{
    comms_process();
    telemetry_poll();
    pdc_poll_tick();
    HAL_Delay(1);
}
```

- [ ] **Step 5: Commit**

```
git add Core/Src/main.c
git commit -m "feat(fw): dispatch LASER_TIMER ISR + call pdc_poll_tick from main loop"
```

---

### Task FW-5: New `OW_CTRL_GET_PDC_BUFFER` opcode

**Files:**
- Modify: `Core/Inc/common.h`
- Modify: `Core/Src/if_commands.c`
- Modify: `CommandHandling.md`

- [ ] **Step 1: Add the opcode**

In `Core/Inc/common.h` after `OW_CTRL_PDUMON = 0x24`:

```c
OW_CTRL_GET_PDC_BUFFER = 0x25,
```

- [ ] **Step 2: Include `pdc_buffer.h` in `if_commands.c`**

Add at the top of `if_commands.c` near existing includes:

```c
#include "pdc_buffer.h"
```

- [ ] **Step 3: Add the handler**

Insert in the `OW_CONTROLLER` switch statement after the existing `OW_CTRL_PDUMON` case (around line 441):

```c
case OW_CTRL_GET_PDC_BUFFER: {
    uartResp->command = OW_CTRL_GET_PDC_BUFFER;
    uartResp->addr = cmd->addr;
    uartResp->reserved = cmd->reserved;

    /* Request payload: 1 byte = max_samples (clamped to 64). */
    uint8_t requested = (cmd->data_len >= 1) ? cmd->data[0] : 64;
    if (requested == 0 || requested > 64) requested = 64;

    /* Response layout: [dropped:u16 LE][count:u8][count * sizeof(pdc_sample_t)] */
    static pdc_sample_t drained[64];
    size_t n = pdc_buffer_drain(drained, requested);
    uint16_t dropped = pdc_buffer_dropped_since_last_drain();

    static uint8_t resp[3 + 64 * sizeof(pdc_sample_t)];
    resp[0] = (uint8_t)(dropped & 0xFF);
    resp[1] = (uint8_t)((dropped >> 8) & 0xFF);
    resp[2] = (uint8_t)n;
    memcpy(&resp[3], drained, n * sizeof(pdc_sample_t));

    uartResp->data_len = (uint16_t)(3 + n * sizeof(pdc_sample_t));
    uartResp->data = resp;
} break;
```

- [ ] **Step 4: Update `CommandHandling.md`**

In the Motion Controller Commands table (around line 142), add the new row:

```
| `OW_CTRL_GET_PDC_BUFFER` | Drain up to N per-frame PDC samples from the SRAM ring buffer. Request payload: 1 byte = max_samples (1..64). Response: 2-byte LE drop counter + 1-byte sample count + N × 7-byte packed `{u32 frame_idx, u16 pdc_raw, u8 flags}`. `flags` bit 0 = `dark_slot`. |
```

- [ ] **Step 5: Build the firmware**

```
cd C:/Users/ethan/Projects/openmotion-console-fw
cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake
cmake --build build 2>&1 | tail -30
```

Expected: `motion-console-fw.elf` built without warnings about undefined symbols. If the build system doesn't pick up new files in `Core/Src/`, regenerate the build with `cmake -B build --fresh ...`.

- [ ] **Step 6: Commit**

```
git add Core/Inc/common.h Core/Src/if_commands.c CommandHandling.md
git commit -m "feat(fw): OW_CTRL_GET_PDC_BUFFER opcode (0x25)"
```

---

## SDK tasks (openmotion-sdk)

### Task SDK-1: Add `OW_CTRL_GET_PDC_BUFFER` constant

**Files:**
- Modify: `omotion/config.py`

- [ ] **Step 1: Add the constant**

In `omotion/config.py`, immediately after the existing `OW_CTRL_PDUMON = 0x24` line (around line 148):

```python
OW_CTRL_GET_PDC_BUFFER = 0x25
```

- [ ] **Step 2: Commit**

```
git add omotion/config.py
git commit -m "feat(sdk): add OW_CTRL_GET_PDC_BUFFER constant"
```

---

### Task SDK-2: `MotionConsole.get_pdc_buffer`

**Files:**
- Modify: `omotion/MotionConsole.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_get_pdc_buffer.py`:

```python
import struct
from unittest.mock import MagicMock

from omotion.MotionConsole import MotionConsole
from omotion.config import (
    OW_CONTROLLER, OW_CTRL_GET_PDC_BUFFER, OW_ERROR,
)


def _make_console_with_uart_response(payload: bytes):
    console = MotionConsole.__new__(MotionConsole)
    console.uart = MagicMock()

    response = MagicMock()
    response.packet_type = 0  # OW_RESP
    response.data = payload
    response.data_len = len(payload)
    console.uart.send_packet.return_value = response
    console.uart.clear_buffer = MagicMock()
    return console


def test_get_pdc_buffer_parses_empty_response():
    # dropped=0, count=0, no tuples
    payload = struct.pack("<HB", 0, 0)
    console = _make_console_with_uart_response(payload)
    dropped, samples = console.get_pdc_buffer(max_samples=64)
    assert dropped == 0
    assert samples == []
    console.uart.send_packet.assert_called_once()
    _, kwargs = console.uart.send_packet.call_args
    assert kwargs["packetType"] == OW_CONTROLLER
    assert kwargs["command"] == OW_CTRL_GET_PDC_BUFFER
    assert kwargs["data"] == bytes([64])


def test_get_pdc_buffer_parses_three_samples():
    samples_raw = b"".join(
        struct.pack("<IHB", fid, raw, flags)
        for fid, raw, flags in [(100, 0x0123, 0x00), (101, 0x0456, 0x01), (102, 0x07AB, 0x00)]
    )
    payload = struct.pack("<HB", 0, 3) + samples_raw
    console = _make_console_with_uart_response(payload)
    dropped, samples = console.get_pdc_buffer(max_samples=64)
    assert dropped == 0
    assert samples == [
        (100, 0x0123, 0x00),
        (101, 0x0456, 0x01),
        (102, 0x07AB, 0x00),
    ]


def test_get_pdc_buffer_returns_empty_on_error():
    console = _make_console_with_uart_response(b"")
    console.uart.send_packet.return_value.packet_type = OW_ERROR
    dropped, samples = console.get_pdc_buffer(max_samples=8)
    assert dropped == 0
    assert samples == []
```

- [ ] **Step 2: Run, expect failure**

```
cd C:/Users/ethan/Projects/openmotion-sdk
python -m pytest tests/test_get_pdc_buffer.py -v
```

Expected: `AttributeError: 'MotionConsole' object has no attribute 'get_pdc_buffer'`.

- [ ] **Step 3: Add the method**

In `omotion/MotionConsole.py`, find a sensible spot near `read_pdu_mon` (around line 1696). Add:

```python
def get_pdc_buffer(self, max_samples: int = 64) -> tuple[int, list[tuple[int, int, int]]]:
    """Drain up to ``max_samples`` per-frame PDC samples from the console firmware.

    Returns ``(dropped_count_delta, samples)`` where ``samples`` is a list of
    ``(frame_idx, pdc_raw, flags)`` tuples in FIFO order.  Flags bit 0 = dark_slot.
    Returns ``(0, [])`` on transport error.
    """
    if max_samples < 1 or max_samples > 64:
        raise ValueError(f"max_samples must be in [1,64], got {max_samples}")

    try:
        r = self.uart.send_packet(
            id=None,
            packetType=OW_CONTROLLER,
            command=OW_CTRL_GET_PDC_BUFFER,
            data=bytes([max_samples]),
        )
        self.uart.clear_buffer()

        if r.packet_type == OW_ERROR:
            logger.warning("get_pdc_buffer: console returned OW_ERROR")
            return 0, []

        if not r.data or r.data_len < 3:
            return 0, []

        dropped = int.from_bytes(r.data[0:2], "little")
        count = r.data[2]
        expected_len = 3 + count * 7
        if r.data_len < expected_len:
            logger.warning(
                "get_pdc_buffer: short response (got %d bytes, expected %d for count=%d)",
                r.data_len, expected_len, count,
            )
            return dropped, []

        samples: list[tuple[int, int, int]] = []
        for i in range(count):
            offset = 3 + i * 7
            frame_idx = int.from_bytes(r.data[offset:offset+4], "little")
            pdc_raw = int.from_bytes(r.data[offset+4:offset+6], "little")
            flags = r.data[offset+6]
            samples.append((frame_idx, pdc_raw, flags))
        return dropped, samples
    except Exception as e:
        logger.debug("get_pdc_buffer transport error: %s", e)
        return 0, []
```

Also add `OW_CTRL_GET_PDC_BUFFER` to the imports from `omotion.config` at the top of the file.

- [ ] **Step 4: Run tests to verify pass**

```
python -m pytest tests/test_get_pdc_buffer.py -v
```

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```
git add omotion/MotionConsole.py tests/test_get_pdc_buffer.py
git commit -m "feat(sdk): MotionConsole.get_pdc_buffer drain client"
```

---

### Task SDK-3: `PdcSample` dataclass

**Files:**
- Modify: `omotion/ConsoleTelemetry.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_console_telemetry_unit.py`:

```python
from omotion.ConsoleTelemetry import PdcSample, PDC_MA_PER_LSB


def test_pdc_sample_scales_raw_to_mA():
    s = PdcSample.from_raw(frame_idx=42, pdc_raw=100, flags=0x01, host_recv_timestamp=1.23)
    assert s.frame_idx == 42
    assert s.pdc_mA == 100 * PDC_MA_PER_LSB
    assert s.dark_slot is True
    assert s.host_recv_timestamp == 1.23
    assert s.dropped_delta == 0


def test_pdc_sample_dark_slot_false_when_flags_clear():
    s = PdcSample.from_raw(frame_idx=43, pdc_raw=200, flags=0x00, host_recv_timestamp=0.0)
    assert s.dark_slot is False
```

- [ ] **Step 2: Run, expect failure**

```
python -m pytest tests/test_console_telemetry_unit.py -v
```

Expected: `ImportError: cannot import name 'PdcSample'`.

- [ ] **Step 3: Add the dataclass**

In `omotion/ConsoleTelemetry.py`, add near the existing `_PDC_MA_PER_LSB` constant (around line 39):

```python
# Renamed to public for downstream PdcSample use; keep the private alias for
# backwards source-compat with code referencing _PDC_MA_PER_LSB.
PDC_MA_PER_LSB: float = 1.9
_PDC_MA_PER_LSB = PDC_MA_PER_LSB
```

And after the `ConsoleTelemetry` dataclass definition (around line 99):

```python
PDC_FLAG_DARK_SLOT: int = 1 << 0


@dataclass
class PdcSample:
    """One per-frame photodiode-current measurement drained from the console
    firmware's ring buffer.

    See docs/superpowers/specs/2026-05-20-per-frame-pdc-telemetry-design.md.
    """
    frame_idx: int
    pdc_mA: float
    dark_slot: bool
    host_recv_timestamp: float
    dropped_delta: int = 0

    @classmethod
    def from_raw(
        cls,
        frame_idx: int,
        pdc_raw: int,
        flags: int,
        host_recv_timestamp: float,
        dropped_delta: int = 0,
    ) -> "PdcSample":
        return cls(
            frame_idx=int(frame_idx),
            pdc_mA=float(pdc_raw) * PDC_MA_PER_LSB,
            dark_slot=bool(flags & PDC_FLAG_DARK_SLOT),
            host_recv_timestamp=float(host_recv_timestamp),
            dropped_delta=int(dropped_delta),
        )
```

- [ ] **Step 4: Run tests**

```
python -m pytest tests/test_console_telemetry_unit.py -v
```

Expected: existing safety test + 2 new PdcSample tests pass.

- [ ] **Step 5: Commit**

```
git add omotion/ConsoleTelemetry.py tests/test_console_telemetry_unit.py
git commit -m "feat(sdk): PdcSample dataclass + PDC_MA_PER_LSB public constant"
```

---

### Task SDK-4: `ConsoleTelemetryPoller` 10 Hz drain + slow-phase cadence

**Files:**
- Modify: `omotion/ConsoleTelemetry.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_console_telemetry_unit.py`:

```python
from unittest.mock import MagicMock
import time as _time

class _FakeConsoleForDrain:
    """Mock console that returns scripted drain responses on each call."""
    def __init__(self, drain_responses):
        self._drain_responses = list(drain_responses)
        self.drain_calls = 0
        self.tec_calls = 0
        self.pdu_calls = 0
        self.safety_calls = 0
        self.analog_calls = 0

    def is_connected(self):
        return True

    def get_pdc_buffer(self, max_samples=64):
        self.drain_calls += 1
        if self._drain_responses:
            return self._drain_responses.pop(0)
        return 0, []

    # Stubs used by the slow tick — set as MagicMocks externally if needed
    def tec_status(self):
        self.tec_calls += 1
        return 0.0, 0.0, 0.0, 0.0, False

    def read_pdu_mon(self):
        self.pdu_calls += 1
        m = MagicMock(); m.raws = []; m.volts = []
        return m

    def read_i2c_packet(self, mux_index, channel, device_addr, reg_addr, read_len):
        self.safety_calls += 1
        return b"\x00" * read_len, read_len

    def get_lsync_pulsecount(self):
        self.analog_calls += 1
        return 0


def test_poller_drains_pdc_each_tick_and_fires_listeners():
    drain_responses = [
        (0, [(1, 100, 0x00), (2, 200, 0x01)]),
        (0, [(3, 150, 0x00)]),
    ]
    console = _FakeConsoleForDrain(drain_responses)
    poller = ConsoleTelemetryPoller(console)
    received = []
    poller.add_pdc_listener(received.append)

    # Run two ticks synchronously by calling the inner method that processes
    # one drain pass + (optionally) a slow refresh.
    poller._tick_once()
    poller._tick_once()

    assert console.drain_calls == 2
    assert len(received) == 3
    assert received[0].frame_idx == 1 and received[0].dark_slot is False
    assert received[1].dark_slot is True
    assert received[2].frame_idx == 3
    assert poller.get_last_pdc_sample().frame_idx == 3


def test_poller_runs_slow_refresh_every_10th_tick():
    console = _FakeConsoleForDrain([(0, [])] * 25)
    poller = ConsoleTelemetryPoller(console)
    for _ in range(20):
        poller._tick_once()
    # Slow refresh fires on tick 0 and tick 10 (and not in between).
    assert console.tec_calls == 2
    assert console.pdu_calls == 2


def test_poller_attaches_dropped_delta_to_first_sample_only():
    drain_responses = [(7, [(1, 100, 0), (2, 110, 0)])]
    console = _FakeConsoleForDrain(drain_responses)
    poller = ConsoleTelemetryPoller(console)
    received = []
    poller.add_pdc_listener(received.append)
    poller._tick_once()
    assert received[0].dropped_delta == 7
    assert received[1].dropped_delta == 0
```

- [ ] **Step 2: Run, expect failure**

```
python -m pytest tests/test_console_telemetry_unit.py -v -k pdc
```

Expected: AttributeErrors (`_tick_once`, `add_pdc_listener`, `get_last_pdc_sample`).

- [ ] **Step 3: Modify the poller**

In `omotion/ConsoleTelemetry.py`:

(a) Change `_POLL_INTERVAL_S = 0.1` (was `1.0`).

(b) Add module-level constant after the existing interval constants:

```python
_SLOW_TICK_EVERY_N: int = 10  # every 10th 100 ms tick = 1 Hz slow refresh
```

(c) Modify `ConsoleTelemetryPoller.__init__` to add the new state:

```python
self._pdc_listeners: List[Callable[[PdcSample], None]] = []
self._last_pdc: Optional[PdcSample] = None
self._slow_phase: int = 0
```

(d) Add public API methods to the class:

```python
def add_pdc_listener(self, fn: Callable[[PdcSample], None]) -> None:
    with self._lock:
        if fn not in self._pdc_listeners:
            self._pdc_listeners.append(fn)

def remove_pdc_listener(self, fn: Callable[[PdcSample], None]) -> None:
    with self._lock:
        try:
            self._pdc_listeners.remove(fn)
        except ValueError:
            pass

def get_last_pdc_sample(self) -> Optional[PdcSample]:
    with self._lock:
        return self._last_pdc
```

(e) Replace the body of `_poll_loop` so it delegates to `_tick_once` (so tests can call the unit directly):

```python
def _poll_loop(self) -> None:
    logger.debug("ConsoleTelemetryPoller poll loop entered")
    last_poll = 0.0
    while True:
        with self._lock:
            if not self._running:
                break
        now = time.time()
        if (now - last_poll) >= _POLL_INTERVAL_S:
            tick_start = time.time()
            self._tick_once()
            last_poll = tick_start
            duration = time.time() - tick_start
            logger.debug("ConsoleTelemetryPoller tick %.1f ms", duration * 1000.0)
        sleep_s = _POLL_INTERVAL_S - (time.time() - last_poll)
        sleep_s = max(_MIN_SLEEP_S, min(_MAX_SLEEP_S, sleep_s))
        self._wake.wait(timeout=sleep_s)
        self._wake.clear()
    logger.debug("ConsoleTelemetryPoller poll loop exited")
```

(f) Add `_tick_once`:

```python
def _tick_once(self) -> None:
    """One scheduler tick: always drain PDC, occasionally refresh slow telemetry."""
    self._drain_pdc()
    if self._slow_phase == 0:
        self._refresh_slow()
    self._slow_phase = (self._slow_phase + 1) % _SLOW_TICK_EVERY_N

def _drain_pdc(self) -> None:
    try:
        dropped, raw_samples = self._console.get_pdc_buffer(max_samples=64)
    except Exception as exc:
        logger.warning("ConsoleTelemetryPoller drain failed: %s", exc)
        return
    if not raw_samples:
        if dropped:
            logger.info("ConsoleTelemetryPoller dropped %d samples in firmware", dropped)
        return

    host_ts = time.time()
    samples: List[PdcSample] = []
    for i, (frame_idx, pdc_raw, flags) in enumerate(raw_samples):
        sample = PdcSample.from_raw(
            frame_idx=frame_idx,
            pdc_raw=pdc_raw,
            flags=flags,
            host_recv_timestamp=host_ts,
            dropped_delta=dropped if i == 0 else 0,
        )
        samples.append(sample)

    listeners: List[Callable] = []
    with self._lock:
        self._last_pdc = samples[-1]
        listeners = list(self._pdc_listeners)

    for sample in samples:
        for fn in listeners:
            try:
                fn(sample)
            except Exception as exc:
                logger.error("ConsoleTelemetry pdc listener raised: %s", exc)
```

(g) Add a new `_refresh_slow()` method that calls the existing `_read_all()` (unchanged) and then handles the snapshot store + listener fanout that used to live inline in `_poll_loop`:

```python
def _refresh_slow(self) -> None:
    snap = self._read_all()
    listeners: List[Callable] = []
    with self._lock:
        self._snapshot = snap
        listeners = list(self._listeners)
    for fn in listeners:
        try:
            fn(snap)
        except Exception as exc:
            logger.error("ConsoleTelemetry listener raised: %s", exc)
```

`_read_all` keeps its current implementation; its sole caller is now `_refresh_slow`. Delete the `self._snapshot = snap` and listener loop currently embedded in `_poll_loop` around lines 213-223 — that logic now lives in `_refresh_slow`.

- [ ] **Step 4: Run tests**

```
python -m pytest tests/test_console_telemetry_unit.py -v -k pdc
```

Expected: 3 PDC tests pass; existing safety tests still pass.

- [ ] **Step 5: Commit**

```
git add omotion/ConsoleTelemetry.py tests/test_console_telemetry_unit.py
git commit -m "feat(sdk): poller drains PDC at 10 Hz, slow refresh at 1 Hz"
```

---

### Task SDK-5: Remove direct PDC I2C read; derive `snap.pdc` from `_last_pdc`

**Files:**
- Modify: `omotion/ConsoleTelemetry.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_console_telemetry_unit.py`:

```python
def test_refresh_slow_does_not_issue_pdc_i2c_read():
    """The dedicated PDC read in _read_analog must be gone; pdc comes from _last_pdc."""
    console = _FakeConsoleForDrain([(0, [(99, 50, 0x00)])])
    # Track I2C reads issued during slow refresh
    real_read = console.read_i2c_packet
    seen_regs: list[tuple] = []
    def spy(mux_index, channel, device_addr, reg_addr, read_len):
        seen_regs.append((mux_index, channel, device_addr, reg_addr, read_len))
        return real_read(mux_index, channel, device_addr, reg_addr, read_len)
    console.read_i2c_packet = spy

    poller = ConsoleTelemetryPoller(console)
    poller._tick_once()      # drains PDC, runs slow refresh (slow_phase starts at 0)

    # Reg 0x1C (PDC) must NOT appear in the I2C reads during slow refresh.
    assert (1, 7, 0x41, 0x1C, 2) not in seen_regs
    # The snapshot's pdc field should be derived from the drained PdcSample.
    snap = poller.get_snapshot()
    assert snap is not None
    assert snap.pdc == 50 * 1.9


def test_snap_pdc_zero_before_any_drain():
    console = _FakeConsoleForDrain([(0, [])])
    poller = ConsoleTelemetryPoller(console)
    poller._tick_once()
    snap = poller.get_snapshot()
    assert snap.pdc == 0.0
```

- [ ] **Step 2: Run, expect failure**

```
python -m pytest tests/test_console_telemetry_unit.py::test_refresh_slow_does_not_issue_pdc_i2c_read -v
```

Expected: assertion failure — the PDC I2C read is still being issued.

- [ ] **Step 3: Modify `_read_analog`**

In `omotion/ConsoleTelemetry.py`, replace the body of `_read_analog` (around lines 346-378):

```python
def _read_analog(self, snap: ConsoleTelemetry) -> None:
    """Refresh slow analog telemetry: lsync, tcl. PDC is now derived from
    the most recent PdcSample drained from the firmware ring buffer."""
    lsync = self._console.get_lsync_pulsecount()
    snap.tcm = int(lsync) if lsync is not None else 0

    tcl_raw, _ = self._console.read_i2c_packet(
        mux_index=_MUX_IDX,
        channel=_TCL_CHANNEL,
        device_addr=_I2C_ADDR,
        reg_addr=_TCL_REG,
        read_len=_TCL_LEN,
    )
    if not tcl_raw:
        logger.debug("Analog I2C tcl read (channel %d) returned no data", _TCL_CHANNEL)
    else:
        snap.tcl = int.from_bytes(tcl_raw[:_TCL_LEN], byteorder="little")

    # pdc is now sourced from the high-rate PdcSample stream
    last_pdc = self._last_pdc
    snap.pdc = last_pdc.pdc_mA if last_pdc is not None else 0.0
```

- [ ] **Step 4: Run tests**

```
python -m pytest tests/test_console_telemetry_unit.py -v
```

Expected: all pass.

- [ ] **Step 5: Commit**

```
git add omotion/ConsoleTelemetry.py tests/test_console_telemetry_unit.py
git commit -m "feat(sdk): drop dedicated PDC I2C read; derive from PdcSample"
```

---

### Task SDK-6: Per-frame telemetry CSV

**Files:**
- Modify: `omotion/ScanWorkflow.py`
- Create: `tests/test_telemetry_csv_per_frame.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_telemetry_csv_per_frame.py`:

```python
import csv
import io

from omotion.ConsoleTelemetry import ConsoleTelemetry, PdcSample
from omotion.ScanWorkflow import _TELEMETRY_HEADERS, _pdc_row


def test_headers_include_new_columns_in_appended_order():
    assert _TELEMETRY_HEADERS[-5:] == [
        "frame_idx", "dark_slot", "pdc_flags", "pdc_dropped_delta", "slow_age_ms"
    ]
    # Existing columns preserved (subset spot-check).
    assert _TELEMETRY_HEADERS[0] == "timestamp"
    assert _TELEMETRY_HEADERS[1:4] == ["tcm", "tcl", "pdc"]


def test_pdc_row_with_fresh_snapshot():
    snap = ConsoleTelemetry()
    snap.timestamp = 1000.0
    snap.tcm = 555
    snap.tcl = 444
    snap.pdc = 12.34
    snap.tec_v_raw = 1.1
    snap.tec_set_raw = 1.2
    snap.tec_curr_raw = 1.3
    snap.tec_volt_raw = 1.4
    snap.tec_good = True
    snap.pdu_raws = list(range(16))
    snap.pdu_volts = [float(i) * 0.1 for i in range(16)]
    snap.safety_se = 0x00
    snap.safety_so = 0x00
    snap.safety_ok = True
    snap.read_ok = True
    snap.error = None

    sample = PdcSample(
        frame_idx=601,
        pdc_mA=15.0,
        dark_slot=True,
        host_recv_timestamp=1001.025,
        dropped_delta=0,
    )

    row = _pdc_row(sample, snap)
    assert len(row) == len(_TELEMETRY_HEADERS)
    d = dict(zip(_TELEMETRY_HEADERS, row))
    assert d["timestamp"] == 1001.025
    assert d["tcm"] == 601           # tcm == frame_idx for PDC-driven rows
    assert d["tcl"] == 444           # carry-forward from snapshot
    assert d["pdc"] == 15.0          # this row's per-frame value
    assert d["dark_slot"] == 1
    assert d["pdc_flags"] == 1       # bit 0 set
    assert d["pdc_dropped_delta"] == 0
    assert d["slow_age_ms"] == 1025  # 1001.025 - 1000.0 = 1.025 s
    assert d["frame_idx"] == 601


def test_pdc_row_before_first_slow_snapshot_leaves_slow_cols_blank():
    sample = PdcSample(frame_idx=1, pdc_mA=20.0, dark_slot=False,
                       host_recv_timestamp=10.0, dropped_delta=0)
    row = _pdc_row(sample, snap=None)
    d = dict(zip(_TELEMETRY_HEADERS, row))
    assert d["timestamp"] == 10.0
    assert d["tcm"] == 1
    assert d["tcl"] == ""
    assert d["pdc"] == 20.0
    assert d["tec_v_raw"] == ""
    assert d["pdu_raw_0"] == ""
    assert d["safety_se"] == ""
    assert d["dark_slot"] == 0
    assert d["slow_age_ms"] == ""


def test_pdc_row_round_trips_through_csv_writer():
    snap = ConsoleTelemetry()
    snap.timestamp = 100.0
    sample = PdcSample(frame_idx=42, pdc_mA=1.9, dark_slot=False,
                       host_recv_timestamp=100.05, dropped_delta=3)
    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(_TELEMETRY_HEADERS)
    w.writerow(_pdc_row(sample, snap))

    buf.seek(0)
    rdr = csv.DictReader(buf)
    rows = list(rdr)
    assert len(rows) == 1
    assert rows[0]["frame_idx"] == "42"
    assert rows[0]["pdc"] == "1.9"
    assert rows[0]["pdc_dropped_delta"] == "3"
```

- [ ] **Step 2: Run, expect failure**

```
python -m pytest tests/test_telemetry_csv_per_frame.py -v
```

Expected: `ImportError: cannot import name '_pdc_row'` and assertion failure on header tail.

- [ ] **Step 3: Update the headers + write `_pdc_row`**

In `omotion/ScanWorkflow.py`, replace the `_TELEMETRY_HEADERS` definition (around lines 39-47):

```python
_TELEMETRY_HEADERS: list[str] = [
    "timestamp",
    "tcm", "tcl", "pdc",
    "tec_v_raw", "tec_set_raw", "tec_curr_raw", "tec_volt_raw", "tec_good",
    *[f"pdu_raw_{i}" for i in range(16)],
    *[f"pdu_volt_{i}" for i in range(16)],
    "safety_se", "safety_so", "safety_ok",
    "read_ok", "error",
    # New per-frame columns (appended for backwards compatibility).
    "frame_idx", "dark_slot", "pdc_flags", "pdc_dropped_delta", "slow_age_ms",
]
```

Replace `_snap_to_row` with `_pdc_row` (same location, around line 50):

```python
def _pdc_row(sample, snap) -> list:
    """Build one telemetry-CSV row keyed by a PdcSample.

    Slow columns come from the last ConsoleTelemetry snapshot (carry-forward).
    Pass snap=None before the first slow refresh has landed — slow columns will
    be empty strings.
    """
    row: list = [
        sample.host_recv_timestamp,            # timestamp
        sample.frame_idx,                      # tcm (== frame_idx for PDC-driven rows)
    ]
    if snap is not None:
        row.append(snap.tcl)                   # tcl carry-forward
    else:
        row.append("")
    row.append(sample.pdc_mA)                  # pdc — this row's per-frame value

    if snap is not None:
        row.extend([
            snap.tec_v_raw, snap.tec_set_raw, snap.tec_curr_raw,
            snap.tec_volt_raw, int(snap.tec_good),
        ])
        pdu_raws = snap.pdu_raws or []
        pdu_volts = snap.pdu_volts or []
        for i in range(16):
            row.append(pdu_raws[i] if i < len(pdu_raws) else "")
        for i in range(16):
            row.append(pdu_volts[i] if i < len(pdu_volts) else "")
        row.extend([
            snap.safety_se, snap.safety_so, int(snap.safety_ok),
            int(snap.read_ok), snap.error or "",
        ])
        slow_age_ms = int((sample.host_recv_timestamp - snap.timestamp) * 1000)
    else:
        # 5 TEC + 16 pdu_raw + 16 pdu_volt + 3 safety + 2 health = 42 blanks
        row.extend([""] * 42)
        slow_age_ms = ""

    row.extend([
        sample.frame_idx,                      # frame_idx (explicit duplicate of tcm)
        1 if sample.dark_slot else 0,
        1 if sample.dark_slot else 0,          # pdc_flags bit 0
        sample.dropped_delta,
        slow_age_ms,
    ])
    return row
```

- [ ] **Step 4: Run tests**

```
python -m pytest tests/test_telemetry_csv_per_frame.py -v
```

Expected: 3 tests pass.

- [ ] **Step 5: Commit**

```
git add omotion/ScanWorkflow.py tests/test_telemetry_csv_per_frame.py
git commit -m "feat(sdk): per-frame telemetry CSV row builder"
```

---

### Task SDK-7: Switch `ScanWorkflow` to the PDC listener

**Files:**
- Modify: `omotion/ScanWorkflow.py`

- [ ] **Step 1: Update the listener wiring**

In `omotion/ScanWorkflow.py`, replace the existing telemetry-CSV listener block (around lines 365-392).

Find:

```python
                if _telem_poller is not None and request.write_telemetry_csv:
                    try:
                        _telem_fh = open(  # noqa: WPS515
                            telemetry_path, "w", newline="", encoding="utf-8"
                        )
                        _telem_csv = csv.writer(_telem_fh)
                        _telem_csv.writerow(_TELEMETRY_HEADERS)
                        _telem_fh.flush()

                        def _on_telemetry(snap):
                            if _telem_stop.is_set():
                                return
                            with _telem_lock:
                                if _telem_stop.is_set():
                                    return
                                try:
                                    _telem_csv.writerow(_snap_to_row(snap))
                                    _telem_fh.flush()
                                except Exception as _te:
                                    logger.debug("Telemetry CSV write error: %s", _te)

                        _telem_listener = _on_telemetry
                        _telem_poller.add_listener(_telem_listener)
```

Replace with:

```python
                if _telem_poller is not None and request.write_telemetry_csv:
                    try:
                        _telem_fh = open(  # noqa: WPS515
                            telemetry_path, "w", newline="", encoding="utf-8"
                        )
                        _telem_csv = csv.writer(_telem_fh)
                        _telem_csv.writerow(_TELEMETRY_HEADERS)
                        _telem_fh.flush()

                        def _on_pdc(sample):
                            if _telem_stop.is_set():
                                return
                            with _telem_lock:
                                if _telem_stop.is_set():
                                    return
                                try:
                                    snap = _telem_poller.get_snapshot()
                                    _telem_csv.writerow(_pdc_row(sample, snap))
                                    _telem_fh.flush()
                                except Exception as _te:
                                    logger.debug("Telemetry CSV write error: %s", _te)

                        _telem_listener = _on_pdc
                        _telem_poller.add_pdc_listener(_telem_listener)
```

- [ ] **Step 2: Update the teardown to call `remove_pdc_listener`**

Search `ScanWorkflow.py` for `remove_listener(_telem_listener)`. Replace each occurrence with:

```python
_telem_poller.remove_pdc_listener(_telem_listener)
```

- [ ] **Step 3: Run the existing scan-workflow test suite**

```
python -m pytest tests/ -v -k "scan or workflow or telemetry"
```

Expected: all currently-passing tests continue to pass. Any test that explicitly fed a `ConsoleTelemetry` snapshot into the workflow's telemetry path will need a small update — fix in place: replace `add_listener` references with `add_pdc_listener` and `ConsoleTelemetry(...)` snapshot factories with `PdcSample(...)` constructors.

- [ ] **Step 4: Commit**

```
git add omotion/ScanWorkflow.py tests/
git commit -m "feat(sdk): wire telemetry CSV writer to pdc_listener"
```

---

## Integration tasks

### Task INT-1: Verify slot tracking on hardware

**Goal:** Confirm the FW `current_slot_is_dark` bit actually matches the LASER_TIMER cycle that fires immediately after the FSYNC ISR sets it. This is the open-question off-by-one flagged in the spec.

- [ ] **Step 1: Flash the new firmware**

Connect the console board over ST-Link.

```
cd C:/Users/ethan/Projects/openmotion-console-fw
cmake --build build --target flash    # or use the existing flash workflow
```

- [ ] **Step 2: Run a 30-second scan via the SDK**

```
cd C:/Users/ethan/Projects/openmotion-sdk
python -c "
from omotion import MotionInterface
import time
iface = MotionInterface()
iface.connect()
time.sleep(2)
iface.start_scan(...)  # use the existing scan entry-point with subject_id='pdc-int-test', duration_sec=30, both masks 0xFF
"
```

(Use the existing engineering test app, `openmotion-test-app/main.py`, if it's easier than scripting.)

- [ ] **Step 3: Open the telemetry CSV and the raw histogram CSV**

```
python -c "
import pandas as pd
tel = pd.read_csv('scan_data/<the_telemetry>.csv')
print(tel.head(15))
print('dark rows:', tel['dark_slot'].sum(), '/', len(tel))
print('first dark frame_idx values:', tel.loc[tel.dark_slot == 1, 'frame_idx'].head(5).tolist())
"
```

- [ ] **Step 4: Confirm dark rows match the science pipeline's schedule**

With defaults (`discard_count=9`, `dark_interval=600`), the science pipeline expects dark frames at `n = 10, 601, 1201, ...`. Expected outcome: `tel.dark_slot == 1` rows have `frame_idx` matching that schedule (allowing for `lsync_counter`'s 1-based reset).

If the dark rows are systematically off by exactly one frame (e.g. `frame_idx = 11, 602, 1202, ...`), the spec's open question is confirmed: the slot-bit update in `FSYNC_PeriodElapsedCallback` lags the LASER_TIMER cycle it gates by one frame. Apply the one-line fix in `trigger.c`'s `LSYNC_PeriodElapsedCallback`: snapshot the **previous** slot decision by adding a second variable:

```c
static volatile bool s_prev_slot_is_dark = false;
// In FSYNC_PeriodElapsedCallback, before the dark = ... line:
s_prev_slot_is_dark = s_current_slot_is_dark;
// Then in LSYNC_PeriodElapsedCallback:
s_pdc_sample_dark = s_prev_slot_is_dark;   // not s_current_slot_is_dark
```

Re-flash, re-test, confirm dark rows now align.

- [ ] **Step 5: Commit any fix**

```
git add Core/Src/trigger.c
git commit -m "fix(fw): use previous slot decision for PDC dark_slot tag"
```

(Skip if no off-by-one found.)

---

### Task INT-2: Dark-slot consistency diagnostic script

**Files:**
- Create: `scripts/check_dark_slot_consistency.py`

- [ ] **Step 1: Write the script**

```python
#!/usr/bin/env python3
"""Compare a telemetry CSV's dark_slot column against the science pipeline's
predicted dark-frame schedule.

Usage:
    python scripts/check_dark_slot_consistency.py <telemetry.csv> \
        [--discard-count 9] [--dark-interval 600]

Reports:
    - Total rows
    - Rows where firmware dark_slot disagrees with the predicted schedule
    - First 10 mismatched frame_idx values (if any)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd


def predicted_dark(n: int, discard_count: int, dark_interval: int) -> bool:
    if n <= discard_count:
        return False
    if n == discard_count + 1:
        return True
    return (n - 1) % dark_interval == 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("csv", type=Path)
    p.add_argument("--discard-count", type=int, default=9)
    p.add_argument("--dark-interval", type=int, default=600)
    args = p.parse_args()

    df = pd.read_csv(args.csv)
    required = {"frame_idx", "dark_slot"}
    if not required.issubset(df.columns):
        print(f"ERROR: CSV missing columns {required - set(df.columns)}", file=sys.stderr)
        return 2

    df = df[df["frame_idx"].notna()].copy()
    df["predicted_dark"] = df["frame_idx"].astype(int).apply(
        lambda n: predicted_dark(n, args.discard_count, args.dark_interval)
    )
    df["fw_dark"] = df["dark_slot"].astype(int).astype(bool)
    df["mismatch"] = df["fw_dark"] != df["predicted_dark"]

    total = len(df)
    n_fw_dark = int(df["fw_dark"].sum())
    n_pred_dark = int(df["predicted_dark"].sum())
    n_mismatch = int(df["mismatch"].sum())

    print(f"Rows analyzed:        {total}")
    print(f"FW dark_slot count:   {n_fw_dark}")
    print(f"Predicted dark count: {n_pred_dark}")
    print(f"Mismatches:           {n_mismatch}")

    if n_mismatch:
        print("\nFirst 10 mismatched frame_idx values:")
        bad = df.loc[df["mismatch"], ["frame_idx", "fw_dark", "predicted_dark"]].head(10)
        print(bad.to_string(index=False))
        return 1
    print("\nOK — firmware and science-pipeline dark-frame schedule agree.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run against a real scan CSV**

```
python scripts/check_dark_slot_consistency.py scan_data/<the_telemetry>.csv
```

Expected: `OK — firmware and science-pipeline dark-frame schedule agree.` If mismatches, return to Task INT-1.

- [ ] **Step 3: Commit**

```
git add scripts/check_dark_slot_consistency.py
git commit -m "feat(sdk): dark_slot consistency diagnostic script"
```

---

### Task INT-3: 60-second integration scan

- [ ] **Step 1: Run a clean scan**

Use `openmotion-test-app` or a small script to run a 60 s scan with both masks active.

- [ ] **Step 2: Verify row count and continuity**

```
python -c "
import pandas as pd
df = pd.read_csv('scan_data/<the_telemetry>.csv')
print('rows:', len(df))
print('frame_idx min/max:', df['frame_idx'].min(), df['frame_idx'].max())
print('gaps:', int((df['frame_idx'].diff().dropna() > 1).sum()))
print('pdc_dropped_delta total:', df['pdc_dropped_delta'].sum())
print('max slow_age_ms (post first 40 rows):', df['slow_age_ms'].iloc[40:].max())
"
```

Acceptance:

- `rows ≈ 60 * 40 = 2400` (±5%).
- `frame_idx` strictly monotonic, no gaps after the first second.
- `pdc_dropped_delta` totals zero in steady state.
- `slow_age_ms ≤ 1200` for every row after row 40 (the first slow tick has landed).

- [ ] **Step 2: Run the diagnostic**

```
python scripts/check_dark_slot_consistency.py scan_data/<the_telemetry>.csv
```

Expected: OK.

- [ ] **Step 3: Final commit + PRs**

```
# SDK
cd C:/Users/ethan/Projects/openmotion-sdk
git push -u origin feature/per-frame-pdc
gh pr create --base next --title "Per-frame PDC telemetry" --body-file docs/superpowers/specs/2026-05-20-per-frame-pdc-telemetry-design.md

# FW
cd C:/Users/ethan/Projects/openmotion-console-fw
git push -u origin feature/per-frame-pdc
gh pr create --base next --title "Per-frame PDC ring buffer + drain opcode" --body "Implements firmware side of openmotion-sdk spec 2026-05-20-per-frame-pdc-telemetry-design.md"
```

---

## Self-review checklist (run before handing off)

- [ ] **Spec coverage:** Every section of `docs/superpowers/specs/2026-05-20-per-frame-pdc-telemetry-design.md` maps to a task above:
  - Background / FPGA semantics → FW-3 (settle delay), FW-2 (falling-edge ISR)
  - Console FW architecture → FW-1 (buffer), FW-2 (slot/ISR), FW-3 (poll), FW-4 (wire), FW-5 (opcode)
  - SDK changes → SDK-1 (constant), SDK-2 (drain client), SDK-3 (dataclass), SDK-4 (poller), SDK-5 (drop direct read), SDK-6 (CSV row), SDK-7 (wire)
  - CSV writer → SDK-6, SDK-7
  - Data flow → covered structurally across FW-2/3, SDK-4, SDK-7
  - Error handling → drain-fail (SDK-4 logs WARN, doesn't flip read_ok), FW I2C-fail (FW-3 logs sparsely), overflow (FW-1 tested), disconnect (existing exception path in poller untouched)
  - Backward compatibility → existing `add_listener` API preserved (SDK-4 keeps `_listeners` + `_refresh_slow`); CSV column order preserved (SDK-6 appends)
  - Testing → FW-1 host test + SDK unit tests for each module + INT-1/INT-3 hardware
  - "Two sources of dark-frame truth" cross-check → INT-2 diagnostic
  - Open questions → INT-1 step 4 explicitly verifies + fixes the slot off-by-one

- [ ] **Placeholders:** Searched plan for TODO, TBD, "implement later" — none present.

- [ ] **Type/name consistency:**
  - C struct `pdc_sample_t` defined in FW-1, used in FW-3 (push), FW-5 (drain response), and parsed identically in SDK-2 (`<IHB` = 4 + 2 + 1 = 7 bytes).
  - Flag bit constant `PDC_FLAG_DARK_SLOT = 1u << 0` matches Python `PDC_FLAG_DARK_SLOT = 1 << 0` (SDK-3).
  - `PdcSample` field names (`frame_idx`, `pdc_mA`, `dark_slot`, `host_recv_timestamp`, `dropped_delta`) used consistently across SDK-3, SDK-4, SDK-6, SDK-7.
  - `_TELEMETRY_HEADERS` order in SDK-6 matches `_pdc_row` element count: 4 fixed (`timestamp, tcm, tcl, pdc`) + 5 TEC + 16 pdu_raw + 16 pdu_volt + 3 safety + 2 health + 5 new = **51 columns**. The snap=None branch fills 42 blanks (5+16+16+3+2) between the `pdc` column and the trailing 5 new columns. Verified by the round-trip test in SDK-6.
  - `add_pdc_listener` / `remove_pdc_listener` / `get_last_pdc_sample` used identically across SDK-4 (defined) and SDK-7 (consumed).
