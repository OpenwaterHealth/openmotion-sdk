# Disconnect / Reconnect Hardening + Simplification — Design

**Date:** 2026-06-13
**Status:** Approved (design); implementation plan pending
**Scope:** `openmotion-sdk` (primary) + a firmware watchdog spec for `openmotion-console-fw` (companion doc).

## Problem

The SDK behaves badly when hardware is disconnected/reconnected during operation:

1. **Laser keeps firing** when USB is yanked mid-scan.
2. The system **"gets confused" on replug** — it does not reliably come back.

The goal is to **harden and simplify** the autoconnect behavior and the SDK
tooling around it, for stability and ease of use.

## Characterization (empirical, on bench YK27987 + Shelly 192.168.1.81)

All three devices (console on COM12 / USB `(1,1,1)`, left sensor `(1,1,2)`,
right sensor `(1,1,3)`) hang off a **single downstream hub plugged into YKUSH
port 1**. YKUSH ports 2 and 3 are empty. The DUT is **mains-powered through the
Shelly**; USB is the data link. So:

- **YKUSH port 1** = cut the entire system's USB at once (data + VBUS), but the
  console keeps running on mains. This is the faithful "USB unplugged while the
  device is still powered" scenario.
- **Shelly** = full mains power cycle (kills everything, including the laser).
- The current cabling **cannot isolate** "console drops, sensors stay." (See
  *Test plan → cabling*.)

Three distinct failure modes were confirmed:

### A. Reconnect wedge — **SDK bug** (the "gets confused" symptom)

- `ConnectionMonitor` is a **single thread** that runs each handle's connect
  sequence **synchronously and inline** (`_handle_event` → `_drive_connecting`).
- The console connect ping calls `MotionUart.send_packet()` with the **default
  20 s timeout** (the sensor path correctly passes `timeout=2.0`), and
  `MotionUart.read_packet` is **uninterruptible**.
- Result: when a freshly re-enumerated console does not answer the first ping,
  the monitor thread **freezes for ~20 s**, and the sensors cannot reconnect
  because the one thread that connects them is blocked. Observed recovery after
  a USB restore ranged from **0/3 to 2/3** devices — **non-deterministic**.
- A retry storm (connect exhausts → next poll immediately re-submits
  `PollArrived` → immediate retry, no inter-cycle backoff) spams logs and
  re-monopolizes the thread.
- **The hardware is fine throughout.** With no SDK running, USB re-enumerates in
  1–2 s and a direct console ping responds in **0.10 s**. A fresh SDK process (or
  a mains cycle) only "fixes" it by sidestepping the wedged monitor thread.

### B. Laser keeps firing — **firmware gap** (the safety one)

- Confirmed `TriggerStatus=2` (laser ON), then cut USB. The host **cannot send
  `stop_trigger`** to a disconnected console, and firmware has **no host-loss
  watchdog** to stop itself.
- Worse: cutting USB **while the trigger is running** **wedges the console
  command interface** — it re-enumerates (COM port returns) but **never answers
  a command again** within the observation window. Recovery required a **mains
  power cycle** every time. (`get_trigger_json`'s `TriggerStatus` field is a
  reliable live observable: `2` = laser ON, `1` = OFF.)

### C. No force-safe on reconnect — **SDK gap**

- The console connect-on-entry does ping + telemetry but **never resets the
  device to a known-safe state** (never `stop_trigger`). A trigger left running
  survives a reconnect even when the console *is* reachable.

## Goals / non-goals

**Goals**
- A slow or unresponsive device must **never** block another device's connect.
- Reconnect after any USB or mains disruption must be **deterministic** and
  bounded in time.
- Every (re)connect leaves the console in a **known laser-OFF** state.
- Reduce log spam and connect-retry churn.
- Provide **repeatable HIL tests** that exercise the real disconnect paths.

**Non-goals**
- Implementing the firmware watchdog (spec only, see companion doc).
- Per-device-drop test isolation under the *current* cabling (documented as a
  recommended re-cabling).
- Demo-mode end-to-end scan streaming (unchanged).

## Design — SDK

### Core principle

**`ConnectionMonitor` only ever does fast, non-blocking work.** All blocking
device I/O (serial/USB open, ping, ID-cache read, close, `stop_trigger`) moves
onto **per-handle worker threads**. This is the architectural simplification:
the monitor becomes a pure *detector + router*; each handle *owns* its own
connect lifecycle.

### Component 1 — Per-handle connect worker

Each handle (`MotionConsole`, `MotionSensor` ×2) gets **one dedicated daemon
thread** with a tiny intent mailbox (`queue.Queue` of `CONNECT` / `DISCONNECT`).

- The monitor's dispatch path (`_handle_event`, today on the monitor thread)
  no longer calls `_drive_connecting` / `_drive_disconnecting` directly. It
  computes the desired target from the event and **posts an intent**, then
  returns immediately.
- The worker thread consumes intents **serially**, running the existing
  connect/teardown sequences. Exactly **one worker per handle** preserves the
  current invariant — *a handle's state machine is never re-entered* — but now
  per-handle, so one slow connect cannot starve the others.
- The state machine itself is unchanged: `_set_state`, `signal_state_changed`,
  the `ConnectionState` enum, and the `_state_cv` all stay. We only move *where*
  `_drive_*` executes (off the monitor thread, onto the handle's worker).

**Intent coalescing / cancellation.** Before and during a connect attempt the
worker checks whether a newer intent supersedes it (device vanished again →
`DISCONNECT`). A per-handle `threading.Event` ("abort current attempt") is set
when a superseding intent arrives; the bounded read loop polls it so a connect
in its retry/backoff aborts promptly and tears down cleanly.

### Component 2 — Bounded, interruptible UART reads

- `MotionUart.read_packet` gains an optional `cancel_evt: threading.Event` and
  honors a real bounded timeout, mirroring the `_transport_down_evt` pattern
  that already exists in `CommInterface` for the USB path. The connect worker
  passes its abort event.
- The console connect **ping uses `timeout≈2.0`** for parity with the sensor
  path.
- Net effect: no console connect attempt can block more than ~2 s, and it
  aborts instantly on a superseding disconnect. This eliminates the 20 s
  monitor-thread freeze.

### Component 3 — Force-safe on console connect

- Immediately after the console reaches `CONNECTED` (ping OK), the worker calls
  `self.stop_trigger()` (idempotent, bounded). Every (re)connect lands the
  device in a known laser-OFF state. This is the SDK-side mitigation for "laser
  still armed after replug" whenever the console *is* reachable.
- `stop_trigger` failure → log `WARNING`, **stay `CONNECTED`** (the device is
  reachable; a failed safe-off is surfaced, not fatal). Logged at INFO on
  success.
- Decision (approved): **always** force-safe on connect, not conditional on SDK
  scan state — SDK scan state may be stale after a disconnect, and a mid-scan
  disconnect already aborts the scan, so this is consistent.

### Component 4 — Retry-storm collapse

- On a failed connect the worker applies a short **cooldown** (~1 s) before it
  is eligible to retry. The monitor will not re-post `CONNECT` while a worker is
  busy or cooling down (extend the existing `in_progress` guard, which today
  covers only `CONNECTING`/`DISCONNECTING`, to also cover "worker busy /
  cooldown").
- Collapses the tight exhaust→re-poll→retry loop and the associated log spam.
  A device that is present but failing connect retries on a calm cadence rather
  than a spin.

### Data flow (new)

```
topology change
  → hotplug OR 200 ms poll sweep              (monitor thread)
  → diff presence vs handle.state
  → post CONNECT/DISCONNECT intent to handle worker   [non-blocking]
  → monitor immediately continues servicing other handles
handle worker thread:
  → bounded / interruptible connect or teardown
  → _set_state(...) → emits signal_state_changed
```

### Error handling

| Situation | Behavior |
|---|---|
| Connect attempt times out / fails | → `DISCONNECTED` + cooldown; monitor re-detects presence and re-posts `CONNECT` after cooldown |
| Device vanishes mid-connect | superseding `DISCONNECT` intent trips the abort event → bounded read returns → worker tears down cleanly |
| `stop_trigger` fails on connect | `WARNING`, remain `CONNECTED` |
| One handle's connect is slow | other handles' workers run concurrently — **no starvation** |

## Design — Firmware (companion spec)

Written separately as
`docs/superpowers/specs/2026-06-13-console-fw-host-loss-watchdog-spec.md`,
targeted at the `openmotion-console-fw` repo:

1. **Host-loss watchdog.** While the trigger is running, if no host
   command/heartbeat arrives within `N` ms, firmware auto-executes
   `stop_trigger` (laser off). `N` tunable; longer than normal inter-command
   gaps, short enough to be eye-safe. The only true fix for failure mode **B**.
2. **Command-interface wedge.** Investigate + fix the CDC/command parser
   becoming unresponsive after a USB-cut-while-triggering (currently recoverable
   only by mains cycle). Likely a USB suspend/disconnect path that leaves the RX
   state machine / DMA / ISR stuck; on disconnect-detect, reset the
   command-interface RX so re-enumeration restores command handling without a
   power cycle.
3. **Heartbeat protocol** definition: dedicated opcode vs. piggybacking on
   existing telemetry polls.

## Test plan (HIL + software)

### Lifecycle × transport disconnect matrix (acceptance requirement)

The work is not "done" until **both communication links are cut at every stage
of the app lifecycle and shown to be handled correctly.** The two links are:

- **Console UART** — the CDC virtual COM port (`OW_*` command/telemetry traffic).
- **Sensor USB-bulk** — the 3-interface composite link (command + histo + IMU),
  left and/or right.

Lifecycle stages to exercise:

| # | Stage | What is happening |
|---|---|---|
| L0 | Pre-start / discovery | before/while `MotionInterface.start()` enumerates |
| L1 | Mid-`CONNECTING` | cut during a connect attempt (ping / id-cache window) |
| L2 | `CONNECTED` idle | connected, no scan |
| L3 | Scan bring-up | configure: FPGA/camera config, before the laser arms |
| L4 | Mid-scan | trigger running (laser ON) + histogram/IMU streaming |
| L5 | Scan teardown | during `stop_trigger` → `disable_camera` → `close` |
| L6 | Calibration workflow | a `CalibrationWorkflow` run in flight |
| L7 | Rapid repeated cuts | stress / debounce |

Transport dimension for each stage: **console-only**, **sensor-only**
(left, right, or both), **both simultaneously**.

**"Handled correctly" — pass criteria for every cell:**

- **Laser safety:** no laser left firing that the SDK *could* have prevented.
  Where the console is reachable, `TriggerStatus == 1` after handling. Where the
  console link itself is cut (host cannot command it), this is explicitly the
  firmware-watchdog case — assert the SDK does the right thing on its side
  (aborts, force-safes on reconnect) and document the residual firmware gap.
- **No wedge / no starvation:** the monitor thread never freezes; a drop on one
  transport must not stall the other handle's connect (verifiable only with
  independent cuts — see *Cabling*).
- **No worker hang:** scan worker, configure worker, and calibration worker all
  unwind within a bounded time (no 60 s / 20 s hangs).
- **Deterministic recovery:** every still-present device returns to `CONNECTED`
  within a bounded time (target `< ~8 s`); a subsequent fresh scan starts
  cleanly (READY checks pass).

### Hardware-in-the-loop tests

New `tests/test_connection_recovery.py` (markers: `console`, `sensor`, `slow`;
power-control gated, skips cleanly when YKUSH/Shelly absent). Drives the matrix
above. Concrete anchor cases:

1. **System cut + restore** (YKUSH port 1, = both links): all three return to
   `CONNECTED` `< ~8 s`, repeated ≥3× for determinism.
2. **Console-only cut** at L2/L4 (requires re-cabling): sensors stay
   `CONNECTED`; only console drops and recovers; mid-scan abort fires.
3. **Sensor-only cut** at L2/L4 (requires re-cabling): console stays
   `CONNECTED` (and force-safes the trigger); only the cut sensor recovers.
4. **Mid-scan (L4) both-link cut**: scan aborts, no worker hang, full recovery,
   fresh scan afterwards succeeds.
5. **Repeated / rapid cut-restore** (L7): no permanent wedge.
6. **Mains cycle** (Shelly) + restore: all three reconnect.
7. **Force-safe**: trigger armed, reconnect driven → `TriggerStatus == 1`.

### Software-only tests (runs with `-m "not console and not sensor"`)

8. **Connect-worker state machine** with fake transports: connect success,
   connect failure → cooldown → retry, disconnect-mid-connect abort.
9. **Non-blocking monitor**: a fake handle whose connect blocks must not delay
   another handle's connect — the regression test for failure mode **A**.

### Cabling

Under the current single-hub-on-YKUSH-port-1 wiring, only **system-wide
(both-link) cuts** are physically possible — the console-only and sensor-only
rows of the matrix (cases 2, 3, and the starvation check) **cannot** be tested.
To cover the full matrix the bench must be **re-cabled** so the console and each
sensor sit on **separate YKUSH ports**. This is a prerequisite for the
independent-transport cells, captured as decision **D1** below.

The scratch characterization harnesses (`scratch/char_connection.py`,
`scratch/char_laser.py`) are the prototypes these tests derive from.

## Decisions

- **D1 — cabling for independent cuts → RESOLVED: combined-only this pass.**
  Keep the current wiring. The redesign is implemented and **every lifecycle
  stage (L0–L7) is tested with both links cut at once** via YKUSH (USB) and via
  the Shelly (mains). The console-only / sensor-only rows of the matrix and the
  cross-handle starvation *hardware* check are **deferred** until the bench is
  re-cabled onto separate YKUSH ports. Note: the cross-handle starvation
  guarantee (failure mode **A**) is still verified in this pass by the
  **software-only non-blocking-monitor test** (case 9), which does not need
  independent hardware cuts.

## Risks / trade-offs

- **More threads** (one connect worker per handle + the monitor + existing read
  threads). Mitigated by: only 3 handles, dedicated single-consumer mailboxes,
  and per-handle serialization that is *simpler* to reason about than today's
  shared-thread-does-everything model.
- **Force-safe on every connect** will stop a trigger that happened to be
  running across a reconnect. Accepted: safety-first, and consistent with
  mid-scan-disconnect already aborting the scan.
- **Cooldown** slightly delays a legitimate fast reconnect (~1 s). Acceptable
  for determinism and to kill the storm.
```
