# Connect/Disconnect Redesign — Manual Hardware Test Plan

**Date:** 2026-04-22
**Companion to:** `2026-04-21-connect-disconnect-redesign-design.md`
**Run against:** `feature/connection-redesign` branches in
`openmotion-sdk`, `openmotion-bloodflow-app`, and `openmotion-test-app`.

These tests exercise the new connection lifecycle on real hardware
(one console + two sensor modules). They cover the four scenarios
called out in the design spec plus a stress test, and validate that
mid-scan recovery (the 5 s grace window) works end-to-end.

## Setup

1. Restart the bloodflow app fresh so the log starts empty.
2. In a terminal, tail the latest log with a filter that hides per-camera
   firmware printf noise:

   ```bash
   tail -f $(ls -t app-logs/ow-bloodflowapp-*.log | head -1) \
     | grep -E 'state |Handle |USB read error|Streaming stopped|Read thread|UART console|MotionComposite|connect attempt|retry|grace|scan: '
   ```

3. Confirm both sensors and the console are CONNECTED in the UI before
   starting any test.

## Test 1 — Idle replug between scans

**Goal:** confirm a sensor unplug-replug between scans is fast and clean.

1. With everything CONNECTED and idle (no scan running), unplug the LEFT
   sensor USB cable.
2. Wait 3 seconds.
3. Plug it back in.
4. Repeat with the RIGHT sensor.
5. Repeat with the CONSOLE.

**Expect per cycle:**

- One WARNING line per sensor read thread (`USB read error errno=32`)
  on unplug for sensors. For console you'll see
  `state CONNECTED → DISCONNECTING (poll_gone)` instead.
- Three INFO state-machine lines: `… DISCONNECTING`, `… closed`,
  `… DISCONNECTED`.
- App UI: the sensor's controls grey out within ~250 ms of physical unplug.
- On replug: `state DISCONNECTED → CONNECTING (poll_arrived)` followed by
  `→ CONNECTED (ping_ok)` within ~3 s (longer if firmware boots slowly).
- App UI: controls re-enable.
- **No fan polls on the other side** during the unplugged cycle.

**Fail signals:** any ERROR line, any Exception, the handle stuck in
CONNECTING or DISCONNECTING for more than ~5 s, app UI not reflecting state.

## Test 2 — Power-cycle the whole enclosure

**Goal:** all three handles independently recover from simultaneous loss.

1. Power off the enclosure (kills console + both sensors at once).
2. Wait 5 seconds.
3. Power back on.

**Expect:**

- All three disconnect events in <50 ms of each other.
- After power-on, all three reach CONNECTED within ~3-5 s of devices
  showing up on USB. The first sensor to attempt connect typically eats
  one ~2 s ping timeout while firmware is still booting; the others
  connect in ~50 ms each.
- Total recovery time visible to the user: power-button-press to
  all-CONNECTED ≈ (firmware boot ~2 s) + (~3 s SDK) ≈ 5-6 s.

**Fail signals:** any handle stuck DISCONNECTED (the next 200 ms poll
should always retry); `connect_retry_exhausted` repeating more than
once or twice in a row.

## Test 3 — Rapid replug stress

**Goal:** state machine debouncing under fast successive events.

1. Pick one sensor cable.
2. Unplug → replug → unplug → replug → unplug → replug, as fast as you
   physically can (sub-second).
3. End with the cable plugged in. Wait 5 seconds.

**Expect:**

- Final state is CONNECTED.
- No "release failed" / "object deleted" warnings anywhere in the log.
- No half-init state (handle stuck in CONNECTING).
- Intermediate DISCONNECTING/CONNECTING transitions show monotonic
  state — no attempts to `_drive_connecting` while already CONNECTING
  (the queue serializes events).

**Fail signals:** handle ends in any state other than CONNECTED with
the cable plugged in. Repeated retry-exhausted entries.

## Test 4 — Mid-scan disconnect aborts immediately

**Goal:** any device dropping mid-scan aborts the scan cleanly with a
specific reason. There is no grace window.

1. Start a scan with both sensors enabled.
2. Wait ~5 s into the scan.
3. Unplug any one of: LEFT sensor / RIGHT sensor / CONSOLE.
4. Replug it (or don't — replug timing doesn't change scan behavior;
   it's just to get the device back for the next test).

**Expect in log:**

- At scan start: three `scan: subscribed to <name> state changes`
  INFO lines (one per participating handle).
- On unplug: `scan: <name> disconnected mid-scan (...); aborting`
  (ERROR).
- `… <name> state CONNECTED → DISCONNECTING → DISCONNECTED`.
- Scan completes with reason `<name> disconnected mid-scan (...)`.
- A *single* clean abort, not a flurry of retries or "release failed"
  warnings.
- After replug, normal CONNECTING → CONNECTED with no scan-related
  messages (scan already finished).

Repeat the test for each of LEFT, RIGHT, CONSOLE to confirm the abort
behavior is symmetric across handle types.

**Fail signals:** scan hangs instead of aborting; multiple abort log
entries; AttributeError or other exception in the worker.

## What success looks like overall

After running all four tests, the log should contain:

- WARNING lines only for the read-thread `USB read error errno=32` on
  unplug.
- ERROR lines only for Test 4's `scan: <name> disconnected mid-scan
  (...); aborting`.
- No `Unexpected error during X`, no `Serial error in send_packet`, no
  `Error getting fan control status`, no `release failed`, no
  `object deleted`, no AttributeErrors.
- Reconnect detection latency under ~250 ms in every case (poll
  fallback) or under ~50 ms (Win32 hotplug path).

If anything fails, capture the relevant log section (with the filter
above) and the rough wall-clock timing of the physical action you
took, and iterate.
