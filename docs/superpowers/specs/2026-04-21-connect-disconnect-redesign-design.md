# Connect/Disconnect Redesign — Design Spec

**Date:** 2026-04-21 · **Amended:** 2026-04-22
**Status:** Implemented and hardware-verified (see test plan
`2026-04-22-connect-disconnect-redesign-test-plan.md`)
**Scope:** `omotion` SDK (`openmotion-sdk`) and the two consuming apps (`openmotion-bloodflow-app`, `openmotion-test-app`)

## 2026-04-22 amendment — mid-scan recovery dropped

After hardware testing, the **5-second grace window for mid-scan
recovery was removed**. The shipped behavior is: any device dropping
mid-scan aborts the scan immediately with a specific reason. This
spec was written assuming recovery would be supported; the sections
below describe the original design but are annotated where reality
differs. The current behavior is described in the "Mid-scan
disconnect handling" section near the end.

Why dropped:

- The grace-window code added significant complexity (per-handle
  timers, threading races between recovery and abort, re-asserting
  camera streams + console trigger, resuming a writer queue mid-scan).
- The clinical workflow already assumes the user retriggers a fresh
  scan after any interruption, so silent recovery was a developer-
  facing nicety rather than a real product requirement.
- A scan with a gap is statistically suspect for downstream BFI/BVI
  analysis anyway — a clean abort + restart is the safer default.

The state machine and stable-handle parts of the design remain as
described; only the `ScanWorkflow` recovery layer changed.

## Problem

Today, when the console and/or sensors are unplugged and reconnected, the SDK takes a long time to recover and emits a flood of error/warning logs along the way. The current architecture has several specific defects that cause this:

1. **Polling-only reconnect at 1 Hz**, with synchronous `time.sleep(0.5)` blocking inside the asyncio monitor coroutine. Worst-case detection latency is 1–2 s, plus enumeration latency.
2. **Multiple disconnect paths racing.** Read-thread USB error → daemon-thread disconnect callback runs in parallel with the next 1 Hz poll calling disconnect on the same composite. Result: duplicate "Disconnecting LEFT sensor" / "release failed" / "object deleted" warnings.
3. **Reconnect race / half-initialized state.** `DualMotionComposite.connect()` assigns `self.left = MotionComposite(dev)` *before* calling `connect()`. If `set_configuration()` or `claim_interface()` raises (common for ~hundreds of ms after Windows re-enumerates), `self.left` is now a dead, half-initialized object — and the `if not self.left` guard is False forever, so it is never retried until app restart.
4. **Sensor identity by physical port suffix is brittle.** Replug into a different USB port → silently skipped.
5. **Sensor handle replacement.** On reconnect, `interface.sensors["left"]` becomes a *new* `MOTIONSensor` instance. Anything caching the old reference (or signals connected to it) is dead.
6. **No "ready" notion.** `signal_connect` fires the instant USB enumerates; firmware/FPGA may need 50–100 ms more before commands work, so apps that immediately ping/configure see noisy timeouts.
7. **Asyncio coupling.** Reconnect only works if the app drives `start_monitoring` from an asyncio event loop.

The fix touches three layers — the public API (handles + signals), the monitoring layer (thread + hotplug), and the wire layer (race fixes, retry/backoff). Mid-scan disconnect semantics are also formalized: any drop during a scan aborts the scan immediately.

## Goals

- Reconnect detection in tens of milliseconds (event-driven), with a 200 ms poll fallback.
- Zero duplicate disconnect emissions; zero "release failed" / "object deleted" warnings on a normal unplug/replug.
- Stable handle objects that never get replaced — apps cache references safely.
- A single, parameterized state-changed signal (no string descriptors to parse).
- During a scan, any device drop aborts the scan with a clear error
  naming the device that dropped. (Original design called for a 5 s
  grace window with transparent recovery; that was dropped on
  2026-04-22 — see amendment header.)
- No asyncio requirement on the consuming app.

## Non-goals

- Identity by hardware ID. Sensors remain identified by USB port (`port_numbers[-1] == 2 → left`, `== 3 → right`). The clinical workflow has fixed cabling.
- Mid-scan recovery of any kind. The original design specified a 5 s grace window with transparent stream resume; that was dropped on 2026-04-22 in favor of immediate clean abort on any disconnect. See amendment header and the "Mid-scan disconnect handling" section.
- Backwards-compatible `signal_connect` / `signal_disconnect` shims. Both apps are migrated in lockstep with the SDK.
- An asyncio-friendly variant of the monitor. Can be added later if a non-Qt consumer needs it; YAGNI for now.

## Architecture

### Connection state machine

One state machine per device handle. There are exactly three handles for the lifetime of a `MotionInterface`: `console`, `left`, `right`. Each is a stable Python object that is never replaced.

States:

```
                    ┌──────────────┐
                    │ DISCONNECTED │  no USB device present, no resources held
                    └──────┬───────┘
              hotplug or   │
              poll arrival │
                           ▼
                    ┌──────────────┐
                    │  CONNECTING  │  on-entry: open transport → claim
                    └──────┬───────┘  interfaces → ping → (sensor) refresh
                           │           id cache → get version. Any failure
                           │  all on-entry work ok    → DISCONNECTED.
                           ▼
                    ┌──────────────┐
                    │  CONNECTED   │  safe to issue commands and run scans
                    └──────┬───────┘
            usb error OR   │
            hotplug remove │
            OR user stop   │
                           ▼
                    ┌──────────────┐
                    │ DISCONNECTING│  releasing endpoints / stopping read thread.
                    └──────┬───────┘
                           │
                           ▼
                    ┌──────────────┐
                    │ DISCONNECTED │
                    └──────────────┘
```

The intermediate `CONNECTING` state exists to make `CONNECTED` mean "safe to issue commands" — apps binding to `CONNECTED` no longer fire commands prematurely.

`DISCONNECTING` is a brief state covering transport release. The state machine is pure — it does not know about scans. Mid-scan disconnect handling is `ScanWorkflow`-level policy (see "Mid-scan disconnect handling" below), not a state-machine concept.

### One signal, one source of truth

Each handle exposes:

```python
signal_state_changed = pyqtSignal(object, object, object, str)
# (handle, old_state, new_state, reason)
```

`reason` is a short tag: `"hotplug_removed"`, `"usb_io_error:errno=19"`, `"connect_retry_exhausted:resource_busy"`, `"user_stop"`, `"poll_arrived"`. Enough to log usefully without parsing.

The four current paths that can call disconnect (`CommInterface._read_loop` errno 19/5/32, `MotionUart._read_data` SerialException, `send_packet` exception, periodic poll) become *event submitters*. They enqueue typed events into a single per-interface event queue. The monitor thread processes events serially. **Dedup falls out naturally**: an `EVT_IO_ERROR` arriving while the handle is already `DISCONNECTING` is a no-op logged at DEBUG.

### Daemon monitor thread

`MotionInterface` constructs a single `ConnectionMonitor(threading.Thread)` and starts it on `interface.start()`. The monitor owns the lifecycle of all three handles. Apps never see the thread directly.

Event sources, all feeding one queue:

| Event source                                                                      | Produces                                                            |
|-----------------------------------------------------------------------------------|---------------------------------------------------------------------|
| OS hotplug (Win32 `WM_DEVICECHANGE`, libusb hotplug on Linux/Mac)                 | `EVT_DEVICE_ARRIVED(vid, pid, port)` / `EVT_DEVICE_REMOVED(...)`    |
| Read-thread USB error (`CommInterface._read_loop`, `MotionUart._read_data`)       | `EVT_IO_ERROR(handle, errno, message)`                              |
| Send failure (`send_packet` exception)                                            | `EVT_IO_ERROR(handle, ...)`                                         |
| Periodic poll (200 ms cadence, fallback)                                          | `EVT_POLL_ARRIVED(...)` / `EVT_POLL_GONE(...)`                      |
| App-driven                                                                        | `EVT_USER_STOP(handle)`                                             |

The monitor `get()`s from the queue with `timeout=0.2`. On timeout it runs a poll sweep (USB enumeration off-thread). Event-driven path gives tens-of-ms reconnect detection; the 200 ms poll is a safety net for cases where hotplug doesn't fire (e.g., USB-CDC enumeration on Windows after a driver re-bind).

### Hotplug detection per platform

```
omotion/hotplug/
    __init__.py          # detect_hotplug() returns the right impl, or PollOnly fallback
    win32.py             # message-only window, WM_DEVICECHANGE via ctypes/user32
    libusb_hotplug.py    # libusb1 hotplug callbacks (Linux/Mac)
    poll_only.py         # no-op subscriber; the 200 ms poll does the work
```

Each impl exposes:

```python
def subscribe(self, vid_pid_filter: list[tuple[int, int]],
              on_arrival: Callable[[VidPidPort], None],
              on_removal: Callable[[VidPidPort], None]) -> Unsubscribe
```

The monitor thread is the only consumer. Callbacks just submit events to its queue.

If hotplug registration fails (very old Windows, libusb without hotplug), the 200 ms poll still works — just slightly slower detection (~200 ms worst-case instead of ~10 ms). No new external dependencies are needed; `ctypes` and the `pyusb`/`libusb1` packages already in use cover both platforms.

### Synchronous start/stop

```python
interface.start(wait: bool = True, wait_timeout: float = 2.0) -> None
interface.stop() -> None  # blocks until monitor thread joins
interface.wait_for_ready(*, console=True, sensors=0, timeout=10.0) -> bool
```

`start()` does one synchronous `_poll_sweep()` immediately, enqueues `EVT_POLL_ARRIVED` for each found device, and processes them before returning (when `wait=True`). No more "did I remember to await `start_monitoring`?".

### Stable handles & class restructure

The new design collapses the existing `MOTIONConsole`/`MOTIONSensor` command classes into the handle role. The transport layer (`MotionUart`, `MotionComposite`) stays as the inner mechanism owned by the handle.

```python
class MotionConsole(QObject):     # was MOTIONConsole; absorbs handle role
    name = "console"
    state: ConnectionState
    signal_state_changed = pyqtSignal(object, object, object, str)
    telemetry: ConsoleTelemetryPoller
    uart: MotionUart                                    # owned transport
    # all existing command methods (ping, get_version, set_fan_speed, ...) stay

class MotionSensor(QObject):      # was MOTIONSensor; absorbs handle role
    name: str                                           # "left" | "right"
    side: Literal["left", "right"]
    state: ConnectionState
    signal_state_changed = pyqtSignal(object, object, object, str)
    hardware_id: str | None                             # populated during CONNECTING
    uart: MotionComposite                               # owned transport
    # all existing command methods (ping, imu_init, get_histogram, ...) stay

class MotionInterface(QObject):   # was MOTIONInterface
    console: MotionConsole                              # never None, never replaced
    left:    MotionSensor                               # never None, never replaced
    right:   MotionSensor                               # never None, never replaced
    # no `sensors` dict
    def start(self, wait=True, wait_timeout=2.0): ...
    def stop(self): ...
    def wait_for_ready(self, *, console=True, sensors=0, timeout=10.0): ...
    def connected_sensors(self) -> list[MotionSensor]: ...
```

Naming sweep across the SDK (drop SCREAMING_CAPS):

- `MOTIONInterface` → `MotionInterface`
- `MOTIONConsole` → `MotionConsole`
- `MOTIONSensor` → `MotionSensor`
- `MOTIONUart` → `MotionUart`
- `MOTIONSignal` → `MotionSignal`
- `MotionComposite` / `DualMotionComposite` already PascalCase. `DualMotionComposite` is deleted entirely — its responsibilities move to `ConnectionMonitor` (device discovery/dispatch) and `MotionSensor` (per-handle lifecycle).

## Wire-level mechanics

### CONNECTING on-entry sequence

Console:
1. `serial.Serial(port, baudrate, timeout=0.5)` — single attempt with 500 ms timeout.
2. Send `OW_CMD_PING`; expect ACK within 200 ms.
3. Send `OW_CMD_VERSION`; cache version.

→ `CONNECTED`. Start telemetry poller as on-entry side effect.

Sensor:
1. `usb.core.find` by VID/PID, filter by `port_numbers[-1]`.
2. `set_configuration()` + claim three interfaces (comm/histo/imu).
3. Send `OW_CMD_PING` on comm interface; expect ACK within 200 ms.
4. `refresh_id_cache()`:
   - `OW_CMD_HWID` → cached on `self.hardware_id`. HWID failure means firmware not responsive → treat as connect failure → retry.
   - 8× `OW_CAMERA_READ_SECURITY_UID` → cached per camera. Per-camera failures are tolerated (a dead camera marks its UID as `""` but does not fail the connect — same lenient policy as the existing `refresh_id_cache()`).
5. Send `OW_CMD_VERSION`; cache version.

→ `CONNECTED`. Start comm read thread as on-entry side effect.

Time budget for sensor `CONNECTING`: ~100 ms claim + 200 ms ping + ~400 ms for the 8 UID reads + ~50 ms HWID + ~50 ms version ≈ 800 ms typical, with a per-attempt cap of 2.5 s.

### Retry / backoff

```python
def _try_connect(handle):
    backoff = [0.05, 0.1, 0.25, 0.5, 1.0]   # 5 attempts, ~1.9 s total
    last_error = None
    for delay in backoff:
        try:
            do_on_entry_work(handle)
            return SUCCESS
        except (usb.core.USBError, serial.SerialException, TimeoutError) as e:
            last_error = e
            cleanup_partial(handle)            # release anything we claimed
            time.sleep(delay)
        except Exception as e:
            cleanup_partial(handle)
            return FATAL(e)                    # bug, not a retryable transient
    return FAILED_AFTER_RETRY(last_error)
```

The 5-step backoff (50 → 100 → 250 → 500 → 1000 ms) covers the empirical Windows post-enumeration window where libusb sees the device but `set_configuration()` still returns "Resource busy" or "Permission denied" for ~200–500 ms.

On `FAILED_AFTER_RETRY`, the handle goes back to `DISCONNECTED` with `reason="connect_retry_exhausted:<errno>"`. The next hotplug or poll arrival will try again from scratch — clean state. This kills the current "stuck half-built composite" bug.

The cleanup contract (`cleanup_partial`): every step of `do_on_entry_work` registers its inverse, run in LIFO order on failure or on `request_disconnect`. Equivalent to an `ExitStack` pattern.

If a device is permanently wedged (firmware never ACKs PING), retry-exhausted is logged at ERROR for the first cycle, then DEBUG for subsequent cycles within a 30 s window — so a wedged device left plugged in does not spam logs.

### Disconnect race fixes

| Today's bug                                                                                                                                          | Fix                                                                                                                                                |
|------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------|
| `_handle_interface_disconnect` runs `MotionComposite.disconnect()` from a daemon thread, while the next 1 Hz poll calls `disconnect()` on the same composite. Duplicate "release failed" warnings. | Both paths now enqueue `EVT_IO_ERROR(handle, …)`. Monitor thread serializes; the second event is a no-op because handle is already `DISCONNECTING`. |
| `MOTIONUart.check_usb_status` calls `time.sleep(0.5)` inside the asyncio coroutine.                                                                  | Goes away with the asyncio monitor. Monitor thread does not need the anti-thrash delay because the state machine debounces.                        |
| `MOTIONUart._read_data`, `_tx`, and `send_packet` all call `self.disconnect()` directly on serial errors.                                            | They submit `EVT_IO_ERROR` and return; the monitor thread runs the actual disconnect. Read thread exits naturally when the monitor sets `running = False` during teardown. |
| `CommInterface.stop_read_thread` joins the read thread; `_handle_interface_disconnect` runs on a daemon thread spawned exactly to avoid join-self deadlock. | The dispatch-to-daemon-thread workaround in `_trigger_disconnect` is no longer needed — the read thread submits the event and exits. The monitor thread does the join. |
| `signal_disconnect.emit` wrapped in try/except for "C++ object deleted" during shutdown.                                                              | Monitor thread is owned by `MotionInterface`; `interface.stop()` joins it before any Qt object can be deleted. Defensive try/except removed.       |

## Mid-scan disconnect handling (shipped)

The state machine is pure: `DISCONNECTING → DISCONNECTED` happens as soon as transport release completes. `ScanWorkflow` subscribes to `signal_state_changed` on the handles the scan is using — always the console, plus whichever sensors were specified when the scan was started — before arming the scan, and unsubscribes after.

```
on DISCONNECTING(handle):
    if scan is active and handle is participating:
        set _scan_abort_reason = f"{handle.name} disconnected mid-scan ({reason})"
        trip _stop_evt
        # The main scan loop polls _stop_evt and unwinds via its
        # existing finally cleanup. The scan result.error is set
        # from _scan_abort_reason instead of the generic
        # "Capture canceled".
```

That's the entire policy. There is no recovery path, no grace timer, no late-return handling, no `ResumeRecovery`, no per-handle restart of streams or trigger. The next scan starts cleanly from a known-good state once the user retriggers it.

A user-initiated `cancel_scan()` works the same way: it trips `_stop_evt`, the worker unwinds, the result is recorded as canceled. If both `cancel_scan()` and a disconnect race, whichever sets `_scan_abort_reason` first wins; both ultimately set `_stop_evt`.

### Console drop ≠ sensor drop?

In the shipped design, no. Both are handled identically — any participating handle dropping aborts the scan. The original design called for a special console-recovery path that restarted the FSYNC trigger; that was removed alongside the grace window because the simpler "any drop aborts" rule covers both with one code path.

### What the original design called for (not shipped)

> ~~Per-handle scan-recovery logic inside `ScanWorkflow`:~~
>
> ```
> on DISCONNECTING(handle):
>     if scan is active and handle is participating:
>         mark handle as in_recovery
>         record loss_t = now()
>
> on CONNECTED(handle):
>     if handle.in_recovery and gap_duration < 5.0:
>         ResumeRecovery(handle, gap_duration)
>
> on 5s timer fires for in_recovery handle:
>     AbortScan(reason=f"{handle.name} did not return within 5 s grace window")
> ```
>
> ~~`ResumeRecovery` would have flushed stale data, restarted streaming on the same writer queue, and (for sensors) re-asserted `OW_CAMERA_STREAM` with the original mask; for console it would have restarted the FSYNC trigger.~~

That code was implemented (commits `2ca9624`, `ca13972`) and removed (commit `7455055`) after hardware testing showed the simpler abort policy was preferable.

## Public API summary

```python
class MotionInterface(QObject):
    console: MotionConsole         # stable, never None, never replaced
    left:    MotionSensor          # stable, never None, never replaced
    right:   MotionSensor          # stable, never None, never replaced

    def start(self, wait: bool = True, wait_timeout: float = 2.0) -> None
    def stop(self) -> None
    def wait_for_ready(self, *, console: bool = True,
                       sensors: int = 0,
                       timeout: float = 10.0) -> bool
    def connected_sensors(self) -> list[MotionSensor]

class MotionConsole(QObject):       # also: all existing command methods
    name: str = "console"
    state: ConnectionState
    signal_state_changed = pyqtSignal(object, object, object, str)
    telemetry: ConsoleTelemetryPoller
    def is_connected(self) -> bool
    def wait_for(self, state: ConnectionState, timeout: float = 5.0) -> bool
    def request_disconnect(self) -> None  # graceful, idempotent

class MotionSensor(QObject):        # also: all existing command methods
    name: str                        # "left" | "right"
    side: Literal["left", "right"]
    hardware_id: str | None          # populated during CONNECTING
    state: ConnectionState
    signal_state_changed = pyqtSignal(object, object, object, str)
    def is_connected(self) -> bool
    def wait_for(self, state: ConnectionState, timeout: float = 5.0) -> bool
    def request_disconnect(self) -> None
```

## Migration & cutover

### Files added

- `omotion/connection_monitor.py` — `ConnectionMonitor(threading.Thread)` with the event queue.
- `omotion/hotplug/__init__.py`, `win32.py`, `libusb_hotplug.py`, `poll_only.py` — platform-specific arrival/removal subscribers feeding the queue.

### Files deleted

- `omotion/DualMotionComposite.py` — its logic moves into `ConnectionMonitor` + `MotionSensor`.

### File and class renames (PascalCase, drop SCREAMING)

File and class both renamed:
- `omotion/Interface.py` → `omotion/MotionInterface.py`; class `MOTIONInterface` → `MotionInterface`
- `omotion/Console.py` → `omotion/MotionConsole.py`; class `MOTIONConsole` → `MotionConsole`
- `omotion/Sensor.py` → `omotion/MotionSensor.py`; class `MOTIONSensor` → `MotionSensor`

Class only (file already has the correct name):
- `omotion/MotionUart.py` — class `MOTIONUart` → `MotionUart`
- `omotion/MotionSignal.py` — class `MOTIONSignal` → `MotionSignal`

`omotion/__init__.py` re-exports everything under the new names. **No aliases to old names** — apps update in lockstep.

### Files modified (existing classes pared down)

- `omotion/connection_state.py` — `ConnectionState` becomes the 4-state enum (DISCONNECTED, CONNECTING, CONNECTED, DISCONNECTING). `DISCOVERED` and `ERROR` removed.
- `omotion/signal_wrapper.py` — pared down to `signal_state_changed` only. `signal_connect`, `signal_disconnect`, `signal_data_received` removed.
- `omotion/MotionUart.py` — `connect()`, `disconnect()`, `check_usb_status()`, `monitor_usb_status()`, `start_monitoring()`, `stop_monitoring()`, `_set_state()` all removed. Becomes a pure transport: `open(port)` / `close()` / `send_packet()` / `read_packet()`.
- `omotion/MotionComposite.py` — `connect()`, `disconnect()`, `_handle_interface_disconnect`, `_set_state()` removed. Becomes a pure composite-of-three-interfaces wrapper. `on_disconnect` callback path on `CommInterface` becomes `on_io_error` and just submits an event to the monitor's queue.
- `omotion/CommInterface.py` — `_trigger_disconnect`, daemon-thread dispatch hack, `_disconnect_notified` flag all removed. Read loop on USB error submits event and exits.

### App-side migration (one focused PR per app, lands on `next` branches)

Both `motion_connector.py` files (bloodflow-app and test-app) get the same shape of change:

```python
# was:
self._interface.signal_connect.connect(self.on_connected)
self._interface.signal_disconnect.connect(self.on_disconnected)

# becomes:
for handle in (self._interface.console, self._interface.left, self._interface.right):
    handle.signal_state_changed.connect(self._on_state_changed)

# new single handler:
def _on_state_changed(self, handle, old, new, reason):
    name = handle.name  # "console" | "left" | "right"
    is_now_connected = (new == ConnectionState.CONNECTED)
    if name == "console":
        self._consoleConnected = is_now_connected
        if is_now_connected:
            # the existing "on connect" side effects: log info, set TEC, set fan
            ...
    elif name == "left":
        self._leftSensorConnected = is_now_connected
        ...
    elif name == "right":
        self._rightSensorConnected = is_now_connected
        ...
    # Forward to the app's own QML-facing signal in whatever shape the QML expects.
    # (signature unchanged from today; apps decide what to emit.)
    self.signalStateChanged.emit(name, new.name)
```

Other app-side changes:

- `interface.sensors.get("left")` → `interface.left` (always non-None).
- Gate "is it usable?" checks on `interface.left.is_connected()`.
- `interface.console_module.ping()` → `interface.console.ping()` (drop `_module`).
- `interface.console_module.telemetry` → `interface.console.telemetry`.

`main.py` of test-app:

```python
# was:
async def main_async():
    await connector._interface.start_monitoring()
# (plus QEventLoop(app) / asyncio glue)

# becomes:
connector._interface.start()   # synchronous, no asyncio needed
# at shutdown:
connector._interface.stop()
```

bloodflow-app similarly drops asyncio for connection monitoring.

### Cutover order

The user's dev machine has `openmotion-sdk` on PYTHONPATH; both apps import `omotion` directly from disk. No `pip install -e` is needed — edits to the SDK are picked up by the apps immediately.

1. Branch the SDK; make the changes; manual verification on real hardware against both apps still on `main`/`next`.
2. Branch each app; update `motion_connector.py` and `main.py` for the new API. Verify all four reconnect scenarios in both apps:
   - hot replug during a scan (any duration → clean abort with the
     dropped handle's name in the result error message)
   - idle replug between scans
   - power-cycle of the whole console/sensor enclosure
3. Merge SDK to `main`, tag, publish wheel.
4. Both app PRs merge to their respective `next` branches; bump SDK pin.

## Open questions

None — design is fully agreed.
