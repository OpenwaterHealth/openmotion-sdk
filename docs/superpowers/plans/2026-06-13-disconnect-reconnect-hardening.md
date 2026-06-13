# Disconnect / Reconnect Hardening Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the SDK's autoconnect deterministic and laser-safe across USB/UART disconnect and reconnect by turning `ConnectionMonitor` into a pure detector and moving all blocking device I/O onto per-handle connect workers.

**Architecture:** Each handle (`MotionConsole`, `MotionSensor` ×2) owns one daemon **connect worker** thread with an intent mailbox. The monitor thread only detects topology changes and posts `CONNECT`/`DISCONNECT` intents (non-blocking). Workers run bounded, interruptible connect/teardown sequences; the console force-safes the trigger (`stop_trigger`) on every connect. A failed connect applies a short cooldown to kill the retry storm.

**Tech Stack:** Python 3.13, `threading`, `queue`, `pyserial`, `pyusb`/`libusb1`, `pytest` (markers: `console`, `sensor`, `slow`). Hardware: YKUSH YK27987 (USB cut via `tests/ykush.py`), Shelly 192.168.1.81 (mains cut).

**Reference spec:** `docs/superpowers/specs/2026-06-13-disconnect-reconnect-hardening-design.md`
**Companion firmware spec (not implemented here):** `docs/superpowers/specs/2026-06-13-console-fw-host-loss-watchdog-spec.md`

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `omotion/MotionUart.py` | Console UART transport | Add cancellable/bounded reads |
| `omotion/CommInterface.py` | Sensor USB command transport | Add `cancel_evt` to `send_packet` |
| `omotion/connect_worker.py` | Per-handle connect worker thread | **Create** |
| `omotion/MotionConsole.py` | Console handle state machine | Worker wiring, bounded ping, force-safe, bounded `stop_trigger` |
| `omotion/MotionSensor.py` | Sensor handle state machine | Worker wiring, interruptible backoff |
| `omotion/connection_monitor.py` | Detector/router | Stop workers in teardown |
| `omotion/MotionInterface.py` | Facade | (no change expected; verify) |
| `tests/test_connect_worker.py` | Worker unit tests (sw-only) | **Create** |
| `tests/test_uart_cancel.py` | Cancellable read unit tests (sw-only) | **Create** |
| `tests/test_monitor_nonblocking.py` | Cross-handle starvation regression (sw-only) | **Create** |
| `tests/test_connection_recovery.py` | HIL lifecycle × transport matrix | **Create** |

---

## Task 1: Cancellable, bounded UART reads (console transport)

**Why:** `MotionUart.read_packet` is uninterruptible and the console connect ping uses the default 20 s timeout, freezing the monitor for ~20 s. Make reads honor a cancel event so a connect attempt aborts promptly on a superseding disconnect.

**Files:**
- Modify: `omotion/MotionUart.py:140-167` (`read_packet`), `omotion/MotionUart.py:169-228` (`send_packet`)
- Test: `tests/test_uart_cancel.py` (create)

- [ ] **Step 1: Write the failing test**

```python
# tests/test_uart_cancel.py
import threading
import time

import pytest

from omotion.MotionUart import MotionUart
from omotion.CommandError import CommandError


class _FakeSerial:
    """A serial port that is open but never returns data."""
    is_open = True

    def read_all(self):
        return b""

    def close(self):
        self.is_open = False


def _uart_with_fake():
    u = MotionUart(vid=0x0483, pid=0xA53E, timeout=20)
    u.serial = _FakeSerial()
    return u


def test_read_packet_aborts_on_cancel_event():
    u = _uart_with_fake()
    cancel = threading.Event()

    # Cancel after 200 ms from another thread.
    threading.Timer(0.2, cancel.set).start()

    start = time.monotonic()
    with pytest.raises(CommandError):
        u.read_packet(timeout=20, cancel_evt=cancel)
    elapsed = time.monotonic() - start
    # Must abort within ~1 s, NOT wait the full 20 s timeout.
    assert elapsed < 1.0, f"read_packet did not abort promptly ({elapsed:.1f}s)"


def test_read_packet_still_times_out_without_cancel():
    u = _uart_with_fake()
    start = time.monotonic()
    with pytest.raises(ValueError):
        u.read_packet(timeout=0.5)
    assert 0.4 < time.monotonic() - start < 2.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_uart_cancel.py -v`
Expected: FAIL — `read_packet()` got an unexpected keyword argument `cancel_evt`.

- [ ] **Step 3: Implement cancellable read**

In `omotion/MotionUart.py`, change the `read_packet` signature and add the cancel check at the top of the loop:

```python
    def read_packet(self, timeout: int = 20, cancel_evt=None) -> UartPacket:
        """Block until a packet arrives or `timeout` seconds elapse.

        If `cancel_evt` is provided and becomes set, the read aborts promptly
        with a CommandError instead of waiting out the full timeout — used by
        the connect worker so a superseding disconnect cancels an in-flight
        connect attempt.
        """
        if self.demo_mode:
            return UartPacket(
                id=0, packetType=OW_ERROR, command=0, addr=0, reserved=0, data=[]
            )
        if self.serial is None:
            raise CommandError("UART not open")
        with self._io_lock:
            start_time = time.monotonic()
            raw_data = b""
            count = 0

            while timeout == -1 or time.monotonic() - start_time < timeout:
                if cancel_evt is not None and cancel_evt.is_set():
                    raise CommandError("UART read canceled")
                time.sleep(0.05)
                try:
                    raw_data += self.serial.read_all()
                except serial.SerialException as se:
                    self._notify_io_error(getattr(se, "errno", None), str(se))
                    raise
                if raw_data:
                    count += 1
                    if count > 1:
                        break

        if not raw_data:
            raise ValueError("No data received from UART within timeout")
        return UartPacket(buffer=raw_data)
```

Then thread `cancel_evt` through `send_packet`. Change its signature (line ~169) to add `cancel_evt=None` after `timeout`, and update the `read_packet` call (line ~226):

```python
    def send_packet(
        self,
        id=None,
        packetType=OW_ACK,
        command=OW_CMD_NOP,
        addr: int = 0,
        reserved: int = 0,
        data=None,
        timeout: int = 20,
        cancel_evt=None,
    ) -> Optional[UartPacket]:
```

```python
                ret_packet = self.read_packet(timeout=timeout, cancel_evt=cancel_evt)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_uart_cancel.py -v`
Expected: PASS (both tests).

- [ ] **Step 5: Commit**

```bash
git add omotion/MotionUart.py tests/test_uart_cancel.py
git commit -m "feat: cancellable bounded reads in MotionUart"
```

---

## Task 2: `cancel_evt` for the sensor USB command transport

**Why:** Symmetry — a superseding disconnect during a sensor connect ping should also abort promptly. `CommInterface.send_packet` already polls `_transport_down_evt`; add an optional `cancel_evt` checked in the same loops.

**Files:**
- Modify: `omotion/CommInterface.py:121-206` (`send_packet`)
- Test: covered indirectly by Task 5 sensor wiring; no new unit test (USB transport is hardware-bound).

- [ ] **Step 1: Add `cancel_evt` parameter**

Change the signature (line 121-131) to add `cancel_evt=None` after `max_retries=0,`:

```python
    def send_packet(
        self,
        id=None,
        packetType=OW_ACK,
        command=OW_CMD_NOP,
        addr=0,
        reserved=0,
        data=None,
        timeout=10.0,
        max_retries=0,
        cancel_evt=None,
    ) -> UartPacket:
```

- [ ] **Step 2: Check it in both wait loops**

In the `not self.async_mode` loop (line ~170) add, immediately after `while ...:`:

```python
                        if cancel_evt is not None and cancel_evt.is_set():
                            raise ConnectionError(
                                f"{self.desc}: send canceled, packet id 0x{id:04X}"
                            )
```

In the `else` (async) loop (line ~188) add, immediately after `while ...:`:

```python
                    if cancel_evt is not None and cancel_evt.is_set():
                        raise ConnectionError(
                            f"{self.desc}: send canceled, packet id 0x{id:04X}"
                        )
```

- [ ] **Step 3: Verify import compiles**

Run: `python -c "import omotion.CommInterface"`
Expected: no error.

- [ ] **Step 4: Commit**

```bash
git add omotion/CommInterface.py
git commit -m "feat: cancel_evt support in CommInterface.send_packet"
```

---

## Task 3: `ConnectWorker` — per-handle connect thread

**Why:** The core architectural change. One worker thread per handle serializes connect/disconnect off the monitor thread, coalesces redundant intents, aborts in-flight connects on a superseding disconnect, and applies a cooldown after a failed connect.

**Files:**
- Create: `omotion/connect_worker.py`
- Test: `tests/test_connect_worker.py` (create)

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_connect_worker.py
import threading
import time

from omotion.connect_worker import ConnectWorker


def _make_worker(connect_fn, disconnect_fn, cooldown=0.0):
    w = ConnectWorker(
        name="test", do_connect=connect_fn, do_disconnect=disconnect_fn,
        cooldown=cooldown,
    )
    w.start()
    return w


def test_connect_intent_runs_do_connect():
    ran = threading.Event()

    def connect(reason, abort):
        ran.set()
        return True

    w = _make_worker(connect, lambda reason: None)
    try:
        w.post("connect", "test")
        assert ran.wait(2.0)
    finally:
        w.stop()


def test_disconnect_sets_abort_and_runs_do_disconnect():
    in_connect = threading.Event()
    aborted = threading.Event()
    disconnected = threading.Event()

    def connect(reason, abort):
        in_connect.set()
        if abort.wait(2.0):       # block until aborted
            aborted.set()
        return False

    w = _make_worker(connect, lambda reason: disconnected.set())
    try:
        w.post("connect", "go")
        assert in_connect.wait(2.0)
        w.post("disconnect", "stop")
        assert aborted.wait(2.0), "abort event was not set on disconnect"
        assert disconnected.wait(2.0)
    finally:
        w.stop()


def test_one_blocking_connect_does_not_starve_another_worker():
    # Regression for failure mode A: per-handle isolation.
    release = threading.Event()
    b_connected = threading.Event()

    def slow_connect(reason, abort):
        release.wait(3.0)         # A blocks
        return True

    def fast_connect(reason, abort):
        b_connected.set()         # B should proceed immediately
        return True

    a = _make_worker(slow_connect, lambda reason: None)
    b = _make_worker(fast_connect, lambda reason: None)
    try:
        a.post("connect", "a")
        b.post("connect", "b")
        # B connects within 1 s even though A is blocked for 3 s.
        assert b_connected.wait(1.0), "B was starved by A's blocking connect"
    finally:
        release.set()
        a.stop()
        b.stop()


def test_cooldown_delays_retry_after_failure():
    attempts = []

    def connect(reason, abort):
        attempts.append(time.monotonic())
        return False              # always fail

    w = _make_worker(connect, lambda reason: None, cooldown=0.5)
    try:
        w.post("connect", "1")
        time.sleep(0.1)
        w.post("connect", "2")    # arrives during cooldown
        time.sleep(1.0)
        assert len(attempts) >= 2
        # Second attempt is at least ~cooldown after the first.
        assert attempts[1] - attempts[0] >= 0.45
    finally:
        w.stop()
```

- [ ] **Step 2: Run to verify it fails**

Run: `pytest tests/test_connect_worker.py -v`
Expected: FAIL — `No module named 'omotion.connect_worker'`.

- [ ] **Step 3: Implement `ConnectWorker`**

```python
# omotion/connect_worker.py
"""Per-handle connect worker.

Each device handle (console, left sensor, right sensor) owns one of these. It
runs the handle's blocking connect/teardown off the ConnectionMonitor thread so
a slow or unresponsive device can never stall another handle's connect.

Intents ("connect"/"disconnect") are posted from the monitor thread and
processed serially on this worker's own daemon thread:

  - "connect"     -> do_connect(reason, abort_evt) -> bool (True == CONNECTED)
  - "disconnect"  -> abort any in-flight connect, then do_disconnect(reason)

A failed connect applies `cooldown` seconds before the next connect attempt, so
a present-but-unresponsive device retries on a calm cadence instead of spinning
every poll tick.
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable

from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.ConnectWorker" if _log_root else "ConnectWorker"
)


class ConnectWorker(threading.Thread):
    def __init__(
        self,
        name: str,
        do_connect: Callable[[str, threading.Event], bool],
        do_disconnect: Callable[[str], None],
        cooldown: float = 1.0,
    ):
        super().__init__(daemon=True, name=f"connect-{name}")
        self._label = name
        self._do_connect = do_connect
        self._do_disconnect = do_disconnect
        self._cooldown = cooldown

        self._mailbox: "queue.Queue[tuple[str, str]]" = queue.Queue()
        self._abort = threading.Event()
        self._stop = threading.Event()
        self._last_fail_t = 0.0

    # ── posted from the monitor thread (non-blocking) ───────────────────────
    def post(self, intent: str, reason: str) -> None:
        if intent == "disconnect":
            # Cancel any in-flight connect attempt promptly.
            self._abort.set()
        self._mailbox.put((intent, reason))

    def stop(self) -> None:
        self._stop.set()
        self._abort.set()
        self._mailbox.put(("stop", "worker_stop"))
        self.join(timeout=5.0)

    # ── worker thread body ──────────────────────────────────────────────────
    def run(self) -> None:
        while not self._stop.is_set():
            try:
                intent, reason = self._mailbox.get(timeout=0.5)
            except queue.Empty:
                continue
            intent, reason = self._coalesce(intent, reason)
            if intent == "stop":
                break
            if intent == "connect":
                self._run_connect(reason)
            elif intent == "disconnect":
                self._abort.clear()
                try:
                    self._do_disconnect(reason)
                except Exception:
                    logger.exception("%s do_disconnect raised", self._label)

    def _coalesce(self, intent: str, reason: str):
        """Drain queued intents, keeping only the most recent. Collapses the
        burst of PollArrived events the monitor emits every 200 ms."""
        while True:
            try:
                intent, reason = self._mailbox.get_nowait()
            except queue.Empty:
                return intent, reason

    def _run_connect(self, reason: str) -> None:
        # Cooldown after a recent failure (interruptible by stop/disconnect).
        if self._cooldown > 0 and self._last_fail_t:
            remaining = self._cooldown - (time.monotonic() - self._last_fail_t)
            if remaining > 0 and self._abort.wait(remaining):
                return  # superseded by disconnect/stop during cooldown
        self._abort.clear()
        try:
            ok = self._do_connect(reason, self._abort)
        except Exception:
            logger.exception("%s do_connect raised", self._label)
            ok = False
        if not ok:
            self._last_fail_t = time.monotonic()
        else:
            self._last_fail_t = 0.0
```

- [ ] **Step 4: Run to verify it passes**

Run: `pytest tests/test_connect_worker.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add omotion/connect_worker.py tests/test_connect_worker.py
git commit -m "feat: ConnectWorker per-handle connect thread"
```

---

## Task 4: Bounded `stop_trigger` (for force-safe)

**Why:** Force-safe-on-connect calls `stop_trigger`, which today uses the default 20 s `send_packet` timeout. Bound it so a force-safe can't block the console worker for 20 s.

**Files:**
- Modify: `omotion/MotionConsole.py:1188-1222` (`stop_trigger`)
- Test: none (thin passthrough; exercised by Task 5 + HIL Task 8).

- [ ] **Step 1: Add a `timeout` parameter**

Change the `stop_trigger` signature and the `send_packet` call:

```python
    def stop_trigger(self, timeout: int = 20) -> bool:
```

```python
            r = self.uart.send_packet(
                id=None, packetType=OW_CONTROLLER, command=OW_CTRL_STOP_TRIG,
                data=None, timeout=timeout,
            )
```

- [ ] **Step 2: Verify import compiles**

Run: `python -c "import omotion.MotionConsole"`
Expected: no error.

- [ ] **Step 3: Commit**

```bash
git add omotion/MotionConsole.py
git commit -m "feat: bounded timeout for stop_trigger"
```

---

## Task 5: Wire `ConnectWorker` into `MotionConsole`

**Why:** Move the console's connect/teardown off the monitor thread; bound the connect ping; force-safe the trigger on every connect.

**Files:**
- Modify: `omotion/MotionConsole.py` — `__init__` (~165-170), `_attach_monitor` (~202-204), `_handle_event` (240-277), `_drive_connecting` (279-342), `_drive_disconnecting` (363-373); add `_start_worker`, `_shutdown`.
- Test: covered by Task 7 (monitor non-blocking) + HIL Task 8.

- [ ] **Step 1: Add worker field and start hook**

In `__init__` (after `self._monitor = None`):

```python
        self._worker = None  # ConnectWorker, created in _attach_monitor
```

Replace `_attach_monitor`:

```python
    def _attach_monitor(self, monitor) -> None:
        """Called by MotionInterface when starting the monitor."""
        self._monitor = monitor
        self._start_worker()

    def _start_worker(self) -> None:
        from omotion.connect_worker import ConnectWorker

        if self._worker is None:
            self._worker = ConnectWorker(
                name=self.name,
                do_connect=self._drive_connecting,
                do_disconnect=self._drive_disconnecting,
            )
            self._worker.start()
```

- [ ] **Step 2: `_handle_event` posts intents instead of driving inline**

Replace the body of `_handle_event` (240-277) with:

```python
    def _handle_event(self, event) -> None:
        """Entry point called by ConnectionMonitor on its thread. Decides the
        target intent from (state, event) and posts it to the per-handle
        worker — never blocks the monitor thread."""
        from omotion.connection_monitor import (
            IoError,
            PollArrived,
            PollGone,
            UserStop,
        )

        if self._worker is None:
            return
        st = self._state
        if isinstance(event, PollArrived):
            if st == ConnectionState.DISCONNECTED:
                self._worker.post("connect", "poll_arrived")
        elif isinstance(event, (PollGone, IoError)):
            if st == ConnectionState.CONNECTED:
                reason = (
                    f"usb_io_error:errno={event.errno}"
                    if isinstance(event, IoError)
                    else "poll_gone"
                )
                self._worker.post("disconnect", reason)
        elif isinstance(event, UserStop):
            if st in (ConnectionState.CONNECTED, ConnectionState.CONNECTING):
                self._worker.post("disconnect", "user_stop")
```

- [ ] **Step 3: `_drive_connecting` becomes abort-aware, bounded, force-safing, and returns bool**

Replace `_drive_connecting` (279-342):

```python
    def _drive_connecting(self, reason: str, abort_evt=None) -> bool:
        """Runs on the connect worker thread. Returns True iff CONNECTED."""
        if self._state != ConnectionState.DISCONNECTED:
            return self._state == ConnectionState.CONNECTED

        if self.uart.demo_mode:
            self._set_state(ConnectionState.CONNECTING, reason=reason)
            self._set_state(ConnectionState.CONNECTED, reason="demo_mode")
            try:
                self.telemetry.start()
            except Exception:
                logger.exception("telemetry start (demo) failed")
            return True

        self._set_state(ConnectionState.CONNECTING, reason=reason)

        backoff = [0.05, 0.1, 0.25, 0.5, 1.0]
        last_error: Optional[Exception] = None
        for delay in backoff:
            if abort_evt is not None and abort_evt.is_set():
                break
            try:
                port = self.uart.find_port()
                if port is None:
                    raise RuntimeError("console COM port not found")
                self.uart.open(port)
                # Bounded, cancellable ping (was the unbounded 20 s default).
                r = self.uart.send_packet(
                    id=None, packetType=OW_CMD, command=OW_CMD_PING,
                    timeout=2, cancel_evt=abort_evt,
                )
                if r is None or r.packetType == OW_ERROR:
                    raise RuntimeError("console ping failed or returned error")
                self._set_state(ConnectionState.CONNECTED, reason="ping_ok")
                # Force-safe: leave the laser/trigger OFF on every connect.
                try:
                    self.stop_trigger(timeout=2)
                    logger.info("console force-safe stop_trigger on connect")
                except Exception as e:
                    logger.warning("force-safe stop_trigger failed: %s", e)
                try:
                    self.telemetry.start()
                except Exception:
                    logger.exception("telemetry start failed")
                return True
            except Exception as e:
                last_error = e
                logger.warning(
                    "console connect attempt failed (%s); retrying in %.0f ms",
                    e, delay * 1000,
                )
                try:
                    self.uart.close()
                except Exception:
                    pass
                if abort_evt is not None and abort_evt.wait(delay):
                    break
                elif abort_evt is None:
                    time.sleep(delay)

        self._set_state(
            ConnectionState.DISCONNECTED,
            reason=f"connect_retry_exhausted:{last_error}",
        )
        return False
```

- [ ] **Step 4: `_drive_disconnecting` guards against double-teardown**

Replace `_drive_disconnecting` (363-373):

```python
    def _drive_disconnecting(self, reason: str) -> None:
        """Runs on the connect worker thread (or synchronously from _shutdown)."""
        if self._state == ConnectionState.DISCONNECTED:
            return
        self._set_state(ConnectionState.DISCONNECTING, reason=reason)
        try:
            self.telemetry.stop()
        except Exception:
            logger.exception("telemetry stop failed")
        try:
            self.uart.close()
        except Exception:
            logger.exception("uart close failed")
        self._set_state(ConnectionState.DISCONNECTED, reason=reason)
```

- [ ] **Step 5: Add `_shutdown` for clean teardown**

Add after `_drive_disconnecting`:

```python
    def _shutdown(self) -> None:
        """Final teardown: stop the worker, then synchronously release the
        transport. Called by ConnectionMonitor._teardown."""
        if self._worker is not None:
            self._worker.stop()
            self._worker = None
        try:
            self._drive_disconnecting("shutdown")
        except Exception:
            logger.exception("console shutdown disconnect failed")
```

- [ ] **Step 6: Verify import + compile**

Run: `python -c "import omotion.MotionConsole"`
Expected: no error.

- [ ] **Step 7: Commit**

```bash
git add omotion/MotionConsole.py
git commit -m "feat: console connect via per-handle worker + force-safe on connect"
```

---

## Task 6: Wire `ConnectWorker` into `MotionSensor`

**Why:** Same isolation for the sensors; make the connect backoff abort-aware.

**Files:**
- Modify: `omotion/MotionSensor.py` — `__init__` (~150-152), `_attach_monitor` (194-195), `_handle_event` (229-252), `_drive_connecting` (268-342), `_drive_disconnecting` (344-355); add `_start_worker`, `_shutdown`.

- [ ] **Step 1: Add worker field + start hook**

In `__init__` (after `self._monitor = None`):

```python
        self._worker = None  # ConnectWorker, created in _attach_monitor
```

Replace `_attach_monitor`:

```python
    def _attach_monitor(self, monitor) -> None:
        self._monitor = monitor
        self._start_worker()

    def _start_worker(self) -> None:
        from omotion.connect_worker import ConnectWorker

        if self._worker is None:
            self._worker = ConnectWorker(
                name=self.name,
                do_connect=self._drive_connecting,
                do_disconnect=self._drive_disconnecting,
            )
            self._worker.start()
```

- [ ] **Step 2: `_handle_event` posts intents**

Replace `_handle_event` (229-252):

```python
    def _handle_event(self, event) -> None:
        from omotion.connection_monitor import (
            IoError,
            PollArrived,
            PollGone,
            UserStop,
        )

        if self._worker is None:
            return
        st = self._state
        if isinstance(event, PollArrived):
            if st == ConnectionState.DISCONNECTED:
                self._worker.post("connect", "poll_arrived")
        elif isinstance(event, (PollGone, IoError)):
            if st == ConnectionState.CONNECTED:
                reason = (
                    f"usb_io_error:errno={event.errno}"
                    if isinstance(event, IoError)
                    else "poll_gone"
                )
                self._worker.post("disconnect", reason)
        elif isinstance(event, UserStop):
            if st in (ConnectionState.CONNECTED, ConnectionState.CONNECTING):
                self._worker.post("disconnect", "user_stop")
```

- [ ] **Step 3: `_drive_connecting` abort-aware + returns bool**

Replace `_drive_connecting` (268-342). Add the guard, the abort checks, `cancel_evt` on the ping, and a bool return:

```python
    def _drive_connecting(self, reason: str, abort_evt=None) -> bool:
        """Runs on the connect worker thread. Returns True iff CONNECTED."""
        if self._state != ConnectionState.DISCONNECTED:
            return self._state == ConnectionState.CONNECTED

        self._set_state(ConnectionState.CONNECTING, reason=reason)

        backoff = [0.05, 0.1, 0.25, 0.5, 1.0]
        last_error: Optional[Exception] = None
        for delay in backoff:
            if abort_evt is not None and abort_evt.is_set():
                break
            try:
                dev = self._find_dev()
                if dev is None:
                    raise RuntimeError(
                        f"sensor device not found (VID=0x{self.vid:04X} "
                        f"PID=0x{self.pid:04X} port_suffix={self._port_suffix})"
                    )
                composite = MotionComposite(
                    dev,
                    desc=self.side.upper(),
                    async_mode=True,
                    on_io_error=self._on_uart_io_error,
                )
                composite.open()
                self.uart = composite

                r = self.uart.comm.send_packet(
                    id=None, packetType=OW_CMD, command=OW_CMD_PING,
                    timeout=2.0, cancel_evt=abort_evt,
                )
                if r is None or r.packetType in _ERROR_TYPES:
                    raise RuntimeError("sensor ping failed or returned error")

                self.refresh_id_cache()
                if not self._cached_hwid:
                    raise RuntimeError("sensor HWID read returned empty")
                self.hardware_id = self._cached_hwid

                try:
                    self._version = self.get_version()
                except Exception as e:
                    logger.debug("get_version during connect failed: %s", e)

                self._set_state(ConnectionState.CONNECTED, reason="ping_ok")
                return True
            except Exception as e:
                last_error = e
                logger.warning(
                    "%s connect attempt failed (%s); retrying in %.0f ms",
                    self.name, e, delay * 1000,
                )
                try:
                    if self.uart is not None:
                        self.uart.close()
                except Exception:
                    pass
                self.uart = None
                self._cached_camera_uids = None
                self._cached_hwid = None
                self.hardware_id = None
                if abort_evt is not None and abort_evt.wait(delay):
                    break
                elif abort_evt is None:
                    time.sleep(delay)

        self._set_state(
            ConnectionState.DISCONNECTED,
            reason=f"connect_retry_exhausted:{last_error}",
        )
        return False
```

- [ ] **Step 4: `_drive_disconnecting` guard + `_shutdown`**

Replace `_drive_disconnecting` (344-355) and add `_shutdown`:

```python
    def _drive_disconnecting(self, reason: str) -> None:
        if self._state == ConnectionState.DISCONNECTED:
            return
        self._set_state(ConnectionState.DISCONNECTING, reason=reason)
        try:
            if self.uart is not None:
                self.uart.close()
        except Exception:
            logger.exception("uart close failed")
        self.uart = None
        self._cached_camera_uids = None
        self._cached_hwid = None
        self.hardware_id = None
        self._set_state(ConnectionState.DISCONNECTED, reason=reason)

    def _shutdown(self) -> None:
        """Final teardown: stop the worker, then synchronously release USB."""
        if self._worker is not None:
            self._worker.stop()
            self._worker = None
        try:
            self._drive_disconnecting("shutdown")
        except Exception:
            logger.exception("%s shutdown disconnect failed", self.name)
```

- [ ] **Step 5: Verify import compiles**

Run: `python -c "import omotion.MotionSensor"`
Expected: no error.

- [ ] **Step 6: Commit**

```bash
git add omotion/MotionSensor.py
git commit -m "feat: sensor connect via per-handle worker, abort-aware backoff"
```

---

## Task 7: Monitor teardown stops workers + non-blocking regression test

**Why:** The monitor's teardown must stop each handle's worker and release transports cleanly. Add the software regression test that proves a blocked console connect no longer starves the sensors (failure mode A).

**Files:**
- Modify: `omotion/connection_monitor.py:214-229` (`_teardown`)
- Test: `tests/test_monitor_nonblocking.py` (create)

- [ ] **Step 1: Update `_teardown` to call `_shutdown`**

Replace the teardown loop (lines 222-227) so it calls each handle's `_shutdown` (which stops the worker and releases the transport) instead of posting a `UserStop`:

```python
        # Drive each handle to DISCONNECTED so its transport releases cleanly
        # before MotionInterface (and any owning Qt parent) tears down. Each
        # handle stops its own connect worker first, then releases synchronously.
        for handle in (self._console, self._left, self._right):
            try:
                handle._shutdown()
            except Exception as e:
                logger.warning("Final disconnect of %s failed: %s", handle.name, e)
```

- [ ] **Step 2: Write the failing regression test**

```python
# tests/test_monitor_nonblocking.py
"""Failure-mode-A regression: a blocked connect on one handle must not stall
another handle's connect. Uses the real ConnectionMonitor with fake handles."""
import threading
import time

from omotion.connection_monitor import ConnectionMonitor, PollArrived
from omotion.connection_state import ConnectionState
from omotion.connect_worker import ConnectWorker


class _FakeHandle:
    def __init__(self, name, connect_fn):
        self.name = name
        self.state = ConnectionState.DISCONNECTED
        self._worker = ConnectWorker(
            name=name, do_connect=connect_fn, do_disconnect=lambda r: None,
            cooldown=0.0,
        )
        self._worker.start()

    def is_connected(self):
        return self.state == ConnectionState.CONNECTED

    def _handle_event(self, event):
        if isinstance(event, PollArrived) and self.state == ConnectionState.DISCONNECTED:
            self._worker.post("connect", "poll")

    def _shutdown(self):
        self._worker.stop()


def test_blocked_console_connect_does_not_starve_sensor():
    release = threading.Event()
    sensor_connected = threading.Event()

    def slow(reason, abort):
        release.wait(3.0)              # console blocks
        return True

    def fast(reason, abort):
        sensor_connected.set()
        return True

    console = _FakeHandle("console", slow)
    left = _FakeHandle("left", fast)
    right = _FakeHandle("right", fast)

    # Directly dispatch PollArrived to both (bypassing USB enumeration).
    console._handle_event(PollArrived(handle_name="console"))
    left._handle_event(PollArrived(handle_name="left"))

    try:
        assert sensor_connected.wait(1.0), "sensor starved by blocked console"
    finally:
        release.set()
        for h in (console, left, right):
            h._shutdown()
```

- [ ] **Step 3: Run to verify it passes**

Run: `pytest tests/test_monitor_nonblocking.py tests/test_connect_worker.py tests/test_uart_cancel.py -v`
Expected: PASS.

- [ ] **Step 4: Run the full software-only suite for regressions**

Run: `pytest tests/ -m "not console and not sensor and not destructive and not fpga and not imu" -q`
Expected: PASS (no regressions in pipeline/unit tests).

- [ ] **Step 5: Commit**

```bash
git add omotion/connection_monitor.py tests/test_monitor_nonblocking.py
git commit -m "feat: monitor teardown stops workers; add non-blocking regression test"
```

---

## Task 8: HIL lifecycle × transport recovery tests

**Why:** Prove on real hardware that cutting **both links** at every app-lifecycle stage is handled correctly, and that force-safe + deterministic recovery work. Combined-cut only (decision D1).

**Files:**
- Create: `tests/test_connection_recovery.py`

**Note for the worker:** This test requires the bench (YKUSH YK27987 + the Open-Motion system + Shelly). It must `pytest.skip` cleanly when hardware/power-control is absent. Run it explicitly; it is `slow`.

- [ ] **Step 1: Write the HIL test module**

```python
# tests/test_connection_recovery.py
"""Hardware-in-the-loop disconnect/reconnect recovery (combined-link cuts).

Bench: YKUSH YK27987 (port 1 carries the whole system's USB) + Shelly mains
outlet. These tests cut BOTH the console UART and the sensor USB-bulk links at
once (the current cabling cannot isolate them) at several app-lifecycle stages
and assert correct handling: deterministic recovery and laser force-safe.
"""
import os
import sys
import time

import pytest

from omotion.connection_state import ConnectionState

ykush = pytest.importorskip("ykush")  # tests/ on path via conftest

RECOVER_TIMEOUT = 8.0
SYS_PORT = 1  # YKUSH port carrying console + both sensors


def _hub():
    try:
        return ykush.default_hub()
    except Exception as e:
        pytest.skip(f"YKUSH hub not available: {e}")


def _all_connected(motion):
    return (motion.console.is_connected()
            and motion.left.is_connected()
            and motion.right.is_connected())


def _wait_all(motion, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if _all_connected(motion):
            return True
        time.sleep(0.1)
    return False


@pytest.fixture
def hub():
    h = _hub()
    h.all_on()
    time.sleep(2.0)
    yield h
    h.all_on()


@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_system_cut_restore_recovers_deterministically(motion, hub):
    """L2 CONNECTED-idle: cut both links, restore, all 3 back < 8 s, ×3."""
    assert _wait_all(motion, 15.0), "preconditions: all three must start connected"
    for i in range(3):
        hub.off(SYS_PORT)
        time.sleep(4.0)
        assert not _all_connected(motion), f"iter {i}: handles should have dropped"
        hub.on(SYS_PORT)
        assert _wait_all(motion, RECOVER_TIMEOUT), (
            f"iter {i}: did not recover all 3 within {RECOVER_TIMEOUT}s "
            f"(console={motion.console.state.name} "
            f"left={motion.left.state.name} right={motion.right.state.name})"
        )


@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_cut_during_connecting_recovers(motion, hub):
    """L1 mid-CONNECTING: cut again right after restore, then leave up."""
    assert _wait_all(motion, 15.0)
    hub.off(SYS_PORT)
    time.sleep(4.0)
    hub.on(SYS_PORT)
    time.sleep(0.8)          # interrupt mid-connect
    hub.off(SYS_PORT)
    time.sleep(3.0)
    hub.on(SYS_PORT)         # final restore
    assert _wait_all(motion, RECOVER_TIMEOUT + 4)


@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_force_safe_trigger_off_after_reconnect(motion, hub):
    """L4-ish: arm the trigger, cut+restore, assert TriggerStatus==1 (OFF)
    once the console is reachable again (SDK force-safe on connect)."""
    assert _wait_all(motion, 15.0)
    cfg = motion.resolve_trigger_config()
    motion.console.set_trigger_json(data=cfg)
    motion.console.start_trigger()
    try:
        assert motion.console.get_trigger_json()["TriggerStatus"] == 2
        hub.off(SYS_PORT)
        time.sleep(4.0)
        hub.on(SYS_PORT)
        assert motion.console.wait_for(ConnectionState.CONNECTED, timeout=RECOVER_TIMEOUT)
        time.sleep(0.5)
        assert motion.console.get_trigger_json()["TriggerStatus"] == 1, (
            "console did not force-safe the trigger on reconnect"
        )
    finally:
        try:
            motion.console.stop_trigger(timeout=2)
        except Exception:
            pass


@pytest.mark.console
@pytest.mark.sensor
@pytest.mark.slow
def test_mains_cycle_recovers(motion, hub):
    """Full mains power cycle via Shelly + restore -> all reconnect."""
    sys.path.insert(0, r"C:\Users\ethan\Projects\openmotion-bloodflow-app\tests")
    try:
        import shelly
        outlet = shelly.ShellyOutlet(os.environ.get("SHELLY_IP_ADDRESS", "192.168.1.81"))
        outlet.is_on()
    except Exception as e:
        pytest.skip(f"Shelly outlet not available: {e}")
    assert _wait_all(motion, 15.0)
    outlet.power_cycle(off_time=6.0, settle_time=1.0)
    assert _wait_all(motion, 25.0), "system did not reconnect after mains cycle"
```

- [ ] **Step 2: Confirm the bench is healthy, then run**

```bash
python -c "import time; from omotion.MotionInterface import MotionInterface; m=MotionInterface(); m.start(wait=False); time.sleep(3); print(m.console.state.name, m.left.state.name, m.right.state.name); m.stop()"
```
Expected: `CONNECTED CONNECTED CONNECTED`. If not, mains-cycle via `python C:/Users/ethan/Projects/openmotion-bloodflow-app/tests/shelly.py cycle --off-time 6` first.

Run: `pytest tests/test_connection_recovery.py -v -s`
Expected: PASS (or clean SKIP if hardware absent).

- [ ] **Step 3: Commit**

```bash
git add tests/test_connection_recovery.py
git commit -m "test: HIL disconnect/reconnect recovery + force-safe"
```

---

## Task 9: Manual full lifecycle verification + docs + cleanup

**Why:** The acceptance requirement is that **both links are cut at every lifecycle stage and handled correctly**. Automated tests cover L1/L2/L4 and force-safe; this task drives the remaining stages (L0, L3, L5, L6, L7) once on the bench, records results, updates docs, and removes scratch.

**Files:**
- Modify: `docs/TestSuite.md`, `docs/Architecture.md` (connection-monitor section), `CLAUDE.md` (transport/hotplug notes)
- Delete: `scratch/char_connection.py`, `scratch/char_laser.py` (superseded by `tests/test_connection_recovery.py`)

- [ ] **Step 1: Drive the remaining lifecycle stages manually and record**

For each of L0 (cut before `start()`), L3 (cut during scan bring-up/configure), L5 (cut during scan teardown), L6 (cut during a CalibrationWorkflow run), L7 (rapid repeated cut), run a short script that: starts the system, reaches the stage, cuts `hub.off(1)` + `hub.on(1)`, and asserts (a) no Python exception escapes, (b) all three return to CONNECTED within ~8 s (after final restore), (c) `get_trigger_json()["TriggerStatus"] == 1`, and (d) a fresh `start_scan` succeeds afterwards. Capture a short pass/fail note per stage. Mains-cycle via Shelly between trials if any wedge is observed (and record it as a residual firmware issue per the companion spec).

Expected: every stage recovers; any stage that needs a mains cycle is the firmware-watchdog case and is recorded, not a blocker for the SDK changes.

- [ ] **Step 2: Update docs**

In `docs/Architecture.md`, update the ConnectionMonitor section: monitor is now a pure detector/router; each handle owns a `ConnectWorker`; connects are bounded/interruptible; console force-safes the trigger on connect. In `docs/TestSuite.md`, add `test_connect_worker.py`, `test_uart_cancel.py`, `test_monitor_nonblocking.py` (software) and `test_connection_recovery.py` (HIL). In `CLAUDE.md`, update the "Three transport threads" gotcha to note the per-handle connect workers and the combined-cut bench constraint.

- [ ] **Step 3: Remove scratch harnesses**

```bash
git rm scratch/char_connection.py scratch/char_laser.py
```

- [ ] **Step 4: Final regression run**

Run: `pytest tests/ -m "not console and not sensor and not destructive and not fpga and not imu" -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add docs/ CLAUDE.md
git commit -m "docs: connection-monitor redesign; remove scratch harnesses"
```

---

## Self-review notes

- **Spec coverage:** monitor-as-detector + per-handle workers (Tasks 3,5,6,7); bounded/interruptible reads (Tasks 1,2); force-safe on connect (Tasks 4,5); retry-storm cooldown (Task 3); lifecycle × transport matrix combined-cut (Tasks 7,9); software non-blocking regression for failure mode A (Task 7); firmware watchdog is spec-only (companion doc, not in this plan). All covered.
- **Decision D1:** combined-cut only — reflected in Task 7/9 (no console-only/sensor-only cells).
- **Type/signature consistency:** `do_connect(reason, abort_evt) -> bool` and `do_disconnect(reason)` match `_drive_connecting`/`_drive_disconnecting` in both handles; `ConnectWorker.post(intent, reason)` / `.stop()` used consistently; `cancel_evt` param name identical across `MotionUart` and `CommInterface`; `stop_trigger(timeout=...)` matches its force-safe call site.
```
