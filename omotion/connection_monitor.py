"""Single daemon thread that owns the connection lifecycle of all device handles.

All transitions on console/left/right go through this thread, fed by a single
event queue. Event sources (read-thread USB errors, OS hotplug callbacks,
periodic poll sweep, and explicit user stop) just submit events; the monitor
thread serializes them so a handle's state machine is never re-entered.

The monitor does not own the handles — `MotionInterface` does. The monitor
holds references and routes events to the right handle by name or by
VID/PID/port match.
"""
from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Protocol, runtime_checkable

from omotion import _log_root
from omotion.connection_state import ConnectionState

if TYPE_CHECKING:
    from omotion.MotionConsole import MotionConsole
    from omotion.MotionSensor import MotionSensor

logger = logging.getLogger(
    f"{_log_root}.ConnectionMonitor" if _log_root else "ConnectionMonitor"
)


# ──────────────────────────────────────────────────────────────────────────────
# Events
# ──────────────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class _Event:
    """Base for all monitor events. Subclasses are frozen dataclasses so they
    hash and compare cleanly and so accidental mutation is caught."""


@dataclass(frozen=True)
class HotplugWake(_Event):
    """An OS-level hotplug notification: USB topology changed. The monitor
    responds by running an immediate poll sweep. Carries no VID/PID/port —
    that lets platform-specific hotplug code stay tiny (no need to parse
    DEV_BROADCAST_DEVICEINTERFACE structs); the poll sweep figures out
    what actually changed."""


@dataclass(frozen=True)
class IoError(_Event):
    """A read or write thread saw an unrecoverable USB/serial error. The
    handle's state machine treats this as a request to drop to DISCONNECTING."""

    handle_name: str
    errno: Optional[int]
    message: str


@dataclass(frozen=True)
class PollArrived(_Event):
    """The 200 ms poll sweep saw a device the matching handle did not know
    about. Either the OS-level hotplug missed an event, or the device was
    plugged in before the monitor started."""

    handle_name: str


@dataclass(frozen=True)
class PollGone(_Event):
    handle_name: str


@dataclass(frozen=True)
class UserStop(_Event):
    """An app explicitly asked the SDK to stop using this handle."""

    handle_name: str


# ──────────────────────────────────────────────────────────────────────────────
# Hotplug provider protocol
# ──────────────────────────────────────────────────────────────────────────────


@runtime_checkable
class HotplugProvider(Protocol):
    """Implemented by `omotion.hotplug.win32`, `libusb_hotplug`, and
    `poll_only`. The monitor uses whichever is selected by
    `omotion.hotplug.detect_hotplug()`. Providers do not need to know
    VID/PID or device identity — they just notify on any USB topology
    change, and the monitor's poll sweep diffs against the current
    handle states."""

    def subscribe(self, on_change) -> "callable":
        """Begin delivering on_change() callbacks. Returns an
        unsubscribe() callable. on_change takes no args and is safe to
        call from any thread."""
        ...


# ──────────────────────────────────────────────────────────────────────────────
# Handle protocol (what the monitor expects)
# ──────────────────────────────────────────────────────────────────────────────


@runtime_checkable
class _MonitoredHandle(Protocol):
    """The minimal surface a handle must expose to be driven by the monitor.
    `MotionConsole` and `MotionSensor` both satisfy this."""

    name: str
    state: ConnectionState

    def is_connected(self) -> bool: ...
    def _handle_event(self, event: _Event) -> None: ...


# ──────────────────────────────────────────────────────────────────────────────
# Monitor
# ──────────────────────────────────────────────────────────────────────────────


class ConnectionMonitor(threading.Thread):
    POLL_INTERVAL = 0.2  # seconds; doubles as the queue.get() timeout

    def __init__(
        self,
        console: "_MonitoredHandle",
        left: "_MonitoredHandle",
        right: "_MonitoredHandle",
        *,
        console_vid: int,
        console_pid: int,
        sensor_vid: int,
        sensor_pid: int,
        hotplug: Optional[HotplugProvider] = None,
    ):
        super().__init__(daemon=True, name="MotionConnectionMonitor")
        self._console = console
        self._left = left
        self._right = right
        self._console_vid = console_vid
        self._console_pid = console_pid
        self._sensor_vid = sensor_vid
        self._sensor_pid = sensor_pid
        self._hotplug = hotplug
        self._hotplug_unsub = None

        self._queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()

    # ── Public API (called from any thread) ─────────────────────────────────

    def submit(self, event: _Event) -> None:
        """Submit an event from any thread. Safe before run() starts and
        after stop() is called (events submitted post-stop are dropped on
        shutdown drain)."""
        self._queue.put(event)

    def request_stop(self) -> None:
        """Request shutdown. Caller should `.join()` afterwards."""
        self._stop_event.set()
        # Wake the queue.get(timeout) loop immediately with a sentinel.
        self._queue.put(None)

    # ── Thread body ─────────────────────────────────────────────────────────

    def run(self) -> None:
        self._wire_hotplug()

        # Initial sweep so already-attached devices are seen immediately,
        # without waiting for the first POLL_INTERVAL tick.
        self._poll_sweep()

        while not self._stop_event.is_set():
            try:
                event = self._queue.get(timeout=self.POLL_INTERVAL)
            except queue.Empty:
                self._poll_sweep()
                continue

            if event is None:
                # Sentinel from request_stop()
                continue

            try:
                self._dispatch(event)
            except Exception:
                logger.exception("Unhandled exception processing %s", event)

        self._teardown()

    # ── Internals ───────────────────────────────────────────────────────────

    def _wire_hotplug(self) -> None:
        if self._hotplug is None:
            logger.info("ConnectionMonitor running without OS hotplug (poll-only)")
            return
        try:
            self._hotplug_unsub = self._hotplug.subscribe(
                on_change=lambda: self.submit(HotplugWake()),
            )
            logger.info("ConnectionMonitor hotplug subscription active")
        except Exception as e:
            # Falling back to poll-only is correct and visible — log loud.
            logger.warning(
                "Hotplug subscribe failed; falling back to poll-only: %s", e
            )
            self._hotplug_unsub = None

    def _teardown(self) -> None:
        if self._hotplug_unsub is not None:
            try:
                self._hotplug_unsub()
            except Exception as e:
                logger.warning("Hotplug unsubscribe failed: %s", e)

        # Drive each handle to DISCONNECTED so its transport releases cleanly
        # before MotionInterface (and any owning Qt parent) tears down.
        for handle in (self._console, self._left, self._right):
            try:
                handle._handle_event(UserStop(handle_name=handle.name))
            except Exception as e:
                logger.warning("Final disconnect of %s failed: %s", handle.name, e)

        logger.info("ConnectionMonitor exited")

    def _dispatch(self, event: _Event) -> None:
        if isinstance(event, HotplugWake):
            # OS noticed USB topology changed; figure out what before the
            # next 200 ms tick.
            self._poll_sweep()
        elif isinstance(event, (IoError, PollArrived, PollGone, UserStop)):
            handle = self._handle_by_name(event.handle_name)
            if handle is None:
                logger.warning(
                    "Event for unknown handle %r: %s", event.handle_name, event
                )
                return
            handle._handle_event(event)
        else:
            logger.warning("Unknown event type: %s", type(event).__name__)

    def _handle_by_name(self, name: str) -> Optional["_MonitoredHandle"]:
        if name == "console":
            return self._console
        if name == "left":
            return self._left
        if name == "right":
            return self._right
        return None

    # ── Poll sweep: USB enumeration as a hotplug fallback ───────────────────

    def _poll_sweep(self) -> None:
        """Enumerate USB and synthesize PollArrived/PollGone events for any
        observed disagreements with current handle state."""
        self._poll_console()
        self._poll_sensors()

    def _poll_console(self) -> None:
        try:
            import serial.tools.list_ports

            present = any(
                getattr(p, "vid", None) == self._console_vid
                and getattr(p, "pid", None) == self._console_pid
                for p in serial.tools.list_ports.comports()
            )
        except Exception as e:
            logger.debug("Console poll sweep failed: %s", e)
            return

        currently_connected = self._console.is_connected()
        in_progress = self._console.state in (
            ConnectionState.CONNECTING,
            ConnectionState.DISCONNECTING,
        )
        if in_progress:
            return  # Don't perturb a state machine that's already mid-transition.

        if present and not currently_connected:
            self.submit(PollArrived(handle_name="console"))
        elif (not present) and currently_connected:
            self.submit(PollGone(handle_name="console"))

    def _poll_sensors(self) -> None:
        try:
            import usb.core
            from omotion.usb_backend import get_libusb1_backend

            backend = get_libusb1_backend()
            devices = list(
                usb.core.find(
                    find_all=True,
                    idVendor=self._sensor_vid,
                    idProduct=self._sensor_pid,
                    backend=backend,
                )
            )
        except Exception as e:
            logger.debug("Sensor poll sweep failed: %s", e)
            return

        left_present = False
        right_present = False
        for dev in devices:
            try:
                ports = getattr(dev, "port_numbers", []) or []
                if not ports:
                    continue
                if ports[-1] == 2:
                    left_present = True
                elif ports[-1] == 3:
                    right_present = True
            except Exception:
                # Defensive: a device whose port_numbers attribute throws
                # (rare libusb edge case) should not abort the whole sweep.
                continue

        self._maybe_submit_for_sensor(self._left, left_present)
        self._maybe_submit_for_sensor(self._right, right_present)

    def _maybe_submit_for_sensor(
        self, handle: "_MonitoredHandle", present: bool
    ) -> None:
        in_progress = handle.state in (
            ConnectionState.CONNECTING,
            ConnectionState.DISCONNECTING,
        )
        if in_progress:
            return
        if present and not handle.is_connected():
            self.submit(PollArrived(handle_name=handle.name))
        elif (not present) and handle.is_connected():
            self.submit(PollGone(handle_name=handle.name))
