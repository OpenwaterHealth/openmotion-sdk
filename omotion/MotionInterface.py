"""Top-level facade: stable handles + a single connection monitor.

`MotionInterface` constructs three handles (`console`, `left`, `right`)
that live for the entire lifetime of the interface and are **never
replaced**. Apps subscribe to each handle's `signal_state_changed` once
and cache the reference forever.

Connection lifecycle is owned by a single daemon `ConnectionMonitor`
thread, fed by an event queue. OS hotplug (Win32 `WM_DEVICECHANGE` or
libusb hotplug) wakes the monitor for sub-50 ms detection; a 200 ms
poll sweep handles fallbacks.
"""
from __future__ import annotations

import logging
import platform
import socket
from typing import Any, Iterable, Optional

from omotion.MotionConsole import MotionConsole
from omotion.MotionSensor import MotionSensor
from omotion.connection_monitor import ConnectionMonitor
from omotion.connection_state import ConnectionState
from omotion.Calibration import Calibration
from omotion.config import (
    CONSOLE_MODULE_PID,
    DEFAULT_TRIGGER_CONFIG,
    SENSOR_MODULE_PID,
    merge_trigger_config,
)
from omotion import __version__ as _SDK_VERSION, _log_root

logger = logging.getLogger(
    f"{_log_root}.Interface" if _log_root else "Interface"
)


class MotionInterface:
    """Top-level entry point. Construct once, call ``start()`` once, then
    use ``console``/``left``/``right`` as long-lived handles."""

    def __init__(
        self,
        vid: int = 0x0483,
        sensor_pid: int = SENSOR_MODULE_PID,
        console_pid: int = CONSOLE_MODULE_PID,
        baudrate: int = 921600,
        timeout: int = 30,
        demo_mode: bool = False,
        default_trigger_config: Optional[dict] = None,
    ):
        self.vid = vid
        self.sensor_pid = sensor_pid
        self.console_pid = console_pid

        # Resolved default trigger config used by every workflow whose
        # request doesn't carry a ``trigger_config`` override. Stored
        # as the merge of (SDK default, app-supplied override) so the
        # lookup is a single dict access. ``default_trigger_config``
        # property returns a fresh defensive copy.
        self._default_trigger_config: dict = merge_trigger_config(
            default_trigger_config
        )

        # The three stable handles. None of these are ever replaced — apps
        # cache them once and connect signals once.
        self.console = MotionConsole(
            vid=vid,
            pid=console_pid,
            baudrate=baudrate,
            timeout=timeout,
            demo_mode=demo_mode,
        )
        self.left = MotionSensor(side="left", vid=vid, pid=sensor_pid)
        self.right = MotionSensor(side="right", vid=vid, pid=sensor_pid)

        # ScanWorkflow is constructed lazily so we don't pull in Qt-heavy
        # dependencies at import time for users who only want device control.
        self._scan_workflow = None
        self._calibration_workflow = None
        self._monitor: Optional[ConnectionMonitor] = None
        self._started = False

    @property
    def default_trigger_config(self) -> dict:
        """A defensive copy of the resolved default trigger config —
        :data:`omotion.config.DEFAULT_TRIGGER_CONFIG` shallow-merged
        with whatever the constructor caller passed for
        ``default_trigger_config``. Workflows fall back to this when
        their request doesn't override."""
        return dict(self._default_trigger_config)

    def resolve_trigger_config(self, override: Optional[dict] = None) -> dict:
        """Return a complete trigger-config dict — the resolved default
        with ``override`` shallow-merged on top. Callers that know they
        want a specific tweak (e.g. ``TriggerStatus: 1`` to disarm)
        pass just the changed keys; missing keys fall through to the
        default."""
        return merge_trigger_config(self._default_trigger_config, override)

    # ──────────────────────────────────────────────────────────────────
    # Lifecycle
    # ──────────────────────────────────────────────────────────────────

    def start(self, wait: bool = True, wait_timeout: float = 2.0) -> None:
        """Spin up the connection monitor and (by default) block until any
        already-attached devices have reached CONNECTED, or ``wait_timeout``
        seconds elapse — whichever comes first.

        Synchronous; no asyncio loop required by the caller.
        """
        if self._started:
            return

        # Lazy hotplug discovery so import works on systems where ctypes
        # bindings or libusb hotplug are unavailable.
        from omotion.hotplug import detect_hotplug

        hotplug = detect_hotplug()

        self._monitor = ConnectionMonitor(
            console=self.console,
            left=self.left,
            right=self.right,
            console_vid=self.vid,
            console_pid=self.console_pid,
            sensor_vid=self.vid,
            sensor_pid=self.sensor_pid,
            hotplug=hotplug,
        )

        # Wire each handle to the monitor so they can submit IO/UserStop
        # events from any thread (read threads, app callers, etc.).
        self.console._attach_monitor(self._monitor)
        self.left._attach_monitor(self._monitor)
        self.right._attach_monitor(self._monitor)

        self._monitor.start()
        self._started = True
        logger.info("MotionInterface started")

        if wait:
            self.wait_for_ready(
                console=False, sensors=0, timeout=wait_timeout,
                require_attached_only=True,
            )

    def stop(self) -> None:
        """Stop the monitor and tear down all handles. Blocks until the
        monitor thread joins."""
        if not self._started or self._monitor is None:
            return
        self._monitor.request_stop()
        self._monitor.join(timeout=5.0)
        self._monitor = None
        self._started = False
        logger.info("MotionInterface stopped")

    # ──────────────────────────────────────────────────────────────────
    # State queries
    # ──────────────────────────────────────────────────────────────────

    def is_device_connected(self) -> tuple[bool, bool, bool]:
        """Return (console_connected, left_connected, right_connected)."""
        return (
            self.console.is_connected(),
            self.left.is_connected(),
            self.right.is_connected(),
        )

    def connected_sensors(self) -> list[MotionSensor]:
        """Return the list of currently-connected sensor handles."""
        return [s for s in (self.left, self.right) if s.is_connected()]

    def wait_for_ready(
        self,
        *,
        console: bool = True,
        sensors: int = 0,
        timeout: float = 10.0,
        require_attached_only: bool = False,
    ) -> bool:
        """Block until requested handles reach CONNECTED, or ``timeout``.

        Args:
            console: if True, require the console to be CONNECTED.
            sensors: minimum number of sensors required to be CONNECTED
                (0/1/2). Ignored if ``require_attached_only`` is True.
            timeout: hard cap on wait time, in seconds.
            require_attached_only: if True, only wait for handles that have
                begun their CONNECTING transition (i.e. devices already
                attached at start time) — used by ``start(wait=True)`` so
                an unplugged sensor doesn't block startup.

        Returns:
            True if requirements were met before timeout; False otherwise.
        """
        import time

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if require_attached_only:
                # We're done as soon as nothing is in CONNECTING; either it
                # made it to CONNECTED or fell back to DISCONNECTED.
                if all(
                    h.state != ConnectionState.CONNECTING
                    for h in (self.console, self.left, self.right)
                ):
                    return True
            else:
                ok_console = (not console) or self.console.is_connected()
                ok_sensors = len(self.connected_sensors()) >= sensors
                if ok_console and ok_sensors:
                    return True
            time.sleep(0.05)
        return False

    # ──────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────

    def run_on_sensors(
        self,
        func_name: str,
        *args,
        target: str | Iterable[str] | None = None,
        include_disconnected: bool = True,
        **kwargs,
    ) -> dict[str, Any]:
        """Run a MotionSensor method on selected sensors and return results.

        Args:
            func_name: Name of the MotionSensor method to call.
            *args: Positional args.
            target: ``None``/``"all"``/``"*"`` for both, or ``"left"``/
                ``"right"``, or an iterable of side names.
            include_disconnected: include disconnected sensors with value
                None in the result, vs. skipping them.
            **kwargs: Keyword args.
        """
        all_sensors = {"left": self.left, "right": self.right}
        if target is None or (
            isinstance(target, str) and target.lower() in ("all", "*")
        ):
            selected = set(all_sensors.keys())
        elif isinstance(target, str):
            selected = {target.lower()}
        else:
            selected = {str(t).lower() for t in target}

        unknown = selected - set(all_sensors.keys())
        if unknown:
            logger.warning(f"Unknown sensor target(s): {sorted(unknown)}")

        results: dict[str, Any] = {}
        for name, sensor in all_sensors.items():
            if name not in selected:
                continue
            if sensor.is_connected():
                method = getattr(sensor, func_name, None)
                if callable(method):
                    try:
                        results[name] = method(*args, **kwargs)
                    except Exception as e:
                        logger.error(f"Error running {func_name} on {name}: {e}")
                        results[name] = None
                else:
                    logger.error(f"{func_name} is not a valid MotionSensor method")
                    results[name] = None
            elif include_disconnected:
                logger.warning(f"{name} sensor not connected.")
                results[name] = None
        return results

    # ──────────────────────────────────────────────────────────────────
    # Scan workflow passthroughs (lazy ScanWorkflow construction)
    # ──────────────────────────────────────────────────────────────────

    @property
    def scan_workflow(self):
        if self._scan_workflow is None:
            from omotion.ScanWorkflow import ScanWorkflow

            self._scan_workflow = ScanWorkflow(self)
        return self._scan_workflow

    @property
    def calibration_workflow(self):
        if self._calibration_workflow is None:
            from omotion.CalibrationWorkflow import CalibrationWorkflow

            self._calibration_workflow = CalibrationWorkflow(self)
        return self._calibration_workflow

    @property
    def calibration_running(self) -> bool:
        return (
            self._calibration_workflow is not None
            and self._calibration_workflow.running
        )

    def start_scan(self, request, **kwargs) -> bool:
        return self.scan_workflow.start_scan(request, **kwargs)

    def cancel_scan(self, **kwargs) -> None:
        self.scan_workflow.cancel_scan(**kwargs)

    def start_calibration(self, request, **kwargs) -> bool:
        return self.calibration_workflow.start_calibration(request, **kwargs)

    def cancel_calibration(self, **kwargs) -> None:
        if self._calibration_workflow is not None:
            self._calibration_workflow.cancel_calibration(**kwargs)

    def get_single_histogram(
        self,
        side: str,
        camera_id: int,
        test_pattern_id: int = 4,
        auto_upload: bool = True,
    ):
        return self.scan_workflow.get_single_histogram(
            side=side,
            camera_id=camera_id,
            test_pattern_id=test_pattern_id,
            auto_upload=auto_upload,
        )

    def start_configure_camera_sensors(self, request, **kwargs) -> bool:
        return self.scan_workflow.start_configure_camera_sensors(request, **kwargs)

    def cancel_configure_camera_sensors(self, **kwargs) -> None:
        self.scan_workflow.cancel_configure_camera_sensors(**kwargs)

    # ──────────────────────────────────────────────────────────────────
    # Logging helpers
    # ──────────────────────────────────────────────────────────────────

    def log_system_info(self) -> None:
        """Log host system and SDK version information."""
        try:
            logger.info("--- System Information ---")
            logger.info("Hostname:    %s", socket.gethostname())
            logger.info("Platform:    %s", platform.platform())
            logger.info("System:      %s %s", platform.system(), platform.release())
            logger.info("Arch:        %s", platform.machine())
            logger.info("Processor:   %s", platform.processor())
            logger.info(
                "Python:      %s (%s)",
                platform.python_version(),
                platform.python_implementation(),
            )
            logger.info("SDK version: %s", _SDK_VERSION)

            if platform.system() == "Windows":
                try:
                    import ctypes

                    class _MEMSTATUSEX(ctypes.Structure):
                        _fields_ = [
                            ("dwLength", ctypes.c_ulong),
                            ("dwMemoryLoad", ctypes.c_ulong),
                            ("ullTotalPhys", ctypes.c_ulonglong),
                            ("ullAvailPhys", ctypes.c_ulonglong),
                            ("ullTotalPageFile", ctypes.c_ulonglong),
                            ("ullAvailPageFile", ctypes.c_ulonglong),
                            ("ullTotalVirtual", ctypes.c_ulonglong),
                            ("ullAvailVirtual", ctypes.c_ulonglong),
                            ("ullAvailExtendedVirtual", ctypes.c_ulonglong),
                        ]

                    mem = _MEMSTATUSEX()
                    mem.dwLength = ctypes.sizeof(_MEMSTATUSEX)
                    ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(mem))
                    logger.info("RAM:         %.2f GB", mem.ullTotalPhys / (1024 ** 3))
                except Exception:
                    pass
        except Exception as e:
            logger.warning("Failed to log system information: %s", e)

    def log_console_info(self) -> None:
        if self.console.is_connected():
            self.console.log_device_info()
            self._load_calibration_from_console()

    def _load_calibration_from_console(self) -> None:
        """Read calibration from the console and install it into ScanWorkflow.

        Best-effort: any failure is logged and the existing cache is kept.
        Called automatically on console-connect via ``log_console_info``;
        also exposed publicly via ``refresh_calibration``.
        """
        try:
            cal = self.console.read_calibration()
        except Exception as e:
            logger.warning(
                "Could not load calibration from console: %s. "
                "Keeping existing cached calibration (source=%s).",
                e, self.scan_workflow._calibration.source,
            )
            return
        self.scan_workflow._install_calibration(cal)

    def refresh_calibration(self) -> Calibration:
        """Re-read calibration from the console and update the cache.

        Returns the resulting :class:`Calibration` (the same value
        accessible via :meth:`get_calibration`).
        """
        self._load_calibration_from_console()
        return self.scan_workflow._calibration

    def get_calibration(self) -> Calibration:
        """Return the currently cached calibration."""
        return self.scan_workflow._calibration

    def write_calibration(
        self, c_min, c_max, i_min, i_max
    ) -> Calibration:
        """Validate inputs, write the calibration to the console EEPROM,
        then read it back into the cache. Returns the cached value.
        """
        self.console.write_calibration(c_min, c_max, i_min, i_max)
        return self.refresh_calibration()

    def log_sensor_info(self, side: str) -> None:
        sensor = self.left if side == "left" else self.right if side == "right" else None
        if sensor and sensor.is_connected():
            sensor.log_device_info()

    @staticmethod
    def get_sdk_version() -> str:
        return _SDK_VERSION
