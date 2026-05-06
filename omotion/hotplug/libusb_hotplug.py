"""libusb1-based hotplug listener for Linux/macOS.

Uses the lower-level ``libusb1`` package (a project dep) to register a
hotplug callback and pump events on a background thread. Calls
``on_change()`` on every USB add/remove. No VID/PID filter is set;
the monitor's poll sweep handles identification.

This module raises at import time if libusb1 hotplug is unavailable
(for example, very old libusb without ``libusb_has_capability(HOTPLUG)``),
which causes ``detect_hotplug()`` to fall back to ``PollOnlyHotplugProvider``.
"""
from __future__ import annotations

import logging
import threading

import libusb1
import usb1

from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.Hotplug.Libusb" if _log_root else "Hotplug.Libusb"
)


class LibusbHotplugProvider:
    def __init__(self):
        self._context = usb1.USBContext()
        if not self._context.hasCapability(libusb1.LIBUSB_CAP_HAS_HOTPLUG):
            raise RuntimeError("libusb on this system does not support hotplug")
        self._on_change = None
        self._handle = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def subscribe(self, on_change):
        if self._thread is not None:
            raise RuntimeError("LibusbHotplugProvider is single-subscription")
        self._on_change = on_change

        def _callback(context, device, event):
            cb = self._on_change
            if cb is not None:
                try:
                    cb()
                except Exception:
                    logger.exception("on_change callback raised")
            return 0  # keep the callback registered

        self._handle = self._context.hotplugRegisterCallback(
            _callback,
            events=(
                libusb1.LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED
                | libusb1.LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT
            ),
        )

        self._thread = threading.Thread(
            target=self._pump, name="MotionHotplugLibusb", daemon=True
        )
        self._thread.start()

        def _unsubscribe():
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            if self._handle is not None:
                try:
                    self._context.hotplugDeregisterCallback(self._handle)
                except Exception:
                    logger.exception("hotplugDeregisterCallback failed")
                self._handle = None
            try:
                self._context.close()
            except Exception:
                logger.exception("USBContext.close failed")

        return _unsubscribe

    def _pump(self):
        # Block in libusb event handling for up to 200 ms at a time so the
        # stop event is checked promptly without consuming CPU.
        while not self._stop.is_set():
            try:
                self._context.handleEventsTimeout(tv_sec=0, tv_usec=200000)
            except Exception:
                logger.exception("libusb handleEventsTimeout failed")
                break
