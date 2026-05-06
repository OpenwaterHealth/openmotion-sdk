"""Platform-specific hotplug providers for the connection monitor.

A provider is anything implementing ``subscribe(on_change) -> unsubscribe``.
The monitor uses ``detect_hotplug()`` to pick the best impl available; if
nothing platform-specific works, it falls back to ``PollOnlyHotplugProvider``,
which is a no-op (the monitor's 200 ms poll sweep handles everything).
"""
from __future__ import annotations

import logging
import sys

from omotion import _log_root
from omotion.hotplug.poll_only import PollOnlyHotplugProvider

logger = logging.getLogger(
    f"{_log_root}.Hotplug" if _log_root else "Hotplug"
)


def detect_hotplug():
    """Return the best hotplug provider for this platform, or
    PollOnlyHotplugProvider if none is available. Never raises."""
    if sys.platform == "win32":
        try:
            from omotion.hotplug.win32 import Win32HotplugProvider

            provider = Win32HotplugProvider()
            logger.info("Using Win32 hotplug provider")
            return provider
        except Exception as e:
            logger.warning(
                "Win32 hotplug provider unavailable, falling back to poll-only: %s", e
            )
    else:
        try:
            from omotion.hotplug.libusb_hotplug import LibusbHotplugProvider

            provider = LibusbHotplugProvider()
            logger.info("Using libusb hotplug provider")
            return provider
        except Exception as e:
            logger.warning(
                "libusb hotplug provider unavailable, falling back to poll-only: %s", e
            )

    return PollOnlyHotplugProvider()


__all__ = ["detect_hotplug", "PollOnlyHotplugProvider"]
