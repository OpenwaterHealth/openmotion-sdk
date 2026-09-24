"""Pure composite-of-three-USB-interfaces wrapper.

`MotionComposite` no longer owns connection lifecycle — the owning
`MotionSensor` drives `open()` / `close()` from its state machine.
Read-thread USB errors on the comm interface are reported up to the sensor
via the `on_io_error(errno, message)` callback, which the sensor wires to
submit an `IoError` event to the `ConnectionMonitor`.
"""
from __future__ import annotations

import logging
from typing import Callable, Optional

import usb.core
import usb.util

from omotion.CommInterface import CommInterface
from omotion.StreamInterface import StreamInterface
from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.MotionComposite" if _log_root else "MotionComposite"
)


class MotionComposite:
    """A USB device with three interfaces: comm (cmd/resp), histo (stream),
    imu (stream). The owning `MotionSensor` provides lifecycle management."""

    def __init__(
        self,
        dev,
        desc: str = "COMPOSITE",
        async_mode: bool = False,
        on_io_error: Optional[Callable[[Optional[int], str], None]] = None,
        on_stream_stall: Optional[Callable[[str, float], None]] = None,
    ):
        self.dev = dev
        self.desc = desc
        self.async_mode = async_mode
        self.demo_mode = False
        self.on_io_error = on_io_error
        self.on_stream_stall = on_stream_stall

        self.comm = CommInterface(
            dev, 0, desc=f"{desc}-COMM", async_mode=True
        )
        self.histo = StreamInterface(dev, 1, desc=f"{desc}-HISTO")
        self.imu = StreamInterface(dev, 2, desc=f"{desc}-IMU")
        self.comm.on_io_error = self._forward_io_error
        # #192: until now only the command interface had any route to report
        # a fault, so a histogram stream that went silent mid-scan reached
        # nobody at all. Only the histo stream is watched — it has a
        # guaranteed ~40 fps cadence to measure against, whereas the IMU
        # stream can be legitimately idle and would false-positive.
        self.histo.on_stream_stall = self._forward_stream_stall
        self.imu.stall_timeout_sec = 0

        self.packet_count = 0

    # ────────────────────────────────────────────────────────────────────
    # Lifecycle (driven by MotionSensor state machine)
    # ────────────────────────────────────────────────────────────────────

    def open(self) -> None:
        """Configure the device and claim all three interfaces. Raises on
        failure; the caller should clean up via `close()`."""
        self.dev.set_configuration()
        self.comm.claim()
        self.histo.claim()
        self.imu.claim()
        if self.comm.async_mode:
            self.comm.start_read_thread()
        logger.info(f"{self.desc}: opened")

    def close(self) -> None:
        """Stop and join all transport threads, release all three interfaces
        and free USB resources. Idempotent, and safe on a composite that was
        never (or only partially) opened — the connect retry loop relies on
        that to roll back a failed open(). Each step runs regardless of
        whether earlier steps fail — failures are logged with enough context
        to diagnose without aborting cleanup."""
        steps = []
        if getattr(self.comm, "async_mode", False):
            steps.append(("stop comm threads", self.comm.stop_read_thread))
        steps += [
            ("stop histo streaming", self.histo.stop_streaming),
            ("stop imu streaming", self.imu.stop_streaming),
            ("release comm", self.comm.release),
            ("release histo", self.histo.release),
            ("release imu", self.imu.release),
            ("dispose usb resources", lambda: usb.util.dispose_resources(self.dev)),
        ]
        for label, step in steps:
            try:
                step()
            except Exception as e:
                logger.warning("%s: close step '%s' failed: %s", self.desc, label, e)
        logger.info(f"{self.desc}: closed")

    # ────────────────────────────────────────────────────────────────────

    def _forward_io_error(self, errno: Optional[int], message: str) -> None:
        cb = self.on_io_error
        if cb is None:
            return
        try:
            cb(errno, message)
        except Exception as e:
            logger.warning("on_io_error callback raised: %s", e)

    def _forward_stream_stall(self, stalled_for_sec: float) -> None:
        """Forward a stalled histogram stream, tagged with this side's desc
        so the consumer knows which sensor went quiet (#192)."""
        cb = self.on_stream_stall
        if cb is None:
            return
        try:
            cb(self.histo.desc, stalled_for_sec)
        except Exception as e:
            logger.warning("on_stream_stall callback raised: %s", e)
