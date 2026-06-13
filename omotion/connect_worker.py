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

        self._mailbox: queue.Queue[tuple[str, str]] = queue.Queue()
        self._abort = threading.Event()
        self._stop = threading.Event()
        self._last_fail_t = 0.0

    # posted from the monitor thread (non-blocking)
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
        if self.is_alive():
            # do_connect ignored the abort and outran the join. The thread is
            # a daemon so the process can still exit, but a caller that
            # proceeds to tear the handle down may race this still-running
            # connect — surface it loudly.
            logger.warning(
                "%s connect worker still alive 5s after stop(); a connect "
                "attempt is ignoring the abort signal", self._label,
            )

    # worker thread body
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
                # Don't clear _abort here — _run_connect owns clearing it (so a
                # disconnect that coalescing collapsed into a later connect is
                # still consumed correctly). A lingering set abort with no
                # connect pending is harmless: only _run_connect and stop()
                # read it.
                try:
                    self._do_disconnect(reason)
                except Exception:
                    logger.exception("%s do_disconnect raised", self._label)

    def _coalesce(self, intent: str, reason: str):
        """Drain queued intents, keeping only the most recent. Collapses the
        burst of PollArrived events the monitor emits every 200 ms. The kept
        intent is the latest desired state; any abort set by a superseded
        disconnect is consumed by _run_connect."""
        dropped = 0
        while True:
            try:
                intent, reason = self._mailbox.get_nowait()
                dropped += 1
            except queue.Empty:
                if dropped:
                    logger.debug(
                        "%s coalesced %d superseded intent(s); latest=%s",
                        self._label, dropped, intent,
                    )
                return intent, reason

    def _run_connect(self, reason: str) -> None:
        # Consume any abort set by a superseded disconnect before doing
        # anything: coalescing may have collapsed a "disconnect" into this
        # "connect" without running the disconnect branch, leaving _abort set.
        # Clearing it here (before the cooldown wait) keeps a stale abort from
        # silently skipping the connect we're about to attempt.
        self._abort.clear()
        # Cooldown after a recent failure (interruptible by stop/disconnect).
        if self._cooldown > 0 and self._last_fail_t:
            remaining = self._cooldown - (time.monotonic() - self._last_fail_t)
            if remaining > 0 and self._abort.wait(remaining):
                return  # superseded by disconnect/stop during cooldown
        try:
            ok = self._do_connect(reason, self._abort)
        except Exception:
            logger.exception("%s do_connect raised", self._label)
            ok = False
        if not ok:
            self._last_fail_t = time.monotonic()
        else:
            self._last_fail_t = 0.0
