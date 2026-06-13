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
