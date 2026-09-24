"""StreamInterface host-side backpressure instrumentation.

The USB read loop puts each chunk onto the per-side packet queue. When the
downstream parser/runner stalls, that put blocks. Previously a put only
surfaced anything after blocking a full second (the drop path); sub-second
stalls — which are exactly what overflowed the sensor firmware's 4-deep
histo queue in the 2026-06 soak — were invisible. These tests cover the
per-scan counters that make those stalls (and any host-side drops) visible.
"""

from __future__ import annotations

import queue
import threading
import time

from omotion.StreamInterface import StreamInterface


def _si() -> StreamInterface:
    # dev is never touched by the enqueue path; construction is side-effect free.
    return StreamInterface(dev=None, interface_index=0, desc="Test")


def test_enqueue_chunk_no_stall_when_queue_has_space():
    si = _si()
    q: queue.Queue = queue.Queue(maxsize=4)

    assert si._enqueue_chunk(q, b"frame") is True
    assert si.backpressure_stalls == 0
    assert si.dropped_chunks == 0
    assert si.packets_received == 1
    assert q.get_nowait() == b"frame"


def test_enqueue_chunk_counts_a_sub_second_stall():
    si = _si()
    si._backpressure_warn_s = 0.05
    q: queue.Queue = queue.Queue(maxsize=1)
    q.put(b"occupied")  # queue starts full so the next put must block

    # Free a slot ~0.12 s later — a sub-second (but >warn) host stall.
    def _free_slot():
        time.sleep(0.12)
        q.get()

    threading.Thread(target=_free_slot, daemon=True).start()

    t0 = time.monotonic()
    assert si._enqueue_chunk(q, b"frame") is True
    elapsed = time.monotonic() - t0

    assert elapsed >= 0.05
    assert si.backpressure_stalls == 1
    assert si.backpressure_stall_s >= 0.05
    assert si.dropped_chunks == 0
    assert si.packets_received == 1


def test_enqueue_chunk_drops_when_queue_full_past_timeout():
    si = _si()
    si._put_timeout_s = 0.05  # keep the test fast; production drops only after 1 s
    q: queue.Queue = queue.Queue(maxsize=1)
    q.put(b"occupied")  # never freed → put times out → chunk dropped

    assert si._enqueue_chunk(q, b"frame") is False
    assert si.dropped_chunks == 1
    assert si.backpressure_stalls == 0
    assert si.packets_received == 0


def test_drop_warning_not_suppressed_by_recent_stall_warning(caplog):
    """Stall and drop warnings rate-limit on separate tokens: a benign
    sub-second stall warning moments earlier must not swallow the first
    real data-loss (drop) warning."""
    import logging

    si = _si()
    si._backpressure_warn_s = 0.01
    si._put_timeout_s = 0.05

    # 1) A stall: queue full, freed after ~30 ms -> stall warning fires.
    q: queue.Queue = queue.Queue(maxsize=1)
    q.put(b"occupied")

    def _free_slot():
        time.sleep(0.03)
        q.get()

    threading.Thread(target=_free_slot, daemon=True).start()
    with caplog.at_level(logging.WARNING):
        assert si._enqueue_chunk(q, b"frame") is True
        assert si.backpressure_stalls == 1

        # 2) Immediately after: a real drop (the queue still holds the chunk
        #    from step 1, so it is full and never freed this time). Must warn
        #    despite the stall warning having just consumed its own token.
        assert si._enqueue_chunk(q, b"frame2") is False
        assert si.dropped_chunks == 1

    drop_warnings = [r for r in caplog.records if "dropping" in r.getMessage()]
    assert len(drop_warnings) == 1


def test_start_streaming_resets_backpressure_counters(monkeypatch):
    si = _si()
    si.backpressure_stalls = 7
    si.backpressure_stall_s = 3.0
    si.dropped_chunks = 2
    si.packets_received = 99

    # Don't run the real USB loop — just verify start_streaming clears stats.
    monkeypatch.setattr(si, "_stream_loop", lambda: None)
    si.start_streaming(queue.Queue(), expected_size=10)
    si.stop_streaming()

    assert si.backpressure_stalls == 0
    assert si.backpressure_stall_s == 0.0
    assert si.dropped_chunks == 0
    assert si.packets_received == 0
