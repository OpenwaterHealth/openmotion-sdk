"""Host-side buffering between the USB reader and the scan thread (#116).

The scan thread runs every pipeline stage and sink inline, so when it is busy
(a dark-interval close with its DB writes, a GC pause, a slow disk) nothing
drains the queues behind it. Once the per-side packet queue fills, the USB
reader stops reading and the sensor, which buffers only ~5 frames, drops
frames on both modules at once. These tests pin the queue to a time budget
and show that a scan-thread stall no longer costs frames.
"""

from __future__ import annotations

import array
import collections
import logging
import queue
import struct
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest
import usb.core

from omotion.config import TYPE_HISTO
from omotion.MotionProcessing import (
    EOF, EOH, HISTO_SIZE_WORDS, SOF, SOH, _crc16,
)
from omotion.StreamInterface import StreamInterface
from omotion.pipeline.sinks import ScanMetadata
from omotion.pipeline.sources import (
    PACKET_BUFFER_SECONDS, LiveUsbSource, packet_queue_depth_for,
)

FULL_SUM = 1920 * 1280 + 6
PERIOD_S = 0.025


def _meta():
    return ScanMetadata(
        scan_id="x", subject_id="y", operator="z",
        started_at_iso="2026-09-24T00:00:00Z", duration_sec=5,
        left_camera_mask=0xFF, right_camera_mask=0xFF, reduced_mode=False,
    )


class _FakeSensor:
    def __init__(self, histo):
        self.uart = SimpleNamespace(histo=histo)


# ---------------------------------------------------------------------------
# Sizing
# ---------------------------------------------------------------------------

def test_default_packet_queue_holds_ten_seconds_of_uncompressed_output():
    # An uncompressed 8-camera packet (32837 B) takes 9 reads of 4096 B; at
    # 40 Hz that is 360 reads/s, so 10 s is 3600 reads.
    assert packet_queue_depth_for(10.0) == 3600
    src = LiveUsbSource(console=None, left=_FakeSensor(None), right=_FakeSensor(None),
                        metadata=_meta())
    expected = packet_queue_depth_for(PACKET_BUFFER_SECONDS)
    assert {q.maxsize for q in src._packet_queues.values()} == {expected}


def test_explicit_packet_queue_size_is_honoured():
    src = LiveUsbSource(console=None, left=_FakeSensor(None), right=None,
                        packet_queue_size=64, metadata=_meta())
    assert src._packet_queues["left"].maxsize == 64


# ---------------------------------------------------------------------------
# High-water reporting
# ---------------------------------------------------------------------------

class _EndlessDev:
    """A HISTO endpoint that always has data."""

    def read(self, ep, size, timeout=None):
        time.sleep(0.001)
        return array.array("B", b"\x00" * 64)


def test_stream_interface_reports_queue_high_water(caplog):
    histo = StreamInterface(_EndlessDev(), 1, desc="TEST-HISTO")
    histo.ep_in = SimpleNamespace(bEndpointAddress=0x81)
    q = queue.Queue(maxsize=8)                    # nobody drains it
    with caplog.at_level(logging.INFO):
        histo.start_streaming(q, expected_size=64)
        deadline = time.monotonic() + 5.0
        while q.qsize() < 8 and time.monotonic() < deadline:
            time.sleep(0.005)
        histo.stop_streaming()
    assert histo.queue_high_water == 8
    text = caplog.text
    assert "packet queue high-water 8/8" in text
    assert "packet queue reached 8/8" in text     # the >= half-full warning


# ---------------------------------------------------------------------------
# A scan-thread stall no longer drops sensor frames
# ---------------------------------------------------------------------------

def _packet(frame: int, ts_ms: int) -> bytes:
    """Uncompressed 8-camera TYPE_HISTO packet with a wire timestamp."""
    blocks = []
    for cam_id in range(8):
        hist = np.zeros(HISTO_SIZE_WORDS, dtype=np.uint32)
        hist[0] = FULL_SUM
        hist[-1] = (frame & 0xFF) << 24
        blocks.append(bytes([SOH, cam_id]) + hist.tobytes()
                      + struct.pack("<f", 30.0) + bytes([EOH]))
    payload = struct.pack("<I", ts_ms) + b"".join(blocks)
    pkt_len = 6 + len(payload) + 3
    header = struct.pack("<BBI", SOF, TYPE_HISTO, pkt_len)
    crc = _crc16(memoryview(header + payload[:-1]))
    return header + payload + struct.pack("<H", crc) + bytes([EOF])


class _Clock:
    """One frame clock for both sides, started by the first host read."""

    def __init__(self):
        self.t0 = None
        self.lock = threading.Lock()

    def start(self) -> float:
        with self.lock:
            if self.t0 is None:
                self.t0 = time.perf_counter()
            return self.t0


class _EmulatedHistoEndpoint:
    """The sensor side of one HISTO endpoint: a frame every 25 ms into the
    firmware TX queue (1 in flight + 4 waiting), which drops new frames while
    full, like `HISTO enqueue fail: queue full`. A read returns at most the
    rest of the in-flight packet (transfers end at the packet boundary)."""

    def __init__(self, packets, clock):
        self.packets = packets
        self.clock = clock
        self.t0 = None
        self.next_i = 0
        self.inflight = None
        self.off = 0
        self.waiting = collections.deque()
        self.drops = []
        self.lock = threading.Lock()

    def _produce_until(self, now):
        while self.next_i < len(self.packets):
            if self.t0 + (self.next_i + 1) * PERIOD_S > now:
                break
            pkt = self.packets[self.next_i]
            if self.inflight is None:
                self.inflight, self.off = pkt, 0
            elif len(self.waiting) < 4:
                self.waiting.append(pkt)
            else:
                self.drops.append(self.next_i + 1)
            self.next_i += 1

    def read(self, ep, size, timeout=None):
        if self.t0 is None:
            self.t0 = self.clock.start()
        deadline = time.perf_counter() + (timeout or 1000) / 1000.0
        while True:
            with self.lock:
                now = time.perf_counter()
                self._produce_until(now)
                if self.inflight is not None:
                    chunk = self.inflight[self.off:self.off + size]
                    self.off += len(chunk)
                    if self.off >= len(self.inflight):
                        self.inflight = self.waiting.popleft() if self.waiting else None
                        self.off = 0
                    return array.array("B", chunk)
                nxt = (self.t0 + (self.next_i + 1) * PERIOD_S
                       if self.next_i < len(self.packets) else None)
            if now >= deadline:
                raise usb.core.USBTimeoutError("Operation timed out", 10060, -7)
            time.sleep(max(0.0, (deadline if nxt is None else min(nxt, deadline)) - now))

    def finish(self):
        if self.t0 is not None:
            with self.lock:
                self._produce_until(time.perf_counter())


@pytest.mark.parametrize("queue_size, expect_loss", [
    (64, True),      # the old fixed depth: ~0.4 s of slack before the reads stop
    (None, False),   # the time-sized default
])
def test_scan_thread_stall_loses_frames_only_with_the_old_queue_depth(queue_size, expect_loss):
    n_frames = 80                                   # 2 s per side
    stall_s = 1.2                                   # a slow dark-interval close
    clock = _Clock()
    endpoints, sensors = {}, {}
    for i, side in enumerate(("left", "right")):
        packets = [_packet(k, 1000 * (i + 1) + k * 25) for k in range(1, n_frames + 1)]
        ep = _EmulatedHistoEndpoint(packets, clock)
        histo = StreamInterface(ep, 1, desc=f"{side.upper()}-HISTO")
        histo.ep_in = SimpleNamespace(bEndpointAddress=0x81 + i)
        endpoints[side], sensors[side] = ep, _FakeSensor(histo)
    src = LiveUsbSource(console=None, left=sensors["left"], right=sensors["right"],
                        packet_queue_size=queue_size, metadata=_meta())

    seen = {"rows": 0}

    def consume():
        stalled = False
        for batch in src:
            seen["rows"] += int(batch.frame_ids.shape[0])
            if not stalled and seen["rows"] >= 16 * 16:  # ~16 frames in
                stalled = True
                time.sleep(stall_s)                      # the scan thread is busy

    t = threading.Thread(target=consume, daemon=True)
    t.start()
    deadline = time.monotonic() + 10.0
    while clock.t0 is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert clock.t0 is not None, "streaming never started"
    time.sleep(max(0.0, clock.t0 + n_frames * PERIOD_S + 0.5 - time.perf_counter()))
    src.close()
    t.join(timeout=15.0)
    assert not t.is_alive()
    for ep in endpoints.values():
        ep.finish()

    dropped = {side: len(ep.drops) for side, ep in endpoints.items()}
    if expect_loss:
        assert dropped["left"] > 0 and dropped["right"] > 0, dropped
    else:
        assert dropped == {"left": 0, "right": 0}, dropped
        assert seen["rows"] == 2 * 8 * n_frames      # every frame reached the consumer
