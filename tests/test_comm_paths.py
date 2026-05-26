"""
Communication path tests (Section 4 of the test plan).

Verifies that packets travel over the correct transport, that
no commands are silently dropped, and that the dual-sensor USB
topology maps devices to the expected side.
"""

import struct
import threading
import time

import numpy as np
import pytest

from omotion.CommandError import CommandError

pytestmark = pytest.mark.sensor

# Minimal BFI calibration arrays for SciencePipeline (shape: modules × cameras)
_BFI_ZEROS = np.zeros((2, 8), dtype=np.float32)
_BFI_ONES = np.ones((2, 8), dtype=np.float32) * 10.0


def _decode_raw_histogram(raw):
    """Decode the 4100-byte payload returned by camera_get_histogram.

    Layout: 4096 bytes of histogram (1024 × uint32 little-endian)
            followed by 4 bytes of float32 temperature.

    Returns:
        (histogram, temperature_c) — numpy uint32 array of length 1024
        and a float temperature in degrees Celsius.
    """
    assert len(raw) == 4100, f"Expected 4100 bytes from camera_get_histogram, got {len(raw)}"
    histogram = np.frombuffer(bytes(raw[:4096]), dtype=np.uint32)
    temperature_c = struct.unpack_from("<f", bytes(raw), 4096)[0]
    return histogram, temperature_c


# ===========================================================================
# 4.1 UART framing (console path)
# ===========================================================================

@pytest.mark.console
def test_uart_sync_mode_blocking(console):
    """A synchronous ping must return True and not hang."""
    assert console.ping() is True


# ===========================================================================
# 4.2 USB bulk command path (sensor)
# ===========================================================================

def test_comm_interface_response_routing(any_sensor):
    """Back-to-back pings should both return True without cross-contamination."""
    assert any_sensor.ping() is True
    assert any_sensor.ping() is True


@pytest.mark.slow
def test_comm_interface_no_missed_acks(any_sensor):
    """
    100 echo commands with unique payloads — every response must arrive
    and match its request.  This is the primary 'no missed comms' check
    for the USB command endpoint.
    """
    n = 100
    results = []

    for i in range(n):
        payload = bytes([i & 0xFF, (i >> 8) & 0xFF])
        data, length = any_sensor.echo(payload)
        results.append((payload, data, length))

    for i, (sent, received, length) in enumerate(results):
        assert length == len(sent), f"Echo {i}: length mismatch ({length} != {len(sent)})"
        assert received == sent, f"Echo {i}: payload mismatch"

    assert len(results) == n


def test_comm_interface_concurrent_pings(any_sensor):
    """
    Two threads each send 10 pings concurrently.  All must succeed,
    verifying the per-request queue routing inside MotionComposite.
    """
    errors = []

    def ping_worker():
        for _ in range(10):
            try:
                result = any_sensor.ping()
                if result is not True:
                    errors.append(f"ping returned {result!r}")
            except Exception as exc:
                errors.append(str(exc))

    threads = [threading.Thread(target=ping_worker) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=15)

    assert not errors, f"Concurrent ping failures: {errors}"


# ===========================================================================
# 4.3 Stream endpoint isolation
# ===========================================================================

@pytest.mark.slow
def test_stream_histogram_data_returned(any_sensor):
    """
    After a full camera bring-up and histogram capture, the raw bytes
    returned by camera_get_histogram must be 4100 bytes and decode to
    1024 histogram bins + float32 temperature.
    """
    ok = any_sensor.enable_camera_power(0x01)
    if ok is False:
        pytest.fail(
            "enable_camera_power returned False — "
            "TCA9548A I2C mux may be stuck (err: HAL_ERROR); power cycle the sensor"
        )
    time.sleep(0.5)
    ok = any_sensor.program_fpga(camera_position=0x01, manual_process=False)
    if ok is False:
        any_sensor.disable_camera_power(0x01)
        pytest.fail("program_fpga returned False — FPGA bitstream load failed")
    time.sleep(0.1)
    ok = any_sensor.camera_configure_registers(0x01)
    if ok is False:
        any_sensor.disable_camera_power(0x01)
        pytest.fail("camera_configure_registers returned False")

    try:
        any_sensor.camera_capture_histogram(0x01)
        raw = any_sensor.camera_get_histogram(0x01)
        assert isinstance(raw, (bytes, bytearray)) and len(raw) == 4100, (
            f"camera_get_histogram returned {len(raw) if raw else 0} bytes (expected 4100)"
        )
        histogram, temperature_c = _decode_raw_histogram(raw)
        assert len(histogram) == 1024, "Expected 1024 histogram bins"
        assert isinstance(temperature_c, float)
    finally:
        any_sensor.disable_camera_power(0x01)


@pytest.mark.slow
def test_stream_interface_no_data_loss(any_sensor, motion):
    """
    Stream histograms for 2 s and assert SciencePipeline receives frames
    with no gaps in absolute_frame_id.
    """
    import queue as _queue
    from omotion.MotionProcessing import (
        create_science_pipeline,
        parse_histogram_packet_structured,
        MIN_PACKET_SIZE,
    )

    # Determine which side this parametrised sensor is on, so the pipeline
    # mask and enqueue() calls use the correct side name.
    side = next(
        (s for s, sensor in (motion.sensors or {}).items() if sensor is any_sensor),
        "left",
    )
    left_mask = 0x01 if side == "left" else 0x00
    right_mask = 0x01 if side == "right" else 0x00

    # Full bring-up: power → 500 ms settle → program_fpga → 100 ms settle → configure
    any_sensor.enable_camera_power(0x01)
    time.sleep(0.5)
    ok = any_sensor.program_fpga(camera_position=0x01, manual_process=False)
    if ok is False:
        pytest.fail("program_fpga(0x01) returned False")
    time.sleep(0.1)
    any_sensor.camera_configure_registers(0x01)
    any_sensor.enable_aggregator_fsin()

    frames = []

    def on_science_frame(frame):
        frames.append(frame)

    # Pipeline expects only the one active camera (mask bit 0 = cam_id 0) on
    # this side.  Using 0xFF on both sides would require 16 cameras to report
    # before a ScienceFrame is emitted, which will never happen here.
    pipeline = create_science_pipeline(
        left_camera_mask=left_mask,
        right_camera_mask=right_mask,
        bfi_c_min=_BFI_ZEROS,
        bfi_c_max=_BFI_ONES,
        bfi_i_min=_BFI_ZEROS,
        bfi_i_max=_BFI_ONES,
        on_science_frame_fn=on_science_frame,
    )

    # Wire the sensor's USB histogram stream into the pipeline.
    # StreamInterface puts raw USB bytes into stream_q; the parser thread
    # converts them into HistogramSamples and calls pipeline.enqueue().
    stream_q = _queue.Queue()
    stream_stop = threading.Event()
    any_sensor.uart.histo.start_streaming(stream_q, expected_size=32837)

    def _parser_loop():
        buf = bytearray()
        while not stream_stop.is_set() or not stream_q.empty():
            try:
                chunk = stream_q.get(timeout=0.1)
                buf.extend(chunk)
                stream_q.task_done()
            except _queue.Empty:
                continue
            offset = 0
            while offset + MIN_PACKET_SIZE <= len(buf):
                try:
                    packet = parse_histogram_packet_structured(memoryview(buf[offset:]))
                    for sample in packet.samples:
                        pipeline.enqueue(
                            side,
                            sample.cam_id,
                            sample.frame_id,
                            sample.timestamp_s,
                            sample.histogram,
                            sample.row_sum,
                            sample.temperature_c,
                        )
                    offset += packet.bytes_consumed
                except ValueError:
                    offset += 1
            if offset > 0:
                del buf[:offset]

    parser_t = threading.Thread(target=_parser_loop, daemon=True)
    parser_t.start()

    any_sensor.enable_camera(0x01)

    try:
        time.sleep(2.0)
    finally:
        try:
            any_sensor.disable_camera(0x01)
        finally:
            try:
                any_sensor.disable_aggregator_fsin()
            finally:
                try:
                    any_sensor.uart.histo.stop_streaming()
                finally:
                    stream_stop.set()
                    parser_t.join(timeout=3.0)
                    try:
                        pipeline.stop()
                    finally:
                        any_sensor.disable_camera_power(0x01)

    assert len(frames) > 0, "No science frames received during 2 s stream"

    abs_ids = [f.absolute_frame for f in frames]
    for prev, curr in zip(abs_ids, abs_ids[1:]):
        assert curr == prev + 1, (
            f"Frame ID gap detected: {prev} → {curr} "
            f"(missing {curr - prev - 1} frame(s))"
        )


@pytest.mark.slow
def test_dual_sensor_independent_pings(sensor_left, sensor_right):
    """Both sensors must respond to ping simultaneously without interference."""
    results = {}
    errors = {}

    def do_ping(side, sensor):
        try:
            results[side] = sensor.ping()
        except Exception as exc:
            errors[side] = str(exc)

    threads = [
        threading.Thread(target=do_ping, args=("left", sensor_left)),
        threading.Thread(target=do_ping, args=("right", sensor_right)),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=10)

    assert not errors, f"Ping errors: {errors}"
    assert results.get("left") is True, "Left sensor ping failed"
    assert results.get("right") is True, "Right sensor ping failed"
