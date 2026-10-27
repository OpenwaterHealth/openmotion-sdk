from types import SimpleNamespace

import usb.core

from omotion.StreamInterface import StreamInterface


def test_drain_final_keeps_endpoint_snapshot_after_concurrent_release():
    """Interface release must not invalidate an in-progress final drain."""

    class ReleasingDevice:
        def __init__(self):
            self.read_calls = []

        def read(self, endpoint_address, length, *, timeout):
            self.read_calls.append((endpoint_address, length, timeout))
            if len(self.read_calls) == 1:
                stream.ep_in = None
                return b"final histogram bytes"
            raise usb.core.USBTimeoutError("endpoint empty", -7, 110)

    device = ReleasingDevice()
    stream = StreamInterface(device, interface_index=1, desc="TEST-HISTO")
    stream.ep_in = SimpleNamespace(bEndpointAddress=0x81)

    chunks = stream.drain_final(expected_size=4096, timeout_ms=7)

    assert chunks == [b"final histogram bytes"]
    assert device.read_calls == [(0x81, 4096, 7), (0x81, 4096, 7)]
