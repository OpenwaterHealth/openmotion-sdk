"""Unit tests for the Win32 hotplug provider's WNDPROC lifetime.

Regression coverage for the dangling-WNDPROC crash (test-app: reliable
0xC0000005 when starting a procedure after a sensor unplug): the window
class ``OmotionHotplugListener`` is registered once per process and never
unregistered, so its WNDPROC must outlive every provider instance. Before
the fix, the first provider's per-instance ctypes thunk became the class
WNDPROC forever; once that instance was garbage-collected, the next
teardown's DestroyWindow dispatched WM_DESTROY through freed memory and
killed the process. The same stale routing also delivered WM_DEVICECHANGE
callbacks to the dead first subscriber instead of the live one.

No hardware needed: WM_DEVICECHANGE is synthesized with SendMessageTimeoutW
(it is a sent-only message — PostMessageW rejects it with
ERROR_MESSAGE_SYNC_ONLY). Note a regression here is fatal by nature: the
subscribe/unsubscribe cycle test kills the whole pytest process with an
access violation rather than failing an assert.
"""
import gc
import sys
import time

import pytest

pytestmark = [
    pytest.mark.unit,
    pytest.mark.skipif(sys.platform != "win32", reason="Win32-only provider"),
]

if sys.platform == "win32":
    import ctypes
    import ctypes.wintypes as wt

    from omotion.hotplug.win32 import (
        DBT_DEVICEARRIVAL,
        DBT_DEVTYP_DEVICEINTERFACE,
        WM_DEVICECHANGE,
        Win32HotplugProvider,
        _DEV_BROADCAST_DEVICEINTERFACE_W,
        _WNDPROC,
        _instances,
    )

    _user32 = ctypes.WinDLL("user32", use_last_error=True)
    _user32.SendMessageTimeoutW.argtypes = [
        wt.HWND, wt.UINT, wt.WPARAM, wt.LPARAM, wt.UINT, wt.UINT,
        ctypes.POINTER(ctypes.c_size_t),
    ]
    _user32.SendMessageTimeoutW.restype = ctypes.c_size_t


def _send_devicechange(hwnd) -> bool:
    """Deliver a synthetic DBT_DEVICEARRIVAL to ``hwnd`` (blocking send)."""
    bcast = _DEV_BROADCAST_DEVICEINTERFACE_W()
    bcast.dbcc_size = ctypes.sizeof(_DEV_BROADCAST_DEVICEINTERFACE_W)
    bcast.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE
    result = ctypes.c_size_t(0)
    ok = _user32.SendMessageTimeoutW(
        hwnd, WM_DEVICECHANGE, DBT_DEVICEARRIVAL,
        ctypes.addressof(bcast), 0, 2000, ctypes.byref(result),
    )
    return bool(ok)


def _gc_and_closure_churn():
    """Collect cyclic garbage (frees dropped providers) and recycle libffi
    closure memory — the in-test stand-in for the allocation pressure a real
    app builds up between procedure runs."""
    gc.collect()
    junk = [_WNDPROC(lambda *a: 0) for _ in range(64)]
    del junk


def test_repeated_subscribe_cycles_survive_provider_gc():
    """MotionInterface.start()/stop() per procedure run means a fresh
    provider per cycle; earlier providers get garbage-collected. Teardown of
    a later provider must not dispatch through a freed WNDPROC."""
    for _ in range(4):
        provider = Win32HotplugProvider()
        unsub = provider.subscribe(lambda: None)
        assert provider._hwnd, "hotplug window was not created"
        _gc_and_closure_churn()
        unsub()  # pre-fix: fatal access violation from cycle 2 onward
        del provider, unsub


def test_events_route_to_current_subscriber_across_cycles():
    """Each cycle's WM_DEVICECHANGE must reach that cycle's on_change, not a
    previously unsubscribed (possibly dead) instance's callback."""
    for cycle in range(1, 4):
        provider = Win32HotplugProvider()
        hits = []
        # Early-bound list: a stale subscriber firing instead of this one
        # must not be able to satisfy the check.
        unsub = provider.subscribe(lambda lst=hits: lst.append(1))
        assert _send_devicechange(provider._hwnd)
        deadline = time.time() + 2.0
        while not hits and time.time() < deadline:
            time.sleep(0.02)
        assert hits, f"cycle {cycle}: on_change never reached the live subscriber"
        _gc_and_closure_churn()
        unsub()
        del provider, unsub


def test_teardown_unregisters_window_from_trampoline_registry():
    """The hwnd -> instance routing entry must not outlive the window."""
    provider = Win32HotplugProvider()
    unsub = provider.subscribe(lambda: None)
    hwnd = provider._hwnd
    assert hwnd in _instances
    unsub()
    assert hwnd not in _instances
