"""Win32 USB hotplug listener via ``WM_DEVICECHANGE``.

Owns a daemon thread that creates a hidden message-only window and registers
for USB device-interface notifications using ``DEVICE_NOTIFY_ALL_INTERFACE_CLASSES``
(no GUID filter — we just want to know that USB topology changed; the
monitor's poll sweep figures out exactly what). On every ``DBT_DEVICEARRIVAL``
or ``DBT_DEVICEREMOVECOMPLETE`` we call the registered ``on_change()``.

The thread pumps ``GetMessage``/``DispatchMessage`` until ``unsubscribe()``
posts ``WM_QUIT``.
"""
from __future__ import annotations

import ctypes
import ctypes.wintypes as wt
import logging
import threading

from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.Hotplug.Win32" if _log_root else "Hotplug.Win32"
)

# ── Win32 constants ─────────────────────────────────────────────────────────
WM_DEVICECHANGE = 0x0219
WM_DESTROY = 0x0002
WM_QUIT = 0x0012

DBT_DEVICEARRIVAL = 0x8000
DBT_DEVICEREMOVECOMPLETE = 0x8004

DBT_DEVTYP_DEVICEINTERFACE = 0x00000005
DEVICE_NOTIFY_WINDOW_HANDLE = 0x00000000
DEVICE_NOTIFY_ALL_INTERFACE_CLASSES = 0x00000004

HWND_MESSAGE = wt.HWND(-3)

CW_USEDEFAULT = -2147483648  # 0x80000000 as signed int

# ── Structs ─────────────────────────────────────────────────────────────────


class _GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", ctypes.c_ulong),
        ("Data2", ctypes.c_ushort),
        ("Data3", ctypes.c_ushort),
        ("Data4", ctypes.c_ubyte * 8),
    ]


class _DEV_BROADCAST_DEVICEINTERFACE_W(ctypes.Structure):
    _fields_ = [
        ("dbcc_size", wt.DWORD),
        ("dbcc_devicetype", wt.DWORD),
        ("dbcc_reserved", wt.DWORD),
        ("dbcc_classguid", _GUID),
        ("dbcc_name", wt.WCHAR * 1),
    ]


# LRESULT is pointer-sized (LONG_PTR); c_long would truncate on 64-bit.
_LRESULT = ctypes.c_ssize_t

_WNDPROC = ctypes.WINFUNCTYPE(
    _LRESULT, wt.HWND, wt.UINT, wt.WPARAM, wt.LPARAM
)


class _WNDCLASSEXW(ctypes.Structure):
    _fields_ = [
        ("cbSize", wt.UINT),
        ("style", wt.UINT),
        ("lpfnWndProc", _WNDPROC),
        ("cbClsExtra", ctypes.c_int),
        ("cbWndExtra", ctypes.c_int),
        ("hInstance", wt.HINSTANCE),
        ("hIcon", wt.HICON),
        ("hCursor", wt.HANDLE),
        ("hbrBackground", wt.HBRUSH),
        ("lpszMenuName", wt.LPCWSTR),
        ("lpszClassName", wt.LPCWSTR),
        ("hIconSm", wt.HICON),
    ]


# ── DLL bindings ────────────────────────────────────────────────────────────
_user32 = ctypes.WinDLL("user32", use_last_error=True)
_kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

_user32.RegisterClassExW.argtypes = [ctypes.POINTER(_WNDCLASSEXW)]
_user32.RegisterClassExW.restype = wt.ATOM
_user32.UnregisterClassW.argtypes = [wt.LPCWSTR, wt.HINSTANCE]
_user32.UnregisterClassW.restype = wt.BOOL

_user32.CreateWindowExW.argtypes = [
    wt.DWORD, wt.LPCWSTR, wt.LPCWSTR, wt.DWORD,
    ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    wt.HWND, wt.HMENU, wt.HINSTANCE, wt.LPVOID,
]
_user32.CreateWindowExW.restype = wt.HWND
_user32.DestroyWindow.argtypes = [wt.HWND]
_user32.DestroyWindow.restype = wt.BOOL

_user32.DefWindowProcW.argtypes = [wt.HWND, wt.UINT, wt.WPARAM, wt.LPARAM]
_user32.DefWindowProcW.restype = _LRESULT

_user32.GetMessageW.argtypes = [
    ctypes.POINTER(wt.MSG), wt.HWND, wt.UINT, wt.UINT,
]
_user32.GetMessageW.restype = wt.BOOL
_user32.TranslateMessage.argtypes = [ctypes.POINTER(wt.MSG)]
_user32.TranslateMessage.restype = wt.BOOL
_user32.DispatchMessageW.argtypes = [ctypes.POINTER(wt.MSG)]
_user32.DispatchMessageW.restype = ctypes.c_long
_user32.PostThreadMessageW.argtypes = [
    wt.DWORD, wt.UINT, wt.WPARAM, wt.LPARAM,
]
_user32.PostThreadMessageW.restype = wt.BOOL

_user32.RegisterDeviceNotificationW.argtypes = [
    wt.HANDLE, wt.LPVOID, wt.DWORD,
]
_user32.RegisterDeviceNotificationW.restype = wt.HANDLE
_user32.UnregisterDeviceNotification.argtypes = [wt.HANDLE]
_user32.UnregisterDeviceNotification.restype = wt.BOOL

_kernel32.GetModuleHandleW.argtypes = [wt.LPCWSTR]
_kernel32.GetModuleHandleW.restype = wt.HMODULE
_kernel32.GetCurrentThreadId.restype = wt.DWORD


# ── Module-level WNDPROC trampoline ─────────────────────────────────────────
# The window class is registered once per process and (deliberately) never
# unregistered, so the WNDPROC it points at must outlive every provider
# instance. A per-instance ctypes thunk cannot: the first provider's thunk
# becomes the class WNDPROC forever, and once that instance is garbage
# collected the pointer dangles — the next message dispatched through the
# class (including the WM_DESTROY that DestroyWindow sends during a *newer*
# instance's teardown) jumps into freed memory and kills the process with an
# access violation. Route everything through this immortal module-level
# trampoline instead, and look up the owning instance by window handle.
_instances: dict[int, "Win32HotplugProvider"] = {}


def _trampoline(hwnd, msg, wparam, lparam):
    provider = _instances.get(hwnd)
    if provider is not None:
        return provider._wndproc_impl(hwnd, msg, wparam, lparam)
    # Sent before CreateWindowExW returns (WM_NCCREATE/WM_CREATE) or after
    # teardown has unregistered the hwnd.
    return _user32.DefWindowProcW(hwnd, msg, wparam, lparam)


_MODULE_WNDPROC = _WNDPROC(_trampoline)


# ────────────────────────────────────────────────────────────────────────────


class Win32HotplugProvider:
    """Hidden message-only window that calls on_change() on USB add/remove."""

    _CLASS_NAME = "OmotionHotplugListener"

    def __init__(self):
        self._on_change = None
        self._thread: threading.Thread | None = None
        self._tid: int | None = None
        self._ready = threading.Event()
        self._hinstance = _kernel32.GetModuleHandleW(None)
        self._hwnd = None
        self._notify_handle = None
        self._class_atom = 0

    # ── Public ──────────────────────────────────────────────────────────────

    def subscribe(self, on_change):
        if self._thread is not None:
            raise RuntimeError("Win32HotplugProvider is single-subscription")
        self._on_change = on_change
        self._thread = threading.Thread(
            target=self._run, name="MotionHotplugWin32", daemon=True
        )
        self._thread.start()
        # Wait for the message loop to actually be up so subscribe() does not
        # return before notifications can be received.
        self._ready.wait(timeout=2.0)

        def _unsubscribe():
            if self._tid is not None:
                # Posting WM_QUIT to the thread breaks GetMessage out of its
                # blocking wait so the loop exits.
                _user32.PostThreadMessageW(self._tid, WM_QUIT, 0, 0)
            if self._thread is not None:
                self._thread.join(timeout=2.0)

        return _unsubscribe

    # ── Thread body ─────────────────────────────────────────────────────────

    def _run(self):
        self._tid = _kernel32.GetCurrentThreadId()
        try:
            self._register_class()
            self._create_window()
            self._register_device_notifications()
        except Exception:
            logger.exception("Win32 hotplug setup failed")
            # Release whatever partial setup succeeded (window, registry
            # entry) so a failed provider doesn't leak its hwnd.
            self._teardown()
            self._ready.set()
            return

        self._ready.set()

        # Pump messages until WM_QUIT.
        msg = wt.MSG()
        while True:
            ret = _user32.GetMessageW(ctypes.byref(msg), None, 0, 0)
            if ret == 0:
                # WM_QUIT
                break
            if ret == -1:
                # GetMessage error; bail.
                logger.error(
                    "GetMessageW returned -1 (GetLastError=%d); exiting hotplug loop",
                    ctypes.get_last_error(),
                )
                break
            _user32.TranslateMessage(ctypes.byref(msg))
            _user32.DispatchMessageW(ctypes.byref(msg))

        self._teardown()

    def _register_class(self):
        wc = _WNDCLASSEXW()
        wc.cbSize = ctypes.sizeof(_WNDCLASSEXW)
        wc.lpfnWndProc = _MODULE_WNDPROC
        wc.hInstance = self._hinstance
        wc.lpszClassName = self._CLASS_NAME
        atom = _user32.RegisterClassExW(ctypes.byref(wc))
        if not atom:
            err = ctypes.get_last_error()
            # 1410 == ERROR_CLASS_ALREADY_EXISTS — fine, reuse it: the class
            # WNDPROC is the module-level trampoline for every instance.
            if err != 1410:
                raise OSError(err, "RegisterClassExW failed")
        self._class_atom = atom or 0

    def _create_window(self):
        hwnd = _user32.CreateWindowExW(
            0,
            self._CLASS_NAME,
            "OmotionHotplug",
            0,
            CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
            HWND_MESSAGE,
            None,
            self._hinstance,
            None,
        )
        if not hwnd:
            err = ctypes.get_last_error()
            raise OSError(err, "CreateWindowExW failed")
        self._hwnd = hwnd
        # Route messages for this window to this instance. Must happen before
        # device notifications are registered so no WM_DEVICECHANGE can race
        # the registry entry.
        _instances[hwnd] = self

    def _register_device_notifications(self):
        nf = _DEV_BROADCAST_DEVICEINTERFACE_W()
        nf.dbcc_size = ctypes.sizeof(_DEV_BROADCAST_DEVICEINTERFACE_W)
        nf.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE
        # dbcc_classguid is ignored when DEVICE_NOTIFY_ALL_INTERFACE_CLASSES is set.
        h = _user32.RegisterDeviceNotificationW(
            self._hwnd,
            ctypes.byref(nf),
            DEVICE_NOTIFY_WINDOW_HANDLE | DEVICE_NOTIFY_ALL_INTERFACE_CLASSES,
        )
        if not h:
            err = ctypes.get_last_error()
            raise OSError(err, "RegisterDeviceNotificationW failed")
        self._notify_handle = h

    def _teardown(self):
        if self._notify_handle is not None:
            try:
                _user32.UnregisterDeviceNotification(self._notify_handle)
            except Exception:
                logger.exception("UnregisterDeviceNotification failed")
            self._notify_handle = None
        if self._hwnd is not None:
            # Unregister first: WM_DESTROY/WM_NCDESTROY from DestroyWindow
            # then fall through the trampoline to DefWindowProcW instead of
            # dispatching into a half-torn-down instance.
            _instances.pop(self._hwnd, None)
            try:
                _user32.DestroyWindow(self._hwnd)
            except Exception:
                logger.exception("DestroyWindow failed")
            self._hwnd = None
        # Don't UnregisterClassW — the class is shared by every instance in
        # the process and its WNDPROC (the module-level trampoline) is
        # immortal, so leaving it registered is safe and harmless.

    # ── Window proc ─────────────────────────────────────────────────────────

    def _wndproc_impl(self, hwnd, msg, wparam, lparam):
        if msg == WM_DEVICECHANGE:
            if wparam in (DBT_DEVICEARRIVAL, DBT_DEVICEREMOVECOMPLETE):
                # Don't do work in the WindowProc — message dispatch must
                # remain quick. The on_change callback is expected to just
                # submit an event to a queue, which is non-blocking.
                cb = self._on_change
                if cb is not None:
                    try:
                        cb()
                    except Exception:
                        logger.exception("on_change callback raised")
            return 1  # TRUE: we handled it
        if msg == WM_DESTROY:
            return 0
        return _user32.DefWindowProcW(hwnd, msg, wparam, lparam)
