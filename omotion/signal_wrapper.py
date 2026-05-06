"""Per-handle Qt signal base class.

Each handle (`MotionConsole`, `MotionSensor`) is a `SignalWrapper` so apps
can connect Qt slots to its `signal_state_changed` signal in the usual way.
When PyQt6 is not installed, `SignalWrapper` falls back to the lightweight
`MotionSignal` shim (synchronous, single-threaded delivery).

The signal signature is:
    signal_state_changed(handle, old_state, new_state, reason: str)

`handle` is the emitting handle itself, so a single slot can dispatch on
`handle.name`. `old_state` and `new_state` are `ConnectionState` values.
`reason` is a short tag like ``"poll_arrived"``, ``"usb_io_error:errno=19"``,
``"connect_retry_exhausted"``, ``"user_stop"`` — useful for logging without
parsing.
"""
import logging
from omotion import _log_root

logger = logging.getLogger(
    f"{_log_root}.SignalWrapper" if _log_root else "SignalWrapper"
)

try:
    from PyQt6.QtCore import QObject, pyqtSignal

    PYQT_AVAILABLE = True
    logger.info("PyQt6 is available. SignalWrapper will emit real Qt signals.")
except ImportError:
    PYQT_AVAILABLE = False
    logger.warning("PyQt6 is NOT available. SignalWrapper will use shim signals.")
    QObject = object
    from omotion.MotionSignal import MotionSignal


class SignalWrapper(QObject if PYQT_AVAILABLE else object):
    """Base for any class that needs to emit `signal_state_changed`."""

    if PYQT_AVAILABLE:
        # (handle, old_state, new_state, reason)
        signal_state_changed = pyqtSignal(object, object, object, str)

        def __init__(self):
            super().__init__()
    else:

        def __init__(self):
            self.signal_state_changed = MotionSignal()
