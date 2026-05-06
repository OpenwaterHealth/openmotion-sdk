import logging
from omotion import _log_root

logger = logging.getLogger(f"{_log_root}.Signal" if _log_root else "Signal")


class MotionSignal:
    """Lightweight signal/slot shim used when PyQt6 is unavailable.

    Mirrors the subset of `pyqtSignal` that the SDK relies on:
    `connect(slot)`, `disconnect(slot)`, and `emit(*args, **kwargs)`.
    Slots are invoked synchronously on the thread that calls `emit()`.
    """

    def __init__(self):
        self._slots = []

    def connect(self, slot):
        if callable(slot) and slot not in self._slots:
            self._slots.append(slot)

    def disconnect(self, slot):
        if slot in self._slots:
            self._slots.remove(slot)

    def emit(self, *args, **kwargs):
        for slot in self._slots:
            try:
                slot(*args, **kwargs)
            except Exception as e:
                logger.error("Signal emit error in slot %s: %s", slot, e)
