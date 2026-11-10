"""Progress reporting shared by the DFU steps and :mod:`omotion.firmware_update`."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

__all__ = ["DFUProgress", "ProgressCallback", "RawProgressCallback", "progress_adapter"]


@dataclass(frozen=True)
class DFUProgress:
    #: "erase", "download", "verify", "wait" or "power_cycle".
    phase: str
    percent: int | None
    bytes_written: int | None
    message: str
    elapsed_s: float


ProgressCallback = Callable[[DFUProgress], None]

#: What :class:`omotion.dfu.STM32DFU` calls: ``(done, total, msg="")``.
RawProgressCallback = Callable[..., None]


def progress_adapter(
    progress_cb: ProgressCallback | None,
    phase: str,
    *,
    t0: float | None = None,
) -> RawProgressCallback | None:
    """Wrap ``progress_cb`` as the ``(done, total, msg)`` callback STM32DFU wants.

    ``done``/``total`` are sectors while erasing and bytes while downloading; a
    call carrying ``msg`` is an erase announcement even inside ``download()``.
    """
    if progress_cb is None:
        return None
    start = time.monotonic() if t0 is None else t0

    def _cb(done: int, total: int, msg: str = "") -> None:
        percent = int(done * 100 / total) if total else 100
        is_erase = phase == "erase" or bool(msg)
        progress_cb(DFUProgress(
            phase="erase" if is_erase else phase,
            percent=percent,
            bytes_written=None if is_erase else done,
            message=msg,
            elapsed_s=time.monotonic() - start,
        ))

    return _cb
