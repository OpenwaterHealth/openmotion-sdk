"""Correction-status vocabulary for the corrected record (bloodflow-app#589).

``session_data.correction_status`` and the exported ``correction_status``
column hold a comma-separated list of the pipeline corrections applied to a
sample. An empty string means the sample is clean. The value used to be a
single worst-wins ``quality`` flag whose clean value was ``"ok"``; readers
still accept ``"ok"`` because rows written before schema migration 3 hold it.

A row that describes one camera (per-camera ``session_data`` rows) lists the
statuses without a camera tag, since the row's ``side`` / ``cam_id`` already
say which camera it is. A value that aggregates several cameras (side-average
rows, one export row per frame) tags each entry with the camera it came from:
``l3:ts_corrected,r5:nan_filled``. Tags are ``l``/``r`` plus the 1-based
camera number, the same suffix as the export's ``bfi_l3`` columns.
"""

from __future__ import annotations

from typing import Iterable

STATUS_TS_CORRECTED = "ts_corrected"
STATUS_NAN_FILLED = "nan_filled"

# Values that mean "no correction" in a single-status field.
_CLEAN = frozenset({"", "ok"})


def parse(value) -> list[str]:
    """Split a stored status list into its entries, dropping clean markers.

    Accepts None, the legacy ``"ok"``, and entries with or without a camera
    tag.
    """
    if value is None:
        return []
    out = []
    for token in str(value).split(","):
        token = token.strip()
        if token and token not in _CLEAN:
            out.append(token)
    return out


def join(entries: Iterable[str]) -> str:
    """Comma-join entries, dropping clean markers and duplicates (first wins)."""
    seen: dict[str, None] = {}
    for entry in entries:
        for token in parse(entry):
            seen.setdefault(token, None)
    return ",".join(seen)


def cam_tag(side, cam_id: int) -> str:
    """``l3`` / ``r5`` for a side (``"left"``/``"right"`` or 0/1) and a
    0-based camera id."""
    if side in ("left", 0):
        prefix = "l"
    elif side in ("right", 1):
        prefix = "r"
    else:
        raise ValueError(f"unknown side {side!r}")
    return f"{prefix}{int(cam_id) + 1}"


def tagged(tag: str, value) -> list[str]:
    """Prefix each entry of ``value`` with ``tag:``."""
    return [f"{tag}:{token}" for token in parse(value)]


def split_tagged(value) -> dict[str, list[str]]:
    """Group a camera-tagged list by tag: ``{"l3": ["ts_corrected"], ...}``.
    Untagged entries are ignored."""
    out: dict[str, list[str]] = {}
    for token in parse(value):
        tag, sep, status = token.partition(":")
        if sep and status:
            out.setdefault(tag, []).append(status)
    return out
