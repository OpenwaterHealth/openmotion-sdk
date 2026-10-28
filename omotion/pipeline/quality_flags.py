"""The ``quality`` flag list of a corrected frame (bloodflow-app#589).

A corrected frame's ``quality``, and with it ``session_data.quality`` and
the export's ``quality_l1`` … ``quality_r8`` cells, is a ``;``-separated
list of flags. A frame with no flag reads ``"ok"``. Flags always come out in
the same order, so one set of flags is always stored as the same string
(section numbers are SciencePipeline.md's):

  * ``ts_corrected``: the frame's timestamp was re-anchored (§5.4);
  * ``nan_filled``: a placeholder for a frame that never arrived (§5.4);
  * ``low_light``: an unlit camera (§5.8.4, #341);
  * ``poor_contact`` / ``ambient_light``: the live contact-quality warning
    latched for the camera at that frame, i.e. what the operator was shown
    (``omotion.contact_quality.ContactQualityMonitor``, bloodflow-app#589).
    Stamped by ScanDBSink on per-camera rows only.

A flag not listed here sorts after these, in the order it was first seen.

``FrameBatch.quality`` (TimestampRepairStage output) holds a single flag per
row; lists only appear once frames are corrected.
"""

from __future__ import annotations

OK = "ok"
SEP = ";"

TS_CORRECTED = "ts_corrected"
NAN_FILLED = "nan_filled"
LOW_LIGHT = "low_light"
POOR_CONTACT = "poor_contact"
AMBIENT_LIGHT = "ambient_light"

_ORDER = {flag: i for i, flag in enumerate(
    (TS_CORRECTED, NAN_FILLED, LOW_LIGHT, POOR_CONTACT, AMBIENT_LIGHT))}


def parse(value) -> list[str]:
    """The flags in a stored value, without ``ok``. Accepts None."""
    if value is None:
        return []
    flags = (token.strip() for token in str(value).split(SEP))
    return [flag for flag in flags if flag and flag != OK]


def join(*values) -> str:
    """Merge flag lists into one value: each flag once, in the fixed order,
    or ``ok`` when there is none."""
    seen: dict[str, None] = {}
    for value in values:
        for flag in parse(value):
            seen.setdefault(flag, None)
    flags = sorted(seen, key=lambda flag: _ORDER.get(flag, len(_ORDER)))
    return SEP.join(flags) if flags else OK


def has(value, flag: str) -> bool:
    """True when ``value`` carries ``flag``."""
    return flag in parse(value)
