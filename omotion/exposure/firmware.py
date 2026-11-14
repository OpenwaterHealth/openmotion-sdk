"""Console firmware gate for Optimize Laser's extended laser limits.

Optimize Laser drives the laser past the stock safety settings (bloodflow-app
docs/auto_exposure/IMPLEMENTATION_PLAN.md section 5): pulses up to 2096 us
with the Safety EE/OPT ``PULSE_WIDTH_UL`` raised, ``RATE_LL`` lowered for the
2400 us dark skip, and ``DRIVE_CL`` raised for a 5000 mA start. Console
firmware 1.8.2-rc.3 and later compile in a safety envelope
(``Core/Inc/safety_envelope.h``) that refuses those register writes, while
the dark skip, a trigger setting rather than a register write, would still
go through. A 2400 us skip under an unlowered ``RATE_LL`` latches the
interlock (E-202) on the first dark frame. So the SDK refuses the whole set
on such firmware before anything touches the hardware.

The check fails closed. Extended limits are allowed only on a console that
reports an exact release tag known to predate the envelope (checked tag by
tag in openmotion-console-fw). A build between tags (git describe
``1.8.1-12-gabc1234``), a dirty build, ``unknown``, the SDK's ``v0.0.0``
placeholder for a console that didn't answer, or anything else unparseable
is refused, because nothing says what such a build contains.

1.8.2-rc.2 adds an I2C pass-through filter (CFW-08) that checks register
map, length and trigger state but not values, so it is allowed here; the
actuator's readbacks catch a write it refuses.
"""

from __future__ import annotations

import re

# Exact release tags only: MAJOR.MINOR.PATCH, optional pre-release, optional "v".
_TAG_RE = re.compile(r"^[vV]?(\d+)\.(\d+)\.(\d+)(?:-([0-9A-Za-z]+(?:\.[0-9A-Za-z]+)*))?$")
_GIT_DESCRIBE_TAIL = re.compile(r"-\d+-g[0-9a-f]+(?:-dirty)?$", re.IGNORECASE)

ENVELOPE_RELEASE = (1, 8, 2)
ENVELOPE_SINCE = "1.8.2-rc.3"

# 1.8.2 pre-releases without safety_envelope.h. Every release below 1.8.2
# predates it; every other 1.8.2 build, and everything after, is refused.
PRE_ENVELOPE_PRERELEASES = frozenset({"dev.1", "dev.3", "dev.4", "dev.5", "rc.1", "rc.2"})


class ExtendedLimitsRefused(RuntimeError):
    """The console's firmware can't take Optimize Laser's extended limits."""


def extended_limits_problem(version: str | None) -> str | None:
    """Why a console reporting ``version`` can't take the extended limits,
    or ``None`` if it can. ``version`` is ``MotionConsole.get_version()``."""
    text = (version or "").strip()
    if not text or text.lower() == "unknown":
        return "the console did not report a firmware version"
    if text.lower().endswith("-dirty") or _GIT_DESCRIBE_TAIL.search(text):
        return (f"console firmware {text!r} is a build between release tags, so whether "
                "it has the safety envelope is unknown")
    m = _TAG_RE.match(text)
    if not m:
        return f"console firmware version {text!r} is not a release tag"
    release = tuple(int(g) for g in m.group(1, 2, 3))
    prerelease = m.group(4)
    if release == (0, 0, 0):
        return "the console did not report a firmware version"
    if release < ENVELOPE_RELEASE:
        return None
    if release == ENVELOPE_RELEASE and prerelease in PRE_ENVELOPE_PRERELEASES:
        return None
    return (f"console firmware {text} has the compiled-in safety envelope "
            f"(since {ENVELOPE_SINCE}), which refuses the extended laser limits "
            "Optimize Laser needs; flash the study firmware, 1.8.1")


def require_extended_limits(version: str | None) -> None:
    """Raise :class:`ExtendedLimitsRefused` unless the console can take the
    extended limits. Call it before the first laser or safety-limit write."""
    problem = extended_limits_problem(version)
    if problem:
        raise ExtendedLimitsRefused(problem)
