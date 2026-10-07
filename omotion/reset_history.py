"""Why did a sensor module last go down?

``OW_CMD_RESET_HISTORY`` (0x10, openmotion-sensor-fw#137) returns the
firmware's persistent reset record. It lives in ``.noinit`` RAM, so it
survives every reset except a power loss, and its counters run since the last
power-on.

The headline field is ``last_shutdown``: how the *previous* session ended. The
firmware records it itself, because under the custom bootloader the RCC reset
flags are already cleared by the time the app runs:

  power_off      nothing survived in RAM, so the module was unpowered (or this
                 is its first boot)
  unexpected     a warm reset nobody asked for: the watchdog after a hang, an
                 external reset, or a brown-out short enough to keep RAM
  host_reset     OW_CMD_RESET
  host_dfu       OW_CMD_DFU
  ecc            the RAM-ECC double-bit-error reset
  fault          a CPU fault handler, then the watchdog
  error_handler  the firmware's Error_Handler(), then the watchdog

``prev_alive_ms`` is the previous session's uptime at its last 1 s main-loop
heartbeat. 0 means that session never reached the main loop, which is what a
watchdog boot loop looks like.
"""

from __future__ import annotations

import struct

__all__ = [
    "SHUTDOWN_REASONS",
    "ABNORMAL_SHUTDOWNS",
    "parse_reset_history",
    "describe_shutdown",
    "format_reset_history",
]

#: ``last_shutdown`` codes — ``sysmon_shutdown_t`` in sensor-fw
#: ``Core/Inc/system_monitor.h``. Append-only on both sides.
SHUTDOWN_REASONS = {
    0: "power_off",
    1: "unexpected",
    2: "host_reset",
    3: "host_dfu",
    4: "ecc",
    5: "fault",
    6: "error_handler",
}

#: Shutdowns that mean something went wrong, as opposed to a power-off or a
#: reset the host asked for.
ABNORMAL_SHUTDOWNS = frozenset({"unexpected", "ecc", "fault", "error_handler"})

_DESCRIPTIONS = {
    "power_off": "power off (module was unpowered, or first boot)",
    "unexpected": "unexpected reset (watchdog after a hang, external reset, or brown-out)",
    "host_reset": "host reset (OW_CMD_RESET)",
    "host_dfu": "host DFU request (OW_CMD_DFU)",
    "ecc": "RAM ECC error reset",
    "fault": "CPU fault, then watchdog reset",
    "error_handler": "firmware Error_Handler, then watchdog reset",
}

_SHUTDOWN_SLOTS = 8

# RCC->RSR flag bits (STM32H743, RM0433). Listed in the order they read best.
_RCC_FLAGS = (
    (22, "PIN"), (23, "POR"), (21, "BOR"), (24, "SFT"), (26, "IWDG"),
    (28, "WWDG"), (30, "LPWR"), (17, "CPU"), (19, "D1"), (20, "D2"),
)

_RCC_COUNTS = ("por", "pin", "sft", "iwdg", "wwdg", "bor", "lpwr")

# sysmon_reset_history_t v1, 104 bytes:
#   u8 struct_version; u8 last_shutdown; u8 reserved[2];
#   u32 boot_count, prev_alive_ms, uptime_ms; u32 shutdown_count[8];
#   u32 por, pin, sft, iwdg, wwdg, bor, lpwr counts; u32 last_rcc_rsr;
#   u32 ecc_sbe, ecc_dbe, ecc_last_addr, ecc_last_monitor;
#   u32 dma_err_count, usb_recover_count
_V1 = struct.Struct("<BB2x3I8I7II4III")


def parse_reset_history(payload) -> dict | None:
    """Decode an ``OW_CMD_RESET_HISTORY`` reply, or ``None`` if it isn't one.

    Never raises. A short/empty payload (which is what firmware without the
    command produces: it answers OW_UNKNOWN) gives ``None``. The struct version
    is only required to be >= 1, so a future reply that appends fields still
    parses. An unrecognised ``last_shutdown`` code is reported as
    ``"code_<n>"`` rather than dropped.
    """
    if not payload or len(payload) < _V1.size:
        return None
    f = _V1.unpack_from(bytes(payload))
    version, code, boot_count, prev_alive_ms, uptime_ms = f[0:5]
    if version < 1:
        return None
    slots = f[5:5 + _SHUTDOWN_SLOTS]
    rcc = f[13:20]
    rsr = f[20]
    ecc_sbe, ecc_dbe, ecc_addr, ecc_mon, dma_err, usb_recover = f[21:27]

    counts = {}
    for i, n in enumerate(slots):
        name = SHUTDOWN_REASONS.get(i)
        if name is not None or n:
            counts[name or f"code_{i}"] = n

    return {
        "version": version,
        "last_shutdown": SHUTDOWN_REASONS.get(code, f"code_{code}"),
        "last_shutdown_code": code,
        "boot_count": boot_count,
        "prev_alive_ms": prev_alive_ms,
        "uptime_ms": uptime_ms,
        "shutdown_counts": counts,
        "rcc_counts": dict(zip(_RCC_COUNTS, rcc)),
        "last_rcc_rsr": rsr,
        "last_rcc_flags": [name for bit, name in _RCC_FLAGS if rsr & (1 << bit)],
        "ecc_sbe_count": ecc_sbe,
        "ecc_dbe_count": ecc_dbe,
        "ecc_last_addr": ecc_addr,
        "ecc_last_monitor": ecc_mon,
        "dma_err_count": dma_err,
        "usb_recover_count": usb_recover,
    }


def _fmt_duration(ms: int) -> str:
    s = ms / 1000.0
    if s < 60:
        return f"{s:.1f} s"
    m, s = divmod(int(s), 60)
    h, m = divmod(m, 60)
    return f"{h}h{m:02d}m{s:02d}s" if h else f"{m}m{s:02d}s"


def describe_shutdown(history: dict) -> str:
    """One line: how the previous session ended and how long it ran."""
    reason = history["last_shutdown"]
    text = _DESCRIPTIONS.get(reason, f"unrecognised shutdown {reason}")
    if reason == "power_off":
        return text  # nothing about the previous session survived
    alive = history["prev_alive_ms"]
    if alive == 0:
        return f"{text}; the previous session never reached the main loop"
    return f"{text}; the previous session ran {_fmt_duration(alive)}"


def format_reset_history(history: dict) -> list[str]:
    """``key = value`` lines for the sensor's connect-time device block."""
    counts = ", ".join(
        f"{name} {n}" for name, n in history["shutdown_counts"].items() if n
    )
    flags = "|".join(history["last_rcc_flags"]) or "none (cleared by the bootloader)"
    return [
        f"shutdown = {describe_shutdown(history)}",
        f"boots    = {history['boot_count']} since power-on ({counts or 'none'})",
        (
            f"health   = uptime {_fmt_duration(history['uptime_ms'])}, "
            f"usb recoveries {history['usb_recover_count']}, "
            f"ecc sbe/dbe {history['ecc_sbe_count']}/{history['ecc_dbe_count']}, "
            f"dma errors {history['dma_err_count']}, rcc flags {flags}"
        ),
    ]
