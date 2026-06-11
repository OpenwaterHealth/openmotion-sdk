#!/usr/bin/env python3
"""
ykush.py — Yepkit YKUSH USB hub driver for hardware-in-the-loop tests.

Controls individual downstream ports on a YKUSH switchable USB hub so
tests can unplug/replug the device under test (DUT) in software — a
true USB disconnect, unlike the Shelly outlet
(openmotion-bloodflow-app/tests/shelly.py) which cuts mains power and
reboots the whole DUT. Use this when:

  - The DUT (console or sensor module) is plugged into a YKUSH
    downstream port on the test runner.
  - A test needs to verify USB disconnect / reconnect handling
    (ConnectionMonitor, stream-reader teardown, hotplug events)
    without power-cycling the DUT.
  - You want to detach a device into a known state between tests.

Hardware setup
--------------
Required on the runner / dev machine:

  - A Yepkit YKUSH-family hub (YKUSH, YKUSH3, YKUSHXS) on a USB port.
    Its control interface is standard HID — the stock Windows driver
    works, no Zadig/WinUSB swap needed (unlike the sensor modules).
  - The ``hidapi`` package (in the SDK's ``[dev]`` extra):
    ``pip install -e ".[dev]"`` or ``pip install hidapi``.
  - The DUT plugged into one of the hub's downstream ports.

Optional environment variables:

  - ``$YKUSH_SERIAL`` — hub serial number (e.g. ``YK27987``) when more
    than one YKUSH is attached. With a single hub, auto-detected.
  - ``$YKUSH_PORT``   — default downstream port for the module-level
    helpers (default 1).

CLI
---
::

    python ykush.py                        # default: cycle port 1 (off 3s, on)
    python ykush.py status                 # all ports
    python ykush.py on 2
    python ykush.py off 2
    python ykush.py toggle 2
    python ykush.py cycle 2 --off-time 5.0
    python ykush.py on all                 # all ports up (also: off all)

    # Override hub selection (otherwise uses $YKUSH_SERIAL / auto-detect)
    python ykush.py --serial YK27987 status

Library — quickest path
-----------------------
::

    from ykush import on, off, toggle, power_cycle, is_on

    off()                             # detach the DUT's port
    on()                              # reattach it
    toggle()                          # invert state, return new state
    power_cycle(off_time=3.0)         # off → wait 3s → on
    if is_on(): ...
    off(port=2)                       # any helper takes an explicit port

These module-level functions all share one ``YkushHub`` opened lazily
on first call (honoring ``$YKUSH_SERIAL`` / ``$YKUSH_PORT``). A missing
or unopenable hub raises ``YkushNotFound`` (a ``RuntimeError``) — tests
should ``pytest.skip(...)`` rather than fail in that case. To target a
specific hub inside one process, use the class directly::

    from ykush import YkushHub
    hub = YkushHub(serial="YK27987")
    hub.power_cycle(2, off_time=3.0)

Pytest patterns
---------------
A fixture that skips cleanly when no hub is attached and leaves all
ports UP at teardown so downstream tests are usable::

    import pytest, ykush

    @pytest.fixture(scope="module")
    def hub():
        try:
            h = ykush.default_hub()      # raises YkushNotFound if absent
        except RuntimeError as e:
            pytest.skip(f"YKUSH hub not available: {e}")
        yield h
        try: h.all_on()
        except Exception: pass

Verifying the *host* noticed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The hub only switches the port. It does not know whether Windows
dropped the device or whether the SDK reacted. Pair every port action
with a host-side observation — e.g. wait for the ConnectionMonitor
disconnect signal, or poll ``MotionInterface`` state. The same
snapshot → action → wait-for-pattern shape used with shelly.py applies.

Return-value contract
---------------------
Identical to shelly.py:

- ``on(port)`` / ``off(port)`` / ``set(port, on)`` re-read the port
  after writing and return ``True`` only if the post-write state
  matches the request. ``False`` means the hub did not actually change
  state — treat as a hardware problem, not a test failure.
- ``toggle(port)`` returns the **new** state (``True`` == port up).
- ``is_on(port)`` returns the **current** state from a fresh read.
- ``power_cycle(port, off_time, settle_time)`` returns the post-cycle
  state. ``True`` means the cycle completed successfully.
- ``status()`` returns ``{port: bool}`` for every downstream port.

Failure modes & gotchas
-----------------------
- **No hub / wrong serial**: ``YkushNotFound`` from the constructor and
  module-level helpers. Skip, do not fail.
- **Ports come up UP**: when the hub itself loses power or
  re-enumerates, all downstream ports default to UP — a test that died
  mid-detach self-heals on replug, but don't rely on port state
  persisting across hub reboots.
- **Hold off-phases ≥ 2–3 s**: Windows needs time to fully drop the
  device (surprise-removal, driver teardown) before re-attach. Sub-
  second flaps exercise the OS hotplug path, not the SDK's reconnect
  logic. ``power_cycle`` defaults to 3 s off.
- **Stale handle after hub replug**: the HID handle held by a
  ``YkushHub`` goes dead if the hub itself is unplugged. Subsequent
  calls raise ``OSError``/``ValueError`` from hidapi — build a fresh
  instance (or call ``reset_default_hub()`` for the shared one).
- **Singleton hub**: ``default_hub()`` returns a process-wide cached
  instance. Concurrent tests on one runner contend for the same
  physical ports; serialize them — there is no locking.
- **Port numbering is 1-based**, matching the labels on the YKUSH
  board (ports 1..3 on the classic YKUSH).

Wire protocol
-------------
One-byte commands in 64-byte HID reports (per Yepkit's official
pykush): port down ``0x01–0x03``, port up ``0x11–0x13``, all down
``0x0A``, all up ``0x1A``, port state ``0x20 | port``, port count
``0xF1`` (older firmware doesn't answer it — falls back to the model's
known count). Replies start ``[status, value, ...]`` with status 1 on
success; a state value > 0x10 means UP.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

import hid

ENV_SERIAL = "YKUSH_SERIAL"
ENV_PORT = "YKUSH_PORT"

YKUSH_VID = 0x04D8
# Normal-operation PIDs: YKUSH beta, YKUSH, YKUSH3, YKUSHXS.
YKUSH_PIDS = (0x0042, 0xF2F7, 0xF11B, 0xF0CD)
# YKUSHXS has one port and predates the port-count command.
_SINGLE_PORT_PIDS = (0xF0CD,)

_PACKET_SIZE = 64
_PROTO_OK = 0x01
_CMD_ALL_DOWN = 0x0A
_CMD_ALL_UP = 0x1A
_CMD_GET_PORT = 0x20
_CMD_PORT_COUNT = 0xF1


class YkushNotFound(RuntimeError):
    """No matching YKUSH hub attached (or it could not be opened)."""


class YkushHub:
    """Minimal Yepkit YKUSH switchable-hub client.

    One instance controls one hub over its HID control interface. The
    constructor enumerates, opens the device, and reads the downstream
    port count, so an unplugged hub fails fast with ``YkushNotFound``.

    Parameters
    ----------
    serial:
        Hub serial number string (e.g. ``"YK27987"``). ``None``
        auto-detects, raising if zero — or more than one — YKUSH is
        attached, so a multi-hub bench must pick explicitly.
    timeout:
        Per-command reply timeout in seconds (HID round-trips are
        local; the 1 s default is generous).
    """

    def __init__(self, serial: Optional[str] = None, timeout: float = 1.0) -> None:
        self.timeout = timeout
        infos = [
            d
            for d in hid.enumerate(YKUSH_VID)
            if d["product_id"] in YKUSH_PIDS
            and (serial is None or d["serial_number"] == serial)
        ]
        if not infos:
            raise YkushNotFound(
                f"No YKUSH hub found"
                + (f" with serial {serial!r}" if serial else "")
                + f" (looked for VID {YKUSH_VID:#06x})."
            )
        serials = sorted({d["serial_number"] for d in infos})
        if len(serials) > 1:
            raise YkushNotFound(
                f"Multiple YKUSH hubs attached ({', '.join(serials)}); "
                f"pass serial= or set ${ENV_SERIAL}."
            )
        info = infos[0]
        self.serial: str = info["serial_number"]
        self.product_id: int = info["product_id"]
        self._dev = hid.device()
        try:
            self._dev.open_path(info["path"])
        except OSError as e:
            raise YkushNotFound(
                f"YKUSH {self.serial} found but could not be opened: {e}"
            ) from e
        self.port_count: int = self._read_port_count()

    @classmethod
    def from_env(cls, timeout: float = 1.0) -> "YkushHub":
        """Construct a hub honoring ``$YKUSH_SERIAL`` (auto-detect if unset)."""
        return cls(serial=os.environ.get(ENV_SERIAL), timeout=timeout)

    def close(self) -> None:
        """Release the HID handle. Safe to call more than once."""
        if getattr(self, "_dev", None) is not None:
            self._dev.close()
            self._dev = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    # ------------------------------------------------------------------ internals
    def _sendrecv(self, cmd: int) -> list[int]:
        """Send a one-byte command, return the reply bytes.

        YKUSH commands are a single byte in a zero-padded 64-byte HID
        report (plus the leading report-ID zero on write). Replies are
        ``[status, value, ...]``. A timeout returns all-0xFF, which no
        valid reply starts with, so callers just see a failed status.
        """
        self._dev.write([0x00, cmd] + [0x00] * (_PACKET_SIZE - 1))
        reply = self._dev.read(_PACKET_SIZE + 1, timeout_ms=int(self.timeout * 1000))
        if not reply:
            return [0xFF, 0xFF]
        return list(reply)

    def _read_port_count(self) -> int:
        if self.product_id in _SINGLE_PORT_PIDS:
            return 1
        reply = self._sendrecv(_CMD_PORT_COUNT)
        if reply[0] == _PROTO_OK:
            return reply[1]
        return 3  # v1 firmware predates the command; classic YKUSH has 3

    def _check_port(self, port: int) -> None:
        if port not in range(1, self.port_count + 1):
            raise ValueError(
                f"Port {port} out of range (hub {self.serial} has ports "
                f"1..{self.port_count})."
            )

    # ------------------------------------------------------------------ actions
    def is_on(self, port: int) -> bool:
        """Return the port's current up/down state from a fresh read."""
        self._check_port(port)
        reply = self._sendrecv(_CMD_GET_PORT | port)
        if reply[0] != _PROTO_OK:
            raise IOError(
                f"YKUSH {self.serial} did not answer a port-{port} status read."
            )
        return reply[1] > 0x10

    def set(self, port: int, on: bool) -> bool:
        """Drive the port up/down and confirm by reading back.

        Returns ``True`` only if the post-write state matches the
        requested state — ``False`` means the hub didn't comply.
        """
        self._check_port(port)
        self._sendrecv((0x10 if on else 0x00) | port)
        return self.is_on(port) == on

    def on(self, port: int) -> bool:
        """Power the port up. Returns ``True`` if the port is now up."""
        return self.set(port, True)

    def off(self, port: int) -> bool:
        """Power the port down. Returns ``True`` if the port is now down."""
        return self.set(port, False)

    def toggle(self, port: int) -> bool:
        """Invert the port's state. Returns the **new** state (True == up)."""
        self.set(port, not self.is_on(port))
        return self.is_on(port)

    def all_on(self) -> bool:
        """Power every downstream port up. ``True`` if all read back up."""
        self._sendrecv(_CMD_ALL_UP)
        return all(self.status().values())

    def all_off(self) -> bool:
        """Power every downstream port down. ``True`` if all read back down."""
        self._sendrecv(_CMD_ALL_DOWN)
        return not any(self.status().values())

    def status(self) -> dict[int, bool]:
        """Return ``{port: is_up}`` for every downstream port."""
        return {p: self.is_on(p) for p in range(1, self.port_count + 1)}

    def power_cycle(
        self, port: int, off_time: float = 3.0, settle_time: float = 0.5
    ) -> bool:
        """Down → wait ``off_time`` → up → wait ``settle_time``.

        Returns ``True`` if the port is up after the cycle. Keep
        ``off_time`` ≥ 2–3 s for reconnect tests so Windows fully drops
        the device before it re-enumerates (see module docstring).
        """
        self.off(port)
        time.sleep(off_time)
        self.on(port)
        time.sleep(settle_time)
        return self.is_on(port)


# ----------------------------------------------- Module-level test helpers ---
# Convenience wrappers for test scripts, mirroring shelly.py. All forward
# to one shared YkushHub opened lazily on first call, targeting
# $YKUSH_PORT (default 1) unless a port is passed explicitly.

_default_hub: Optional[YkushHub] = None


def default_hub() -> YkushHub:
    """Return the process-wide shared hub, opening it on first call.

    Honors ``$YKUSH_SERIAL`` / ``$YKUSH_PORT``. Raises ``YkushNotFound``
    if no hub is attached — callers in pytest should ``pytest.skip(...)``.
    """
    global _default_hub
    if _default_hub is None:
        _default_hub = YkushHub.from_env()
    return _default_hub


def reset_default_hub() -> None:
    """Drop the shared hub so the next helper call reopens it.

    Use after the hub itself was unplugged/replugged (the old HID
    handle is dead at that point).
    """
    global _default_hub
    if _default_hub is not None:
        _default_hub.close()
    _default_hub = None


def _resolve_port(port: Optional[int]) -> int:
    return port if port is not None else int(os.environ.get(ENV_PORT, "1"))


def is_on(port: Optional[int] = None) -> bool:
    """Current state of the default (or given) port — fresh read."""
    return default_hub().is_on(_resolve_port(port))


def on(port: Optional[int] = None) -> bool:
    """Power the default (or given) port up; ``True`` on success."""
    return default_hub().on(_resolve_port(port))


def off(port: Optional[int] = None) -> bool:
    """Power the default (or given) port down; ``True`` on success."""
    return default_hub().off(_resolve_port(port))


def toggle(port: Optional[int] = None) -> bool:
    """Toggle the default (or given) port; returns the **new** state."""
    return default_hub().toggle(_resolve_port(port))


def power_cycle(
    port: Optional[int] = None, off_time: float = 3.0, settle_time: float = 0.5
) -> bool:
    """Power-cycle the default (or given) port (down → wait → up → settle)."""
    return default_hub().power_cycle(
        _resolve_port(port), off_time=off_time, settle_time=settle_time
    )


# --------------------------------------------------------------------------- CLI
def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control a Yepkit YKUSH USB hub (toggle/on/off/status/cycle)."
    )
    parser.add_argument(
        "action",
        nargs="?",
        default="cycle",
        choices=["toggle", "on", "off", "status", "cycle"],
        help="What to do with the port (default: cycle).",
    )
    parser.add_argument(
        "port",
        nargs="?",
        default=os.environ.get(ENV_PORT, "1"),
        help=f"Downstream port number, or 'all' for on/off. "
        f"Defaults to ${ENV_PORT} or 1.",
    )
    parser.add_argument(
        "--serial",
        default=os.environ.get(ENV_SERIAL),
        help=f"Hub serial number (e.g. YK27987). Defaults to ${ENV_SERIAL} "
        f"or auto-detect.",
    )
    parser.add_argument(
        "--off-time",
        type=float,
        default=3.0,
        help="Seconds to hold the port down during 'cycle' (default 3.0).",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_cli()

    if args.port != "all":
        try:
            args.port = int(args.port)
        except ValueError:
            print(f"ERROR: Invalid port {args.port!r}.", file=sys.stderr)
            return 2
    elif args.action not in ("on", "off"):
        print("ERROR: 'all' only works with on/off.", file=sys.stderr)
        return 2

    try:
        hub = YkushHub(serial=args.serial)
    except YkushNotFound as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    label = f"{hub.serial} port {args.port}"
    try:
        if args.action == "status":
            for port, up in hub.status().items():
                print(f"[{hub.serial}] port {port}: {'ON' if up else 'OFF'}")
            return 0

        if args.action == "on":
            ok = hub.all_on() if args.port == "all" else hub.on(args.port)
            print(f"[{label}] on -> {'ON' if ok else 'FAILED'}")
            return 0 if ok else 1

        if args.action == "off":
            ok = hub.all_off() if args.port == "all" else hub.off(args.port)
            print(f"[{label}] off -> {'OFF' if ok else 'FAILED'}")
            return 0 if ok else 1

        if args.action == "toggle":
            state = hub.toggle(args.port)
            print(f"[{label}] toggle -> {'ON' if state else 'OFF'}")
            return 0

        if args.action == "cycle":
            print(f"[{label}] power-cycling (off {args.off_time:.1f}s) ...")
            ok = hub.power_cycle(args.port, off_time=args.off_time)
            print(f"[{label}] cycle -> {'ON' if ok else 'FAILED'}")
            return 0 if ok else 1

    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        hub.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
