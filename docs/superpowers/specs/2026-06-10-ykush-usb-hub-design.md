# YKUSH controllable USB hub driver — design

**Date:** 2026-06-10
**Status:** Approved
**Owner:** ethan

## Problem

Disconnect/reconnect testing today relies on a Shelly WiFi outlet
(`openmotion-bloodflow-app/tests/shelly.py`) that kills mains power to the
DUT. That power-cycles the whole device, is slow, and can't simulate a pure
USB unplug. A Yepkit YKUSH hub (classic 3-port model, serial `YK27987`,
VID `04D8` / PID `F2F7`) is now on the bench: it can switch individual
downstream USB ports off and on under software control — a true cable-pull
without touching device power.

## Goal

A self-contained Python module in `openmotion-sdk` that controls YKUSH
ports from both test code and the command line, mirroring the proven
`shelly.py` pattern so anyone familiar with one can use the other.

## Decisions (made with Ethan, 2026-06-10)

| Decision | Choice |
|---|---|
| Placement | `tests/ykush.py` in openmotion-sdk (helper module like shelly.py; not shipped in wheel) |
| Transport | Direct HID via the `hid` (hidapi) package — no external binary, stock Windows HID driver |
| Scope | Driver + CLI only. Pytest fixtures, reconnect tests, named-port mapping deferred. |
| Bench setup | A Logitech USB webcam is on one port as the test canary |
| Branch | Off `next`, PR to `next` (repo convention) |

## Design

### `YkushHub` class

- Constructor: `YkushHub(serial=None, port_count=auto)`. With no serial,
  auto-detect; raise if zero or multiple hubs found. Knows the Yepkit
  VID and per-model PIDs / port counts (classic YKUSH = 3).
- Actions, all per-port: `is_on(port)`, `on(port)`, `off(port)`,
  `toggle(port)`, `power_cycle(port, off_time, settle_time)`, and
  `status()` → dict of all ports.
- Return contract identical to shelly.py: `on()/off()` write then read
  back, returning `True` only if the port is in the requested state;
  `toggle()` returns the new state; `power_cycle()` returns post-cycle
  state.
- Wire protocol: 64-byte HID reports, one-byte command (port down
  `0x01–0x03`, up `0x11–0x13`, status read). Exact bytes confirmed
  against Yepkit's official `pykush` source during implementation and
  verified live on the bench hub.

### Module-level helpers + env config

- `default_hub()` lazy process-wide singleton; `on()/off()/toggle()/
  power_cycle()/is_on()` forward to it.
- `YKUSH_SERIAL` (optional, disambiguates multiple hubs),
  `YKUSH_PORT` (default port for module helpers, default `1`).
- Hub absent/unopenable → raise; pytest callers `pytest.skip(...)`.

### CLI

```
python tests/ykush.py on|off|toggle|status|cycle [port] [--serial YK…] [--off-time 3.0]
```

`status` with no port prints all ports. Exit codes follow shelly.py
(0 success, 1 action failed, 2 usage error).

### Dependency

`hid` added to the `[dev]` extra in `pyproject.toml`. Runtime wheel
unchanged.

## Documented gotchas (go in the module docstring)

- All ports come up ON when the hub itself reboots or re-enumerates.
- Hold off-phases ≥ 2–3 s so Windows fully drops the device before
  re-enumeration; sub-second flaps test the OS, not the SDK.
- Only downstream ports are switched; the hub's upstream link (and its
  own HID control interface) stays alive throughout.

## Testing

Live verification on the bench hub: every CLI action exercised; the
Logitech webcam's presence/absence in Windows PnP is the ground truth
for port state. Error paths: no-hub (unplugged serial mismatch) and
explicit wrong serial. No mocked protocol unit tests — hardware is the
authority and is permanently available on this bench.

## Out of scope

- Pytest fixture (skip-if-absent, restore-on-teardown) — later, after
  hands-on use.
- First HIL reconnect test against `ConnectionMonitor` — later.
- Named-port mapping (`YKUSH_PORT_CONSOLE=…`) — later, once real DUTs
  occupy the ports.
