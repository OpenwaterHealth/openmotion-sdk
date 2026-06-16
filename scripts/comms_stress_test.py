#!/usr/bin/env python3
"""Comms stress test for the MOTION Console module.

Runs a tight loop of command/response transactions over the standard
`MotionInterface` connection path to quantify:

- success/failure rate
- round-trip latency distribution
- sustained request rate

Typical usage (Windows cmd):

    set PYTHONPATH=%cd%;%PYTHONPATH%
    python scripts/comms_stress_test.py --duration 60 --pattern ping-echo --echo-bytes 128

Notes:
- Default pattern avoids stateful actions (no LED/fan) to keep the test non-invasive.
- `--timeout` is injected into the underlying `uart.send_packet()` used by `console.ping()` and
    `console.echo()` so API calls still respect the CLI timeout.
- The test also reads temperatures via `console.get_temperatures()` every 10 packets.
- If you set a very small timeout, remember `MotionUart.read_packet()` sleeps in 50 ms
  increments; effective timeout resolution is ~50 ms.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import random
import time
from dataclasses import dataclass, field
from pathlib import Path

from omotion import MotionInterface


logger = logging.getLogger("comms_stress_test")


TEMP_EVERY_PACKETS = 10


def _percentile(sorted_values: list[float], p: float) -> float | None:
    """Linear-interpolated percentile with p in [0, 1]."""
    if not sorted_values:
        return None
    if p <= 0:
        return sorted_values[0]
    if p >= 1:
        return sorted_values[-1]

    k = (len(sorted_values) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_values[int(k)]
    return sorted_values[f] + (sorted_values[c] - sorted_values[f]) * (k - f)


@dataclass
class OpStats:
    name: str
    count: int = 0
    ok: int = 0
    failed: int = 0
    timeouts: int = 0
    errors: int = 0
    mismatches: int = 0
    exceptions: int = 0
    lat_ms: list[float] = field(default_factory=list)

    def record_ok(self, latency_ms: float) -> None:
        self.count += 1
        self.ok += 1
        self.lat_ms.append(latency_ms)

    def record_timeout(self) -> None:
        self.count += 1
        self.failed += 1
        self.timeouts += 1

    def record_error_packet(self) -> None:
        self.count += 1
        self.failed += 1
        self.errors += 1

    def record_mismatch(self) -> None:
        self.count += 1
        self.failed += 1
        self.mismatches += 1

    def record_exception(self) -> None:
        self.count += 1
        self.failed += 1
        self.exceptions += 1

    def summary_line(self) -> str:
        if self.count == 0:
            return f"{self.name}: no samples"

        lat_sorted = sorted(self.lat_ms) if self.lat_ms else []
        p50 = _percentile(lat_sorted, 0.50)
        p95 = _percentile(lat_sorted, 0.95)
        p99 = _percentile(lat_sorted, 0.99)
        avg = (sum(self.lat_ms) / len(self.lat_ms)) if self.lat_ms else None
        mn = min(self.lat_ms) if self.lat_ms else None
        mx = max(self.lat_ms) if self.lat_ms else None
        err_rate = (self.failed / self.count) * 100.0

        def _fmt(v: float | None) -> str:
            return "-" if v is None else f"{v:7.2f}"

        return (
            f"{self.name}: n={self.count} ok={self.ok} fail={self.failed} "
            f"({err_rate:0.2f}%) "
            f"lat_ms min/avg/p50/p95/p99/max="
            f"{_fmt(mn)}/{_fmt(avg)}/{_fmt(p50)}/{_fmt(p95)}/{_fmt(p99)}/{_fmt(mx)}"
        )


def _build_payload(size: int, *, seed: int) -> bytes:
    # Deterministic payload, but varied per iteration.
    rng = random.Random(seed)
    return bytes(rng.randrange(0, 256) for _ in range(size))


def parse_cli(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="MOTION console comms stress test")
    p.add_argument(
        "--iterations",
        type=int,
        default=None,
        help="Number of pattern cycles to run. If omitted, uses --duration.",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Seconds to run (ignored if --iterations is provided).",
    )
    p.add_argument(
        "--pattern",
        choices=["ping", "echo", "ping-echo"],
        default="ping-echo",
        help="Which command sequence to stress.",
    )
    p.add_argument(
        "--echo-bytes",
        type=int,
        default=64,
        help="Echo payload size in bytes.",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Per-command response timeout in seconds.",
    )
    p.add_argument(
        "--sleep-ms",
        type=float,
        default=0.0,
        help="Optional sleep between commands (ms).",
    )
    p.add_argument(
        "--report-every",
        type=int,
        default=0,
        help="Print intermediate summary every N pattern cycles (0 disables).",
    )
    p.add_argument(
        "--max-consecutive-failures",
        type=int,
        default=50,
        help="Abort after this many consecutive failures.",
    )
    p.add_argument(
        "--csv-out",
        type=Path,
        default=None,
        help="Optional CSV path for per-command results.",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="Enable info-level logging (otherwise warnings only).",
    )
    return p.parse_args(argv)


def _pattern_ops(pattern: str) -> list[tuple[str, int]]:
    if pattern == "ping":
        return ["ping"]
    if pattern == "echo":
        return ["echo"]
    if pattern == "ping-echo":
        return ["ping", "echo"]
    raise ValueError(f"Unknown pattern: {pattern}")


def _ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def run() -> int:
    args = parse_cli()

    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    # Keep chatty library loggers quiet unless explicitly asked.
    if not args.verbose:
        logging.getLogger("omotion").setLevel(logging.WARNING)

    if args.echo_bytes < 0:
        raise SystemExit("--echo-bytes must be >= 0")
    if args.timeout <= 0:
        raise SystemExit("--timeout must be > 0")
    if args.timeout < 0.1:
        logger.warning("Timeout < 0.1s is likely too small for current read loop resolution")
    if args.iterations is not None and args.iterations <= 0:
        raise SystemExit("--iterations must be > 0")
    if args.duration <= 0:
        raise SystemExit("--duration must be > 0")
    if args.sleep_ms < 0:
        raise SystemExit("--sleep-ms must be >= 0")

    interface = MotionInterface()
    interface.start()
    if not interface.console.is_connected():
        print("Console Module not connected.")
        interface.stop()
        return 2

    console = interface.console
    # Ensure console API calls respect the requested timeout.
    # The Console methods call `self.uart.send_packet(...)` without passing timeout,
    # so we wrap the instance method to inject `timeout=args.timeout`.
    if getattr(console, "uart", None) is not None:
        _orig_send_packet = console.uart.send_packet

        def _send_packet_with_timeout(*a, **kw):
            kw.setdefault("timeout", args.timeout)
            return _orig_send_packet(*a, **kw)

        console.uart.send_packet = _send_packet_with_timeout

    ops = _pattern_ops(args.pattern)
    stats: dict[str, OpStats] = {name: OpStats(name=name) for name in ops}

    csv_file = None
    csv_writer: csv.writer | None = None
    if args.csv_out is not None:
        _ensure_parent_dir(args.csv_out)
        csv_file = open(args.csv_out, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            [
                "ts_unix",
                "cycle",
                "op",
                "ok",
                "latency_ms",
                "error",
                "echo_bytes",
                "echo_match",
            ]
        )

    print("Starting comms stress test")
    print(f"Pattern: {args.pattern}")
    print(f"Timeout: {args.timeout:0.3f}s | Sleep: {args.sleep_ms:0.1f}ms | Echo bytes: {args.echo_bytes}")
    print(f"Temperatures: every {TEMP_EVERY_PACKETS} packets")
    if args.iterations is None:
        print(f"Stop condition: duration {args.duration:0.1f}s")
    else:
        print(f"Stop condition: {args.iterations} pattern cycles")
    if args.csv_out is not None:
        print(f"CSV: {args.csv_out}")
    print()

    start_time = time.perf_counter()
    consecutive_failures = 0
    cycle = 0
    packet_counter = 0

    temp_stats = OpStats(name="temperatures")

    def should_stop() -> bool:
        if args.iterations is not None:
            return cycle >= args.iterations
        return (time.perf_counter() - start_time) >= args.duration

    try:
        while not should_stop():
            cycle += 1
            for op_name in ops:
                st = stats[op_name]
                ts_unix = time.time()
                payload = b""
                echo_match: bool | None = None
                err_label: str | None = None
                latency_ms: float | None = None
                ok = False

                if op_name == "echo":
                    payload = _build_payload(args.echo_bytes, seed=cycle)

                t0 = time.perf_counter()
                try:
                    if op_name == "ping":
                        resp = console.ping()
                        t1 = time.perf_counter()
                        latency_ms = (t1 - t0) * 1000.0
                        if resp is True:
                            ok = True
                            st.record_ok(latency_ms)
                        else:
                            err_label = "ping_false"
                            st.record_error_packet()

                    elif op_name == "echo":
                        echoed, echoed_len = console.echo(payload)
                        t1 = time.perf_counter()
                        latency_ms = (t1 - t0) * 1000.0

                        if echoed is None:
                            err_label = "echo_none"
                            st.record_timeout()
                        else:
                            rx = bytes(echoed)
                            if echoed_len != len(payload):
                                echo_match = False
                                err_label = "echo_len_mismatch"
                                st.record_mismatch()
                            else:
                                echo_match = rx == payload
                                if echo_match:
                                    ok = True
                                    st.record_ok(latency_ms)
                                else:
                                    err_label = "echo_mismatch"
                                    st.record_mismatch()

                    else:
                        raise ValueError(f"Unknown operation: {op_name}")

                except Exception as e:  # noqa: BLE001 - stress test: keep going
                    err_label = f"exception:{type(e).__name__}"
                    st.record_exception()
                    logger.debug("Command failed", exc_info=e)

                if ok:
                    consecutive_failures = 0
                else:
                    consecutive_failures += 1
                    if consecutive_failures >= args.max_consecutive_failures:
                        print(f"Aborting: hit {consecutive_failures} consecutive failures")
                        raise KeyboardInterrupt

                packet_counter += 1

                # Periodic temperature read
                if TEMP_EVERY_PACKETS > 0 and (packet_counter % TEMP_EVERY_PACKETS == 0):
                    t_ts_unix = time.time()
                    t_err_label: str | None = None
                    t_latency_ms: float | None = None
                    t_ok = False
                    t0 = time.perf_counter()
                    try:
                        temps = console.get_temperatures(return_all=True)
                        t1 = time.perf_counter()
                        t_latency_ms = (t1 - t0) * 1000.0

                        if temps:
                            t_ok = True
                            temp_stats.record_ok(t_latency_ms)
                            last = temps[-1]
                            print(last)
                        else:
                            print("No telemetry samples received.")
                            temp_stats.record_error_packet()

                    except Exception as e:  # noqa: BLE001
                        t_err_label = f"exception:{type(e).__name__}"
                        temp_stats.record_exception()
                        logger.debug("Temperature read failed", exc_info=e)

                    if csv_writer is not None:
                        csv_writer.writerow(
                            [
                                f"{t_ts_unix:0.3f}",
                                str(cycle),
                                "temperatures",
                                "1" if t_ok else "0",
                                f"{t_latency_ms:0.3f}" if t_latency_ms is not None else "",
                                t_err_label or "",
                                "0",
                                "",
                            ]
                        )

                if csv_writer is not None:
                    csv_writer.writerow(
                        [
                            f"{ts_unix:0.3f}",
                            str(cycle),
                            op_name,
                            "1" if ok else "0",
                            f"{latency_ms:0.3f}" if latency_ms is not None else "",
                            err_label or "",
                            str(args.echo_bytes if op_name == "echo" else 0),
                            "" if echo_match is None else ("1" if echo_match else "0"),
                        ]
                    )

                if args.sleep_ms > 0:
                    time.sleep(args.sleep_ms / 1000.0)

            if args.report_every and (cycle % args.report_every == 0):
                now = time.perf_counter()
                elapsed = now - start_time
                cycles_per_s = cycle / elapsed if elapsed > 0 else 0.0
                print(f"--- {cycle} cycles in {elapsed:0.1f}s ({cycles_per_s:0.2f} cycles/s) ---")
                for op_name in ops:
                    print(stats[op_name].summary_line())
                if temp_stats.count:
                    print(temp_stats.summary_line())

    except KeyboardInterrupt:
        pass
    finally:
        if csv_file is not None:
            csv_file.flush()
            csv_file.close()
        interface.stop()

    end_time = time.perf_counter()
    elapsed = end_time - start_time
    total_cmds = sum(s.count for s in stats.values()) + temp_stats.count
    total_ok = sum(s.ok for s in stats.values()) + temp_stats.ok
    total_fail = sum(s.failed for s in stats.values()) + temp_stats.failed
    cmds_per_s = total_cmds / elapsed if elapsed > 0 else 0.0

    print("\n=== Summary ===")
    print(f"Elapsed: {elapsed:0.2f}s | Cycles: {cycle} | Commands: {total_cmds} ({cmds_per_s:0.2f} cmd/s)")
    if total_cmds:
        print(f"OK: {total_ok} | Fail: {total_fail} | Fail rate: {(total_fail/total_cmds)*100.0:0.2f}%")
    for op_name in ops:
        print(stats[op_name].summary_line())
    if temp_stats.count:
        print(temp_stats.summary_line())

    return 0 if total_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(run())
