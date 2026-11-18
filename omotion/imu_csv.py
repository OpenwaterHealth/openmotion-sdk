"""Per-scan IMU CSV logger (bloodflow-app#779).

During a scan each sensor module streams one JSON line per IMU sample on USB
interface 2 (``MotionComposite.imu``) once ``imu_on()`` has started the
firmware's 40 Hz sample timer::

    {"F":frame,"G":[gx,gy,gz],"M":[mx,my,mz],"A":[ax,ay,az],"T":temp}\\r\\n

Future firmware adds ``"ms"``: the sensor's TIM5 millisecond clock, the same
clock that stamps histogram frames. This module starts that stream on each
active side, parses it, puts every sample on the scan's ``timestamp_s`` axis
and writes ``{scan_id}_{subject_id}_imu.csv`` (both sides, one row per
sample).

Timestamps (column ``time_source``):

``device``
    The line carries ``ms``: ``timestamp_s = ms / 1000 - t0`` where ``t0`` is
    the device time of that side's first histogram sample — exactly the zero
    ``LiveUsbSource._t0_normalize`` gives the histogram rows, so IMU and
    histogram rows share one clock.
``host``
    Older firmware (no ``ms``): ``timestamp_s`` is the host
    ``time.monotonic()`` at receipt minus the host monotonic time at which
    that side's first histogram sample was parsed. Good to roughly the USB
    delivery jitter (~10-20 ms).

Samples that arrive before the side's first histogram sample are held until
the anchor exists, so they get negative times. If a side never produces a
histogram sample, its held samples fall back to the scan's start time
(``fallback_t0_monotonic``) with ``time_source=host`` when the logger stops.

Motion magnitude (bloodflow-app#783): each resolved sample also lands in its
side's 1/8 s bin (``MOTION_BIN_S``, bins aligned to multiples of 0.125 s on
the same ``timestamp_s`` axis). A bin is emitted as an ``ImuMotionBin`` once
a sample past its end arrives (the partial last bin at stop) with the bin's
peak ``accel_dev_g`` and ``gyro_dps`` from ``motion_magnitude``; the app
applies its own threshold. ``motion_bins_from_imu_csv`` rebuilds the same
bins from a recorded IMU CSV.

Every failure here is logged and swallowed: the IMU path must never abort or
stall a scan. Lifecycle mirrors ``CameraTelemetryCsvLogger``: ScanWorkflow
constructs it in ``start_scan`` when ``ScanRequest.write_imu_csv`` is True,
``start()``s it before the trigger and ``stop()``s it in the worker's
``finally``. Standalone use::

    log = ImuCsvLogger([("left", sensor)], "bench_imu.csv")
    log.start(); ...; log.stop()
"""

from __future__ import annotations

import csv
import json
import math
import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Sequence, Tuple

from omotion import _log_root
from omotion.pipeline.batch import ImuMotionBin

logger = logging.getLogger(f"{_log_root}.ImuCsv" if _log_root else "ImuCsv")

# ICM-20948 full-scale settings configured by sensor firmware.
ACCEL_LSB_PER_G: float = 2048.0     # ±16 g
GYRO_LSB_PER_DPS: float = 16.4      # ±2000 dps

# Firmware IMU packets are <= 200 B; a 512 B read returns one transfer.
IMU_READ_SIZE: int = 512
# Raw chunk queue per side: 40 Hz x ~100 s of backlog before reads stall.
_CHUNK_QUEUE_MAXSIZE: int = 4096
# A line longer than this is garbage (real lines are < 128 B): drop it.
_MAX_LINE_BYTES: int = 1024
# Samples held per side while waiting for the first histogram sample.
# Beyond this the oldest are resolved against the fallback anchor.
_MAX_PENDING: int = 2400            # 60 s at 40 Hz

# TIM5 32-bit counter at 100 kHz, reported in ms: wraps every ~11.9 h.
_DEVICE_ROLLOVER_S: float = (2**32) / 100.0 / 1000.0

# Sensor firmware before 1.6.2 did the ICM I2C read inside the TIM14 ISR
# (and could printf from it), which races camera I2C mid-scan and can wedge
# the comm interface (sdk#142). Don't start the stream on those.
MIN_SENSOR_FW_FOR_IMU_STREAM: Tuple[int, int, int] = (1, 6, 2)

# Motion-magnitude bin width (s): 5 samples at the firmware's 40 Hz.
MOTION_BIN_S: float = 0.125

IMU_CSV_HEADERS: List[str] = [
    "side", "imu_frame", "timestamp_s", "time_source", "device_ms",
    "host_time_unix",
    "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
    "accel_x_g", "accel_y_g", "accel_z_g",
    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
    "temp_c",
]

# (side) -> (t0_device_s, t0_host_monotonic) or None until the side's first
# histogram sample has been parsed.
AnchorFn = Callable[[str], Optional[Tuple[float, float]]]


@dataclass
class ImuSample:
    """One parsed IMU line (raw sensor counts)."""
    frame: int
    accel: Tuple[int, int, int]
    gyro: Tuple[int, int, int]
    mag: Tuple[int, int, int]
    temp_c: float
    device_ms: Optional[int] = None


def _triplet(v: Any) -> Tuple[int, int, int]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"expected 3-element list, got {v!r}")
    return (int(v[0]), int(v[1]), int(v[2]))


def parse_imu_line(line: str | bytes) -> Optional[ImuSample]:
    """Parse one IMU JSON line. Returns None for anything malformed."""
    if isinstance(line, (bytes, bytearray)):
        try:
            line = bytes(line).decode("ascii")
        except UnicodeDecodeError:
            return None
    line = line.strip().strip("\x00")
    if not line:
        return None
    try:
        obj = json.loads(line)
        if not isinstance(obj, dict):
            return None
        ms = obj.get("ms")
        return ImuSample(
            frame=int(obj["F"]),
            accel=_triplet(obj["A"]),
            gyro=_triplet(obj["G"]),
            mag=_triplet(obj["M"]),
            temp_c=float(obj["T"]),
            device_ms=None if ms is None else int(ms),
        )
    except (ValueError, KeyError, TypeError):
        return None


class ImuLineParser:
    """Reassembles IMU lines from USB chunks.

    A line may be split across reads, a read may carry several lines, and the
    firmware may pad with NULs. ``feed`` returns the samples completed by the
    chunk; malformed lines are counted in ``bad_lines`` and skipped.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.good_lines = 0
        self.bad_lines = 0

    def feed(self, chunk: bytes) -> List[ImuSample]:
        out: List[ImuSample] = []
        self._buf.extend(bytes(chunk).replace(b"\x00", b""))
        while True:
            nl = self._buf.find(b"\n")
            if nl < 0:
                break
            raw = bytes(self._buf[:nl])
            del self._buf[:nl + 1]
            if not raw.strip():
                continue
            s = parse_imu_line(raw)
            if s is None:
                self.bad_lines += 1
            else:
                self.good_lines += 1
                out.append(s)
        if len(self._buf) > _MAX_LINE_BYTES:
            # No newline in a kilobyte: resync on the next line.
            self.bad_lines += 1
            self._buf.clear()
        return out

    def flush(self) -> List[ImuSample]:
        """Parse whatever is left (a final line with no terminator)."""
        raw = bytes(self._buf)
        self._buf.clear()
        if not raw.strip():
            return []
        s = parse_imu_line(raw)
        if s is None:
            self.bad_lines += 1
            return []
        self.good_lines += 1
        return [s]


def device_timestamp_s(device_ms: int, t0_device_s: float) -> float:
    """Device ms -> scan-relative seconds, unwrapping one TIM5 rollover."""
    dt = device_ms / 1000.0 - t0_device_s
    if dt < -_DEVICE_ROLLOVER_S / 2:
        dt += _DEVICE_ROLLOVER_S
    elif dt > _DEVICE_ROLLOVER_S / 2:
        dt -= _DEVICE_ROLLOVER_S
    return dt


def resolve_timestamp(sample: ImuSample, host_monotonic: float,
                      anchor: Optional[Tuple[float, float]],
                      fallback_t0_monotonic: Optional[float] = None,
                      ) -> Optional[Tuple[float, str]]:
    """Return ``(timestamp_s, time_source)`` or None if it must wait.

    ``anchor`` is the side's ``(t0_device_s, t0_host_monotonic)``. With no
    anchor, ``fallback_t0_monotonic`` (if given) yields a host timestamp.
    """
    if anchor is not None:
        t0_dev, t0_host = anchor
        if sample.device_ms is not None:
            return device_timestamp_s(sample.device_ms, t0_dev), "device"
        return host_monotonic - t0_host, "host"
    if fallback_t0_monotonic is not None:
        return host_monotonic - fallback_t0_monotonic, "host"
    return None


def imu_row(side: str, sample: ImuSample, timestamp_s: float,
            time_source: str, host_time_unix: float) -> List[Any]:
    """Build one CSV row in ``IMU_CSV_HEADERS`` order."""
    ax, ay, az = sample.accel
    gx, gy, gz = sample.gyro
    mx, my, mz = sample.mag
    return [
        side, sample.frame, round(timestamp_s, 6), time_source,
        "" if sample.device_ms is None else sample.device_ms,
        round(host_time_unix, 6),
        ax, ay, az, gx, gy, gz, mx, my, mz,
        round(ax / ACCEL_LSB_PER_G, 6), round(ay / ACCEL_LSB_PER_G, 6),
        round(az / ACCEL_LSB_PER_G, 6),
        round(gx / GYRO_LSB_PER_DPS, 4), round(gy / GYRO_LSB_PER_DPS, 4),
        round(gz / GYRO_LSB_PER_DPS, 4),
        round(sample.temp_c, 2),
    ]


# ── motion magnitude ──────────────────────────────────────────────────────

def motion_magnitude(accel_g: Sequence[float], gyro_dps: Sequence[float],
                     ) -> Tuple[float, float]:
    """The per-sample motion magnitudes, in scaled units.

    ``accel_g``: accelerometer (x, y, z) in g; ``gyro_dps``: gyroscope
    (x, y, z) in deg/s. Returns ``(accel_dev_g, gyro_mag_dps)``:

    - ``accel_dev_g = | |a| - 1 |``: how far the specific-force magnitude is
      from 1 g. Orientation-independent, so gravity drops out without a
      filter; ~0 at rest in any pose.
    - ``gyro_mag_dps = |omega|``: rotation rate about any axis. No bias
      removal (expect a few deg/s at rest).

    Pure and dependency-free so other apps (e.g. wellness) can reuse it.
    """
    ax, ay, az = accel_g
    gx, gy, gz = gyro_dps
    return (abs(math.sqrt(ax * ax + ay * ay + az * az) - 1.0),
            math.sqrt(gx * gx + gy * gy + gz * gz))


def raw_motion_magnitude(accel_raw: Sequence[int], gyro_raw: Sequence[int],
                         ) -> Tuple[float, float]:
    """``motion_magnitude`` from raw ICM-20948 counts (±16 g, ±2000 dps)."""
    return motion_magnitude([v / ACCEL_LSB_PER_G for v in accel_raw],
                            [v / GYRO_LSB_PER_DPS for v in gyro_raw])


def motion_bin_index(timestamp_s: float, bin_s: float = MOTION_BIN_S) -> int:
    """Index of the bin ``[k*bin_s, (k+1)*bin_s)`` holding ``timestamp_s``."""
    # The epsilon keeps an exact edge (0.375 / 0.125) from rounding down.
    return math.floor(timestamp_s / bin_s + 1e-9)


class MotionBinner:
    """Folds one side's time-ordered samples into ``bin_s`` bins.

    ``add`` returns the bin a sample completes (the first sample past the
    open bin's end closes it; bins with no samples are skipped). ``flush``
    closes the partial last bin.
    """

    def __init__(self, side: str, bin_s: float = MOTION_BIN_S) -> None:
        self.side = side
        self.bin_s = float(bin_s)
        self._reset()

    def _reset(self) -> None:
        self._idx: Optional[int] = None
        self._n = 0
        self._acc = 0.0
        self._gyr = 0.0
        self._host = False

    def add(self, timestamp_s: float, time_source: str,
            accel_raw: Sequence[int], gyro_raw: Sequence[int],
            ) -> List[ImuMotionBin]:
        idx = motion_bin_index(timestamp_s, self.bin_s)
        out: List[ImuMotionBin] = []
        if self._idx is not None and idx != self._idx:
            out = self.flush()
        if self._idx is None:
            self._idx = idx
        acc, gyr = raw_motion_magnitude(accel_raw, gyro_raw)
        self._n += 1
        self._acc = max(self._acc, acc)
        self._gyr = max(self._gyr, gyr)
        self._host = self._host or time_source != "device"
        return out

    def flush(self) -> List[ImuMotionBin]:
        if self._idx is None or self._n == 0:
            self._reset()
            return []
        t0 = self._idx * self.bin_s
        b = ImuMotionBin(
            side=self.side, t_start_s=round(t0, 9),
            t_end_s=round(t0 + self.bin_s, 9),
            accel_dev_g=self._acc, gyro_dps=self._gyr, n_samples=self._n,
            time_source="host" if self._host else "device",
        )
        self._reset()
        return [b]


def motion_bins_from_imu_csv(path: str, bin_s: float = MOTION_BIN_S,
                             ) -> List[ImuMotionBin]:
    """Rebuild the motion bins from a recorded ``*_imu.csv``.

    The same bins, per side, a live scan produced; rows that don't parse are
    skipped. Sorted by side, then time.
    """
    by_side: dict = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                by_side.setdefault(r["side"], []).append((
                    float(r["timestamp_s"]), r["time_source"],
                    (int(r["ax"]), int(r["ay"]), int(r["az"])),
                    (int(r["gx"]), int(r["gy"]), int(r["gz"])),
                ))
            except (KeyError, TypeError, ValueError):
                continue
    bins: List[ImuMotionBin] = []
    for side in sorted(by_side):
        binner = MotionBinner(side, bin_s)
        for ts, src, a, g in sorted(by_side[side], key=lambda x: x[0]):
            bins.extend(binner.add(ts, src, a, g))
        bins.extend(binner.flush())
    return bins


def sensor_fw_supports_imu_stream(version: Optional[str]) -> bool:
    """True when the sensor firmware samples the IMU outside the ISR."""
    try:
        from omotion.firmware_update import parse_version
        return parse_version(version or "") >= MIN_SENSOR_FW_FOR_IMU_STREAM
    except (TypeError, ValueError):
        return False


class _Side:
    def __init__(self, side: str, sensor: Any) -> None:
        self.side = side
        self.sensor = sensor
        self.stream = None          # StreamInterface (sensor.uart.imu)
        self.chunks: "queue.Queue[bytes]" = queue.Queue(maxsize=_CHUNK_QUEUE_MAXSIZE)
        self.parser = ImuLineParser()
        # (sample, host_monotonic, host_unix) waiting for the anchor.
        self.pending: List[Tuple[ImuSample, float, float]] = []
        self.lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None
        self.imu_on = False
        self.fallback_warned = False
        self.rows = 0
        self.device_rows = 0
        self.binner = MotionBinner(side)
        self.bins = 0


class ImuStreamLogger:
    """Streams IMU from one or more sensors into a per-sample CSV and/or
    1/8 s motion bins.

    ``sensors`` is a sequence of ``(side, MotionSensor)``. ``anchor_fn(side)``
    returns that side's histogram zero as ``(t0_device_s, t0_host_monotonic)``
    or None until it exists (``LiveUsbSource.side_t0``). ``fallback_t0_monotonic``
    anchors host timestamps for a side that never gets a histogram anchor
    (defaults to the time ``start()`` was called).

    ``path`` is the per-sample CSV, or None for none. ``motion_fn`` (or
    None) receives every completed ``ImuMotionBin`` on that side's consumer
    thread, in time order per side; it must not block for long, and an
    exception from it is logged once and swallowed.

    Nothing raises out of ``start()``/``stop()``; failures are logged.
    One USB reader per side feeds both outputs.
    """

    def __init__(self, sensors: Sequence[Tuple[str, Any]], path: Optional[str],
                 anchor_fn: Optional[AnchorFn] = None,
                 fallback_t0_monotonic: Optional[float] = None,
                 motion_fn: Optional[Callable[[ImuMotionBin], None]] = None,
                 ) -> None:
        self.path = path
        self._motion_fn = motion_fn
        self._motion_fn_failed = False
        self._anchor_fn = anchor_fn or (lambda _side: None)
        self._fallback_t0 = fallback_t0_monotonic
        self._sides = [_Side(side, s) for side, s in sensors]
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._started = False
        self._stopped = False
        self._stop_lock = threading.Lock()
        self._fh = self._writer = None
        if path is not None:
            self._fh = open(path, "w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._fh)
            self._writer.writerow(IMU_CSV_HEADERS)
            self._fh.flush()

    def set_motion_fn(self, fn: Optional[Callable[[ImuMotionBin], None]]) -> None:
        """Replace (or with None, detach) the bin callback."""
        self._motion_fn = fn

    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the IMU stream on every side (idempotent, never raises)."""
        if self._started:
            return
        self._started = True
        if self._fallback_t0 is None:
            self._fallback_t0 = time.monotonic()
        for st in self._sides:
            try:
                self._start_side(st)
            except Exception:
                logger.exception("IMU: failed to start IMU stream on %s", st.side)
                self._stop_side_hw(st)

    def _start_side(self, st: _Side) -> None:
        version = getattr(st.sensor, "_version", None)
        if not sensor_fw_supports_imu_stream(version):
            logger.warning(
                "IMU: %s sensor firmware %s predates %d.%d.%d (ISR-side IMU "
                "sampling, sdk#142); not streaming IMU on this side",
                st.side, version, *MIN_SENSOR_FW_FOR_IMU_STREAM)
            return
        stream = st.sensor.uart.imu
        if not st.sensor.imu_init():
            logger.warning("IMU: imu_init failed on %s; continuing", st.side)
        try:
            stream.flush_stale_data(expected_size=IMU_READ_SIZE, max_total_ms=300)
        except Exception:
            logger.warning("IMU: stale flush failed on %s", st.side, exc_info=True)
        # Host reader first, so the endpoint is drained from the first sample.
        stream.start_streaming(st.chunks, IMU_READ_SIZE)
        st.stream = stream
        st.thread = threading.Thread(
            target=self._consume, args=(st,), daemon=True,
            name=f"ImuCsv-{st.side}")
        st.thread.start()
        st.imu_on = True  # set before the call: stop must imu_off on any outcome
        if not st.sensor.imu_on():
            logger.warning("IMU: imu_on failed on %s; no IMU rows expected",
                           st.side)
        logger.info("IMU: streaming IMU on %s -> %s", st.side, self.path)

    def stop(self) -> None:
        """Stop streams, flush held samples and the last motion bins, close
        the file (idempotent; safe to call from several threads)."""
        with self._stop_lock:
            if self._stopped:
                return
            self._stopped = True
        for st in self._sides:
            self._stop_side_hw(st)
        self._stop.set()
        for st in self._sides:
            if st.thread is not None:
                st.thread.join(timeout=2.0)
                if st.thread.is_alive():
                    logger.warning("IMU: %s consumer did not exit", st.side)
        for st in self._sides:
            try:
                tail = st.parser.flush()
                now_m, now_u = time.monotonic(), time.time()
                with st.lock:
                    st.pending.extend((s, now_m, now_u) for s in tail)
                self._drain_pending(st, final=True)
            except Exception:
                logger.exception("IMU: final flush failed on %s", st.side)
            if st.parser.bad_lines:
                logger.warning("IMU: %s dropped %d malformed IMU line(s)",
                               st.side, st.parser.bad_lines)
            if st.stream is not None or st.rows:
                logger.info("IMU: %s %d sample(s) (%d device-timed), "
                            "%d motion bin(s)", st.side, st.rows,
                            st.device_rows, st.bins)
        with self._write_lock:
            if self._fh is not None:
                try:
                    self._fh.close()
                except Exception:
                    pass

    close = stop

    def _stop_side_hw(self, st: _Side) -> None:
        # imu_off first so the endpoint goes quiet, then stop the reader
        # (the firmware IMU endpoint can wedge if left pushing unread).
        if st.imu_on:
            st.imu_on = False
            try:
                if not st.sensor.imu_off():
                    logger.warning("IMU: imu_off failed on %s", st.side)
            except Exception:
                logger.warning("IMU: imu_off raised on %s", st.side, exc_info=True)
        if st.stream is not None:
            try:
                st.stream.stop_streaming()
            except Exception:
                logger.warning("IMU: stop_streaming raised on %s",
                               st.side, exc_info=True)

    # ------------------------------------------------------------------

    def _consume(self, st: _Side) -> None:
        while True:
            try:
                chunk = st.chunks.get(timeout=0.1)
            except queue.Empty:
                if self._stop.is_set():
                    return
                # Anchor may have appeared with no new IMU data.
                if st.pending:
                    self._safe_drain(st)
                continue
            now_m, now_u = time.monotonic(), time.time()
            try:
                samples = st.parser.feed(chunk)
                with st.lock:
                    st.pending.extend((s, now_m, now_u) for s in samples)
            except Exception:
                logger.exception("IMU: parse failed on %s", st.side)
            self._safe_drain(st)

    def _safe_drain(self, st: _Side) -> None:
        try:
            self._drain_pending(st)
        except Exception:
            logger.exception("IMU: write failed on %s", st.side)

    def _drain_pending(self, st: _Side, final: bool = False) -> None:
        """Write every held sample whose timestamp can now be resolved and
        emit the motion bins it completes (and the open bin when ``final``).

        Without an anchor, samples stay held, except the oldest beyond
        ``_MAX_PENDING`` (and all of them when ``final``), which fall back to
        the scan-start anchor.
        """
        with st.lock:
            if not st.pending:
                tail = st.binner.flush() if final else []
                st.bins += len(tail)
                pending = False
            else:
                pending = True
        if not pending:
            self._emit([], tail)
            return
        with st.lock:
            try:
                anchor = self._anchor_fn(st.side)
            except Exception:
                anchor = None
            n_fallback = 0
            if anchor is None:
                n_fallback = (len(st.pending) if final
                              else max(0, len(st.pending) - _MAX_PENDING))
                if n_fallback and not st.fallback_warned:
                    st.fallback_warned = True
                    logger.warning(
                        "IMU: %s has no histogram anchor; %d IMU sample(s) "
                        "timed against scan start instead", st.side, n_fallback)
            rows: List[List[Any]] = []
            bins: List[ImuMotionBin] = []
            keep: List[Tuple[ImuSample, float, float]] = []
            for i, (s, host_m, host_u) in enumerate(st.pending):
                fb = self._fallback_t0 if i < n_fallback else None
                res = resolve_timestamp(s, host_m, anchor, fb)
                if res is None:
                    keep.append((s, host_m, host_u))
                    continue
                ts, source = res
                rows.append(imu_row(st.side, s, ts, source, host_u))
                bins.extend(st.binner.add(ts, source, s.accel, s.gyro))
                st.rows += 1
                if source == "device":
                    st.device_rows += 1
            st.pending = keep
            if final:
                bins.extend(st.binner.flush())
            st.bins += len(bins)
            # Deliver under the side lock so a final flush on the stopping
            # thread can't interleave with the consumer: time order per side.
            self._emit(rows, bins)

    def _emit(self, rows: List[List[Any]], bins: List[ImuMotionBin]) -> None:
        if rows and self._fh is not None:
            with self._write_lock:
                if not self._fh.closed:
                    self._writer.writerows(rows)
                    self._fh.flush()
        fn = self._motion_fn
        if fn is None:
            return
        for b in bins:
            try:
                fn(b)
            except Exception:
                if not self._motion_fn_failed:
                    self._motion_fn_failed = True
                    logger.exception("IMU motion callback raised (logged once)")

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass


# Original name (bloodflow-app#779); the logger now also produces motion bins.
ImuCsvLogger = ImuStreamLogger
