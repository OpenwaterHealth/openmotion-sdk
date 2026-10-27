"""NvcmProgrammer — burn a Lattice CrossLink NVCM image via a MotionSensor.

Replays Diamond-generated .iea/.ied I2C transactions over the sensor's
factory commands (OW_FACTORY_*). The default image is bundled in
omotion/nvcm/ (see README there for provenance).

NVCM is ONE-TIME programmable: a successful burn is permanent. The replay
performs full readback verification (omotion.i2c_parser), so a non-blank
or already-programmed device fails fast and corrupt burns cannot PASS.

The replay is deliberately paced (~2 ms per factory transaction, ~3 min per
camera) and each burn starts with a hard power-cycle of the target camera.
Burns faster than the paced envelope silently corrupt the array partway
through and permanently brick the part — see _SensorI2CDriver.
MIN_DISPATCH_INTERVAL_S and issue #245 before touching either behavior.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Callable, Optional

from omotion import _log_root
from omotion.i2c_parser import I2CDriver, isp_entry_point, ERR_MESSAGES
from omotion.MotionSensor import _ERROR_TYPES

logger = logging.getLogger(
    f"{_log_root}.NvcmProgrammer" if _log_root else "NvcmProgrammer")

_NVCM_DIR = Path(__file__).resolve().parent / "nvcm"
DEFAULT_ALGO_PATH = _NVCM_DIR / "impl1_algo.iea"
DEFAULT_DATA_PATH = _NVCM_DIR / "impl1_data.ied"

#: Failures within this many transactions are almost always the algorithm's
#: initial IDCODE/status checks rejecting a non-blank (already programmed)
#: device rather than a mid-burn error.
_EARLY_FAIL_TX = 200

#: Settle after powering the target camera off, before powering it back on.
_POWER_OFF_SETTLE_S = 0.5
#: Settle after power-on before the replay toggles CRESET: firmware delays
#: ~200 ms per camera on power-on, and the CrossLink boot-scan must park.
_POWER_ON_SETTLE_S = 1.0


class NvcmTransportError(RuntimeError):
    """A MotionSensor factory call reported failure mid-replay.

    Raised by _SensorI2CDriver when i2c_write/i2c_read/i2c_write_read/creset/
    switch_camera return an error indication. Callers of NvcmProgrammer.burn()
    never see this — burn() converts it into a failed NvcmResult.
    """


@dataclass(frozen=True)
class NvcmResult:
    """Outcome of an NVCM burn.

    On real hardware, polling-loop retries (busy-status reads that mismatch
    and are retried) make the executed transaction count exceed the simulated
    total used for progress reporting; progress callbacks are clamped to 100%.
    """

    success: bool
    error: Optional[str]
    #: Countable transactions executed (stop + read + creset). May exceed the
    #: simulated total when polling loops retried — see class docstring.
    transactions: int


class _TxState(Enum):
    IDLE = auto()
    AFTER_START = auto()
    WRITE_PHASE = auto()
    READ_ONLY = auto()
    AFTER_RESTART = auto()
    READ_PHASE = auto()


class _CountingSimDriver(I2CDriver):
    """Simulation driver that counts countable transactions.

    The counting rule (stop + read + creset) must stay identical to
    _SensorI2CDriver's so the sim total matches the hardware run.
    """

    def __init__(self) -> None:
        self.count = 0

    def is_simulation(self) -> bool:
        return True

    def start(self) -> None:
        pass

    def restart(self) -> None:
        pass

    def stop(self) -> None:
        self.count += 1

    def write(self, data: bytes) -> None:
        pass

    def read(self, num_bytes: int) -> bytes:
        self.count += 1
        return bytes([0xFF] * num_bytes)

    def creset(self, value: int) -> None:
        self.count += 1

    def wait(self, ms: int) -> None:
        pass


class _SensorI2CDriver(I2CDriver):
    """Group raw parser signals into MotionSensor factory I2C transactions.

    The .iea encodes the I2C address as the first WRITE byte after each
    START/RESTART (0x80 = 0x40 write, 0x81 = 0x40 read). This driver strips
    that byte, accumulates payload, and dispatches on STOP/READ:
        pure write       -> sensor.i2c_write(addr, data)
        write then read  -> sensor.i2c_write_read(addr, data, n)
        pure read        -> sensor.i2c_read(addr, n)

    Every dispatch to the sensor is paced (MIN_DISPATCH_INTERVAL_S) so the
    NVCM replay cannot run faster than the envelope it was validated at,
    no matter how fast the underlying transport is.
    """

    #: Minimum start-to-start spacing between factory-I2C dispatches, in
    #: seconds. This is a safety floor, not an optimization knob: CrossLink
    #: NVCM burns are only proven good at >=1.4 ms/transaction (June 2026
    #: validation ran at 2.26 ms/tx; the 1.4.2 release exe passes at
    #: 1.43-1.60 ms/tx), and every burn observed at <=0.67 ms/tx corrupted
    #: the array from ~28% in and permanently bricked the OTP part
    #: (2026-08-13 incident, issue #245: 12 cameras lost). 2.0 ms sits in
    #: the proven-good zone and reproduces the validated ~9 ms/page program
    #: cadence. Do NOT lower without hardware revalidation on expendable
    #: parts; 0.67-1.4 ms/tx is untested territory.
    MIN_DISPATCH_INTERVAL_S = 0.002

    def __init__(self, sensor, total: int = 0,
                 progress_cb: Optional[Callable[[int, int], None]] = None,
                 default_addr: int = 0x40) -> None:
        self._sensor = sensor
        self._default_addr = default_addr
        self._state = _TxState.IDLE
        self._addr = default_addr
        self._write_buf = bytearray()
        self.count = 0
        self._total = total
        self._progress_cb = progress_cb
        self._last_pct = -1
        self._last_dispatch = 0.0

    def _pace(self) -> None:
        """Sleep so consecutive sensor dispatches start >= the floor apart."""
        now = time.monotonic()
        wait = self._last_dispatch + self.MIN_DISPATCH_INTERVAL_S - now
        if wait > 0:
            time.sleep(wait)
            now = time.monotonic()
        self._last_dispatch = now

    def is_simulation(self) -> bool:
        return False

    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self.count += 1
        if self._progress_cb is None or self._total <= 0:
            return
        # Clamp: polling-loop retries on hardware can push count past the
        # simulated total. done is monotonic and never exceeds total.
        done = min(self.count, self._total)
        pct = done * 100 // self._total
        if pct != self._last_pct:
            self._last_pct = pct
            self._progress_cb(done, self._total)

    # ------------------------------------------------------------------

    def start(self) -> None:
        self._state = _TxState.AFTER_START
        self._addr = self._default_addr
        self._write_buf = bytearray()

    def restart(self) -> None:
        self._state = _TxState.AFTER_RESTART

    def stop(self) -> None:
        # Empty-buffer STOPs (e.g. the parser's EnableHardware bus test)
        # dispatch nothing — counted on both sim and hardware drivers.
        if self._state == _TxState.WRITE_PHASE and self._write_buf:
            self._pace()
            # MotionSensor.i2c_write returns None on success, False on a
            # firmware error packet (despite its docstring claiming it raises).
            if self._sensor.i2c_write(self._addr, bytes(self._write_buf)) is False:
                raise NvcmTransportError(
                    f"i2c_write failed at transaction {self.count}")
        self._state = _TxState.IDLE
        self._write_buf = bytearray()
        self._tick()

    def write(self, data: bytes) -> None:
        if not data:
            return
        if self._state == _TxState.AFTER_START:
            addr_byte = data[0]
            self._addr = addr_byte >> 1
            if addr_byte & 0x01:
                self._state = _TxState.READ_ONLY
            else:
                self._state = _TxState.WRITE_PHASE
            if len(data) > 1:
                self._write_buf += data[1:]
        elif self._state == _TxState.WRITE_PHASE:
            self._write_buf += data
        elif self._state == _TxState.AFTER_RESTART:
            addr_byte = data[0]
            self._addr = addr_byte >> 1
            self._state = _TxState.READ_PHASE
            if len(data) > 1:
                self._write_buf += data[1:]
        else:
            logger.warning("write() in unexpected state %s", self._state)

    def read(self, num_bytes: int) -> bytes:
        self._pace()
        if self._state == _TxState.READ_PHASE and self._write_buf:
            result = self._sensor.i2c_write_read(
                self._addr, bytes(self._write_buf), num_bytes)
            what = "i2c_write_read"
        else:
            result = self._sensor.i2c_read(self._addr, num_bytes)
            what = "i2c_read"
        # MotionSensor returns bytes on success, False on a firmware error
        # packet (despite its docstring claiming it raises).
        if result is False or result is None:
            raise NvcmTransportError(
                f"{what} failed at transaction {self.count}")
        if len(result) != num_bytes:
            raise NvcmTransportError(
                f"short read: got {len(result)} of {num_bytes} bytes"
                f" at transaction {self.count}")
        self._write_buf = bytearray()
        self._tick()
        return result

    def select_camera(self, camera: int) -> None:
        if not (1 <= camera <= 8):
            raise ValueError(f"camera must be 1-8, got {camera}")
        self._pace()
        # MotionSensor.switch_camera returns the raw response packet; a NAK
        # shows up as packetType in _ERROR_TYPES (same check the other
        # MotionSensor methods use).
        r = self._sensor.switch_camera(camera - 1)
        if r is None or r is False or getattr(r, "packetType", None) in _ERROR_TYPES:
            raise NvcmTransportError(f"switch_camera({camera}) failed")

    def creset(self, value: int) -> None:
        self._pace()
        # MotionSensor.creset returns the pin state int (0 or 1) on success,
        # False on a firmware error packet — identity check only, 0 is valid.
        if self._sensor.creset(value != 0) is False:
            raise NvcmTransportError(
                f"creset({value}) failed at transaction {self.count}")
        self._tick()

    def wait(self, ms: int) -> None:
        time.sleep(ms / 1000.0)


class NvcmProgrammer:
    """Burns one camera's CrossLink NVCM. PERMANENT — see module docstring."""

    def __init__(self, sensor) -> None:
        self._sensor = sensor

    def burn(self, camera: int,
             algo_path: Optional[str] = None,
             data_path: Optional[str] = None,
             progress_cb: Optional[Callable[[int, int], None]] = None,
             ) -> NvcmResult:
        """Burn `camera` (1-8). progress_cb(done, total) fires per percent.

        Always returns an NvcmResult — never raises, except ValueError for an
        out-of-range camera. Transport failures (sensor NAKs, short reads,
        unexpected exceptions) come back as a failed NvcmResult.

        The target camera is hard power-cycled first (a refused power command
        fails the burn before any I2C reaches the FPGA), and the replay is
        paced to ~2 ms/transaction — a full burn takes ~3 minutes per camera
        BY DESIGN. Both behaviors are OTP-safety requirements (issue #245),
        not overhead to optimize away.

        progress_cb is invoked synchronously on the calling thread; GUI
        consumers must marshal updates to their UI thread. `done` is clamped
        to `total` (hardware polling retries can execute more transactions
        than the simulated total). There is deliberately no cancellation
        mid-burn: an OTP write must not be aborted partway.
        """
        if not (1 <= camera <= 8):
            raise ValueError(f"camera must be 1-8, got {camera}")
        algo = str(algo_path or DEFAULT_ALGO_PATH)
        data = str(data_path or DEFAULT_DATA_PATH)

        # Deterministic sim pre-pass: exact transaction total for progress,
        # and a file sanity check before touching hardware.
        try:
            sim = _CountingSimDriver()
            ret = isp_entry_point(algo, data, driver=sim)
            if ret < 0:
                msg = ERR_MESSAGES.get(ret, f"error {ret}")
                return NvcmResult(False, f"image pre-check failed: {msg}", 0)
            total = sim.count

            # Hard power-cycle the target camera so the burn always starts
            # from a cold, unconfigured FPGA. Every validated PASS followed a
            # fresh power state; a warm/long-idle part is exactly the regime
            # where the 2026-08-13 bricking burns also showed anomalous
            # busy-poll behavior (issue #245). A refused power command aborts
            # BEFORE any I2C reaches the FPGA — never burn on a guess.
            mask = 1 << (camera - 1)
            if not self._sensor.disable_camera_power(mask):
                return NvcmResult(
                    False, "failed to power-cycle camera (power-off refused)", 0)
            time.sleep(_POWER_OFF_SETTLE_S)
            if not self._sensor.enable_camera_power(mask):
                return NvcmResult(False, "failed to power camera", 0)
        except Exception as exc:  # disconnected sensor, corrupt image, ...
            logger.exception("NVCM burn pre-flight failed for camera %d", camera)
            return NvcmResult(False, str(exc), 0)
        time.sleep(_POWER_ON_SETTLE_S)

        driver = _SensorI2CDriver(self._sensor, total=total,
                                  progress_cb=progress_cb)

        def pace_ms(elapsed: float) -> float:
            # Average ms per countable transaction, incl. the .iea's own
            # waits. Regression tripwire: every validated PASS logs >=1.4;
            # the 2026-08-13 bricking burns logged 0.42-0.86 (issue #245).
            return elapsed * 1000.0 / driver.count if driver.count else 0.0

        t0 = time.monotonic()
        try:
            driver.select_camera(camera)
            logger.info("NVCM burn start: camera %d, %d transactions",
                        camera, total)
            ret = isp_entry_point(algo, data, driver=driver)
        except Exception as exc:  # incl. NvcmTransportError — never leak
            logger.exception("NVCM burn ABORTED: camera %d after %d tx "
                             "(avg %.2f ms/tx)", camera, driver.count,
                             pace_ms(time.monotonic() - t0))
            return NvcmResult(False, str(exc), driver.count)
        elapsed = time.monotonic() - t0

        if ret < 0:
            msg = ERR_MESSAGES.get(ret, f"error {ret}")
            if driver.count < _EARLY_FAIL_TX:
                msg += (" (failed during initial checks — device may already"
                        " be programmed / not blank)")
            logger.warning("NVCM burn FAILED: camera %d after %d tx "
                           "(avg %.2f ms/tx): %s",
                           camera, driver.count, pace_ms(elapsed), msg)
            return NvcmResult(False, msg, driver.count)

        if progress_cb is not None:
            progress_cb(total, total)
        logger.info("NVCM burn PASSED: camera %d (%d tx, %.1f s, avg %.2f "
                    "ms/tx)", camera, driver.count, elapsed, pace_ms(elapsed))
        return NvcmResult(True, None, driver.count)
