"""SDK-level background poller for console health data."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, TYPE_CHECKING

from omotion import _log_root

if TYPE_CHECKING:
    from omotion.MotionConsole import MotionConsole

logger = logging.getLogger(f"{_log_root}.ConsoleTelemetry" if _log_root else "ConsoleTelemetry")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_POLL_INTERVAL_S: float = 1.0          # target cadence
_MIN_SLEEP_S: float = 0.05             # floor to avoid tight spin
_MAX_SLEEP_S: float = 1.0              # ceiling

# I2C parameters for analog telemetry (from firmware knowledge)
_MUX_IDX: int = 1
_I2C_ADDR: int = 0x41

# tcl: laser trigger counter on channel 4, register 0x10, 4 bytes LE
_TCL_CHANNEL: int = 4
_TCL_REG: int = 0x10
_TCL_LEN: int = 4

# pdc: photodiode current on channel 7, register 0x1C, 2 bytes LE → × 1.9 mA
_PDC_CHANNEL: int = 7
_PDC_REG: int = 0x1C
_PDC_LEN: int = 2
_PDC_MA_PER_LSB: float = 1.9

# safety interlock on channels 6 and 7, register 0x24, 1 byte each
_SAFETY_SE_CHANNEL: int = 6
_SAFETY_SO_CHANNEL: int = 7
_SAFETY_REG: int = 0x24
_SAFETY_LEN: int = 1
_SAFETY_FAULT_MASK: int = 0x07
_SAFETY_RESERVED_MASK: int = 0xF8
_SAFETY_FAULTS = {
    0x01: "POWER_PEAK_CURRENT_LIMIT_FAIL",
    0x02: "PULSE_UPPER_LIMIT_FAIL_OR_PULSE_LOWER_LIMIT_FAIL",
    0x04: "RATE_LOWER_LIMIT_FAIL",
}


def _decode_safety_faults(raw_byte: int) -> List[str]:
    """Decode safety interlock fault bits [2:0] into readable labels."""
    return [label for bit, label in _SAFETY_FAULTS.items() if (raw_byte & bit) != 0]


# ---------------------------------------------------------------------------
# Data class
# ---------------------------------------------------------------------------

@dataclass
class ConsoleTelemetry:
    """Immutable snapshot of one console telemetry poll."""

    timestamp: float = 0.0          # time.time() when the poll completed

    # --- Analog telemetry ---
    tcm: int = 0                    # MCU trigger (lsync) count
    tcl: int = 0                    # laser trigger count (I2C)
    pdc: float = 0.0                # photodiode current in mA

    # --- TEC raw ADC values (app converts to °C) ---
    tec_v_raw: float = 0.0          # OUT1 / thermistor voltage (→ measured temp)
    tec_set_raw: float = 0.0        # IN2P / setpoint voltage (→ target temp)
    tec_curr_raw: float = 0.0       # V_itec (→ TEC current)
    tec_volt_raw: float = 0.0       # V_vtec (→ TEC voltage)
    tec_good: bool = False          # TMPGD pin (abs(OUT1-IN2P) < 100 mV)

    # --- PDU monitor (16 raw counts + 16 calibrated volts) ---
    pdu_raws: List[int] = field(default_factory=list)
    pdu_volts: List[float] = field(default_factory=list)

    # --- Safety interlock ---
    safety_se: int = 0              # raw byte from channel 6
    safety_so: int = 0              # raw byte from channel 7
    safety_ok: bool = True          # True if both low-nibbles are zero (interlock clear)
    # safety_known is False until _read_safety has actually heard back
    # from the interlock chip on this poll. Lets callers distinguish
    # "no data, defaulted to OK" (don't trust safety_ok) from "chip
    # responded, faults absent" (trust safety_ok). See issue
    # OpenwaterHealth/openmotion-bloodflow-app#107.
    safety_known: bool = False

    # --- Read health ---
    read_ok: bool = True            # False if any sub-read threw an exception
    error: Optional[str] = None     # last exception message if read_ok is False


# ---------------------------------------------------------------------------
# Poller
# ---------------------------------------------------------------------------

class ConsoleTelemetryPoller:
    """
    Background thread that polls the console at ~1 Hz.

    Lifecycle
    ---------
    start()  – called by MOTIONInterface when the console USB connects
    stop()   – called by MOTIONInterface when the console USB disconnects
               (also called automatically on object deletion)

    Thread safety
    -------------
    All public methods acquire ``_lock`` before touching shared state.
    Listener callbacks are invoked *outside* the lock to avoid deadlocks;
    they run on the poller thread, so they should be fast / non-blocking.
    """

    def __init__(self, console: "MotionConsole") -> None:
        self._console = console
        self._lock = threading.Lock()
        self._snapshot: Optional[ConsoleTelemetry] = None
        self._listeners: List[Callable[[ConsoleTelemetry], None]] = []

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._wake = threading.Event()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the polling thread (idempotent)."""
        with self._lock:
            if self._running:
                logger.debug("ConsoleTelemetryPoller.start() called but already running")
                return
            self._running = True
            self._wake.clear()
            self._thread = threading.Thread(
                target=self._poll_loop,
                name="ConsoleTelemetryPoller",
                daemon=True,
            )
            self._thread.start()
            logger.info("ConsoleTelemetryPoller started")

    def stop(self) -> None:
        """Stop the polling thread and block until it exits (idempotent)."""
        thread_to_join: Optional[threading.Thread] = None
        with self._lock:
            if not self._running:
                return
            self._running = False
            self._wake.set()
            thread_to_join = self._thread
            self._thread = None

        if thread_to_join and thread_to_join.is_alive():
            thread_to_join.join(timeout=5.0)
        logger.info("ConsoleTelemetryPoller stopped")

    def get_snapshot(self) -> Optional[ConsoleTelemetry]:
        """Return the most recent telemetry snapshot, or None if none yet."""
        with self._lock:
            return self._snapshot

    def add_listener(self, fn: Callable[[ConsoleTelemetry], None]) -> None:
        """Register a callback invoked on every successful poll."""
        with self._lock:
            if fn not in self._listeners:
                self._listeners.append(fn)

    def remove_listener(self, fn: Callable[[ConsoleTelemetry], None]) -> None:
        """Unregister a previously registered callback."""
        with self._lock:
            try:
                self._listeners.remove(fn)
            except ValueError:
                pass

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        logger.debug("ConsoleTelemetryPoller poll loop entered")
        last_poll = 0.0

        while True:
            with self._lock:
                if not self._running:
                    break

            now = time.time()
            elapsed = now - last_poll
            if elapsed >= _POLL_INTERVAL_S:
                tick_start = time.time()
                snap = self._read_all()
                last_poll = tick_start

                # Store snapshot and collect listeners under lock
                listeners: List[Callable] = []
                with self._lock:
                    self._snapshot = snap
                    listeners = list(self._listeners)

                # Notify listeners outside the lock
                for fn in listeners:
                    try:
                        fn(snap)
                    except Exception as exc:
                        logger.error("ConsoleTelemetry listener raised: %s", exc)

                duration = time.time() - tick_start
                logger.debug("ConsoleTelemetryPoller tick %.1f ms", duration * 1000.0)

            # Smart sleep: wait until next scheduled tick, clamped
            sleep_s = _POLL_INTERVAL_S - (time.time() - last_poll)
            sleep_s = max(_MIN_SLEEP_S, min(_MAX_SLEEP_S, sleep_s))
            self._wake.wait(timeout=sleep_s)
            self._wake.clear()

        logger.debug("ConsoleTelemetryPoller poll loop exited")

    def _read_all(self) -> ConsoleTelemetry:
        """Perform one complete poll; always returns a ConsoleTelemetry."""
        snap = ConsoleTelemetry(timestamp=time.time())

        try:
            self._read_tec(snap)
            self._read_pdu(snap)
            self._read_safety(snap)
            self._read_analog(snap)
            snap.read_ok = True
            snap.error = None
        except Exception as exc:
            snap.read_ok = False
            snap.error = str(exc)
            # If the underlying UART is no longer connected, this poll
            # tick caught the device on its way out — log at INFO not
            # ERROR (the state machine will already log the disconnect),
            # then stop the loop so we don't keep retrying.
            if not self._console.is_connected():
                logger.info(
                    "ConsoleTelemetryPoller: console disconnected mid-poll (%s); stopping",
                    exc,
                )
                with self._lock:
                    self._running = False
                self._wake.set()
            else:
                logger.error("ConsoleTelemetryPoller _read_all error: %s", exc)

        snap.timestamp = time.time()
        return snap

    def _read_tec(self, snap: ConsoleTelemetry) -> None:
        result = self._console.tec_status()
        # tec_status() returns (volt, temp_set, tec_curr, tec_volt, tec_good)
        # All raw ADC floats except tec_good which is bool.
        v, temp_set, tec_curr, tec_volt, tec_good = result
        snap.tec_v_raw = float(v)
        snap.tec_set_raw = float(temp_set)
        snap.tec_curr_raw = float(tec_curr)
        snap.tec_volt_raw = float(tec_volt)
        snap.tec_good = bool(tec_good)

    def _read_pdu(self, snap: ConsoleTelemetry) -> None:
        pdu = self._console.read_pdu_mon()
        if pdu is None:
            raise RuntimeError("read_pdu_mon returned None")
        snap.pdu_raws = list(pdu.raws)
        snap.pdu_volts = list(pdu.volts)

    def _read_safety(self, snap: ConsoleTelemetry) -> None:
        se_raw, _ = self._console.read_i2c_packet(
            mux_index=_MUX_IDX,
            channel=_SAFETY_SE_CHANNEL,
            device_addr=_I2C_ADDR,
            reg_addr=_SAFETY_REG,
            read_len=_SAFETY_LEN,
        )
        so_raw, _ = self._console.read_i2c_packet(
            mux_index=_MUX_IDX,
            channel=_SAFETY_SO_CHANNEL,
            device_addr=_I2C_ADDR,
            reg_addr=_SAFETY_REG,
            read_len=_SAFETY_LEN,
        )
        if not se_raw or not so_raw:
            # Safety interlock chip not responding — treat as unknown/unavailable.
            # Leave safety_ok at its default True so legacy callers don't falsely
            # trip on absent hardware; new callers should gate on safety_known
            # to distinguish "no data, defaulted to OK" from "chip responded,
            # faults absent". See issue
            # OpenwaterHealth/openmotion-bloodflow-app#107.
            logger.warning(
                "Safety I2C read returned no data (se=%s so=%s) — interlock state unknown",
                se_raw,
                so_raw,
            )
            return
        snap.safety_se = se_raw[0]
        snap.safety_so = so_raw[0]
        se_faults = _decode_safety_faults(snap.safety_se & _SAFETY_FAULT_MASK)
        so_faults = _decode_safety_faults(snap.safety_so & _SAFETY_FAULT_MASK)
        snap.safety_ok = not se_faults and not so_faults
        snap.safety_known = True

        if se_faults:
            logger.error(
                "Safety interlock SE faults: %s (raw=0x%02X)",
                ", ".join(se_faults),
                snap.safety_se,
            )
        if so_faults:
            logger.error(
                "Safety interlock SO faults: %s (raw=0x%02X)",
                ", ".join(so_faults),
                snap.safety_so,
            )

        # Bits [7:3] are reserved and expected to be zero.
        if (snap.safety_se & _SAFETY_RESERVED_MASK) != 0:
            logger.warning(
                "Safety interlock SE has unexpected reserved bits set: raw=0x%02X",
                snap.safety_se,
            )
        if (snap.safety_so & _SAFETY_RESERVED_MASK) != 0:
            logger.warning(
                "Safety interlock SO has unexpected reserved bits set: raw=0x%02X",
                snap.safety_so,
            )

    def _read_analog(self, snap: ConsoleTelemetry) -> None:
        # get_lsync_pulsecount swallows transient errors and returns None;
        # default to 0 so a single bad poll doesn't raise TypeError.
        lsync = self._console.get_lsync_pulsecount()
        snap.tcm = int(lsync) if lsync is not None else 0

        tcl_raw, _ = self._console.read_i2c_packet(
            mux_index=_MUX_IDX,
            channel=_TCL_CHANNEL,
            device_addr=_I2C_ADDR,
            reg_addr=_TCL_REG,
            read_len=_TCL_LEN,
        )
        pdc_raw, _ = self._console.read_i2c_packet(
            mux_index=_MUX_IDX,
            channel=_PDC_CHANNEL,
            device_addr=_I2C_ADDR,
            reg_addr=_PDC_REG,
            read_len=_PDC_LEN,
        )

        # These I2C channels may not be populated in all hardware configurations.
        # Leave tcl/pdc at their zero defaults and log at DEBUG — the UART layer
        # already logged the underlying fault at a higher level.
        if not tcl_raw:
            logger.debug("Analog I2C tcl read (channel %d) returned no data", _TCL_CHANNEL)
        else:
            snap.tcl = int.from_bytes(tcl_raw[:_TCL_LEN], byteorder="little")

        if not pdc_raw:
            logger.debug("Analog I2C pdc read (channel %d) returned no data", _PDC_CHANNEL)
        else:
            snap.pdc = int.from_bytes(pdc_raw[:_PDC_LEN], byteorder="little") * _PDC_MA_PER_LSB

    # ------------------------------------------------------------------

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
