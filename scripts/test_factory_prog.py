#!/usr/bin/env python3
"""
test_factory_prog.py - Deploy a Lattice FPGA bitstream via the OpenMotion Sensor board.

Parses the .iea/.ied algorithm/data files produced by Lattice Diamond and replays
every I2C transaction against the real FPGA over the STM32H7-based programmer board.

Usage:
    python lattice_deploy.py <algo.iea> <data.ied> [--port COMx] [--addr 0x40]

If --port is omitted the OWInterface auto-discovers the USB-CDC device.
"""

import argparse
import sys
import logging
import os
import time
from enum import Enum, auto

# ---------------------------------------------------------------------------
# Locate i2c_parser in the sibling i2cem project
# ---------------------------------------------------------------------------
_THIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_PARSER_DIR = os.path.join(_THIS_DIR, "..", "..", "i2cem")
if _PARSER_DIR not in sys.path:
    sys.path.insert(0, _PARSER_DIR)

from omotion.i2c_parser import I2CDriver, isp_entry_point, ERR_MESSAGES  # noqa: E402

# ---------------------------------------------------------------------------
# OPENMotion SDK 
# ---------------------------------------------------------------------------
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)
from omotion import MotionInterface  # noqa: E402
from omotion.MotionSensor import MotionSensor  # noqa: E402

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Transaction state machine
# ---------------------------------------------------------------------------

class _TxState(Enum):
    IDLE        = auto()
    AFTER_START = auto()   # received START, waiting for address byte
    WRITE_PHASE = auto()   # collecting write payload
    READ_ONLY   = auto()   # START + read-address → pure read
    AFTER_RESTART = auto() # received RESTART, waiting for address byte
    READ_PHASE  = auto()   # after RESTART+address → combined write-read


class HardwareDriver(I2CDriver):
    """I2CDriver that groups raw I2C signals into MotionSensor transactions.

    The .iea file encodes the I2C address as the first WRITE byte after each
    START/RESTART:
        0x80  =  (0x40 << 1) | 0  =  address 0x40, write
        0x81  =  (0x40 << 1) | 1  =  address 0x40, read

    The driver strips that address byte, accumulates subsequent write bytes,
    then on STOP (or READ) dispatches the appropriate MotionSensor call:
        Pure write:        i2c_write(dev_addr, write_data)
        Write then read:   i2c_write_read(dev_addr, write_data, read_len)
        Pure read:         i2c_read(dev_addr, read_len)

    A START immediately followed by STOP (EnableHardware bus-test) is silently
    ignored because write_data will be empty when stop() is called.
    """

    def __init__(self, ifc: MotionSensor, default_addr: int = 0x40):
        self._ifc          = ifc
        self._default_addr = default_addr
        self._state        = _TxState.IDLE
        self._addr         = default_addr
        self._write_buf    = bytearray()

    # ------------------------------------------------------------------

    def is_simulation(self) -> bool:
        return False

    # ------------------------------------------------------------------

    def start(self) -> None:
        self._state     = _TxState.AFTER_START
        self._addr      = self._default_addr
        self._write_buf = bytearray()
        logger.debug("START")

    def restart(self) -> None:
        self._state = _TxState.AFTER_RESTART
        logger.debug("RESTART")

    def stop(self) -> None:
        logger.debug("STOP (state=%s, write_len=%d)", self._state, len(self._write_buf))
        if self._state == _TxState.WRITE_PHASE and self._write_buf:
            logger.info("i2c_write addr=0x%02X len=%d data=%s",
                        self._addr, len(self._write_buf),
                        self._write_buf.hex())
            self._ifc.i2c_write(self._addr, bytes(self._write_buf))
        # READ_PHASE: already dispatched inside read(); just reset
        self._state     = _TxState.IDLE
        self._write_buf = bytearray()

    # ------------------------------------------------------------------

    def write(self, data: bytes) -> None:
        if not data:
            return

        if self._state == _TxState.AFTER_START:
            # First WRITE after START: the address byte
            addr_byte  = data[0]
            self._addr = addr_byte >> 1            # extract 7-bit address
            if addr_byte & 0x01:
                self._state = _TxState.READ_ONLY  # read-address without prior RESTART
            else:
                self._state = _TxState.WRITE_PHASE
            # Remaining bytes in the same call (rare) are payload
            if len(data) > 1:
                self._write_buf += data[1:]

        elif self._state == _TxState.WRITE_PHASE:
            self._write_buf += data

        elif self._state == _TxState.AFTER_RESTART:
            # First WRITE after RESTART: the read-address byte
            addr_byte  = data[0]
            self._addr = addr_byte >> 1
            self._state = _TxState.READ_PHASE
            if len(data) > 1:
                # Unexpected payload after restart address — keep it
                self._write_buf += data[1:]

        else:
            logger.warning("write() called in unexpected state %s", self._state)

        logger.debug("write %d bytes: %s  [state=%s]",
                     len(data), data.hex(), self._state)

    # ------------------------------------------------------------------

    def read(self, num_bytes: int) -> bytes:
        logger.debug("read %d bytes (state=%s, write_len=%d)",
                     num_bytes, self._state, len(self._write_buf))

        if self._state == _TxState.READ_PHASE:
            if self._write_buf:
                # Combined write-then-read with RESTART
                logger.info("i2c_write_read addr=0x%02X write_len=%d read_len=%d data=%s",
                            self._addr, len(self._write_buf), num_bytes,
                            self._write_buf.hex())
                result = self._ifc.i2c_write_read(
                    self._addr, bytes(self._write_buf), num_bytes)
            else:
                # Degenerate case: RESTART + read-addr + read, no prior write
                logger.info("i2c_read addr=0x%02X len=%d", self._addr, num_bytes)
                result = self._ifc.i2c_read(self._addr, num_bytes)
        elif self._state == _TxState.READ_ONLY:
            # START + read-addr (no RESTART) + read
            logger.info("i2c_read addr=0x%02X len=%d", self._addr, num_bytes)
            result = self._ifc.i2c_read(self._addr, num_bytes)
        else:
            logger.warning("read() called in unexpected state %s — falling back to i2c_read",
                           self._state)
            result = self._ifc.i2c_read(self._addr, num_bytes)

        # After dispatching the read the write buffer is consumed
        self._write_buf = bytearray()
        logger.debug("read result: %s", result.hex() if result else "(empty)")
        return result

    # ------------------------------------------------------------------

    def select_camera(self, camera: int) -> None:
        """Select the active camera. User-facing cameras are 1-8; firmware uses 0-7."""
        if not (1 <= camera <= 8):
            raise ValueError(f"camera must be 1–8, got {camera}")
        camera_id = camera - 1
        logger.info("select_camera %d -> firmware camera_id %d", camera, camera_id)
        self._ifc.switch_camera(camera_id)

    def creset(self, value: int) -> None:
        state = value != 0
        logger.info("creset %s", "HIGH (release)" if state else "LOW (assert)")
        self._ifc.creset(state)

    def wait(self, ms: int) -> None:
        logger.info("wait %d ms", ms)
        time.sleep(ms / 1000.0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:

    parser = argparse.ArgumentParser(
        description="Program a Lattice FPGA Sensor board.")
    parser.add_argument("algo", metavar="ALGO.IEA",
                        help="Lattice algorithm file (.iea)")
    parser.add_argument("data", metavar="DATA.IED",
                        help="Lattice data file (.ied)")
    parser.add_argument("--sensor", default="left",
                        help="Sensor Module [left, right] (default left).")
    parser.add_argument("--cam", default=1, type=int,
                        help="Sensor Module [1-8] (default 1).")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="Per-command timeout in seconds (default 5.0).")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable DEBUG logging.")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s")

    if args.sensor not in ("left", "right"):
        print(f"Error: --sensor must be 'left' or 'right', got '{args.sensor}'", file=sys.stderr)
        sys.exit(1)

    if not (1 <= args.cam <= 8):
        print(f"Error: --cam must be 1–8, got {args.cam}", file=sys.stderr)
        sys.exit(1)

    # Validate input files
    for path in (args.algo, args.data):
        if not os.path.isfile(path):
            print(f"Error: file not found: {path}", file=sys.stderr)
            sys.exit(1)

    # Connect to programmer board
    print("Connecting to Motion Sensor...")

    _CONNECT_TIMEOUT = 12.0
    iface = MotionInterface()
    iface.start(wait=True, wait_timeout=_CONNECT_TIMEOUT)

    # Poll for connection
    def _await(handle, label):
        deadline = time.monotonic() + _CONNECT_TIMEOUT
        while time.monotonic() < deadline:
            if handle.is_connected():
                return True
            time.sleep(0.1)
        return False

    console_connected = _await(iface.console, "Console")
    left_connected    = _await(iface.left,    "Left sensor")
    right_connected   = _await(iface.right,   "Right sensor")

    if console_connected and left_connected and right_connected:
        print("MOTION System fully connected.")
    else:
        print(f"MOTION System NOT Fully Connected. CONSOLE: {console_connected}, SENSOR (LEFT,RIGHT): {left_connected}, {right_connected}")

    if not left_connected and not right_connected:
        print("Sensor Modules not connected.")
        iface.stop()
        exit(1)

    sensor = iface.left if args.sensor == "left" else iface.right
    if not sensor.is_connected():
        print(f"Requested sensor '{args.sensor}' is not connected.")
        iface.stop()
        exit(1)

    print(f"Connected.  Sensor: {args.sensor}  Camera: {args.cam}\n")

    # Run the bitstream deployment
    driver = HardwareDriver(sensor)
    print(f"Programming FPGA from:\n  algo: {args.algo}\n  data: {args.data}\n")

    print(f"Selecting camera {args.cam}...")
    driver.select_camera(args.cam)

    try:
        ret = isp_entry_point(args.algo, args.data, driver=driver)
    finally:
        iface.stop()

    if ret < 0:
        msg = ERR_MESSAGES.get(ret, "UNKNOWN ERROR")
        print(f"\nProgramming failed: {msg}", file=sys.stderr)
        print("+=======+")
        print("| FAIL! |")
        print("+=======+\n")
        sys.exit(abs(ret))
    else:
        print("\n+=========+")
        print("| PASSED! |")
        print("+=========+\n")



if __name__ == "__main__":
    main()
