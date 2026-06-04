#!/usr/bin/env python3
"""
i2c_parser.py - Parse Lattice .iea/.ied files and log all I2C transactions.
Reads return 0xFF since no hardware is present.

Usage: python i2c_parser.py <algo.iea> <data.ied>
"""

import sys


# ---------------------------------------------------------------------------
# Driver interface
# ---------------------------------------------------------------------------

class I2CDriver:
    """Hardware abstraction layer for I2C operations.

    Default implementation logs all transactions to stdout and returns 0xFF
    bytes for reads (simulation mode). Override this class to drive real
    hardware.
    """

    def is_simulation(self) -> bool:
        """Return True if this is a simulation driver (reads return 0xFF)."""
        return True

    def start(self) -> None:
        print("[I2C] START")

    def restart(self) -> None:
        print("[I2C] RESTART")

    def stop(self) -> None:
        print("[I2C] STOP")

    def write(self, data: bytes) -> None:
        n = len(data)
        print(f"[I2C] WRITE {n} bytes: "
              + " ".join(f"{b:02X}" for b in data) + " ")

    def read(self, num_bytes: int) -> bytes:
        result = bytes([0xFF] * num_bytes)
        print(f"[I2C] READ  {num_bytes} bytes: "
              + " ".join(f"{b:02X}" for b in result) + " ")
        return result

    def creset(self, value: int) -> None:
        """Toggle CRESET pin.  value != 0 → high (release), 0 → low (assert)."""
        pass  # no-op in simulation

    def wait(self, ms: int) -> None:
        """Wait for the given number of milliseconds."""
        pass  # no-op in simulation


# ---------------------------------------------------------------------------
# Opcodes
# ---------------------------------------------------------------------------
I2C_STARTTRAN    = 0x10
I2C_RESTARTTRAN  = 0x11
I2C_ENDTRAN      = 0x12
I2C_TRANSOUT     = 0x13
I2C_TRANSIN      = 0x14
I2C_RUNCLOCK     = 0x15
I2C_WAIT         = 0x16
I2C_LOOP         = 0x17
I2C_ENDLOOP      = 0x18
I2C_TDI          = 0x19
I2C_CONTINUE     = 0x1A
I2C_TDO          = 0x1B
I2C_MASK         = 0x1C
I2C_BEGIN_REPEAT = 0x1D
I2C_END_REPEAT   = 0x1E
I2C_END_FRAME    = 0x1F
I2C_DATA         = 0x20
I2C_PROGRAM      = 0x21
I2C_VERIFY       = 0x22
I2C_DTDI         = 0x23
I2C_DTDO         = 0x24
I2C_COMMENT      = 0x25
I2C_ENDCOMMENT   = 0x26
I2C_TRST         = 0x27
I2C_ENDVME       = 0x7F

# ---------------------------------------------------------------------------
# Data type register bits
# ---------------------------------------------------------------------------
SDR_DATA       = 0x0001
TDI_DATA       = 0x0002
TDO_DATA       = 0x0004
MASK_DATA      = 0x0008
DTDI_DATA      = 0x0010
DTDO_DATA      = 0x0020
COMPRESS       = 0x0040
COMPRESS_FRAME = 0x0080

SUPPORTED_VERSION = "_I2C1.0"

# ---------------------------------------------------------------------------
# Error codes
# ---------------------------------------------------------------------------
ERR_VERIFY_FAIL     = -1
ERR_FIND_ALGO_FILE  = -2
ERR_FIND_DATA_FILE  = -3
ERR_WRONG_VERSION   = -4
ERR_ALGO_FILE_ERROR = -5
ERR_DATA_FILE_ERROR = -6
ERR_OUT_OF_MEMORY   = -7
ERR_VERIFY_ACK_FAIL = -8

ERR_MESSAGES = {
    ERR_VERIFY_FAIL:     "VERIFY FAIL",
    ERR_FIND_ALGO_FILE:  "CANNOT FIND ALGO FILE",
    ERR_FIND_DATA_FILE:  "CANNOT FIND DATA FILE",
    ERR_WRONG_VERSION:   "WRONG FILE TYPE/VERSION",
    ERR_ALGO_FILE_ERROR: "ALGO FILE ERROR",
    ERR_DATA_FILE_ERROR: "DATA FILE ERROR",
    ERR_OUT_OF_MEMORY:   "OUT OF MEMORY",
    ERR_VERIFY_ACK_FAIL: "VERIFY ACK FAIL",
}


class I2CParser:
    def __init__(self, algo_data: bytes, data_data: bytes,
                 driver: I2CDriver = None):
        self.algo_array = algo_data
        self.data_array = data_data
        self.algo_size  = len(algo_data)
        self.data_size  = len(data_data)

        self.g_iMovingAlgoIndex    = 0
        self.g_iMovingDataIndex    = 0
        self.g_iMainDataIndex      = 0
        self.g_iRepeatIndex        = 0
        self.g_iTDIIndex           = 0
        self.g_iTDOIndex           = 0
        self.g_iMASKIndex          = 0
        self.g_ucCompressCounter   = 0
        self.g_usDataType          = 0
        self.g_usLCOUNTSize        = 0
        self.g_iLoopMovingIndex    = 0
        self.g_iLoopDataMovingIndex = 0
        self.driver = driver if driver is not None else I2CDriver()

    # -----------------------------------------------------------------------
    # Low-level helpers
    # -----------------------------------------------------------------------
    def get_byte(self, index: int, algo: bool) -> int:
        if algo:
            return self.algo_array[index] if index < self.algo_size else 0xFF
        else:
            return self.data_array[index] if index < self.data_size else 0xFF

    def ispVMDataSize(self) -> int:
        """Read a variable-length encoded integer from the algo array."""
        size  = 0
        shift = 0
        while True:
            b = self.get_byte(self.g_iMovingAlgoIndex, True)
            self.g_iMovingAlgoIndex += 1
            size |= (b & 0x7F) << shift
            if not (b & 0x80):
                break
            shift += 7
        return size

    # -----------------------------------------------------------------------
    # ispVMShiftExec: parse TDI/TDO/MASK/DTDI/DTDO sub-opcodes
    # -----------------------------------------------------------------------
    def ispVMShiftExec(self, data_size: int) -> int:
        self.g_usDataType &= ~(TDI_DATA | TDO_DATA | MASK_DATA |
                               DTDI_DATA | DTDO_DATA | COMPRESS_FRAME)
        num_bytes = (data_size + 7) // 8

        while True:
            b = self.get_byte(self.g_iMovingAlgoIndex, True)
            self.g_iMovingAlgoIndex += 1

            if b == I2C_CONTINUE:
                break
            elif b == I2C_TDI:
                self.g_usDataType |= TDI_DATA
                self.g_iTDIIndex = self.g_iMovingAlgoIndex
                self.g_iMovingAlgoIndex += num_bytes
            elif b == I2C_DTDI:
                self.g_usDataType |= DTDI_DATA
                if self.get_byte(self.g_iMovingAlgoIndex, True) != I2C_DATA:
                    return ERR_ALGO_FILE_ERROR
                self.g_iMovingAlgoIndex += 1
                if self.g_usDataType & COMPRESS:
                    if self.get_byte(self.g_iMovingDataIndex, False):
                        self.g_usDataType |= COMPRESS_FRAME
                    self.g_iMovingDataIndex += 1
            elif b == I2C_TDO:
                self.g_usDataType |= TDO_DATA
                self.g_iTDOIndex = self.g_iMovingAlgoIndex
                self.g_iMovingAlgoIndex += num_bytes
            elif b == I2C_DTDO:
                self.g_usDataType |= DTDO_DATA
                if self.get_byte(self.g_iMovingAlgoIndex, True) != I2C_DATA:
                    return ERR_ALGO_FILE_ERROR
                self.g_iMovingAlgoIndex += 1
                if self.g_usDataType & COMPRESS:
                    if not (self.g_usDataType & DTDI_DATA):
                        if self.get_byte(self.g_iMovingDataIndex, False):
                            self.g_usDataType |= COMPRESS_FRAME
                        self.g_iMovingDataIndex += 1
            elif b == I2C_MASK:
                self.g_usDataType |= MASK_DATA
                self.g_iMASKIndex = self.g_iMovingAlgoIndex
                self.g_iMovingAlgoIndex += num_bytes
            else:
                return ERR_ALGO_FILE_ERROR
        return 0

    # -----------------------------------------------------------------------
    # ispVMRead: simulate a read (returns all 0xFF), advance data indexes
    # -----------------------------------------------------------------------
    def ispVMRead(self, data_size: int) -> int:
        num_bytes = (data_size + 7) // 8

        # Consume TDI / DTDI bytes (mirrors C InData loop)
        for bit_idx in range(data_size):
            if bit_idx % 8 == 0:
                if self.g_usDataType & TDI_DATA:
                    self.g_iTDIIndex += 1
                elif self.g_usDataType & DTDI_DATA:
                    if self.g_ucCompressCounter:
                        self.g_ucCompressCounter -= 1
                    else:
                        b = self.get_byte(self.g_iMovingDataIndex, False)
                        self.g_iMovingDataIndex += 1
                        if (self.g_usDataType & COMPRESS_FRAME) and b == 0xFF:
                            self.g_ucCompressCounter = self.get_byte(
                                self.g_iMovingDataIndex, False)
                            self.g_iMovingDataIndex += 1
                            self.g_ucCompressCounter -= 1

        # Check END_FRAME for DTDI and advance any DTDO compression flag
        if self.g_usDataType & DTDI_DATA:
            if self.get_byte(self.g_iMovingDataIndex, False) != I2C_END_FRAME:
                return ERR_DATA_FILE_ERROR
            self.g_iMovingDataIndex += 1
            if self.g_usDataType & COMPRESS:
                if self.g_usDataType & DTDO_DATA:
                    self.g_usDataType &= ~COMPRESS_FRAME
                    if self.get_byte(self.g_iMovingDataIndex, False):
                        self.g_usDataType |= COMPRESS_FRAME
                    self.g_iMovingDataIndex += 1

        # Get read data from driver (0xFF in simulation, real bytes on hardware)
        read_data = self.driver.read(num_bytes)

        # Consume TDO / DTDO expected bytes (verification skipped in simulation)
        for bit_idx in range(data_size):
            if bit_idx % 8 == 0:
                if self.g_usDataType & TDO_DATA:
                    self.g_iTDOIndex += 1
                elif self.g_usDataType & DTDO_DATA:
                    if self.g_ucCompressCounter:
                        self.g_ucCompressCounter -= 1
                    else:
                        b = self.get_byte(self.g_iMovingDataIndex, False)
                        self.g_iMovingDataIndex += 1
                        if (self.g_usDataType & COMPRESS_FRAME) and b == 0xFF:
                            self.g_ucCompressCounter = self.get_byte(
                                self.g_iMovingDataIndex, False)
                            self.g_iMovingDataIndex += 1
                            self.g_ucCompressCounter -= 1

        # Simulation mode: always pass verification
        return 0

    # -----------------------------------------------------------------------
    # ispVMSend: send data to device (log the bytes)
    # -----------------------------------------------------------------------
    def ispVMSend(self, data_size: int) -> int:
        num_bytes = (data_size + 7) // 8
        out_data  = []

        for bit_idx in range(data_size):
            if bit_idx % 8 == 0:
                if self.g_usDataType & TDI_DATA:
                    b = self.get_byte(self.g_iTDIIndex, True)
                    self.g_iTDIIndex += 1
                else:
                    # DTDI_DATA
                    if self.g_ucCompressCounter:
                        self.g_ucCompressCounter -= 1
                        b = 0xFF
                    else:
                        b = self.get_byte(self.g_iMovingDataIndex, False)
                        self.g_iMovingDataIndex += 1
                        if (self.g_usDataType & COMPRESS_FRAME) and b == 0xFF:
                            self.g_ucCompressCounter = self.get_byte(
                                self.g_iMovingDataIndex, False)
                            self.g_iMovingDataIndex += 1
                            self.g_ucCompressCounter -= 1
                out_data.append(b)

        self.driver.write(bytes(out_data))
        return 0

    # -----------------------------------------------------------------------
    # ispVMShift: entry point for I2C_TRANSOUT / I2C_TRANSIN
    # -----------------------------------------------------------------------
    def ispVMShift(self, command: int) -> int:
        data_size = self.ispVMDataSize()
        self.g_usDataType = (self.g_usDataType & ~SDR_DATA) | SDR_DATA

        ret = self.ispVMShiftExec(data_size)
        if ret < 0:
            return ret

        if (self.g_usDataType & TDO_DATA) or (self.g_usDataType & DTDO_DATA):
            ret = self.ispVMRead(data_size)
            if self.g_usDataType & DTDO_DATA:
                if self.get_byte(self.g_iMovingDataIndex, False) != I2C_END_FRAME:
                    return ERR_DATA_FILE_ERROR
                self.g_iMovingDataIndex += 1
        else:
            ret = self.ispVMSend(data_size)
            if self.g_usDataType & DTDI_DATA:
                if self.get_byte(self.g_iMovingDataIndex, False) != I2C_END_FRAME:
                    return ERR_DATA_FILE_ERROR
                self.g_iMovingDataIndex += 1
        return ret

    # -----------------------------------------------------------------------
    # ispVMComment: print comment string from algo
    # -----------------------------------------------------------------------
    def ispVMComment(self):
        chars = []
        while True:
            b = self.get_byte(self.g_iMovingAlgoIndex, True)
            self.g_iMovingAlgoIndex += 1
            if b == I2C_ENDCOMMENT:
                break
            chars.append(chr(b))
        print("".join(chars))

    # -----------------------------------------------------------------------
    # ispVMLoop: polling loop (break early on first TRANSIN success)
    # -----------------------------------------------------------------------
    def ispVMLoop(self, loop_count: int) -> int:
        self.g_iLoopMovingIndex     = self.g_iMovingAlgoIndex
        self.g_iLoopDataMovingIndex = self.g_iMovingDataIndex

        for _ in range(loop_count):
            self.g_iMovingAlgoIndex = self.g_iLoopMovingIndex
            self.g_iMovingDataIndex = self.g_iLoopDataMovingIndex
            cont = True

            while cont:
                opcode = self.get_byte(self.g_iMovingAlgoIndex, True)
                self.g_iMovingAlgoIndex += 1

                if opcode == I2C_STARTTRAN:
                    self.driver.start()
                elif opcode == I2C_RESTARTTRAN:
                    self.driver.restart()
                elif opcode == I2C_ENDTRAN:
                    self.driver.stop()
                elif opcode == I2C_TRANSOUT:
                    ret = self.ispVMShift(opcode)
                    if ret < 0:
                        return ret
                elif opcode == I2C_TRANSIN:
                    ret = self.ispVMShift(opcode)
                    if ret >= 0:
                        # Success - exit polling loop immediately
                        return ret
                    else:
                        cont = False
                elif opcode == I2C_WAIT:
                    ms = self.ispVMDataSize()
                    self.driver.wait(ms)
                elif opcode == I2C_COMMENT:
                    self.ispVMComment()
                else:
                    cont = False

        return 0

    # -----------------------------------------------------------------------
    # ispProcessI2C: main opcode dispatch loop
    # -----------------------------------------------------------------------
    def ispProcessI2C(self) -> int:
        c_program = 0

        while True:
            opcode = self.get_byte(self.g_iMovingAlgoIndex, True)
            self.g_iMovingAlgoIndex += 1

            if opcode == 0xFF:
                return ERR_ALGO_FILE_ERROR

            ret = 0

            if opcode == I2C_STARTTRAN:
                self.driver.start()

            elif opcode == I2C_RESTARTTRAN:
                self.driver.restart()

            elif opcode == I2C_ENDTRAN:
                self.driver.stop()

            elif opcode in (I2C_TRANSOUT, I2C_TRANSIN):
                ret = self.ispVMShift(opcode)

            elif opcode == I2C_WAIT:
                ms = self.ispVMDataSize()
                self.driver.wait(ms)

            elif opcode == I2C_BEGIN_REPEAT:
                count = self.ispVMDataSize()
                sub   = self.get_byte(self.g_iMovingAlgoIndex, True)
                self.g_iMovingAlgoIndex += 1
                if sub == I2C_PROGRAM:
                    self.g_iMainDataIndex = self.g_iMovingDataIndex
                    c_program = 1
                elif sub == I2C_VERIFY:
                    if c_program:
                        self.g_iMovingDataIndex = self.g_iMainDataIndex
                        c_program = 0
                self.g_iRepeatIndex = self.g_iMovingAlgoIndex
                for _ in range(count):
                    self.g_iMovingAlgoIndex = self.g_iRepeatIndex
                    ret = self.ispProcessI2C()
                    if ret < 0:
                        break

            elif opcode == I2C_END_REPEAT:
                return 0  # bubble up to the repeat-count loop

            elif opcode == I2C_LOOP:
                self.g_usLCOUNTSize = self.ispVMDataSize()
                ret = self.ispVMLoop(self.g_usLCOUNTSize)
                if ret != 0:
                    return ret

            elif opcode == I2C_ENDLOOP:
                pass  # handled inside ispVMLoop

            elif opcode == I2C_COMMENT:
                self.ispVMComment()

            elif opcode == I2C_TRST:
                trst = self.get_byte(self.g_iMovingAlgoIndex, True)
                self.g_iMovingAlgoIndex += 1
                self.driver.creset(trst)

            elif opcode == I2C_ENDVME:
                if self.g_iMovingAlgoIndex >= self.algo_size:
                    return ret
                # more devices in chain - continue

            else:
                return ERR_ALGO_FILE_ERROR

            if ret < 0:
                return ret

        return ERR_ALGO_FILE_ERROR


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def isp_entry_point(algo_file: str, data_file: str,
                    driver: I2CDriver = None) -> int:
    try:
        with open(algo_file, "rb") as f:
            algo_data = f.read()
    except OSError:
        return ERR_FIND_ALGO_FILE

    try:
        with open(data_file, "rb") as f:
            data_data = f.read()
    except OSError:
        return ERR_FIND_DATA_FILE

    parser = I2CParser(algo_data, data_data, driver=driver)

    # Read compression flag from data file
    compress_flag = parser.get_byte(parser.g_iMovingDataIndex, False)
    parser.g_iMovingDataIndex += 1
    if compress_flag:
        parser.g_usDataType |= COMPRESS

    # Read and verify version string from algo file
    version = "".join(
        chr(parser.get_byte(parser.g_iMovingAlgoIndex + i, True))
        for i in range(len(SUPPORTED_VERSION))
    )
    parser.g_iMovingAlgoIndex += len(SUPPORTED_VERSION)

    if version != SUPPORTED_VERSION:
        print(f"Error: Wrong version '{version}', expected '{SUPPORTED_VERSION}'")
        return ERR_WRONG_VERSION

    # EnableHardware (START + STOP bus test), then run, then DisableHardware (STOP)
    parser.driver.start()
    parser.driver.stop()
    ret = parser.ispProcessI2C()
    parser.driver.stop()
    return ret


def main():
    print("         Lattice Semiconductor Corp.")
    print("       ispI2C Parser (Python) v1.0\n")

    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <algo.iea> <data.ied>")
        sys.exit(1)

    algo_file = sys.argv[1]
    data_file = sys.argv[2]

    for path in (algo_file, data_file):
        if not path.lower().endswith((".iea", ".ied")):
            print(f"Error: I2C files must end with .iea or .ied\n")
            sys.exit(-1)

    ret = isp_entry_point(algo_file, data_file)

    if ret < 0:
        msg = ERR_MESSAGES.get(ret, "UNKNOWN ERROR")
        print(f"\nProcessing failure: {msg}")
        print("+=======+")
        print("| FAIL! |")
        print("+=======+\n")
    else:
        print("\n+=======+")
        print("| PASS! |")
        print("+=======+\n")

    sys.exit(ret)


if __name__ == "__main__":
    main()
