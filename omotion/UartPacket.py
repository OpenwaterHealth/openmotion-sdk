import logging

from omotion.config import OW_END_BYTE, OW_START_BYTE
from omotion.utils import util_crc16
from omotion import _log_root

# Set up logging
logger = logging.getLogger(f"{_log_root}.UARTPACKET" if _log_root else "UARTPACKET")


class UartPacket:
    def __init__(
        self,
        id=None,
        packetType=None,
        command=None,
        addr=None,
        reserved=None,
        data=None,
        buffer=None,
    ):
        if buffer:
            self.from_buffer(buffer)
        else:
            self.id = id
            self.packetType = packetType
            self.command = command
            self.addr = addr
            self.reserved = reserved
            self.data = data if data is not None else []
            self.data_len = len(self.data)
            self.crc = self.calculate_crc()

    def calculate_crc(self) -> int:
        crc_value = 0xFFFF
        packet = bytearray()
        packet.append(OW_START_BYTE)
        packet.extend(self.id.to_bytes(2, "big"))
        packet.append(self.packetType)
        packet.append(self.command)
        packet.append(self.addr)
        packet.append(self.reserved)
        packet.extend(self.data_len.to_bytes(2, "big"))
        if self.data_len > 0:
            packet.extend(self.data)
        crc_value = util_crc16(packet[1:])
        return crc_value

    def to_bytes(self) -> bytes:
        buffer = bytearray()
        buffer.append(OW_START_BYTE)
        buffer.extend(self.id.to_bytes(2, "big"))
        buffer.append(self.packetType)
        buffer.append(self.command)
        buffer.append(self.addr)
        buffer.append(self.reserved)
        buffer.extend(self.data_len.to_bytes(2, "big"))
        if self.data_len > 0:
            buffer.extend(self.data)
        crc_value = util_crc16(buffer[1:])
        buffer.extend(crc_value.to_bytes(2, "big"))
        buffer.append(OW_END_BYTE)
        return bytes(buffer)

    def from_buffer(self, buffer: bytes):
        if buffer[0] != OW_START_BYTE or buffer[-1] != OW_END_BYTE:
            logger.error(f"Missing Start or End Byte Packet LEN {str(len(buffer))}")
            logger.debug(buffer)
            raise ValueError("Invalid buffer format")

        self.id = int.from_bytes(buffer[1:3], "big")
        self.packetType = buffer[3]
        self.command = buffer[4]
        self.addr = buffer[5]
        self.reserved = buffer[6]
        self.data_len = int.from_bytes(buffer[7:9], "big")
        self.data = bytearray(buffer[9 : 9 + self.data_len])
        crc_value = util_crc16(buffer[1 : 9 + self.data_len])
        self.crc = int.from_bytes(buffer[9 + self.data_len : 11 + self.data_len], "big")
        if self.crc != crc_value:
            logger.error(
                f"Packet CRC: {str(self.crc)}, Calculated CRC: {str(crc_value)}"
            )
            raise ValueError("CRC mismatch")

    def print_packet(self, full=False):
        logger.debug(
            "UartPacket: id=%s type=0x%02X cmd=0x%02X addr=0x%02X reserved=0x%02X "
            "data_len=%s data=%s crc=0x%04X",
            self.id,
            self.packetType,
            self.command,
            self.addr,
            self.reserved,
            self.data_len,
            self.data.hex() if self.data else "",
            self.crc,
        )

    def __str__(self):
        return (
            f"UartPacket(id={self.id}, "
            f"type=0x{self.packetType:02X}, "
            f"cmd=0x{self.command:02X}, "
            f"addr=0x{self.addr:02X}, "
            f"reserved=0x{self.reserved:02X}, "
            f"data_len={self.data_len}, "
            f"data={self.data.hex()}"
            f"crc=0x{self.crc:04X}"
        )
