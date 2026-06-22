"""Shared wire-framing helpers for the OpenMotion packet protocol.

Both host transports speak the same framed protocol and receive the same kind
of unsolicited firmware log packets, so the framing and log-demux logic lives
here once instead of being copied into each transport:

  * the console's synchronous serial link (:class:`omotion.MotionUart`), and
  * the sensor's asynchronous USB-bulk link (:class:`omotion.CommInterface`).

Frame layout (matches the firmware ``common.h``)::

    [0xAA][id:2][type:1][cmd:1][addr:1][rsvd:1][len:2][payload:N][crc16:2][0xDD]

Firmware mirrors ``printf()`` over the same link as an *unsolicited* packet with
``id == 0``, ``packetType == OW_DATA`` and ``command == OW_CMD_ECHO`` (see the
``DEBUG_FLAG_USB_PRINTF`` debug flag). Because those packets share the link with
command responses, every receiver has to recognise and skip them — that is what
:func:`is_log_packet` / :func:`emit_log_packet` are for.
"""
from __future__ import annotations

import logging
from typing import Optional

from omotion import _log_root
from omotion.UartPacket import UartPacket
from omotion.config import (
    OW_CMD_ECHO,
    OW_DATA,
    OW_END_BYTE,
    OW_START_BYTE,
)

logger = logging.getLogger(f"{_log_root}.framing" if _log_root else "framing")

# Bytes before the payload: start(1)+id(2)+type(1)+cmd(1)+addr(1)+rsvd(1)+len(2)
_HEADER_LEN = 9
# Bytes after the payload: crc(2)+end(1)
_TRAILER_LEN = 3
# Total framing overhead around the payload.
FRAME_OVERHEAD = _HEADER_LEN + _TRAILER_LEN  # 12

# Sanity ceiling on the length field used only to reject a garbled stream while
# resynchronising. Set to the larger of the two firmwares' limits (sensor =
# 8192) so this is safe for both transports; it is not a protocol max.
MAX_FRAME_DATA_LEN = 8192


def extract_frame(buf: bytearray) -> Optional[bytes]:
    """Pop one complete framed packet from the front of ``buf``.

    Skips leading garbage and resynchronises past false start bytes (a 0xAA
    whose declared length doesn't land an 0xDD where expected). Returns the raw
    frame bytes (including the 0xAA/0xDD) and removes them from ``buf``, or
    returns ``None`` when no complete frame is available yet — in which case the
    caller should read more bytes and try again. Mutates ``buf`` in place.

    CRC is **not** validated here; construct a :class:`UartPacket` from the
    returned bytes to validate it.
    """
    while True:
        if not buf:
            return None
        if buf[0] != OW_START_BYTE:
            idx = buf.find(OW_START_BYTE)
            if idx == -1:
                buf.clear()  # no frame start anywhere — drop it all
                return None
            del buf[:idx]
        if len(buf) < _HEADER_LEN:
            return None  # not enough yet to read the length field
        data_len = int.from_bytes(buf[7:9], "big")
        if data_len > MAX_FRAME_DATA_LEN:
            del buf[:1]  # implausible length: this 0xAA wasn't a real start
            continue
        frame_len = FRAME_OVERHEAD + data_len
        if len(buf) < frame_len:
            return None  # whole frame hasn't arrived yet
        if buf[frame_len - 1] != OW_END_BYTE:
            del buf[:1]  # end byte not where the length said: false start
            continue
        frame = bytes(buf[:frame_len])
        del buf[:frame_len]
        return frame


def is_log_packet(pkt: UartPacket) -> bool:
    """True if ``pkt`` is an unsolicited firmware log/printf packet rather than
    a command response.

    Firmware mirrors ``printf()`` as ``id == 0`` / ``OW_DATA`` / ``OW_CMD_ECHO``
    on the same link that carries responses (``DEBUG_FLAG_USB_PRINTF``).
    """
    return (
        pkt.id == 0
        and pkt.packetType == OW_DATA
        and pkt.command == OW_CMD_ECHO
    )


def emit_log_packet(pkt: UartPacket, desc: str) -> None:
    """Surface a firmware log packet (see :func:`is_log_packet`) via the logger.

    The payload is the firmware ``printf`` text; decode it leniently so a
    partial/garbled line never raises.
    """
    raw = bytes(pkt.data[: pkt.data_len]) if pkt.data_len > 0 else b""
    try:
        text = raw.decode("utf-8", errors="replace").rstrip("\x00").strip()
    except Exception:
        text = ""
    if text:
        logger.warning("[%s PRINTF] %s", desc, text)
    else:
        logger.warning("[%s] MCU echo: data=%s", desc, raw.hex() if raw else "")
