"""
FPGAProgrammer.py

High-level FPGA page-by-page programming workflow.

Uses :class:`FpgaPageProgrammer` to drive the Lattice MachXO2 configuration
sequence one 16-byte page at a time over the UART link.

Usage::

    from transport import SerialTransport
    from api import HardwareAPI, FpgaPageProgrammer

    with SerialTransport("/dev/ttyACM0", timeout=5.0) as transport:
        hw   = HardwareAPI(transport)
        prog = FpgaPageProgrammer(hw)
        prog.program_from_jedec("my_design.jed")
"""

from __future__ import annotations

import logging
import sys
import os
from pathlib import Path
import time
from typing import Callable, Optional
from omotion import _log_root
from omotion.CommandError import CommandError
from omotion.MotionConsole import MotionConsole
from omotion.config import (
    XO2_FLASH_PAGE_SIZE,
    ERASE_ALL,
    FPGA_PROG_BATCH_PAGES,
    MuxChannel,
)


logger = logging.getLogger(f"{_log_root}.FPGAProgrammer" if _log_root else "FPGAProgrammer")

# --------------------------------------------------------------------------- #
# Make the project-root jedec_parser importable from the py-demo subtree
# --------------------------------------------------------------------------- #
_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from omotion.jedecParser import parse_jedec_file  # noqa: E402


# --------------------------------------------------------------------------- #
# Exceptions
# --------------------------------------------------------------------------- #


class FpgaUpdateError(RuntimeError):
    """Raised when the FPGA update sequence fails at any step."""


# --------------------------------------------------------------------------- #
# Type alias for optional progress callback
# --------------------------------------------------------------------------- #
ProgressCallback = Callable[[int, int], None]
"""
Signature: ``callback(pages_done: int, total_pages: int)``

Called after each batch of pages written so callers can render a progress bar.
"""


# --------------------------------------------------------------------------- #
# Helpers: feature-row / feabits conversion
# --------------------------------------------------------------------------- #


def _bitstring_to_bytes(bitstr: str) -> bytes:
    """
    Convert an MSB-first binary string into bytes.

    Matches the ``bitstring_to_bytes()`` function inside ``jedec_parser.py``.

    Parameters
    ----------
    bitstr:
        A string of ``'0'`` and ``'1'`` characters whose length is a multiple
        of 8 (after left-padding with zeros if necessary).

    Returns
    -------
    bytes
    """
    # Pad to a multiple of 8 bits
    padded = bitstr.zfill((len(bitstr) + 7) // 8 * 8)
    out = []
    p = len(padded) - 1
    for _ in range(len(padded) // 8):
        val = 0
        for _ in range(8):
            val = (val << 1) | (1 if padded[p] == "1" else 0)
            p -= 1
        out.append(val)
    return bytes(out)


def _parse_extra(extra: dict) -> tuple[bytes, bytes]:
    """
    Extract and convert the ``feature_row`` / ``feabits`` entries from the
    *extra* dictionary returned by :func:`~jedec_parser.parse_jedec_file`.

    Returns
    -------
    tuple[bytes, bytes]
        ``(feature_row_bytes, feabits_bytes)`` where each value is already
        in the binary format expected by the FPGA programming commands.
    """
    default_feature_row = bytes(8)
    default_feabits = bytes(2)

    raw_fr = extra.get("feature_row", None)
    raw_fb = extra.get("feabits", None)

    feature_row = _bitstring_to_bytes(raw_fr) if raw_fr else default_feature_row
    feabits = _bitstring_to_bytes(raw_fb.zfill(16)) if raw_fb else default_feabits

    return feature_row, feabits


# --------------------------------------------------------------------------- #
# FpgaPageProgrammer
# --------------------------------------------------------------------------- #


class FpgaPageProgrammer:
    """
    Page-by-page FPGA programming orchestrator.

    Instead of buffering the entire bitstream on the MCU and programming it
    in one shot, this class drives the Lattice MachXO2 configuration sequence
    one 16-byte page at a time over the UART link.

    Sequence (mirrors ``XO2ECA_apiProgram``):

    1. Open config interface (offline mode)
    2. Erase flash sectors (CFG + UFM + FeatureRow)
    3. Reset CFG address → write all CFG pages
    4. Optionally verify CFG pages (reset addr → read back each page)
    5. (If UFM data present) Reset UFM address → write all UFM pages
    6. Optionally verify UFM pages
    7. Write Feature Row
    8. Optionally verify Feature Row
    9. Set DONE bit
    10. Refresh (load config/boot to user mode)

    On any failure an attempt is made to close the config interface so the
    device is left in a clean state.

    Parameters
    ----------
    api:
        An initialised :class:`~api.commands.HardwareAPI` instance whose
        transport is already open.
    verify:
        If True, read back each sector after writing and compare with the
        written data.  Adds one round-trip per page.  Default True.
    erase_mode:
        Erase bitmap passed to ``FPGA_PROG_ERASE``.  Default
        :data:`~protocol.constants.ERASE_ALL` (CFG + UFM + FeatureRow).
    erase_timeout:
        Seconds to wait for ``FPGA_PROG_ERASE`` to complete.  For
        MachXO2-7000 the firmware applies a 4800 ms fixed delay then polls
        for BUSY to clear (up to ``XO2ECA_CMD_LOOP_TIMEOUT`` = 25 000 ms)
        giving a worst-case of ~30 s.  Default 35.0 to give a 5 s margin.
    """

    def __init__(
        self,
        api: MotionConsole,
        verify: bool = True,
        erase_mode: int = ERASE_ALL,
        erase_timeout: float = 35.0,
        refresh_timeout: float = 10.0,
    ) -> None:
        self._api = api
        self._verify = verify
        self._erase_mode = erase_mode
        self._erase_timeout = erase_timeout
        self._refresh_timeout = refresh_timeout

    # ------------------------------------------------------------------ #

    def program_from_jedec(
        self,
        target_fpga: MuxChannel,
        jedec_path: str | os.PathLike,
        on_progress: Optional[ProgressCallback] = None,
    ) -> None:
        """
        Parse a JEDEC file and program the FPGA page by page.

        Parameters
        ----------
        jedec_path:
            Path to the ``.jed`` JEDEC bitstream file.
        on_progress:
            Optional ``(pages_written, total_pages)`` callback invoked after
            every CFG page write.

        Raises
        ------
        FpgaUpdateError
            On parse failure, communication error, or verify mismatch.
        """
        jedec_path = Path(jedec_path)
        if not jedec_path.exists():
            raise FpgaUpdateError(f"JEDEC file not found: {jedec_path}")

        logger.info("Parsing JEDEC file: %s", jedec_path)
        try:
            image, extra = parse_jedec_file(str(jedec_path))
        except Exception as exc:
            raise FpgaUpdateError(f"Failed to parse JEDEC file: {exc}") from exc

        feature_row, feabits = _parse_extra(extra)
        cfg_data = image.data  # all fuse data → CFG sector
        ufm_data = b""  # UFM not separately included in parsed JEDEC

        self.program_raw(
            target_fpga, cfg_data, ufm_data, feature_row, feabits, on_progress
        )

    # ------------------------------------------------------------------ #

    def program_raw(
        self,
        target_fpga: MuxChannel,
        cfg_data: bytes,
        ufm_data: bytes,
        feature_row: bytes,
        feabits: bytes,
        on_progress: Optional[ProgressCallback] = None,
    ) -> None:
        """
        Program the FPGA from raw sector data.

        Parameters
        ----------
        cfg_data:
            Configuration sector bytes.  Must be a multiple of 16.
        ufm_data:
            UFM sector bytes.  May be empty.  Must be a multiple of 16.
        feature_row:
            8-byte feature row data.
        feabits:
            2-byte FEABITS data.
        on_progress:
            Optional ``(pages_written, total_pages)`` callback.
        """
        if len(cfg_data) % XO2_FLASH_PAGE_SIZE != 0:
            raise FpgaUpdateError(
                f"cfg_data length {len(cfg_data)} is not a multiple of "
                f"{XO2_FLASH_PAGE_SIZE}"
            )
        if len(ufm_data) % XO2_FLASH_PAGE_SIZE != 0:
            raise FpgaUpdateError(
                f"ufm_data length {len(ufm_data)} is not a multiple of "
                f"{XO2_FLASH_PAGE_SIZE}"
            )

        cfg_pages = len(cfg_data) // XO2_FLASH_PAGE_SIZE
        ufm_pages = len(ufm_data) // XO2_FLASH_PAGE_SIZE
        total_pages = cfg_pages + ufm_pages
        written = 0

        logger.info(
            "Page programmer: cfg=%d pages, ufm=%d pages, verify=%s",
            cfg_pages,
            ufm_pages,
            self._verify,
        )

        api = self._api

        # ---------------------------------------------------------------- #
        # Step 1 – Open config interface
        # ---------------------------------------------------------------- #
        logger.info("Step 1: Opening config interface (offline mode) …")
        print("  [1/10] Opening config interface …", flush=True)
        try:
            # Some devices may be slow to respond right after connection —
            # perform a small retry loop to improve robustness.
            last_exc = None
            for attempt in range(3):
                try:
                    api.fpga_prog_open(fpga_chan=target_fpga)
                    last_exc = None
                    break
                except CommandError as exc:
                    last_exc = exc
                    time.sleep(0.5)
            if last_exc:
                raise FpgaUpdateError(
                    f"FPGA_PROG_OPEN failed after retries: {last_exc}"
                ) from last_exc
        except CommandError as exc:
            # Fallback: ensure any CommandError is wrapped as FpgaUpdateError
            raise FpgaUpdateError(f"FPGA_PROG_OPEN failed: {exc}") from exc

        # Read status register immediately after OPEN for diagnostic baseline.
        try:
            sr = api.fpga_prog_read_status(fpga_chan=target_fpga)
            isc_en = (sr >> 14) & 1
            fail = (sr >> 13) & 1
            busy = (sr >> 12) & 1
            print(
                f"         Status after OPEN: 0x{sr:08X}  "
                f"ISC_EN={isc_en} FAIL={fail} BUSY={busy}",
                flush=True,
            )
            if fail:
                print(
                    "  [!] WARNING: FAIL bit already set after OPEN — "
                    "FPGA may be in a bad state. Proceeding anyway.",
                    flush=True,
                )
        except CommandError:
            pass  # diagnostic only; don't abort

        try:
            # ------------------------------------------------------------ #
            # Step 2 – Erase flash
            # ------------------------------------------------------------ #
            logger.info("Step 2: Erasing flash (mode=0x%02X) …", self._erase_mode)
            print(
                f"  [2/10] Erasing flash (timeout={self._erase_timeout:.0f} s) …",
                flush=True,
            )

            try:
                api.fpga_prog_erase(fpga_chan=target_fpga, mode=self._erase_mode)
            except CommandError as exc:
                # Read status register so we can report FAIL/BUSY bits.
                try:
                    sr = api.fpga_prog_read_status(fpga_chan=target_fpga)
                    isc_en = (sr >> 14) & 1
                    fail = (sr >> 13) & 1
                    busy = (sr >> 12) & 1
                    detail = (
                        f"Status Register: 0x{sr:08X}  "
                        f"ISC_EN={isc_en} FAIL={fail} BUSY={busy}"
                    )
                except CommandError:
                    detail = "(status register unreadable)"
                raise FpgaUpdateError(
                    f"FPGA_PROG_ERASE failed: {exc}  [{detail}]"
                ) from exc

            print("         Erase done.", flush=True)

            # ------------------------------------------------------------ #
            # Step 3 – Program CFG sector
            # ------------------------------------------------------------ #
            logger.info("Step 3: Programming CFG sector (%d pages) …", cfg_pages)
            print(
                f"  [3/10] Write CFG: {cfg_pages} pages (batch={FPGA_PROG_BATCH_PAGES}) …",
                flush=True,
            )
            try:
                api.fpga_prog_cfg_reset(fpga_chan=target_fpga)
            except CommandError as exc:
                raise FpgaUpdateError(f"CFG reset address failed: {exc}") from exc

            i = 0
            while i < cfg_pages:
                batch = min(FPGA_PROG_BATCH_PAGES, cfg_pages - i)
                chunk = cfg_data[
                    i * XO2_FLASH_PAGE_SIZE : (i + batch) * XO2_FLASH_PAGE_SIZE
                ]
                try:
                    api.fpga_prog_cfg_write_pages(fpga_chan=target_fpga, pages=chunk)
                except CommandError as exc:
                    raise FpgaUpdateError(
                        f"CFG write failed at page {i}: {exc}"
                    ) from exc
                written += batch
                i += batch
                if on_progress:
                    on_progress(written, total_pages)

            # ------------------------------------------------------------ #
            # Step 4 – Verify CFG sector
            # ------------------------------------------------------------ #
            if self._verify and cfg_pages > 0:
                logger.info("Step 4: Verifying CFG sector …")
                print(f"  [4/10] Verify CFG: {cfg_pages} pages …", flush=True)
                try:
                    api.fpga_prog_cfg_reset(fpga_chan=target_fpga)
                except CommandError as exc:
                    raise FpgaUpdateError(
                        f"CFG reset address (verify) failed: {exc}"
                    ) from exc

                for i in range(cfg_pages):
                    expected = cfg_data[
                        i * XO2_FLASH_PAGE_SIZE : (i + 1) * XO2_FLASH_PAGE_SIZE
                    ]
                    try:
                        read_back = api.fpga_prog_cfg_read_page(fpga_chan=target_fpga)
                    except CommandError as exc:
                        raise FpgaUpdateError(
                            f"CFG read-back failed at page {i}: {exc}"
                        ) from exc
                    if read_back != expected:
                        raise FpgaUpdateError(
                            f"CFG verify mismatch at page {i}: "
                            f"expected={expected.hex()} got={read_back.hex()}"
                        )
                    if i % 500 == 0 or i == cfg_pages - 1:
                        pct = 100.0 * (i + 1) / cfg_pages
                        print(
                            f"\r         Verify CFG: {i + 1:>5}/{cfg_pages} "
                            f"({pct:5.1f}%)",
                            end="",
                            flush=True,
                        )
                print(flush=True)  # newline after verify progress
                logger.info("CFG verify passed (%d pages).", cfg_pages)
                print("         CFG verify passed.", flush=True)

            # ------------------------------------------------------------ #
            # Step 5 – Program UFM sector
            # ------------------------------------------------------------ #
            if ufm_pages > 0:
                logger.info("Step 5: Programming UFM sector (%d pages) …", ufm_pages)
                print(f"  [5/10] Write UFM: {ufm_pages} pages …", flush=True)
                try:
                    api.fpga_prog_ufm_reset(fpga_chan=target_fpga)
                except CommandError as exc:
                    raise FpgaUpdateError(f"UFM reset address failed: {exc}") from exc

                i = 0
                while i < ufm_pages:
                    batch = min(FPGA_PROG_BATCH_PAGES, ufm_pages - i)
                    chunk = ufm_data[
                        i * XO2_FLASH_PAGE_SIZE : (i + batch) * XO2_FLASH_PAGE_SIZE
                    ]
                    try:
                        api.fpga_prog_ufm_write_pages(
                            fpga_chan=target_fpga, pages=chunk
                        )
                    except CommandError as exc:
                        raise FpgaUpdateError(
                            f"UFM write failed at page {i}: {exc}"
                        ) from exc
                    written += batch
                    i += batch
                    if on_progress:
                        on_progress(written, total_pages)

                # -------------------------------------------------------- #
                # Step 6 – Verify UFM sector
                # -------------------------------------------------------- #
                if self._verify:
                    logger.info("Step 6: Verifying UFM sector …")
                    print(f"  [6/10] Verify UFM: {ufm_pages} pages …", flush=True)
                    try:
                        api.fpga_prog_ufm_reset(fpga_chan=target_fpga)
                    except CommandError as exc:
                        raise FpgaUpdateError(
                            f"UFM reset address (verify) failed: {exc}"
                        ) from exc

                    for i in range(ufm_pages):
                        expected = ufm_data[
                            i * XO2_FLASH_PAGE_SIZE : (i + 1) * XO2_FLASH_PAGE_SIZE
                        ]
                        try:
                            read_back = api.fpga_prog_ufm_read_page(
                                fpga_chan=target_fpga
                            )
                        except CommandError as exc:
                            raise FpgaUpdateError(
                                f"UFM read-back failed at page {i}: {exc}"
                            ) from exc
                        if read_back != expected:
                            raise FpgaUpdateError(
                                f"UFM verify mismatch at page {i}: "
                                f"expected={expected.hex()} got={read_back.hex()}"
                            )
                    logger.info("UFM verify passed (%d pages).", ufm_pages)

            # ------------------------------------------------------------ #
            # Step 7 – Write Feature Row
            # ------------------------------------------------------------ #
            logger.info("Step 7: Writing Feature Row …")
            print("  [7/10] Writing Feature Row …", flush=True)
            try:
                api.fpga_prog_featrow_write(
                    fpga_chan=target_fpga, feature=feature_row, feabits=feabits
                )
            except CommandError as exc:
                raise FpgaUpdateError(f"Feature Row write failed: {exc}") from exc

            # ------------------------------------------------------------ #
            # Step 8 – Verify Feature Row
            # ------------------------------------------------------------ #
            if self._verify:
                logger.info("Step 8: Verifying Feature Row …")
                print("  [8/10] Verifying Feature Row …", flush=True)
                try:
                    fr_read, fb_read = api.fpga_prog_featrow_read(fpga_chan=target_fpga)
                except CommandError as exc:
                    raise FpgaUpdateError(
                        f"Feature Row read-back failed: {exc}"
                    ) from exc

                if fr_read != bytes(feature_row):
                    raise FpgaUpdateError(
                        f"Feature Row verify mismatch: "
                        f"expected={bytes(feature_row).hex()} got={fr_read.hex()}"
                    )
                if fb_read != bytes(feabits):
                    raise FpgaUpdateError(
                        f"FEABITS verify mismatch: "
                        f"expected={bytes(feabits).hex()} got={fb_read.hex()}"
                    )
                logger.info("Feature Row verify passed.")

            # ------------------------------------------------------------ #
            # Step 9 – Set DONE bit
            # ------------------------------------------------------------ #
            logger.info("Step 9: Setting DONE bit …")
            print("  [9/10] Setting DONE bit …", flush=True)
            try:
                api.fpga_prog_set_done(fpga_chan=target_fpga)
            except CommandError as exc:
                raise FpgaUpdateError(f"Set DONE failed: {exc}") from exc

            # ------------------------------------------------------------ #
            # Step 10 – Refresh (boot from flash)
            # ------------------------------------------------------------ #
            logger.info("Step 10: Refreshing FPGA (loading config from flash) …")
            print(
                f"  [10/10] Refresh FPGA (timeout={self._refresh_timeout:.0f} s) …",
                flush=True,
            )

            try:
                api.fpga_prog_refresh(fpga_chan=target_fpga)
            except CommandError as exc:
                raise FpgaUpdateError(f"Refresh failed: {exc}") from exc

            logger.info(
                "Page-by-page programming complete. CFG=%d pages, UFM=%d pages.",
                cfg_pages,
                ufm_pages,
            )
            print("  Programming complete.", flush=True)

        except FpgaUpdateError:
            # Attempt clean-up: close config interface so the device is not
            # left stranded in config mode.
            logger.warning("Programming failed – attempting to close config interface.")
            try:
                api.fpga_prog_close()
            except Exception:
                pass
            raise
