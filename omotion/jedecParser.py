"""
jedec_parser.py

Simple JEDEC parser for Lattice Crosslink devices.

- Reads JEDEC ASCII file
- Extracts QF (total fuse count)
- Parses Lxxxx lines to build fuse bit array
- Packs bits into rows of 16 bytes (128 bits) for NVCM

Fuse mapping:
- Fuse 0 is first bit of the concatenated L fields.
- Row 0, byte 0 = fuses 0..7 (bit0 = fuse0, bit1 = fuse1, ...).
"""

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

from omotion import _log_root

logger = logging.getLogger(f"{_log_root}.jedecParser" if _log_root else "jedecParser")


class JedecError(Exception):
    pass


@dataclass
class JedecImage:
    total_fuses: int
    rows: int
    row_size_bytes: int
    data: bytes  # rows * 16 bytes


def _is_comment_line(line: str) -> bool:
    line = line.lstrip()
    return not line or line.startswith("*")


def _parse_qf_line(line: str) -> int:
    """
    Parse 'QF1532160*' -> 1532160
    """
    line = line.strip()
    if not line.startswith("QF"):
        raise JedecError("Invalid QF line")
    line = line[2:]
    # read until '*'
    num_str = ""
    for ch in line:
        if ch == "*":
            break
        if not ch.isdigit():
            raise JedecError("Invalid character in QF line")
        num_str += ch
    if not num_str:
        raise JedecError("Empty QF value")
    return int(num_str, 10)


def _parse_L_line(line: str, fuse_bits: List[int]) -> None:
    """
    Parse 'L0000 010101*' into fuse_bits, modifying in place.
    fuse_bits is a list of 0/1 ints, length >= total_fuses.
    """
    line = line.strip()
    if not line.startswith("L"):
        raise JedecError("Invalid L line")
    line = line[1:]

    # parse start index
    idx_str = ""
    i = 0
    while i < len(line) and line[i].isdigit():
        idx_str += line[i]
        i += 1
    if not idx_str:
        raise JedecError("Missing start index in L line")
    start_index = int(idx_str, 10)

    # skip whitespace
    while i < len(line) and line[i].isspace():
        i += 1

    pos = start_index
    while i < len(line) and line[i] != "*":
        ch = line[i]
        if ch in ("0", "1"):
            if pos >= len(fuse_bits):
                raise JedecError("L line exceeds QF fuse count")
            fuse_bits[pos] = 1 if ch == "1" else 0
            pos += 1
        # else ignore whitespace or unexpected chars
        i += 1


def _pack_bits_to_rows(fuse_bits: List[int], total_fuses: int) -> bytes:
    """
    Pack fuse_bits into rows × 16 bytes (128 bits per row).
    Bits are MSB-first in each byte (fuse N maps to bit 7-(N%8) of its byte).
    """
    row_bits = 128
    rows = (total_fuses + row_bits - 1) // row_bits
    out = bytearray(rows * 16)

    for row in range(rows):
        base_bit = row * row_bits
        for byte_idx in range(16):
            val = 0
            bit_base = base_bit + byte_idx * 8
            for b in range(8):
                fuse_idx = bit_base + b
                if fuse_idx < total_fuses:
                    # MSB-first: bit 7 is first, bit 0 is last
                    val = (val << 1) | (1 if fuse_bits[fuse_idx] else 0)
                else:
                    val = val << 1
            out[row * 16 + byte_idx] = val
    return bytes(out)


def parse_jedec_file(path: str) -> Tuple[JedecImage, dict]:
    """
    Parse a JEDEC file into a JedecImage for Crosslink NVCM.

    Diamond JEDEC files use '*' as a field terminator, not a line terminator.
    Multiple fields can appear on a single line (e.g. 'QP144*QF1441280*G0*F0*'),
    so the file is split on '*' into individual fields rather than parsed
    line-by-line.

    Returns:
        (JedecImage, extra_dict)
        extra_dict may contain 'feature_row' and 'feabits' as 64/16-char
        bit-strings respectively.
    """
    with open(path, "r", encoding="latin1") as f:
        content = f.read()

    # Strip JEDEC STX (0x02) byte that Diamond prepends to some files.
    content = content.lstrip("\x02")

    # Split the entire file into '*'-terminated fields.  Using split('*')
    # handles any combination of single-field lines, multi-field lines, and
    # fields that span several lines (e.g. large L fuse-data blocks).
    raw_fields = content.split("*")

    total_fuses: Optional[int] = None
    fuse_bits: Optional[List[int]] = None
    feature_row: Optional[str] = None
    feabits: Optional[str] = None

    i = 0
    while i < len(raw_fields):
        field = raw_fields[i].strip()

        # Skip empty fields and NOTE / comment fields
        if not field or field.upper().startswith("NOTE"):
            i += 1
            continue

        # ---- QF: total fuse count ----------------------------------------
        if field.startswith("QF"):
            num_str = field[2:].strip()
            if num_str.isdigit():
                total_fuses = int(num_str)
                fuse_bits = [0] * total_fuses

        # ---- L: fuse data block ------------------------------------------
        elif field.startswith("L"):
            if fuse_bits is None:
                i += 1
                continue
            rest = field[1:]
            # Parse the start index (decimal digits immediately after 'L')
            j = 0
            while j < len(rest) and rest[j].isdigit():
                j += 1
            if j == 0:
                i += 1
                continue
            fuse_pos = int(rest[:j])
            # All '0'/'1' characters that follow (across embedded newlines)
            # are fuse data for this block.
            for c in rest[j:]:
                if c in ("0", "1"):
                    if fuse_pos < len(fuse_bits):
                        fuse_bits[fuse_pos] = int(c)
                        fuse_pos += 1

        # ---- E: feature row (64 bits) + FEABITS (16 bits on next line) ---
        elif field.startswith("E"):
            # The E field contains the 64-bit feature row on the first line
            # and the 16-bit FEABITS on the second line, both within the
            # same '*'-terminated field.
            field_lines = [ln.strip() for ln in field.splitlines() if ln.strip()]
            if field_lines:
                e_bits = field_lines[0][1:]  # strip leading 'E'
                if len(e_bits) >= 64 and all(c in "01" for c in e_bits[:64]):
                    feature_row = e_bits[:64]
            if len(field_lines) >= 2:
                fb = field_lines[1]
                if all(c in "01" for c in fb):
                    feabits = fb[:16]

        i += 1

    if total_fuses is None or total_fuses <= 0:
        raise JedecError("QF field not found or invalid")
    if fuse_bits is None:
        fuse_bits = [0] * total_fuses

    logger.debug(
        "QF=%d set_fuses=%d feature_row=%s feabits=%s",
        total_fuses,
        sum(fuse_bits),
        "present" if feature_row else "absent",
        f"present ({feabits})" if feabits else "absent",
    )

    # Pack into rows × 16 bytes
    data = _pack_bits_to_rows(fuse_bits, total_fuses)
    rows = len(data) // 16

    extra: dict = {}
    if feature_row:
        extra["feature_row"] = feature_row
    if feabits:
        extra["feabits"] = feabits

    return (
        JedecImage(
            total_fuses=total_fuses,
            rows=rows,
            row_size_bytes=16,
            data=data,
        ),
        extra,
    )


# Example CLI usage:
if __name__ == "__main__":
    import sys
    import os

    def write_c_header(
        img: JedecImage,
        out_path: str,
        array_name: str = "jedec_data",
        feature_row: str = None,
        feabits: str = None,
    ):
        with open(out_path, "w") as f:
            f.write("// Auto-generated JEDEC data header\n")
            f.write("#pragma once\n\n")
            f.write(f"#define JEDEC_ROWS {img.rows}\n")
            f.write(f"#define JEDEC_ROW_SIZE {img.row_size_bytes}\n")
            f.write(f"#define JEDEC_TOTAL_FUSES {img.total_fuses}\n\n")
            f.write(f"const unsigned char {array_name}[{len(img.data)}] = {{\n")
            for i, b in enumerate(img.data):
                if i % 16 == 0:
                    f.write("    ")
                f.write(f"0x{b:02X}, ")
                if (i + 1) % 16 == 0:
                    f.write(f"// Row {i // 16}\n")
            if len(img.data) % 16 != 0:
                f.write("\n")
            f.write("};\n\n")
            # Feature row output (if present)
            if feature_row and feabits:

                def bitstring_to_bytes(bitstr):
                    # Convert string of bits (MSB first, right-to-left per C code) to bytes
                    out = []
                    p = len(bitstr) - 1
                    for _ in range(len(bitstr) // 8):
                        val = 0
                        for _ in range(8):
                            val = (val << 1) | (1 if bitstr[p] == "1" else 0)
                            p -= 1
                        out.append(val)
                    return out

                feature_bytes = bitstring_to_bytes(feature_row)
                feabits_bytes = bitstring_to_bytes(feabits.zfill(16))
                f.write(f"const unsigned char {array_name}_feature_row[8] = {{ ")
                f.write(", ".join(f"0x{b:02X}" for b in feature_bytes))
                f.write(" };")
                f.write("\n")
                f.write(f"const unsigned char {array_name}_feabits[2] = {{ ")
                f.write(", ".join(f"0x{b:02X}" for b in feabits_bytes))
                f.write(" };")
                f.write("\n\n")

    def print_usage():
        print("Usage: python jedec_parser.py <file.jed> [--header <output.h>]")

    argc = len(sys.argv)
    if argc not in (2, 4):
        print_usage()
        sys.exit(1)

    jedec_path = sys.argv[1]
    img, extra = parse_jedec_file(jedec_path)

    if argc == 4 and sys.argv[2] == "--header":
        header_path = sys.argv[3]
        array_name = os.path.splitext(os.path.basename(header_path))[0]
        # Ensure valid C identifier
        array_name = array_name.replace(".", "_").replace("-", "_")
        write_c_header(
            img, header_path, array_name, extra.get("feature_row"), extra.get("feabits")
        )
        print(f"Header file written to {header_path}")
    else:
        print(f"Total fuses: {img.total_fuses}")
        print(f"Rows:        {img.rows}")
        print(f"Bytes:       {len(img.data)}")
