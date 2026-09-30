"""Check that a bare-metal sensor-fw flash image carries the expected camera
FPGA bitstream, before flashing it.

usage: check_fw_bitstream.py <build/Debug/motion-sensor-fw.bin> <bitstream.bit>

A bare-metal build merges fpga/openmotion-camera-fpga.bin into the image at
flash address 0x081A0000 (offset 0x1A0000 from the 0x08000000 image base). The
CMake configure step downloads the latest *release* bitstream over that file,
so an image built in the wrong order silently carries the release bitstream
instead of map v3 (captures then fail with "FPGA register map < v3").
Exit code 0 = match, 1 = mismatch.
"""
import hashlib
import sys
from pathlib import Path

OFFSET = 0x1A0000


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    image = Path(sys.argv[1]).read_bytes()
    bit = Path(sys.argv[2]).read_bytes()
    seg = image[OFFSET:OFFSET + len(bit)]
    want = hashlib.sha256(bit).hexdigest()
    got = hashlib.sha256(seg).hexdigest()
    print(f"image {sys.argv[1]}: {len(image)} bytes")
    print(f"expected bitstream {Path(sys.argv[2]).name}: {len(bit)} bytes, sha256 {want[:16]}...")
    print(f"image @0x{OFFSET:06X}:                           sha256 {got[:16]}...")
    if seg == bit:
        print("OK: the image carries this bitstream")
        return 0
    print("MISMATCH: re-copy the bitstream to fpga/openmotion-camera-fpga.bin AFTER cmake configure, "
          "then rebuild with --clean-first")
    return 1


if __name__ == "__main__":
    sys.exit(main())
