Vendored libusb (Windows)

windows/x64/libusb-1.0.dll and windows/x86/libusb-1.0.dll are the DLLs pyusb
loads (omotion/usb_backend.py). They are the VS2022/MS64 and VS2022/MS32 DLLs
from the official libusb 1.0.30 Windows binary release:

  https://github.com/libusb/libusb/releases/tag/v1.0.30
  libusb-1.0.30.7z
  sha256 7fb1dfec805b97983763d7d0ae244320da12add1003d4249c96cc4d586398c79
  signed by the libusb release manager key
  C681 8737 9B23 DE9E FC46  651E 2C80 FF56 C683 0A0E (libusb's KEYS file)

Firmware update (omotion/dfu) goes through pyusb and loads these same DLLs;
they are the only libusb copies in the package.

libusb 1.0.30 is the minimum: it fixes denial-of-service bugs that a malicious
USB device can trigger with malformed descriptors. tests/test_vendored_libusb.py
fails if any libusb-1.0.dll in the package is older.

To update: download the release .7z and its .asc, verify both the signature and
the GitHub digest, then copy those two DLLs over the current ones.
