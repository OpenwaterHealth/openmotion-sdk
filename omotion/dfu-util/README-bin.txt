dfu-util 0.11 binaries

The Windows binaries were built using MinGW on Ubuntu 20.04
and the instructions on http://dfu-util.sourceforge.net/build.html

The macOS (darwin) binaries were built on macOS 10.13.6

The dfu-util source was from the 0.11 release tarball.

The libusb source the tools were built against was from git 2021-09-05
(pre-1.0.25), commit v1.0.24-66-g1a90627.

The lsusb utility was built from usbutils v014
with the patch lsusb_build_on_mingw.patch applied.

libusb at runtime
-----------------
Windows: dfu-util.exe loads libusb-1.0.dll from its own directory. The
win64/win32 DLLs here are the MinGW64/MinGW32 builds from the official
libusb 1.0.30 release, not the 1.0.24-era DLLs built with the tools. Their
provenance is in omotion/_vendor/libusb/README.txt.

macOS: dfu-util links Homebrew's libusb by absolute path
(/usr/local/opt/libusb/lib/libusb-1.0.0.dylib), so the host needs
`brew install libusb`.

Linux: dfu-util uses the system libusb.

The statically linked dfu-util-static / lsusb-static builds, the macOS
libusb dylib and the libusb static and import libraries were removed. Nothing
ran or loaded them, and each one contained 1.0.24-era libusb.
