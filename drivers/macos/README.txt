OpenMotion USB Drivers — macOS
==============================

No driver installation is required on macOS.

macOS provides native support for the OpenMotion hardware through IOKit:

  Sensor Modules (VID 0x0483, PID 0x5A5A)
    Accessible via libusb without any additional drivers.
    The application uses pyusb + libusb to communicate directly with
    the sensor's USB interfaces.

    Prerequisite: Install libusb via Homebrew:
      brew install libusb

  Console Module (VID 0x0483, PID 0xA53E)
    Enumerates automatically as a serial device at:
      /dev/cu.usbmodemXXXX
    No driver needed — macOS includes a built-in CDC/ACM driver.

  DFU Bootloader (VID 0x0483, PID 0xDF11)
    Standard STM32 DFU mode. Firmware updates go through the SDK
    (omotion.FirmwareUpdater), which needs libusb:
      brew install libusb

Application Data
----------------
  When launched from the .app bundle (DMG installer), the Bloodflow
  application stores logs and scan data in:

    ~/Documents/OpenWater Bloodflow/
      app-logs/         Application log files
      scan_data/        Captured histogram data and processed CSVs
      run-logs/         Per-scan run logs

  When running from source, these are created in the current working
  directory.  Set "output_path" in config/app_config.json to override.

Gatekeeper Note
---------------
  Since the app is not notarized with Apple, macOS may block it on
  first launch.  To open it:

    1. Right-click the app and select "Open" (not double-click)
    2. Click "Open" in the confirmation dialog
    3. Subsequent launches will work normally

  Or: System Settings > Privacy & Security > Open Anyway

Troubleshooting
---------------
  If the sensor module is not detected:
    1. Check System Information > USB to verify the device appears
    2. Ensure libusb is installed:  brew list libusb
    3. Try unplugging and replugging the USB cable
    4. Check for macOS privacy prompts about USB accessory access

  If the console is not detected:
    1. List serial ports:  ls /dev/cu.usbmodem*
    2. Check System Information > USB for the device
    3. Try a different USB cable (some are charge-only)
