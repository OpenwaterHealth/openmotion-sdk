# Hardware Runner Setup

This document describes how to configure a self-hosted GitHub Actions runner
with a console module and sensor modules permanently attached.

## Prerequisites

- Ubuntu 22.04 LTS (recommended) or Windows 11
- Python 3.12
- libusb 1.0 (`sudo apt install libusb-1.0-0`)
- A console module attached via USB VCP
- A left sensor on USB port 2, right sensor on USB port 3 (optional)

## Linux udev rules

Create `/etc/udev/rules.d/99-openmotion.rules`:

```
# Console module (USB VCP)
SUBSYSTEM=="tty", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", MODE="0666", GROUP="plugdev"

# Sensor modules (libusb bulk)
SUBSYSTEM=="usb", ATTRS{idVendor}=="<VID>", ATTRS{idProduct}=="<PID>", MODE="0666", GROUP="plugdev"
```

Replace `<VID>` and `<PID>` with the actual vendor/product IDs. Then reload:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Add the runner user to the `plugdev` group:

```bash
sudo usermod -aG plugdev $USER
```

## Runner registration

```bash
mkdir actions-runner && cd actions-runner
curl -o actions-runner-linux-x64.tar.gz -L \
  https://github.com/actions/runner/releases/download/v2.317.0/actions-runner-linux-x64-2.317.0.tar.gz
tar xzf actions-runner-linux-x64.tar.gz

./config.sh \
  --url https://github.com/<org>/<repo> \
  --token <RUNNER_TOKEN> \
  --labels "self-hosted,hardware,openmotion" \
  --name "openmotion-hw-runner-01"

sudo ./svc.sh install
sudo ./svc.sh start
```

## Environment variables

Set these in the runner's environment (or via a `.env` file sourced by the
service unit):

| Variable | Purpose | Default |
|---|---|---|
| `OPENMOTION_DEMO` | Set to `1` to run in demo mode (no hardware required) | `0` |
| `OPENMOTION_SERIAL_PORT` | Override the console serial port (e.g. `/dev/ttyACM0`) | auto-detect |

## Running tests locally

```bash
# Fast tests only (< 10 s each, non-destructive)
pytest tests/ -m "not slow and not destructive"

# All tests including slow ones
pytest tests/ -m "not destructive"

# Full suite (use with caution — modifies flash)
pytest tests/

# Offline dry-run (no hardware)
OPENMOTION_DEMO=1 pytest tests/
```

## Windows notes

- Install the WinUSB driver for the sensor modules using Zadig
- The console module enumerates as a COM port; ensure it is not claimed by
  another application before running tests
- Run the runner service as a user account with access to the COM port and
  USB devices, not as SYSTEM
