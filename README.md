# Overview

This repository contains the Open-Motion SDK, a Python library for interfacing with Open-Motion blood flow imaging hardware. The SDK communicates with sensor modules and the console module to control cameras, FPGA, and data acquisition for near-infrared speckle contrast imaging.

The sensor modules run firmware on STM32H7 processors and communicate over USB. A Python library called `omotion` provides the core interface for hardware control and data streaming.

# Getting started
1. Install requirements.txt (`pip install -r requirements.txt`)
2. Install libusb for your system requires libusb to be installed, for windows install the dll to c:\windows\system32, download the correct dll from github [libusb Releases](https://github.com/libusb/libusb/releases)
3. Plug in your aggregator module. Please wait 10 seconds for it to boot up before continuing.
4. Run `python multicam_setup.py` - this will flash each camera sensor one by one. Alternatively, you may flash just a single camera sensor by usising `python flash_camera.py 1` - this will flash just camera 1
5. Run `python monitor.py 1` - this will flash the camera with a few parameters (test modes, exposure times, gain settings, etc), start the camera streaming, start the frame sync generating, and then put the cameras into streaming mode. It will then recieve the histogram data for the defined number of seconds then close down. Modify the parameters at the top of this file if you want to adjust the gain, exposure time, etc. Change the number in the command line arguments to change the camera you'd like to interrogate. Cameras are numbered 1-8 and correspond to J1-J8 on the aggregator board.

# from repo root rebuild and install
python -m pip install --upgrade build twine
python -m build          # creates wheel + sdist under dist/
python -m pip install --force-reinstall dist/openmotion_sdk-1.3.3-py3-none-any.whl

# quick runtime check (on Windows box with your device bound to WinUSB/libusbK)
python -c "import usb, omotion.usb_backend as ub; print(ub.get_libusb1_backend())"
