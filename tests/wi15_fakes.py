"""Shared fakes for the WI-00015 calibration test modules.

Imported with absolute imports (``from wi15_fakes import FakeDevice``) by the
``test_wi15_*`` files; the tests directory has no ``__init__.py`` so pytest
puts it on ``sys.path``. Evidence/dataclass builders live in
``wi15_builders``; the console-script harness lives in ``wi15_script_harness``.
"""


class FakeClock:
    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeDevice:
    def __init__(self, connected, serial, firmware, hardware_id):
        self.connected = connected
        self.serial = serial
        self.firmware = firmware
        self.hardware_id = hardware_id
        self.serial_error = None
        self.firmware_error = None

    def is_connected(self):
        return self.connected

    def read_serial_number(self):
        if self.serial_error:
            raise self.serial_error
        return self.serial

    def get_version(self):
        if self.firmware_error:
            raise self.firmware_error
        return self.firmware

    def get_hardware_id(self):
        return self.hardware_id


class FakeRecorder:
    """Trivial list-recording workflow recorder (events + checkpoints)."""

    def __init__(self):
        self.events = []
        self.checkpoints = []

    def record(self, event):
        self.events.append(event)

    def checkpoint(self, result):
        self.checkpoints.append(result)
