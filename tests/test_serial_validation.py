import pytest
from omotion.MotionConsole import is_valid_console_serial

pytestmark = pytest.mark.unit

@pytest.mark.parametrize("s", ["QWW04Q10003", "A", "0", "A" * 24, "ABC123XYZ"])
def test_valid_serials(s):
    assert is_valid_console_serial(s) is True

@pytest.mark.parametrize("s", ["", "a" * 5, "AB-12", "ABC 12", "A" * 25, "ÀBC", "ABC\n", "ABC\t"])
def test_invalid_serials(s):
    assert is_valid_console_serial(s) is False
