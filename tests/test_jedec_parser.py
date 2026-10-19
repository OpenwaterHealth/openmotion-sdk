"""Unit tests for jedecParser.parse_jedec_file.

The parser printed an unconditional "[DEBUG] ..." line to stdout on every
call — including every production FPGA-programming run via
FPGAProgrammer.program_from_jedec. Diagnostics must go through logging.
"""
from omotion.jedecParser import parse_jedec_file

MINIMAL_JED = "QF128*\nL0000 10101010*\n"


def _write_jed(path):
    path.write_text(MINIMAL_JED, encoding="latin1")
    return str(path)


def test_parse_minimal_jed(tmp_path):
    image, extra = parse_jedec_file(_write_jed(tmp_path / "m.jed"))
    assert image.total_fuses == 128
    assert image.rows == 1
    assert image.data[0] == 0b10101010
    assert extra == {}


def test_parse_does_not_print_to_stdout(tmp_path, capsys):
    parse_jedec_file(_write_jed(tmp_path / "m.jed"))
    assert capsys.readouterr().out == ""
