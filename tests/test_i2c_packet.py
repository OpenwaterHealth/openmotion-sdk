"""Unit tests for I2C_Packet.read_csv_to_i2c_packets.

The method takes only the CSV path (no self) but had no @staticmethod
decorator, so it only worked when called unbound through the class —
calling it on an instance mis-bound the path argument to ``self``.
"""
from omotion.i2c_packet import I2C_Packet


def _write_csv(path):
    path.write_text(
        "id,device_address,register_address,data\n"
        "1,0x29,0x3012,0x01\n"
        "2,0x29,0x3013,0xFF\n"
    )
    return str(path)


def test_read_csv_callable_on_instance(tmp_path):
    csv_path = _write_csv(tmp_path / "cfg.csv")
    packets = I2C_Packet().read_csv_to_i2c_packets(csv_path)
    assert len(packets) == 2
    assert packets[0].device_address == 0x29
    assert packets[1].data == 0xFF


def test_read_csv_callable_on_class(tmp_path):
    csv_path = _write_csv(tmp_path / "cfg.csv")
    packets = I2C_Packet.read_csv_to_i2c_packets(csv_path)
    assert len(packets) == 2
