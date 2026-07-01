import struct
import crcmod
import csv


class I2C_Packet:
    def __init__(self, id=0, cmd=0, device_address=0, register_address=0, data=0):
        self.id = id
        self.cmd = cmd  # read/write
        self.device_address = device_address
        self.register_address = register_address
        self.data = data

    def calculate_crc(self):
        crc16 = crcmod.predefined.Crc("crc-ccitt-false")
        buffer = struct.pack(
            "<HBHB", self.id, self.device_address, self.register_address, self.data
        )
        crc16.update(buffer)
        return crc16.crcValue

    def to_buffer(self):
        return struct.pack(
            "<HBHBH",
            self.id,
            self.device_address,
            self.register_address,
            self.data,
            self.calculate_crc(),
        )

    def from_buffer(self, buffer):
        packetCrc = 0
        self.id, self.device_address, self.register_address, self.data, packetCrc = (
            struct.unpack("<HBHBH", buffer)
        )
        if packetCrc != self.calculate_crc():
            raise ValueError("CRC validation failed.")
        return self

    def print_packet(self):
        print("Status Packet:")
        print("  ID:", self.id)
        print("  device_address:", hex(self.device_address))
        print("  register_address:", hex(self.register_address))
        print("  data:", hex(self.data))
        print("  CRC:", hex(self.calculate_crc()))

    @staticmethod
    def main():
        # Create an instance of the packet
        packet = I2C_Packet()

        # Set packet data
        packet.id = 1
        packet.cmd = 2
        packet.device_address = 3
        packet.register_address = 0
        packet.data = 0x03

        # Print packet details
        print("Original Packet:")
        packet.print_packet()

        # Convert packet to buffer
        buffer = packet.to_buffer()

        # Create a new packet instance and parse from buffer
        new_packet = I2C_Packet().from_buffer(buffer)

        # Print parsed packet details
        print("\nParsed Packet:")
        new_packet.print_packet()

    @staticmethod
    def read_csv_to_i2c_packets(csv_file_path):
        i2c_packets = []

        with open(csv_file_path, mode="r") as file:
            csv_reader = csv.DictReader(file)
            for row in csv_reader:
                packet = I2C_Packet()
                packet.id = int(row["id"])
                packet.cmd = 1
                packet.device_address = int(
                    row["device_address"], 16
                )  # Assuming hex format
                packet.register_address = int(
                    row["register_address"], 16
                )  # Assuming hex format
                packet.data = int(row["data"], 16)  # Assuming hex format
                i2c_packets.append(packet)

        return i2c_packets


if __name__ == "__main__":
    I2C_Packet.main()

    # Example usage
    csv_file_path = "camera_config_partial.csv"
    i2c_packets = I2C_Packet.read_csv_to_i2c_packets(csv_file_path)

    for packet in i2c_packets:
        packet.print_packet()
