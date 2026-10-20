import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from powder_sampling_control.device_adapters.md30_modbus_rtu import (
    MD30Config, MD30ModbusRTU, _append_crc as md30_crc,
)
from powder_sampling_control.device_adapters.zdt_emm42 import (
    Emm42Config, ZDTEmm42, _append_crc as emm_crc,
)
from powder_sampling_control.device_adapters.qls5800_signal_gen import (
    QLS5800Config, QLS5800SignalGenerator,
)


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.buffer = b""
        self.writes = []

    def reset_input_buffer(self):
        self.buffer = b""

    def write(self, data):
        self.writes.append(data)
        self.buffer += self.responses.pop(0)
        return len(data)

    def flush(self):
        pass

    def read(self, size=1):
        data, self.buffer = self.buffer[:size], self.buffer[size:]
        return data

    def close(self):
        pass


class NewDeviceAdapterTests(unittest.TestCase):
    def test_md30_move_to_uses_standard_modbus_write(self):
        ack = md30_crc(bytes.fromhex("01 06 00 02 01 F4"))
        serial = FakeSerial([ack])
        driver = MD30ModbusRTU(MD30Config(port="COM9"), serial_port=serial)
        driver.move_to(500)
        self.assertEqual(serial.writes[0], ack)

    def test_md30_rejects_unconfirmed_position_conversion(self):
        driver = MD30ModbusRTU(MD30Config(port="COM9"), serial_port=FakeSerial([]))
        with self.assertRaises(ValueError):
            driver.move_to_mm(10)

    def test_emm42_speed_frame_uses_modbus_function_10(self):
        ack = emm_crc(bytes.fromhex("01 10 00 F6 00 03"))
        serial = FakeSerial([ack])
        driver = ZDTEmm42(Emm42Config(port="COM9"), serial_port=serial)
        driver.set_speed(1500, direction=1, acceleration=10)
        self.assertEqual(serial.writes[0], emm_crc(bytes.fromhex(
            "01 10 00 F6 00 03 06 01 0A 05 DC 00 00")))

    def test_qls_write_is_uncrc_and_readback_is_required(self):
        # QLS writes have no acknowledgement frame; only the subsequent read
        # returns data for the adapter's mandatory readback verification.
        serial = FakeSerial([b"", bytes.fromhex("09 03 02 00 00 00 3C")])
        driver = QLS5800SignalGenerator(QLS5800Config(port="COM9"), serial_port=serial)
        driver.set_frequency(60)
        self.assertEqual(serial.writes[0], bytes.fromhex("09 06 02 00 00 00 3C"))
        self.assertEqual(serial.writes[1], bytes.fromhex("09 03 02"))


if __name__ == "__main__":
    unittest.main()
