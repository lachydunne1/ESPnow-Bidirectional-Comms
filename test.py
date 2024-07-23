import struct
import unittest
from unittest.mock import MagicMock, patch
from send import SerialWrapper 

class TestSerialWrapper(unittest.TestCase):

    @patch('serial.Serial')
    def test_send_data(self, mock_serial):
        # Create an instance of the SerialWrapper with the mocked serial port
        ser_wrapper = SerialWrapper('/dev/ttyUSB1')
        ser_wrapper.ser.write = MagicMock()

        # Define test data
        command = 1  # Broadcast
        data1 = 10   # Steps
        data2 = 1    # Direction
        crc = ser_wrapper.calculate_crc([command, data1, data2])
        expected_data = struct.pack('BHBB', command, crc, data1, data2)

        # Call the send_data method
        ser_wrapper.send_data(command, data1, data2)

        # Assert that the write method was called with the correct data
        ser_wrapper.ser.write.assert_called_once_with(expected_data)

    @patch('serial.Serial')
    def test_calculate_crc(self, mock_serial):
        ser_wrapper = SerialWrapper('/dev/ttyUSB1')
        data = [1, 10, 1]
        expected_crc = sum(data) & 0xFFFF

        # Check the CRC calculation
        self.assertEqual(ser_wrapper.calculate_crc(data), expected_crc)

if __name__ == '__main__':
    unittest.main()
