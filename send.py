import struct
import serial

BROADCAST_ADDR = b'\xFF\xFF\xFF\xFF\xFF\xFF'
DATA_FORMAT = 'BHBB' #'B' = uint8_t, 'H = uint16_t


class SerialWrapper:

    def __init__(self, device):
        self.ser = serial.Serial(device, 115200)
    
    def calculate_crc(self, data):
        return sum(data) & 0xFFFF


    def send_data(self, type: int, data1: int, data2: int):
        
        crc = self.calculate_crc([type, data1, data2])
        packed_data = struct.pack(DATA_FORMAT, type, crc, data1, data2)
        self.ser.write(packed_data)  

if __name__ == "__main__":

    #open the serial port with ESP BAUD
    ser = SerialWrapper('/dev/ttyUSB1')

    while True:
        command = int(input("Enter BOOLEAN for [1] Broadcast or [2] Unicast: "))
        data1 = int(input("Enter STEP : "))
        data2 = int(input("Enter BOOLEAN for [1] Left, [2] Right: "))
        ser.send_data(command, data1, data2)
