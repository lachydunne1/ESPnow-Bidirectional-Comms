import struct
import serial

BROADCAST_ADDR = b'\xFF\xFF\xFF\xFF\xFF\xFF'
DATA_FORMAT = 'BBB' #'B' = uint8_t, 'H = uint16_t


class SerialWrapper:

    def __init__(self, device):
        self.ser = serial.Serial(device, 115200)
    
    #def calculate_crc(self, data):
    #   return sum(data) & 0xFFFF

    def send_data(self, type: int, data1: int, data2: int):

        packed_data = struct.pack(DATA_FORMAT, data1, data2, type)
        self.ser.write(packed_data)  

if __name__ == "__main__":

    #open the serial port with ESP BAUD
    ser = SerialWrapper('/dev/ttyUSB1')

    while True:
        command = -1
        while command < 0 and (command != 0 or command != 1):
            command = int(input("Enter BOOLEAN for [1] Broadcast or [2] Unicast: "))
        
        data1 = 0

        while data1 <=0 or data1 > 100 :
            data1 = int(input("Enter percentage ROM to rotate through: "))

        data2 = int(input("Enter BOOLEAN for [1] Left, [2] Right: "))
        print(f"Sending [{command}, {data1}, {data2}] ")
        ser.send_data(command, data1, data2)
