
import serial

import time



ser = serial.Serial('/dev/ttyACM0', 115200)

Angle = 90
Dist = 24

ser.write('C'.encode())
ser.write(int(Angle).to_bytes(4, byteorder='little', signed=True) + int(0).to_bytes(4, signed=True, byteorder='little'))
print("Sent 1", int(Angle*1.7).to_bytes(4, byteorder='little', signed=True) + int(0).to_bytes(4, signed=True, byteorder='little'))
time.sleep(2)
ser.write('C'.encode())
ser.write(int(-0*1.7).to_bytes(4, byteorder='little', signed=True) + int(Dist).to_bytes(4, signed=True, byteorder='little'))
print("sent 2")

