import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200)

while True:
    time.sleep(1.5)

    ARUCO_ANGLE = 0
    ARUCO_DIST = 90
    print("Moving")
    ser.flush()
    ser.write('C'.encode())
    ser.write(int(-ARUCO_ANGLE*10).to_bytes(4, byteorder='little', signed=True) + int(ARUCO_DIST*10).to_bytes(4, signed=True, byteorder='little'))

