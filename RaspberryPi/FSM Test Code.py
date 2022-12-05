#FINAL COPY
#SEED Lab 2pm
#Group 7
#Mini project 1

import serial
import time
import cv2
import numpy as np
import sys
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv2 import aruco
import math
import struct
import os
os.environ['DISPLAY'] = ':0'
#from scipy.interpolate import interp1d

# COMM - This is the address we setup in the Arduino Program
ser = serial.Serial('/dev/ttyACM0', 115200)

ARUCO_MARKER = 0
ARUCO_ANGLE = 0
ARUCO_ID = 0
ARUCO_DIST = 0

UPDATE_RATE = 1.5
SCANNER_TIMEOUT = 8
FUDGE_FACTOR = 0
DISTANCE_AWAY = 0
LOST_MARKER_TIMEOUT = 5

lutx = [21, 47, 69, 88, 107, 124, 138, 152, 164, 175, 185, 194, 203, 211, 218, 225, 231, 237, 243, 248, 254, 259, 263, 268, 272, 276, 280, 284, 287, 290, 293, 296, 299, 301, 304, 307, 310, 311, 314, 316, 318, 320, 322, 324, 326, 328, 330, 331, 333, 334, 336, 337, 339, 340, 342]
luty = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29,30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55]

MARKERS_USED = [0, 1, 2, 3, 4, 5]

# COMM - Wait for connection to complete
time.sleep(3)

# COMM - LCD Stuff
#def LCD_stuff(SENT, RXED):
    # Modify this if you have a different sized Character LCD
    #lcd_columns = 16
    #lcd_rows = 2

    # Initialise I2C bus.
    #i2c = board.I2C()  # uses board.SCL and board.SDA

    # Initialise the LCD class
    #lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
        
    #lcd.clear()
    
    # Set LCD color to red
    #lcd.color = [0, 100, 0]
    
    # Print two line message
    #lcd.message = "SENT: " + str(SENT) + "\nRXED: " + str(RXED)
    #return None

# COMM - Function to read serial
def ReadfromArduino():
    while(ser.in_waiting > 0):
        try:
            line = ser.readline().decode('utf-8').rstrip()
            print("Serial output: ", line)
            
        except:
            print("Communication Error")
        return line

# COMP VIS - set up aruco dictionary
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# COMP VIS - set up camera
arucoDictionary = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_100"]) #"DICT_5X5_100"
arucoParameter = cv2.aruco.DetectorParameters_create()

print("enter\'q\' to exit video capture") 

camera = cv2.VideoCapture(0)

# COMP VIS - Begin ArUCo detection

lastMarker = 0
lastTime = time.time()


index = 0
distancetoMove = [0, 12]
angletoTurn = [90, 0]
lastMarkerSeen = time.time()

currentNum = 1
incrementList = 1

hasMoved = [False, False, False, False, False, False, False, False]
moving = [False, False, False, False, False, False, False, False]

#scanning state
def SCAN_STATE(lastMarkerMatch, timeVsUpdateRate, scannerTimeout, idsFound, matchMarker):
    global lastTime
    global incrementList
    if timeVsUpdateRate:
        lastTime = time.time()
        ser.write('C'.encode())
        ser.write(int(-40*10).to_bytes(4, byteorder='little', signed=True) + int(0).to_bytes(4, signed=True, byteorder='little'))
        ser.flush()
        print("Scanning for marker:", incrementList)
    if  (idsFound == True) and (matchMarker == True):
#        print("starting read to Arduino")
        return WRITE_TO_ARDUINO_STATE
    else:
        return SCAN_STATE
    
def WRITE_TO_ARDUINO_STATE(lastMarkerMatch, timeVsUpdateRate, scannerTimeout, idsFound, matchMarker):
    global lastMarkerSeen
    global lastTime
    global ARUCO_ANGLE
    global ARUCO_DIST
    global incrementList
    if time.time() - lastTime >= UPDATE_RATE:
        lastTime = time.time()

        if ((idsFound == False) or (matchMarker == False)):
            if incrementList > 6:
                return SPIN_STATE
            if hasMoved[incrementList]:
                return INC_MARKER_LIST_STATE
            if time.time() - lastMarkerSeen > LOST_MARKER_TIMEOUT:
                return SCAN_STATE
            return WRITE_TO_ARDUINO_STATE
        lastMarkerSeen = time.time()
        ARUCO_ANGLE += 2

        angleTolerance = 2
        if moving[incrementList]:
            angleTolerance = 3

        if abs(ARUCO_ANGLE) > angleTolerance:
            if not hasMoved[incrementList]:
                ARUCO_DIST = 0
        else:
            hasMoved[incrementList] = True
            moving[incrementList] = True
            #else:
            #    ARUCO_ANGLE = 0
        print ("\nRPI: Hi Arduino, I sent you ", -ARUCO_ANGLE," and ", ARUCO_DIST, " moving towards marker ", incrementList)
        ser.write('C'.encode())
        ser.write(int(-ARUCO_ANGLE*10).to_bytes(4, byteorder='little', signed=True) + int(ARUCO_DIST*10).to_bytes(4, signed=True, byteorder='little'))

            # COMM - Wait for Arduino to set up response
            #time.sleep(2)
            
            #print(ser.readline())

        ser.flush()

            # COMM - Print to LCD
            #LCD_stuff(ARUCO_ANGLE, ARUCO_ID)
        if ((idsFound == False) or (matchMarker == False)):
            if incrementList > 6:
                return SPIN_STATE
            if hasMoved[incrementList]:
                return INC_MARKER_LIST_STATE
            if time.time() - lastMarkerSeen > LOST_MARKER_TIMEOUT:
                return SCAN_STATE
    return WRITE_TO_ARDUINO_STATE

def INC_MARKER_LIST_STATE(lastMarkerMatch, timeVsUpdateRate, scannerTimeout, idsFound, matchMarker):
    global incrementList

    if not scannerTimeout:
        print("waiting")
        return INC_MARKER_LIST_STATE
    scannerTimeout = time.time()

    if (incrementList < 6):
        incrementList += 1
    else:
        incrementList = 0
    print(incrementList)
    if incrementList > 6:
        return SPIN_STATE
    return SCAN_STATE

def SPIN_STATE(lastMarkerMatch, timeVsUpdateRate, scannerTimeout, idsFound, matchMarker):
    global lastTime

    if time.time() - lastTime > UPDATE_RATE/2:
        lastTime = time.time()
        ser.write('C'.encode())
        ser.write(int(180*10).to_bytes(4, byteorder='little', signed=True) + int(0).to_bytes(4, signed=True, byteorder='little'))
        ser.flush()
        print("SPINNING")
    return SPIN_STATE


state = SCAN_STATE

while(True):
    ret, frame = camera.read()
    
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    cv2.imshow('frame',grey)
    
    (coordinates, ids, rejectedIds) = cv2.aruco.detectMarkers(frame, arucoDictionary, parameters=arucoParameter)


    # COMP VIS - verify marker existence and print ids
    if len(coordinates) > 0:
        ids = ids.flatten()
        #ids.sort()
        ARUCO_ID = -1
        i = 0

        #convert marker coordinates to integers
        for markerCoord, markerID in zip(coordinates, ids):
#            print(ids)
            #for i in range(len(ids)):
#            i = np.where(ids == currentNum)
#            print(i)
#            i = i[0]
#            if i < 0:
#                continue
            #print("ID: ", markerID)
            if markerID != incrementList:
                continue
            ARUCO_ID = markerID
            
#            currentNum += 1
            '''currentMarker = ids[i]
#            print(currentMarker)
            if currentMarker == ids[i]:
                if currentMarker == incrementList + 1:
                    ARUCO_ID = currentMarker
                else:
                    i += 1
                    continue'''
            (topLeft, topRight, bottomRight, bottomLeft) = markerCoord.reshape((4,2))
    
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))

            #calculate the position of the marker
            #use marker center location in compas quadrant
            #(height, width) = frame.shape

            #determine quadrant location
            #aruco center gives x then y coordinates of marker center
            arucoCenter = [(topLeft[0] + ((topRight[0] - topLeft[0])/2)), (topRight[1] + ((bottomRight[1] - topRight[1])/2))]
            
            height = 480
            width = 680
            
            #interpolate = interp1d(lutx, luty)
            #calculate the angle of the marker
            objEdgeDist = (int(width)/2) + ((topRight[0] - topLeft[0]) + topLeft[0])
            anglePose = (54/2) * (objEdgeDist/(int(width)/2))
            result = '{0:.3g}'.format(anglePose)
            
            #print calculate the angle, each if statement determines the sign of the angle
            if arucoCenter[0] < (int(width)/2):
                ARUCO_MARKER = 2
                objEdgeDist = ((int(width)/2) - arucoCenter[0])
                ARUCO_ANGLE = (54/2) * (objEdgeDist/(int(width)/2)) * -1
                #print(objEdgeDist)
                #ARUCO_ANGLE = float('{0:.3g}'.format(anglePose))
            elif arucoCenter[0] > (int(width)/2):
                ARUCO_MARKER = 1
                objEdgeDist = (-(int(width)/2) + arucoCenter[0])
                ARUCO_ANGLE = (54/2) * (objEdgeDist/(int(width)/2))
                #ARUCO_ANGLE = float('{0:.3g}'.format(anglePose))
                
            else:
                ARUCO_MARKER = 0
            
            markerHeight = height - bottomLeft[1]
            
            closestIndex = 53
            for j in range(len(luty)):
                if abs(lutx[j] - lutx[closestIndex]) > abs(lutx[j] - markerHeight):
                    closestIndex = j
                    
            ARUCO_DIST = 10 + closestIndex - DISTANCE_AWAY - FUDGE_FACTOR
            if ARUCO_DIST > 3*12:
                ARUCO_DIST -= 25
                hasMoved[incrementList] = False
            #i += 1
                #print("Height:", markerHeight)
                #print(ARUCO_DIST)
                    
                #print("Vertical distance to bottom edge: "+str(height - bottomLeft[1]))
#                else:
#                    i += 1
#                    if i == len(ids):
#                        continue
#                    else:
#                        currentMarker = ids[i]
    #for j in range
#    print(ARUCO_ID, " ", MARKERS_USED[j])
#    print(j, " ", ids)
    global lastMerkerSeen
    global scannerTimeout
    lastMarkerMatch = (lastMarker != ARUCO_ID)
    timeVsUpdateRate = (time.time() - lastTime >= UPDATE_RATE)
    scannerTimeout = ((time.time() - lastMarkerSeen) >= SCANNER_TIMEOUT)
    idsFound = (len(coordinates) != 0)
    matchMarker = (ARUCO_ID == incrementList)


    
    #start fsm
#    print("launching fsm")
    new_state = state(lastMarkerMatch, timeVsUpdateRate, scannerTimeout, idsFound, matchMarker)
    state = new_state
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
camera.release()
cv2.destroyAllWindows()

