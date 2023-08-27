#import RPi.GPIO as GPIO
import time
import datetime
import struct
import serial
import numpy as np
import random
import math


global g
g = 9.80297

def getEuler(ser):
    command = bytearray(':7\n','UTF-8') #1: tared 7: Untared
    #command = '>0,3\n'
    #command = ':64\n'
    ser.write(command)
    #time.sleep(0.0001)
    ser.flushInput()
    """
    result = ser.read(12)
    euler = struct.unpack('>fff', result) # Units rad/s
    return euler
    """
    result = ser.read(24)
    return result
    
    
def get2Vector(ser):
    ser.write(bytearray(':12\n','UTF-8'))
    ser.flushInput() #flush buffer
    result = ser.readline()
    return result



def getGyroCorrected(ser):
    command = bytearray([247,38,38%256])
    ser.write(command)
    time.sleep(0.0001)
    result = ser.read(12)
    gyro = struct.unpack('>fff', result) # Units rad/s
    return gyro

    #tNext += T
    #time.sleep(T-0.01) # what is the -0.01?
    #time.sleep(T-float(stim_per))
    #f.close()
    #GPIO.remove_event_detect(18)
    return

# Start of main program
# Create serial port for RehaStim
# ser = serial.Serial( port = '/dev/ttyUSB0', baudrate = 115200, parity = serial.PARITY_NONE, stopbits = serial.STOPBITS_ONE, bytesize = serial.EIGHTBITS)
# Create serial port for IMU
ser2=serial.Serial('/dev/ttyACM0', baudrate=115200) # imu 


 
 
#ser2.write(bytearray(':116,000\n','UTF-8'))
ser2.write(bytearray(':109,1\n','UTF-8'))
#ser2.write(bytearray(':85\n','UTF-8'))


ser2.write(bytearray(':123,1\n','UTF-8'))   #Setting necessary to initiate (command 123)
#MODE 0) IMU mode
#Mode 1) Kalman Filter mode 
#Mode 2) Q-COMP Filter mode
#Mode 3) Q-GRAD Filter mode

### <1> get Euler
while True:
   
    angle = getEuler(ser2)
    angle_str = str(angle).strip('b')
    #print(angle_str)
    angle_split = angle_str.split(',')
    #print(angle_split[2][:-3])

    try:
        pitch = float(angle_split[0][1:]) /math.pi*180 +11.2
        #print("pitch ",pitch
        yaw = float(angle_split[1]) /math.pi*180
        #print("yaw ",yaw)
        roll = float(angle_split[2][:-3]) /math.pi*180
        print(pitch,yaw,roll)
        #print("roll", roll)
    except:
        #print("fail")
        pass
    #angle = getGyroCorrected(ser2)
    #print(angle)
    
    print("======================")
    time.sleep(0.2)

    
"""

### <2> get Vector
while True:
    vectors = get2Vector(ser2)
    #vectors_str = str(vectors).rstrip()
    print(vectors)
    print("======================")
   
    
    Angle_data = IMU_read.split(",")
    angle_y = float(Angle_data[-2])
    angle_z = float(Angle_data[-1])
    
    #print(angle_y,angle_z)
    

    
    angle = -math.atan(angle_y/angle_z)
    dat_deg = conv_rad_to_deg_set(angle,4)
    print('Angle: '+ str(dat_deg))
    
    
"""
