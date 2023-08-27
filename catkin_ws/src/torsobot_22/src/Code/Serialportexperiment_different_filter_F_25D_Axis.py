#! /usr/bin/python2
# -*- coding: utf-8 -*-
import rospy
import serial
#import time
#import datetime
import math
#import numpy as np

from std_msgs.msg import Int32, Float32
rospy.init_node('imu_data', anonymous = False)
IMU = rospy.Publisher('/IMU_angle', Float32, queue_size = 0)
rate = rospy.Rate(40)

def conv_rad_to_deg_set(x,dec_place): # convert radian to degree
    return round(float(x)*180/3.14159,dec_place)

def init_IMU_port(): # find IMU port address and initialize serial connection if address is found
    port = None
    for count in range(0,6):
        address = '/dev/ttyACM' + str(count)
        for j in range(0,10):
            try:
                ser = serial.Serial(
                    port=address,
                    baudrate = 115200,
                    parity = 'N',
                    stopbits = 1,
                    bytesize = 8,
                    timeout = 0.1)

                ser.write('\x3a\x38\x31\x5c\x6e') #get streaming slot \x3a\x38\x31\x5c\x6e    :81\n
                #ser.write(bytearray(':81\n','UTF-8'))
                rospy.sleep(0.1)
                IMU_read = ser.readline()
                #print(IMU_read)
                message = str(IMU_read).split(',')

                # check whether the IMU is found by checking the reading
                if len(message) == 8 and message[4].isdigit() and message[5].isdigit():
                    port = ser
                    print('IMU Port Found at ' + address)
                    return port
            except:
            
                pass
    print('IMU Port Not Found') 
    return None

def IMU_execute(IMU_ser):
    IMU_ser.write(bytearray(':116,101000\n','UTF-8'))
    rospy.sleep(0.1)
    Flag = True
    while(Flag == True):
        rate.sleep()
        IMU_ser.write(bytearray(':10\n','UTF-8')) 
        IMU_read = str(IMU_ser.readline())
        IMU_ser.flushInput() #flush buffer
        IMU_read = IMU_read.rstrip()
        Angle_data = IMU_read.split(",")
        
        angle_y = float(Angle_data[-2])
        angle_z = float(Angle_data[-1])

        angle = -math.atan(angle_y/angle_z)
        dat_deg = conv_rad_to_deg_set(angle,4)
        print('Angle: '+ str(dat_deg))
        IMU.publish(dat_deg)
        #print(dat_deg)

        if (dat_deg > 80 or dat_deg < -80):
            Flag = False
   
if __name__ == "__main__": 
    try: 
        IMU_func = init_IMU_port()
        #First_read = IMU_set(IMU_func)
        IMU_execute(IMU_func)
    except rospy.ROSInterruptException:
        pass