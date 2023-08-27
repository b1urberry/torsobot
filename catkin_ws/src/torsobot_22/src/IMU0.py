#! /usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 14:03:57 2020

@author: Jun
"""

import rospy
import serial
import math
import traceback
import time

from std_msgs.msg import Int32, Float32, Bool
rospy.init_node('imu_data', anonymous = False)
IMU = rospy.Publisher('/IMU2_angle', Float32, queue_size = 0)
IMU_fail = rospy.Publisher('/IMU_fail', Bool, queue_size = 0)

rate = rospy.Rate(40)




def conv_rad_to_deg_set(x,dec_place): # convert radian to degree
    return round(float(x)*180/3.14159,dec_place)



def init_IMU_port(): # find IMU port address and initialize serial connection if address is found
    port = None
    #for count in range(0,6):
    for count in range(0,10):
        #address = '/dev/ttyACM' + str(count)
        address = '/dev/ttyACM' + str(count)
        #for j in range(0,10):
        for j in range(0,10):
            try:
                ser = serial.Serial(
                    port=address,
                    baudrate = 115200,
                    parity = 'N',
                    stopbits = 1,
                    bytesize = 8,
                    timeout = 0.1)

                #ser.write('\x3a\x38\x31\x5c\x6e') #get streaming slot \x3a\x38\x31\x5c\x6e    :81\n
                ser.write(bytearray(':81\n','UTF-8'))
                rospy.sleep(0.1)
                IMU_read = ser.readline()
                #print("IMU_read",IMU_read)
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
    #IMU_ser.write(bytearray(':116,101000\n','UTF-8'))
    print("")
    rospy.sleep(0.2)
    IMU_ser.write(bytearray(':109,0\n','UTF-8'))
    IMU_ser.write(bytearray(':123,1\n','UTF-8'))
    Flag = True
    time1 = rospy.get_time()
    while(Flag == True):
        rate.sleep()
       
        
        IMU_ser.write(bytearray(':7\n','UTF-8')) # 1: Taared 7: Untared
        IMU_ser.flushInput() #flush buffer
        IMU_read = str(IMU_ser.readline()).strip('b')
        angle_split = IMU_read.split(',')
        #print(float(angle_split[2][:-5]))
        try:
            pitch = float(angle_split[0][1:]) /math.pi*180 +0.7 #0.7 offset
            #print("pitch ",pitch
            yaw = float(angle_split[1]) /math.pi*180
            #print("yaw ",yaw)
            roll = float(angle_split[2][:-5]) /math.pi*180
            #print(pitch,yaw,roll)
            print(pitch)
            IMU.publish(pitch)   #make sure lights goes on to the right of the robot
            IMU_fail.publish(True)
            #print("roll", roll)
        except:
            #print("fail")
            pass
    
        print("======================")
            

if __name__ == "__main__":
    try:
        IMU_func = init_IMU_port()
        #First_read = IMU_set(IMU_func)
        IMU_execute(IMU_func)
    except rospy.ROSInterruptException:
        pass
