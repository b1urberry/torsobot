#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 14:03:57 2020

@author: shane
"""
import rospy
import serial
import math
import numpy as np
from std_msgs.msg import Int32, Float32

rospy.init_node('Encoder', anonymous = False)
Encoder = rospy.Publisher('/Encoder_data', Float32, queue_size = 0)
rate = rospy.Rate(100)

def init_encoder_port(): # find encoder port address and initialize connection if found
    port = None
    for count in range(0,6):
        address = '/dev/ttyUSB' + str(count)
        print(count)
        for j in range(0,20):
            try:
                ser = serial.Serial(
                    port=address,
                    baudrate = 19200,
                    parity = 'N',
                    stopbits = 1,
                    bytesize = 8,
                    timeout = 0.05
                )
                ser.flushInput()
                rospy.sleep(0.1)
                read = ser.readline()
                position_read = str(read).strip()
                if len(position_read) >= 1 and position_read[1:].isdigit() or position_read.isdigit(): # identify whether a given port is connected to the encoder by checking the reading
                    port = ser
                    print('Encoder Port Found at ' + address)
                    return port
            except:
                pass
    print('Encoder Port Not Found')
    return None


def Encoder_Reading(port):
    while True:
        port.flushInput()
        #port.write('\x61')   # send a
        rate.sleep()

        Enc_read = port.readline()
        Enc_read = str(Enc_read).strip()
        if len(Enc_read) >= 1 and Enc_read[1:].isdigit() or Enc_read.isdigit():

            print(Enc_read)
            Encoder.publish(float(Enc_read))

        else:
            continue

if __name__ == "__main__":
    try:
       Enc_ser = init_encoder_port()
       Encoder_Reading(Enc_ser)
    except rospy.ROSInterruptException:
        pass
