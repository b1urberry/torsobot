#! /usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 14:03:57 2020

@author: shane
"""
import rospy
from std_msgs.msg import Int32, Float32
from dual_g2_hpmd_rpi import motors

rospy.init_node('Angle_Reset', anonymous=False)
Complete = False
def listener():
    global Complete
    IMU_message = rospy.Subscriber('/IMU_angle', Float32, Reset)
    rospy.spin()

def Reset(msg_in):
    global Complete
    print('Resetting')
    if msg_in.data >-8.0:
        #motors.motor1.setSpeed(msg_in.data*5.0+100.0)
        motors.motor1.setSpeed(100.0)
        rospy.sleep(0.001)
    if msg_in.data < -10.0:
        #motors.motor1.setSpeed(msg_in.data*7.0-100.0)
        motors.motor1.setSpeed(-100.0)
        rospy.sleep(0.001)
'''
    if (abs(msg_in.data)<10):
        motors.motor1.setSpeed(0)
        Complete = True
        if(Complete == True):
            print('Robot Initial Angle reset complete')
            exit()
'''
if __name__ == "__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        motors.motor1.setSpeed(0)
        print("The program has ended")
        pass
    motors.motor1.setSpeed(0)
