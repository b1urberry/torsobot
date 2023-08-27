#!/usr/bin/env python2
# -*- coding: utf-8 -*-

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
    print(msg_in.data)
    if msg_in.data >4:
        motors.motor1.setSpeed(150)
        rospy.sleep(0.001)
    if msg_in.data < -4:
        motors.motor1.setSpeed(-150)
        rospy.sleep(0.001)
    if (abs(msg_in.data)<4):
        motors.motor1.setSpeed(0)
        Complete = True
        while(Complete == True):
            print('Robot Initial Angle reset complete')

        

if __name__ == "__main__": 
    try:   
        listener()
    except rospy.ROSInterruptException:
        print("The program has ended")
        pass
