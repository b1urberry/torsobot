#! /usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun16 14:03:57 2020

@author:Yuxiao
"""

import rospy
import math

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

# from scipy.spatial.transform import Rotation as R

IMU_angle_pub = rospy.Publisher('/IMU_angle', Float32, queue_size = 0)
rad_to_degree = 180/math.pi


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = -math.atan2(t0, t1) * rad_to_degree
    #roll_x = -math.atan2(t0, t1) * rad_to_degree - 10 #Madgwick -0.5 / -3.5
    #roll_x = -math.atan2(t0, t1) * rad_to_degree -15.1 #Complementary (basic -15.39 - 3) -18
    
    
    #stationary: -0.55
    #moving: -
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2) * rad_to_degree

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4) * rad_to_degree
    return roll_x, pitch_y, yaw_z 

def euler_from_quaternion1(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
 
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
 
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
 
    return roll_x * rad_to_degree, pitch_y * rad_to_degree, yaw_z * rad_to_degree

def callback(data):
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    roll, pitch, yaw = euler_from_quaternion(x,y,z,w)
    #r = R.from_quat([x, y, z, w])
    #print(r.as_matrix())
    #print("x=",x,"y=",y,"z=",z,"w=",w)
    print("roll={} pitch={} yaw={}".format(roll, pitch, yaw))
    
    IMU_angle_pub.publish(float(roll))
    
    
print("Start Imu processing.")
rospy.init_node('imu_processor', anonymous=True)
rospy.Subscriber('imu/data', Imu, callback)

rospy.spin()
