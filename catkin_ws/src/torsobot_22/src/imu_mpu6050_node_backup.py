#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2020-02-11
Updated 2021-02-11
@author: Peter Adamczyk
"""

from mpu6050 import mpu6050
import time
import traceback
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion



def imu_talker():
    rospy.init_node('imu_mpu6050_node',anonymous=False)
    
    # Publisher
#    imu_pub = rospy.Publisher('/imu_mpu6050_raw',Imu,queue_size=1)
    imu_pub = rospy.Publisher('imu/data_raw',Imu,queue_size=1)
    # Message
    imu_msg = Imu()
    
    # set a Rate at which it should be read
    r = rospy.Rate(50)
    
    #imu = mpu6050(0x69)
    imu = mpu6050(0x68)
    imu.set_accel_range(imu.ACCEL_RANGE_4G)
    imu.set_gyro_range(imu.GYRO_RANGE_1000DEG)
    
    counter = 0
    
    # dummy code to print values for a while
    while not rospy.is_shutdown():
        [accel, gyro, temp] = imu.get_all_data()
        counter += 1
#        # this will print the data that are returned
#        print('a: {0}, w: {1}, T: {2}'.format(accel, gyro, temp))
        
        # Pack the essage
        #print('w: {:.2f} {:.2f} {:.2f}'.format( gyro['x'],gyro['y'],gyro['z']))
        imu_msg.header.seq = counter
        imu_msg.header.stamp = rospy.get_rostime()
        imu_msg.header.frame_id = ''
        imu_msg.orientation.x = 0.
        imu_msg.orientation.y = 0.
        imu_msg.orientation.z = 0.
        imu_msg.orientation.w = 0.
        imu_msg.orientation_covariance = np.array([-1,0.,0.,0.,0.,0.,0.,0.,0.])
        
        imu_msg.angular_velocity.x = gyro['x']*np.pi/180
        imu_msg.angular_velocity.y = gyro['y']*np.pi/180
        imu_msg.angular_velocity.z = gyro['z']*np.pi/180
#        imu_msg.angular_velocity.x = 0.03
#        imu_msg.angular_velocity.y = 0.0
#        imu_msg.angular_velocity.z = 0.0
        imu_msg.angular_velocity_covariance = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.])
        imu_msg.linear_acceleration.x = accel['x']
        imu_msg.linear_acceleration.y = accel['y']
        imu_msg.linear_acceleration.z = accel['z']
#        imu_msg.linear_acceleration.x = 0.0
#        imu_msg.linear_acceleration.y = 0.0
#        imu_msg.linear_acceleration.z = 9.8
        imu_msg.linear_acceleration_covariance = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.])
        
        #print('a_x', accel['x'] , 'a_y', accel['y'], 'a_z',accel['z'])
        # print('v_x', gyro['x'] , 'v_y', gyro['y'] , 'v_z', gyro['z'])
        
        imu_pub.publish(imu_msg) 
        
        r.sleep()
        
        
    
# the system literal "__name__" will be "__main__" if the user launches this file. 
# In that case, run the main program. 
# Otherwise it will be something related to whatever program called this one.     
if __name__ == "__main__":
    try:
        imu_talker()
        print("test1")
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
