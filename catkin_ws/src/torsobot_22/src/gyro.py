#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
@source: https://tutorials-raspberrypi.com/measuring-rotation-and-acceleration-raspberry-pi/
@editor: JungWoo

"""
import smbus
import math
import rospy
import traceback

# Publish IMU data to Controller_output.py 

from std_msgs.msg import Int32, Float32, Bool

rospy.init_node('imu_data', anonymous = False)
IMU = rospy.Publisher('/IMU_angle', Float32, queue_size = 0)
IMU_fail = rospy.Publisher('/IMU_fail', Bool, queue_size = 0)

rate = rospy.Rate(40)



# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
 
def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    
    #x_transform= -y*math.sin(math.radians(-90) )
    #y_transform= dist(x,z)*math.sin(math.radians(-90) )
    
    #radians = math.atan2(y_transform, x_transform)
    
    radians= math.atan2(y, dist(x,z))
    
    return math.degrees(radians)
 

bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect
 
# Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)

print("Start measuring Lean Angle")

while True:
    rate.sleep()
    
    gyroskop_xout = read_word_2c(0x43)
    gyroskop_yout = read_word_2c(0x45)
    gyroskop_zout = read_word_2c(0x47)
 
    #print  " skaliert: ", (gyroskop_xout / 131)
    #print "gyroskop_yout: ", ("%5d" % gyroskop_yout), " skaliert: ", (gyroskop_yout / 131)
    #print "gyroskop_zout: ", ("%5d" % gyroskop_zout), " skaliert: ", (gyroskop_zout / 131)
 
    #print
    #print "Beschleunigungssensor"
    #print "---------------------"
 
    beschleunigung_xout = read_word_2c(0x3b)
    beschleunigung_yout = read_word_2c(0x3d)
    beschleunigung_zout = read_word_2c(0x3f)
 
    #beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
    #beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
    #beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
    x = beschleunigung_xout / 16384.0
    y = beschleunigung_yout / 16384.0
    z = beschleunigung_zout / 16384.0
 
    #print "beschleunigung_xout: ", ("%6d" % beschleunigung_xout), " skaliert: ", beschleunigung_xout_skaliert
    #print "beschleunigung_yout: ", ("%6d" % beschleunigung_yout), " skaliert: ", beschleunigung_yout_skaliert
    #print "beschleunigung_zout: ", ("%6d" % beschleunigung_zout), " skaliert: ", beschleunigung_zout_skaliert
    
    #angle = -math.atan(gyroskop_yout/gyroskop_zout)
    #print(angle)
    angle = -get_x_rotation(x,y,z)  
    
    print('Angle: '+ str(angle))
    IMU.publish(angle)
    IMU_fail.publish(True)
        
    """    
    try:
            
        print('Angle: '+ str(angle))
        IMU.publish(angle)
        IMU_fail.publish(True)
            
    except:
        traceback.print_exc()
        print("Failed reading IMU data")
        #sleep(2)
        IMU_fail.publish(False)
        time.sleep(2)
        pass
        
    """