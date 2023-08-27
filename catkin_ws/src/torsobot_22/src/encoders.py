#! /usr/bin/python3
"""
Created on April 21 2022

@author:(implemented from ME 439)
editor: Jun
"""

import sys
import time
import rospy
from std_msgs.msg import Int32, Float32

try:
    f = open("/dev/encoder-driver",'rb')
except Exception as e:
    print(e,file=sys.stderr)
    print("ENCODER DRIVER NOT FOUND... DID YOU FORGET TO sudo make load IT?",file=sys.stderr)
    exit()
    
rospy.init_node('Encoder', anonymous = False)
Encoder = rospy.Publisher('/Encoder_data', Float32, queue_size = 0)
rate = rospy.Rate(100)

    
def readEncoders():
    bytes = f.read(8);
    leftInt = int.from_bytes(bytes[0:3],"little",signed=True)
    rightInt = int.from_bytes(bytes[4:7],"little",signed=True)
    return [leftInt,rightInt]
#    return [leftInt]

[leftEnc, rightEnc] = readEncoders()
#print("%d,%d" % (leftEnc, rightEnc))

#[leftEnc] = readEncoders();
#print("%d,%d" % (leftEnc));


#while 1:
#    [leftEnc, rightEnc] = readEncoders();
#    print("%d,%d" % (leftEnc, rightEnc));
#    time.sleep(1)

time1 = rospy.get_time()
[leftEnc, rightEnc] = readEncoders()
data1= -leftEnc

while 1:
    
    #print("%d,%d" % (leftEnc, rightEnc))
    
    
    rate.sleep()
    
    time2 = rospy.get_time()
    [leftEnc, rightEnc] = readEncoders()
    data2 =  -leftEnc
    
    data = (data2-data1) / ((time2-time1)*20)
    # counts/milliseconds --> RPM
    # 1200 counts per 1 revolution for the output shaft
    
    Encoder.publish(data)

    print(data)
    
    time1= time2
    data1= data2
    


'''
#! /usr/bin/python3
import sys
try:
    f = open("/dev/encoder-driver",'rb');
except Exception as e:
    print(e,file=sys.stderr)
    print("ENCODER DRIVER NOT FOUND... DID YOU FORGET TO sudo make load IT?",file=sys.stderr)
    exit()
    
def readEncoders():
    bytes = f.read(8);
    leftInt = int.from_bytes(bytes[0:3],"little",signed=True)
    rightInt = int.from_bytes(bytes[4:7],"little",signed=True)
    return [leftInt,rightInt]

[leftEnc, rightEnc] = readEncoders();

print("%d,%d" % (leftEnc, rightEnc));
'''