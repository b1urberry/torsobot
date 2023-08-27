#! /usr/bin/env python3
# Sample code to demonstrate Encoder class.  Prints the value every 5 seconds, and also whenever it changes.

import rospy
import time
import RPi.GPIO as GPIO
from encoder import Encoder
from datetime import datetime


GPIO.setmode(GPIO.BCM)

e1 = Encoder(17,27)        

while True:
    starttime = datetime.timestamp(datetime.now())
    oldvalue = e1.getValue()
    time.sleep(0.01)
    endtime = datetime.timestamp(datetime.now())
    value = e1.getValue()
    print("Value is {}".format(value))
    print("avg Value is {}".format((value-oldvalue)*1.0/(endtime-starttime)))

GPIO.cleanup()