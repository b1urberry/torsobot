#!/usr/bin/env python3


from pololu_drv8835_rpi import motors
print("issue the commands")
motors.motor1.setSpeed(300) #150
motors.motor2.setSpeed(300) #150
print("finish the commands")
