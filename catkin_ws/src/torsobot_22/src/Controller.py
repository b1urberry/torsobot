#! /usr/bin/env python3
"""
Created on Mon Mar  2 14:03:57 2020

@author: shane
"""
import rospy
import os
import datetime
import message_filters
from std_msgs.msg import Int32, Float32, Bool
from dual_g2_hpmd_rpi import motors
import operator
import matplotlib.pyplot as plt
import math

rospy.init_node('message_sync', anonymous=False)
IMU_message = message_filters.Subscriber('/IMU_angle', Float32)
#For <Madgwick Filter>
# imu_data_raw (sensor_msgs/Imu) 
# http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
Encoder_message = message_filters.Subscriber('/Encoder_data', Float32)
IMU_fail = rospy.Subscriber('/IMU_fail', Bool)

global motor_output
global angle_desired, speed_desired
global k_p_angle, k_i_angle,  k_d_angle
global k_p_speed, k_i_speed
global time1
global time2
global reset

global speed_desired2
global angle_desired2
global k_p_angle2
global k_i_angle2
global k_d_angle2
global k_p_speed2
global k_i_speed2
global First_PID_run
global flag

Frequency = 40
First_PID_run = True
#################User Input#######################


"""
First PID
"""
speed_desired = 4 # desired wheel speed in rps
angle_desired = 25.0 # desired angle - 0
k_p_angle = 20.0 #10 # propotional gain for angle control
k_i_angle = 13.0#k_p_angle/6. # integral gain for angle control
k_d_angle = 0.9# derivatibe gain for angle control 0.5

k_p_speed = 2.3#15#1.5 # proportional gain for speed control (60) 1:15
k_i_speed = 7#1.5 # integral gain for speed control(30) 1:35





time1 = 0.0
time2 = 0.0
motor_output = 0.0
flag = True
button_not_pressed = True
rate = rospy.Rate(Frequency)

def drive_motor(speed): # send speed command to motor
    global run
    max_speed = 480
    if run:
        if speed > max_speed:
            drive_speed = max_speed
        elif speed < -max_speed:
            drive_speed = -max_speed
        else:
            drive_speed = round(speed)
        motors.motor1.setSpeed(int(drive_speed))
    else:
        motors.motor1.setSpeed(0)

def PID_control(IMU_message,Encoder_message):
    global Fall
    global time1
    global time2
    global flag
    global speed_desired
    global angle_desired
    global k_p_angle
    global k_i_angle
    global k_d_angle
    global k_p_speed
    global k_i_speed
    global speed_desired2
    global angle_desired2
    global k_p_angle2
    global k_i_angle2
    global k_d_angle2
    global k_p_speed2
    global k_i_speed2
    global current_wheel_speed
    global current_imu_angle
    global speed_error_cum
    global angle_error_cum
    global angle_error_prev
    global time_current
    global run
    global file
    global speed_error
    global First_PID_run
    global motor_output


    time2 = rospy.get_time()
    
    if IMU_fail==False:
        motors.motor1.setSpeed(0)
    
        
    

    if run == False:
        if Fall == False:
            file.write('The robot stopped due to BUTTON PRESSED')
            print('Robot stopped')
            file.close()
        elif Fall == True:
            file.write('The robot stopped due to FALLING')
            print('Robot stopped')
            file.close()

#    if Encoder_message.data/60.0 >= 0.25 and flag == True:
#        print('PID switched')
#        k_p_angle = k_p_angle2
#        k_i_angle = k_i_angle2
#        k_d_angle = k_d_angle2
#        k_p_speed = k_p_speed2
#        k_i_speed = k_i_speed2
#        angle_error_cum = 0.0
#        speed_error_cum = 0.0
#        angle_error_prev = 0.0
#        speed_error = 0.0
#        speed_desired = speed_desired2
#        angle_desired = angle_desired2
#        flag = False
#        First_PID_run = True

    current_wheel_speed = Encoder_message.data/60.0  #Wheel speed will be in rps
    angle_prev = current_imu_angle
    current_imu_angle = IMU_message.data

    #### time update
    time_old = time_current # set previous time reading
    time_current = rospy.get_time() # set current time reading
    dt = time_current - time_old # time step

    """
    Outer PID loop
    """

    #Loop time
    if(First_PID_run):
    # P
        speed_error = speed_desired - current_wheel_speed
    # I
        speed_error_cum += speed_error * dt
    # Effort
        angle_desired = 1.0 * (k_p_speed * speed_error + k_i_speed * speed_error_cum)
        #angle_desired = 15
    """
    Inner PID loop
    """
    if abs(current_imu_angle) <= 60.0 :
        # P
        angle_error = angle_desired - current_imu_angle
        # I
        angle_error_cum += angle_error*dt
        # D
        angle_diff = (angle_error - angle_error_prev)/dt

        angle_error_prev = angle_error

        # Output
        motor_output = -(k_p_angle*angle_error + k_i_angle*angle_error_cum + k_d_angle*angle_diff)
        drive_motor(motor_output)
        record(time_current,speed_desired,current_wheel_speed,speed_error,speed_error_cum,angle_desired,current_imu_angle,angle_error,angle_error_cum,angle_diff,motor_output)

    else:
        #Deceleration of the motor to avoid skipping
        if motor_output<0:
            for x in range (int(motor_output),0,10):
                drive_motor(x)
                rospy.sleep(0.01)
        if motor_output>0:
            for x in range (int(motor_output),0,-10):
                drive_motor(x)
                rospy.sleep(0.01)
        drive_motor(0)
        print('angle exceeded, shutting down')
        run = False
        Fall = True

    First_PID_run = operator.not_(First_PID_run)

    print(motor_output, current_imu_angle, current_wheel_speed)
    

def record(time_current,speed_desired,wheel_speed,speed_error,speed_error_cum,angle_desired,imu_angle,angle_error,angle_error_cum,angle_diff,motor_output):
    global file
    file.write(str('%.2f'%time_current)+'\t\t'+str('%.2f'%speed_desired)+'\t\t'+str('%.2f'%wheel_speed)+'\t\t'+str('%.2f'%speed_error)+'\t\t'
        +str('%.2f'%speed_error_cum)+'\t\t'+str('%.2f'%angle_desired)+'\t\t'+str('%.2f'%imu_angle)+'\t\t'+str('%.2f'%angle_error)+'\t\t'+
        str('%.2f'%angle_error_cum)+'\t\t'+str('%.2f'%angle_diff)+'\t\t'+str('%.2f'%motor_output)+'\n')


def initialize_file():
    pass
   
def message_sync():
    global speed_error_cum
    global angle_error_cum
    global time_current
    global current_imu_angle
    global angle_error_prev
    global run
    global Fall
    global file

    Fall = False
    run = True
    speed_error_cum = 0.0
    angle_error_cum = 0.0
    current_imu_angle = 0.0
    angle_error_prev = 0.0
    time_current = rospy.get_time()
    # open a file
    initialize_file()

    sync = message_filters.ApproximateTimeSynchronizer([IMU_message,Encoder_message],queue_size = 4,slop = 0.03, allow_headerless=True)
    sync.registerCallback(PID_control)
    rate.sleep()
    rospy.spin()


if __name__ == "__main__":
    try:
        a = input("Press Enter to Start\n")
        rospy.sleep(0.5)
        time1 = rospy.get_time()
        time2 = rospy.get_time()
        print('Robot is running \n In order to stop the robot, press Ctrl+C')
        message_sync()
    except rospy.ROSInterruptException:
        motors.motor1.setSpeed(0)
        print("The program has ended")
        pass
    motors.motor1.setSpeed(0)
