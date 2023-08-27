#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import os
import datetime
import message_filters
from std_msgs.msg import Int32, Float32
from dual_g2_hpmd_rpi import motors
import RPi.GPIO as GPIO
import operator

rospy.init_node('message_sync', anonymous=False)
IMU_message = message_filters.Subscriber('/IMU_angle', Float32)
Encoder_message = message_filters.Subscriber('/Encoder_data', Float32)


global motor_output
global speed_desired
global angle_desired
global k_p_angle 
global k_i_angle 
global k_d_angle 
global k_p_speed 
global k_i_speed 
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
speed_desired = 0.7 # desired wheel speed in rpm
angle_desired = 0.0 # desired angle - 0
k_p_angle = 20.0#10 # propotional gain for angle control
k_i_angle = 22.0#k_p_angle/6. # integral gain for angle control
k_d_angle = 0.2# derivatibe gain for angle control
k_p_speed = 18.0#15#1.5 # proportional gain for speed control (60) 1:15
k_i_speed = 18.0#1.5 # integral gain for speed control(30) 1:35

"""


Second PID
"""
k_p_angle2 = 10.#10 # propotional gain for angle control // 
k_i_angle2 = 13.#1.5#k_p_angle/6. # integral gain for angle control
k_d_angle2 = 0.7# derivatibe gain for angle control

k_p_speed2 = 6.#1.5 # proportional gain for speed control (60) 1:15
k_i_speed2 = 10#1.5 # integral gain for speed control(30) 1:35
speed_desired2 = 0.6
angle_desired2 = 0.0

########################################

time1 = 0
time2 = 0
motor_output = 0
flag = True
button_not_pressed = True
rate = rospy.Rate(Frequency)

"""
The following code initializes the button press feature
"""
pin_switch_write = 20 #pin 38
pin_switch_read = 21 #pin 40
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin_switch_write, GPIO.OUT)
GPIO.setup(pin_switch_read, GPIO.IN, GPIO.PUD_DOWN) 
GPIO.output(pin_switch_write,1)

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
    
    if GPIO.input(pin_switch_read) == 1:
        run = False    
    if run == False:
        if Fall == False:
            file.write('The robot stopped due to BUTTON PRESSED')
            print('Robot stopped')
            file.close()      
        elif Fall == True:
            file.write('The robot stopped due to FALLING')
            print('Robot stopped')
            file.close()
    
    if Encoder_message.data/60.0 >= 0.1 and flag == True:
        print('PID switched')
        k_p_angle = k_p_angle2
        k_i_angle = k_i_angle2
        k_d_angle = k_d_angle2
        k_p_speed = k_p_speed2
        k_i_speed = k_i_speed2
        angle_error_cum = 0.0
        speed_error_cum = 0.0
        angle_error_prev = 0.0
        speed_error = 0.0
        speed_desired = speed_desired2
        angle_desired = angle_desired2
        flag = False
        First_PID_run = True
       
    current_wheel_speed = -Encoder_message.data/60.0 #Wheel speed will be in rps 
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
        angle_desired = 2.0 * (k_p_speed * speed_error + k_i_speed * speed_error_cum) 
    """
    Inner PID loop
    """
    if abs(current_imu_angle) <= 60 :
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
    
        
def record(time_current,speed_desired,wheel_speed,speed_error,speed_error_cum,angle_desired,imu_angle,angle_error,angle_error_cum,angle_diff,motor_output):
    global file
    file.write(str('%.2f'%time_current)+'\t\t'+str('%.2f'%speed_desired)+'\t\t'+str('%.2f'%wheel_speed)+'\t\t'+str('%.2f'%speed_error)+'\t\t'
        +str('%.2f'%speed_error_cum)+'\t\t'+str('%.2f'%angle_desired)+'\t\t'+str('%.2f'%imu_angle)+'\t\t'+str('%.2f'%angle_error)+'\t\t'+
        str('%.2f'%angle_error_cum)+'\t\t'+str('%.2f'%angle_diff)+'\t\t'+str('%.2f'%motor_output)+'\n')
    

def initialize_file():
    global file
    global speed_desired
    global angle_desired
    gotname = False
    num = 1
    while not gotname: # keep interating until a unique file name is found
        name_temp = 'Test' + str(num) + '.txt'
        if os.path.exists('/home/pi/catkin_ws/src/torsobot/src/'+name_temp): # if file name already exists, add 1 to file number
            num += 1
        else:
            filename = name_temp
            gotname = True
    file = open('/home/pi/catkin_ws/src/torsobot/src/'+filename,'w') # open file
    file.write('Testing Time:' + str(datetime.datetime.now())+'\n') # add date & time
    file.write('--------------------------------------------------\n')
    file.write('------------First PID----------------\n')
    # record angle gain
    file.write('kp_angle = ' + str(round(k_p_angle,3)) + ', ' + 'ki_angle = ' + str(round(k_i_angle,3))+ ', ' + 'kd_angle = ' + str(round(k_d_angle,3)))
    # record speed gain
    file.write(', kp_speed = ' + str(round(k_p_speed,3)) + ', ' + 'ki_speed = ' + str(round(k_i_speed,3)))
    file.write(', Speed Desired = ' + str(speed_desired) + '\n') # desired speed
    file.write('------------Second PID----------------\n')    
    file.write('kp_angle = ' + str(round(k_p_angle2,3)) + ', ' + 'ki_angle = ' + str(round(k_i_angle2,3))+ ', ' + 'kd_angle = ' + str(round(k_d_angle2,3)))
    # record speed gain
    file.write(', kp_speed = ' + str(round(k_p_speed2,3)) + ', ' + 'ki_speed = ' + str(round(k_i_speed2,3)))
    file.write(', Speed Desired = ' + str(speed_desired2) + '\n') # desired speed
    file.write('--------------------------------------------------\n')
    # column headers
    file.write('Time\t\tSpeed_desired\t\tWheel_speed\tSpeed_error\tSpeed_error_cum\tAngle_desired\timu_angle\tangle_error\tangle_error_cum\tangle_diff\tmotor_output\n')

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
        while button_not_pressed == True:
            if GPIO.input(pin_switch_read) == 1:
                rospy.sleep(0.01)
                if GPIO.input(pin_switch_read) == 1:
                    button_not_pressed = False
            print('Push the button to start')
            #only start the program is the switch is pressed
        rospy.sleep(0.5)
        time1 = rospy.get_time()
        time2 = rospy.get_time()
        print('Robot is running \n In order to stop the robot, please press the button again')
        message_sync()
    except rospy.ROSInterruptException:
        motors.motor1.setSpeed(0)
        print("The program has ended")
        pass
    motors.motor1.setSpeed(0)
