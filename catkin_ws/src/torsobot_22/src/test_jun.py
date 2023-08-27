from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
E1A=11
E1B=13

GPIO.setup(E1A,GPIO.IN,)