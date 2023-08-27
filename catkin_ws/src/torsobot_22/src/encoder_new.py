import RPi.GPIO as GPIO
left_pin = 17
right_pin = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(left_pin, GPIO.IN)
GPIO.setup(right_pin, GPIO.IN)

while True:
    A = GPIO.input(left_pin)
    B = GPIO.input(right_pin)
    print("left value = {}, right value = {}".format(A, B))