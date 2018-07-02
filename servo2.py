import RPi.GPIO as GPIO
import time

servo = 40
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo,GPIO.OUT)
GPIO.setwarnings(False)

p=GPIO.PWM(servo,50)# 50hz frequency
p.start(7.2)#

p.ChangeDutyCycle(5.2)
time.sleep(1)
p.ChangeDutyCycle(7.2)
time.sleep(1)

p.stop()