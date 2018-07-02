import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(16, GPIO.OUT)

pwm = GPIO.PWM(16, 50) #sets PWM on pin 3 at 50Hz

pwm.start(0) #starts with 0 duty cycle so it doesn't set any angles on startup

def SetAngle(angle):
	duty = (angle/18)+2.5
	GPIO.output(16, True)
	pwm.ChangeDutyCycle(duty)
	sleep(1)
	GPIO.output(16, False)
	pwm.ChangeDutyCycle(0)

SetAngle(90)

pwm.stop()
GPIO.cleanup()

