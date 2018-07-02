import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
GPIO.setmode(GPIO.BCM)

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

 



TRIG = 23                                #Associate pin 23 to TRIG
ECHO = 24

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN) 
GPIO.output(TRIG,0)



def get_distance():
	print "measuring"

	GPIO.output(TRIG,1)
	time.sleep(0.00001)
	GPIO.output(TRIG,0)

	while GPIO.input(ECHO) == 0:
    	pass
	start =time.time()

	while GPIO.input(ECHO) == 1:
    	pass
	stop=time.time()
	distance = (stop-start)*17150

	print distance

print 'Connecting...'
vehicle = connect("/dev/ttyACM0",baud=baud_rate,wait_ready=True) 

while True:
	print(get_distance())
	time.sleep(0.01)