import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
GPIO.setmode(GPIO.BCM)

TRIG = 23                                  #Associate pin 23 to TRIG
ECHO = 24  

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN)  
GPIO.output(TRIG,False)
while True:

	ChangeMode(vehicle,LOITER)
	time.sleep(10)
	GPIO.output(TRIG, False)                 #Set TRIG as LOW
	print "Waitng For Sensor To Settle"
	time.sleep(0.1)

	GPIO.output(TRIG, True)                  #Set TRIG as HIGH
	time.sleep(0.00001)                      #Delay of 0.00001 seconds
	GPIO.output(TRIG, False)                 #Set TRIG as LOW

	while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW
    	pulse_start = time.time()              #Saves the last known time of LOW pulse

	while GPIO.input(ECHO)==1:               #Check whether the ECHO is HIGH
    	pulse_end = time.time()                #Saves the last known time of HIGH pulse 

    	pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

    	distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
    	distance = round(distance, 2)            #Round to two decimal points
    	print distance
