import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
GPIO.setmode(GPIO.BCM)

TRIG = 23                                  #Associate pin 23 to TRIG
ECHO = 24  

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)


def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True

gnd_speed =0.5# [m/s]
mode      = 'GROUND'  
baud_rate = 115200

print('Connecting...')
vehicle = connect("/dev/ttyACM0",baud=baud_rate,wait_ready=true)  



while not mode =='object' :
    
    GPIO.output(TRIG, False)                 #Set TRIG as LOW
    print "Waitng For Sensor To Settle"
    time.sleep(0.001)                            #Delay of 2 seconds

    GPIO.output(TRIG, True)                  #Set TRIG as HIGH
    time.sleep(0.00001)                      #Delay of 0.00001 seconds
    GPIO.output(TRIG, False)                 #Set TRIG as LOW

    distance = 0 
    while GPIO.input(ECHO)==0:               #Check whether the ECHO is LOW
        pulse_start = time.time()              #Saves the last known time of LOW pulse

    while GPIO.input(ECHO)==1:               #Check whether the ECHO is HIGH
        pulse_end = time.time()                #Saves the last known time of HIGH pulse 

        pulse_duration = pulse_end - pulse_start #Get pulse duration to a variable

        distance = pulse_duration * 17150        #Multiply pulse duration by 17150 to get distance
        distance = round(distance, 2)            #Round to two decimal points
        #print(distance)     

    if mode == 'GROUND':
        #--- Wait until a valid mission has been uploaded
        #n_WP, missionList = get_current_mission(vehicle)
        time.sleep(2)
        #if n_WP > 0:
        #    print ("A valid mission has been uploaded: takeoff!")
        mode = 'TAKEOFF' 
    elif mode == 'TAKEOFF':
    	arm_and_takeoff(3)
    	print("Changing to STABILIZE")
        ChangeMode(vehicle,"STABILIZE")
        mode = 'flight'
    elif mode == 'flight':
    	if distance <= 200:
    		ChangeMode(vehicle,"LOITER")
    		time.sleep(3)
    		ChangeMode(vehicle,"LAND")
    		mode = 'object'
    		
    time.sleep(0.3)	
