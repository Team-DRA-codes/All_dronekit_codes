import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
GPIO.setmode(GPIO.BCM)

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil



TRIG = 23                                  #Associate pin 23 to TRIG
ECHO = 24  

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN)                   #Set pin as GPIO in

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

def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


while not mode =='object' :
    
    GPIO.output(TRIG, False)                 #Set TRIG as LOW
    print "Waitng For Sensor To Settle"
    time.sleep(0.1)                            #Delay of 2 seconds

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
           

    if mode == 'GROUND':
        #--- Wait until a valid mission has been uploaded
        #n_WP, missionList = get_current_mission(vehicle)
        time.sleep(2)
        
        #    print ("A valid mission has been uploaded: takeoff!")
        mode = 'TAKEOFF' 
    elif mode == 'TAKEOFF':
    	  arm_and_takeoff(2)
        #time.sleep(10)
    	  print "Changing to AUTO"
        ChangeMode(vehicle,"AUTO")
        mode = 'flight'
    elif mode == 'flight':
       	print "mode cahnged to AUTO"
    	if distance < 200:
        print 'object detected'
    		ChangeMode(vehicle,'BRAKE')
        time.sleep(0.5)
        ChangeMode(vehicle,'GUIDED')
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    		time.sleep(1)
        set_velocity_body(vehicle, gnd_speed, 0, 0)
        time.sleep(1)
    		ChangeMode(vehicle,"AUTO")
    		
    time.sleep(0.3)	
    
