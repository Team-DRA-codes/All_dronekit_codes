import RPi.GPIO as GPIO                    #Import GPIO library
import time                                #Import time library
GPIO.setmode(GPIO.BCM)

from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil



TRIG = 23                                #Associate pin 23 to TRIG
ECHO = 24

GPIO.setup(TRIG,GPIO.OUT)                  #Set pin as GPIO out
GPIO.setup(ECHO,GPIO.IN) 
GPIO.output(TRIG,0)                  #Set pin as GPIO in

def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print "waiting to be armable"
      time.sleep(1)

   print "Arming motors"
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print "Taking Off"
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print "Target altitude reached"
          break
      time.sleep(1)


def ChangeMode(vehicle, mode):
    while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(0.5)
    return True

gnd_speed =1.5# [m/s]
mode      = 'GROUND'  
baud_rate = 115200

print 'Connecting...'
vehicle = connect("/dev/ttyACM0",baud=baud_rate,wait_ready=true)  

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

def measure_average():
  # This function takes 3 measurements and
  # returns the average.
  distance1=get_distance()
  time.sleep(0.05)
  distance2=get_distance()
  time.sleep(0.05)
  distance = distance1 + distance2
  distance = distance / 2
  return distance

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


'''while not mode =='object' :
    
              #Round to two decimal points
           

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
    		
    time.sleep(0.3)	'''
while True:
    	measure_average()
    	time.sleep(0.1)
    	if distance<=200:
    		ChangeMode("GUIDED")
    		set_velocity_body(vehicle,-gnd_speed,0,0)
    		time.sleep(0.5)
    		ChangeMode("LOITER")