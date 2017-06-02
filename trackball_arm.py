# for more Pololu Micro Maestro info
# see http://afflator.ontopoeticmachines.org/post/9
from sys import *
import math
import serial
import time
import os
import struct
import sys
import ctypes

global Cur_x,Cur_y,Cur_z,Cur_w


#Arm dimensions( mm ) */
BASE_HGT=56.00      #base hight 2.65" (original author had 67.31)
HUMERUS=146.00      #shoulder-to-elbow "bone" 5.75"
ULNA=185.00        #elbow-to-wrist "bone" 7.375" (original author had 187.325)
GRIPPER=150.00      #gripper (incl. my copper chopsticks end effector)

# Servo names/numbers */
# Base servo HS-485HB */
BAS_SERVO=4
#Shoulder Servo HS-5745-MG */
SHL_SERVO=3
#Elbow Servo HS-5745-MG */
ELB_SERVO=2
#Wrist servo HS-645MG */
WRI_SERVO=1
#Wrist rotate servo HS-485HB */
#WRO_SERVO=4
#Gripper servo HS-422 */
GRI_SERVO=0

#pre-calculations */
hum_sq = HUMERUS*HUMERUS
uln_sq = ULNA*ULNA


def setpos(cha,uS):
    """maestro uses is 0.25us increments 
    """
    pos=int(uS) 
    pos=pos*4
    low=pos&0x7f
    high=pos>>7
    if(testMode==0):
    	s.write(chr(0x84)+chr(cha)+chr(low)+chr(high))


# arm positioning routine utilizing inverse kinematics */
# z is height, y is distance from base center out, x is side to side. y,z can only be positive */

def set_arm(x, y, z, grip_angle_d):
	
#        xy_text='X:'+str(int(x))+' Y:'+str(int(y))
#        Label(master,text=xy_text).grid(row=0,column=1)

	print "set_arm(",x,",",y,",",z,",",grip_angle_d,")"

	grip_angle_r = math.radians( grip_angle_d ) #grip angle in radians for use in calculations
	#Base angle and radial distance from x,y coordinates */
	bas_angle_r = math.atan2( x, y )  #note I assume it is atan2(x,y) not atan2(y,x)!
	print "bas_angle:",math.degrees(bas_angle_r)
	rdist = math.sqrt(( x * x ) + ( y * y ))
	print "rdist:",rdist
	# rdist is y coordinate for the arm when projects on the 2D plane of the arm*/
	y = rdist
    
	# Grip offsets calculated based on grip angle */
	grip_off_z = ( math.sin( grip_angle_r )) * GRIPPER
	grip_off_y = ( math.cos( grip_angle_r )) * GRIPPER
    
	print "grip_off_z:",grip_off_z
	print "grip_off_y:",grip_off_y
    
	# Wrist position */
	wrist_z = ( z - grip_off_z ) - BASE_HGT
	wrist_y = y - grip_off_y
    
	# Shoulder to wrist distance ( AKA sw ) */
	s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y )
	s_w_sqrt = math.sqrt( s_w )
    
	# s_w angle to ground */
	#float a1 = atan2( wrist_y, wrist_z )
	a1 = math.atan2( wrist_z, wrist_y )
    
	# s_w angle to humerus */
	try:
		a2 = math.acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ))
	except ValueError:
		print "hum_sq:",hum_sq,"uln_sq:",uln_sq,"s_w:",s_w,"HUMERUS:",HUMERUS,"s_w_sqrt:",s_w_sqrt
		print (( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt )

	# shoulder angle */
	shl_angle_r = a1 + a2
	shl_angle_d = math.degrees( shl_angle_r )
    
	# elbow angle */
	elb_angle_r = math.acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ))
	elb_angle_d = math.degrees( elb_angle_r )
	elb_angle_dn = -( 180.0 - elb_angle_d )
    
	# wrist angle */
	wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d
   
	elb_mult=9.0 #formerly 6.6
	shl_mult=9.0 #formerly 6.6
	wri_mult=12.0 #formerly 11.1 

	# Servo pulses - mid angle was 1500, I have adjusted for my arm
	bas_servopulse = 1538.0 - (( math.degrees( bas_angle_r )) * 11.11 )
	shl_servopulse = 1932.0 + (( shl_angle_d - 90.0 ) * shl_mult )
	elb_servopulse = 1539.0 -  (( elb_angle_d - 90.0 ) * elb_mult )
	wri_servopulse = 2739.0 + ( wri_angle_d  * wri_mult )
    
	print "shl_angle:",shl_angle_d
	print "elb_angle:",elb_angle_d
	print "wri_angle:",wri_angle_d
    
    
	# Set servos */
	setpos( BAS_SERVO, bas_servopulse)
	setpos( WRI_SERVO, wri_servopulse)
	setpos( SHL_SERVO, shl_servopulse)
	setpos( ELB_SERVO, elb_servopulse)

def move_arm(x,y,z,w):
    global Cur_x,Cur_y,Cur_z,Cur_w
    for t in range(0,100):
        tx=(x*(t/100.0))+(Cur_x*((100-t)/100.0))
        ty=(y*(t/100.0))+(Cur_y*((100-t)/100.0))
        tz=(z*(t/100.0))+(Cur_z*((100-t)/100.0))
        tw=(w*(t/100.0))+(Cur_w*((100-t)/100.0))
        set_arm(tx,ty,tz,tw)
        time.sleep(0.01)
    Cur_x=x
    Cur_y=y
    Cur_z=z
    Cur_w=w

### main program starts here

if(len(argv)>1):
        print "TEST MODE"
        testMode=1
else:
        testMode=0

# initiate USB serial connection to Pololu Micro Maestro
if(testMode==0):
        s=serial.Serial()
        # lower port # is command port
        s.port="/dev/ttyACM0"
        s.baudrate=115200
        s.timeout=1
        s.open()

# parking position
Cur_x=0
Cur_y=185
Cur_z=150
Cur_w=-60

grip_closed=2368
grip_open=1600

# path to /dev for Kensington Orbit Trackball USB Mouse with Scroll Ring
infile_path = "/dev/input/event0")

#long int, long int, unsigned short, unsigned short, long int
FORMAT = 'llHHl'

EVENT_SIZE = struct.calcsize(FORMAT)

#open file in binary mode
in_file = open(infile_path, "rb")

event = in_file.read(EVENT_SIZE)

last_sec=0

# note reduction in X/Y speed by 1/10, needed for useful control

while event:
	(tv_sec, tv_usec, type, code, value) = struct.unpack(FORMAT, event)

	Old_x=Cur_x
	Old_y=Cur_y
	Old_z=Cur_z

	if type == 2 and code == 0:
       		print("delta X:",value)
		Cur_x=Cur_x+value/10.0	
	if type == 2 and code == 1:
		print("delta Y:",value)
		Cur_y=Cur_y+value/10.0
	if type == 2 and code == 8:
	        print("delta Z:",value)
		Cur_z=Cur_z+value
	if type == 1 and code == 272:
	        print("grasp open")
		setpos(0,grip_open)
	if type == 1 and code == 273:
	        print("grasp closed")
		setpos(0,grip_closed)

# sometimes the inverse kinetics blows up near the edge of the useful
# arm reach space

	try:	
		set_arm(Cur_x,Cur_y,Cur_z,Cur_w)
	except:
		print("Error!")
		Cur_x=Old_x
		Cur_y=Old_y
		Cur_z=Old_z

	event = in_file.read(EVENT_SIZE)

in_file.close()
