# UAV : QCOORD
# This script sets up the specification of a UAV in the morse simulator
# Simulates a UAV quadcopter that follows and records UGV/USV

# The UAV has the following characteristics:
#	Base: QUAD2012
#	Sensors:
#		Pose - knows own location & rotation (simulates GPS and IMU)
#		VideoCamera - RGBA image video camera
#	Actuators:
#		Waypoint - can got to specified location (simulate autopilot)
#		MotionVW - can move given velocity vector (simulate motion control)

# Import libraries
import pymorse
import argparse
import time
import numpy as nm
import json
import redis
import math
from roboutils import *

# Import interfaces
from roboutils import get_status

# Import behaviors
from roboutils import ping
from roboutils import goto_target
from roboutils import return_home
from roboutils import halt

# Define interface message command codes
class Command:
	# Completely stops and exits current mode
	halt = -1
	# Exit robot
	terminate = 0
	# Ping robot
	ping = 1
	# Will enter mode where it listens for waypoints
	modeWaypoints = 2
	# A specific waypoint to follow
	waypoint = 3
	# Will turn on camera and start recording
	cameraOn = 4

def systems_check(robot):

	# Very that robot is available
	if (ping(robot) == None):
		print ("[-] Robot {0} offline".format(robot['name']))
		return None
	print ("[+] Robot {0} online".format(robot['name']))

	# Verify 'pose' sensor
	try:
		getattr(robot['simu'], robot['name']).pose.get_properties ()
		print("[+] Robot {0} sensor: pose online".format(robot['name']))
	except:
		print("[+] Robot {0} sensor: pose offline".format(robot['name']))
		return None

	# Verify 'waypoint' sensor
	try:
		getattr(robot['simu'], robot['name']).waypoint
		print("[+] Robot {0} actuator: waypoint online".format(robot['name']))
	except:
		print("[+] Robot {0} actuator: waypoint offline".format(robot['name']))
		return None

	# Verify 'motionWM' actuator
	try:
		getattr(robot['simu'], robot['name']).motion
		print("[+] Robot {0} actuator: motion online".format(robot['name']))
	except:
		print("[+] Robot {0} actuator: motion offline".format(robot['name']))
		return None

	# Verify 'VideoCamera' sensor
	try:
		getattr(robot['simu'], robot['name']).videocamera
		print("[+] Robot {0} sensor: video camera online".format(robot['name']))
	except:
		print("[+] Robot {0} sensor: video camera offline".format(robot['name']))
		return None


	print ("[+] Robot {0}: All systems online".format(robot['name']))
	return 0

def get_angle_2D (a, b):
	"""
	Function: get_angle_2D
	Arguments:
	    a: 2D numeric array
	    b: 2D numeric array
	Purpose:
	Calculate the angle between two vectors in (R^2)
	"""
	x = nm.dot(a, b) / nm.linalg.norm(a) / nm.linalg.norm(b)
	x = 1.0 if x > 1.0 else x
	x = -1.0 if x < -1.0 else x
	x = math.acos(x)
	return x

def rotatePoint(centerPoint,point,angle):
	"""Rotates a point around another centerPoint. Angle is in degrees.
	Rotation is counter-clockwise"""
	# Source: https://gist.github.com/somada141/d81a05f172bb2df26a2c
	angle = angle
	temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
	temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
	temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
	return temp_point



def pong (redis, commStr):
	reply = { 'tag':1 }
	reply_json = json.dumps (reply, separators = (',', ':'))
	redis.publish (commStr, reply_json)

def receiveWaypoints(robot, delta, comm):

	halt = False
	speed = 1

	# While there are still targets and the quadcopter has not been told to stop
	while (halt == False):
		msg = comm.get_message ()
		if msg:
			if (msg['type'] == 'message'):
				msg_data = json.loads (msg['data'].decode("utf-8"))
				if (msg_data['tag'] == -1): # Halt tag
					halt (robot)
					halt = True
				#elif (msg_data['tag'] == 4):
					# Start recording
#					getattr(robot['simu'], robot['name']).videocamera.capture (10000)
				elif (msg_data['tag'] == 3): # A Waypoint msg
					# Get own position
					status = get_status(robot);
	
					waypoint = (msg_data['x'], msg_data['y'])
					heading_toward = (msg_data['heading_x'], msg_data['heading_y'])

					# Get heading toward targets
					w = [waypoint[0], waypoint[1]]
					n = [heading_toward[0], heading_toward[1]]
					p = [status['pos_x'], status['pos_y']]
					l = nm.subtract (n, w)
					theta = nm.arctan2 (l[1], l[0])
					if (theta < 0):
						theta = theta + 2 * math.pi

					# Find furthest target from destination
					
					
					# Set waypoint
					dest = {'x':waypoint[0], 'y':waypoint[1]}

					#goto_target(robot, dest, speed);
					getattr(robot['simu'], robot['name']).waypoint.goto(dest['x'], dest['y'], 30, theta, 0.2)
					#getattr(robot['simu'], robot['name']).motion.set_speed (speed, 0)
					robot['destination'] = dest
					#print(str(get_status(robot)))

					# Calculate distance to destination
					distance = nm.sqrt(((dest['x'] - status['pos_x']) ** 2 ) + ((dest['y'] - status['pos_y']) ** 2 ))
				
					# Modify speed based on target distance from destination
					# Commented out because the waypoint navigation does this automatically 
					#if (distance > delta + 1):
					#	speed = min (speed * (distance / (delta + distance) ) + 0.1, robot['MAX_SPEED'])
					#elif (distance < delta + 1):
					#	speed = (speed * ( (delta - distance) / (delta + distance) ) )


					time.sleep(0.5)
	return 0


def main ():
	# Parse arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("-n", "--name", help = "name of robot in MORSE environment")
	parser.add_argument("-s", "--maxSpeed", help = "max speed of QCOORD", default = 3)
	args = parser.parse_args()

	# A robot is a dictionary of info
	# Will fail if args not set => implicit argument verification
	robot = {}

	# Init message passing interface (via Redis)
	r = redis.StrictRedis (host = 'localhost', port = 6379, db = 0)
	p = r.pubsub ()
	# Subscribe to messages from GCS
	GCSconnStr = 'GCS'
	p.subscribe (GCSconnStr)
	# Publish to own channel
	connStr = 'Morse-QCOORD-' + args.name
	r.publish (connStr, 'Boot')

	with pymorse.Morse() as simu:

		# Init robot
		robot = { 'name':args.name,
		 'simu':simu,
		 # Traits
		 'MAX_SPEED': float (args.maxSpeed),
		'destination' : {'x':None, 'y':None} }

		# Ensure expected robot configuraton
		systems_check (robot)
		
		# Listen for messages
		read = False
		for msg in p.listen ():
			if (msg['type'] == 'message'):
				msg_data = json.loads (msg['data'].decode("utf-8"))
				if (msg_data['tag'] == Command.ping):
					pong (r, connStr)
				elif (msg_data['tag'] == Command.modeWaypoints):
					pong (r, connStr)
					receiveWaypoints (robot, 1, p)
				elif (msg_data['tag'] == Command.halt):
					halt (robot)
				elif (msg_data['tag'] == Command.terminate):
					halt (robot)
					break
				time.sleep (0.5)

if __name__ == "__main__":
	main ()
