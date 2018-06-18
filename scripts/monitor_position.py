# Script : Hold_Position

# Import libraries
import pymorse
import numpy as nm
import math
import argparse
import datetime
import time
import ast
import re
import json
import redis
from shapely import geometry
from shapely.geometry import Polygon, Point





def rotatePoint(centerPoint,point,angle):
	angle = angle
	temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
	temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
	temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
	return temp_point

def calcCentroid(points):
	points = [(p[0], p[1]) for p in points]
	x, y = zip(*points)
	l = len(x)
	return sum(x) / l, sum(y) / l


def calcFieldOfView (camera):
	"""
	Function: calcFieldOfView
	Arguments:
	    camera: Camera object
	Purpose:
	    Calculate the field of view (FoV) for a camera 
	    given the x and y sensor dimensions and the focal length
	"""
	xView = (2 * math.atan (camera['xSensor_mm'] / (2 * camera['focallen_mm'])))
	yView = (2 * math.atan (camera['ySensor_mm'] / (2 * camera['focallen_mm'])))
	return (math.degrees(xView), math.degrees(yView))

def calcGroundFootprintDimensions (camera, altitude):
	"""
	Function: calcGroundFootprintDimension
	Arguments:
	    camera: Camera object
	    altitude_m: Altitude of camera in meters
	Purpose: 
	    Calculate the ground footprint of an aerial camera, 
	    such as one mounted on an aerial vehicle. 
	    Finds distances relative to the camera position, but not the actual position
	"""
	FoV = calcFieldOfView (camera)
	FoV = [768, 768]
	print ("FoV", FoV)
	distFront = altitude * (math.tan(nm.radians (camera['xGimbal_deg'] + 0.5 * FoV[0])))
	distBehind = altitude * (math.tan(nm.radians (camera['xGimbal_deg'] - 0.5 * FoV[0])))
	distLeft = altitude * (math.tan(nm.radians (camera['yGimbal_deg'] - 0.5 * FoV[1])))
	distRight = altitude * (math.tan(nm.radians (camera['yGimbal_deg'] + 0.5 * FoV[1])))
	return (distFront, distBehind, distLeft, distRight)

def calcGroundFootprint (camera, altitude, position):
	"""
	Function: calcGroundFootprint
	Arguments: 
	    camera: Camera object
	    altitude_m: Altitude of camera in meters
	    position: ground (x, y) coordinates of camera
	"""
	(distFront, distBehind, distLeft, distRight) = calcGroundFootprintDimensions (camera, altitude)
	print (distFront, distBehind, distLeft, distRight)
	print ("Height", abs (distFront - distBehind), "  Width", abs(distLeft - distRight))
	posLowerLeft = (position[0] + distLeft, position[1] + distBehind)
	posLowerRight = (position[0] + distRight, position[1] + distBehind)
	posUpperLeft = (position[0] + distLeft, position[1] + distFront)
	posUpperRight = (position[0] + distRight, position[1] + distFront)
	return (posLowerLeft, posUpperLeft, posUpperRight, posLowerRight)

def getGroundFootprint (position, target, camera, altitude):
	# Calculate angle
	i = nm.subtract (target, position)
	theta = nm.arctan2(i[1], i[0])
	if (theta < 0):
		theta = theta + 2 * math.pi
	theta = theta - (math.pi / 2)
	print (theta)

	footprint =  calcGroundFootprint (camera, altitude, position)
	footprint = [rotatePoint (position, footprint[0], theta),
	             rotatePoint (position, footprint[1], theta),
	             rotatePoint (position, footprint[2], theta),
	             rotatePoint (position, footprint[3], theta)]
	print(footprint)
	return footprint



def getTargetByName (targets, name):
	for t in targets:
		if t['name'] == name:
			return t
	return None

def main ():

	# Initialize null robots
	quadcopter = {}
	targets = []

	# Parse arguments
	parser = argparse.ArgumentParser ()
	parser.add_argument ("-r", "--quadcopter", help = "Name of quadcopter")
	parser.add_argument ("-x", "--xcoord",     help = "x coordinate of waypoint")
	parser.add_argument ("-y", "--ycoord",     help = "y coordinate of waypoint")
	parser.add_argument ("-a", "--xheading",   help = "x coordinate of heading")
	parser.add_argument ("-b", "--yheading",   help = "x coordinate of heading")
	parser.add_argument ("-t", "--targets",    help = "list of comma-separated target names")

	args = parser.parse_args ()

	# Must have a robot
	if (args.quadcopter is None):
		print ("[-] No quadcopter specified")
		exit (-1)
	
	# Must have a waypoint
	if (args.xcoord is None or \
	   args.ycoord is None or \
	   args.xheading is None or \
	   args.yheading is None):
		print ("[-] Missing or incomplete waypoint")
		exit (-1)

	# Init message passing interface (via Redis)
	r = redis.StrictRedis (host = 'localhost', port = 6379, db = 0)
	p = r.pubsub ()
	GCSconnStr = 'GCS'
	
	quadName = args.quadcopter
	targetNames = args.targets.split (",")
	
	quadConnStr = 'Morse-QCOORD-' + quadName
	p.subscribe (quadConnStr)
	
	# Init quadcopter
	quad = { 'name':quadName, 'connStr':quadConnStr, 'waypoint':{}, 'altitude': 30  }

	# Init targets
	for name in targetNames:
		targetConnStr = 'Morse-Marisa-' + name
		target = {'name':name, 'connStr':targetConnStr}
		targets.append (target)

	# Subscribe to each target
	for t in targets:
		t['pubsub'] = r.pubsub ()
		t['pubsub'].subscribe (t['connStr'])
		t['pos_x'] = None
		t['pos_y'] = None
		t['dest_x'] = None
		t['dest_y'] = None
		t['distance'] = None



	# Camera object
	camera = { 'xSensor_mm': 256,
		   'ySensor_mm': 256,
		   'focallen_mm': 97,
		   'xGimbal_deg': 0,
		   'yGimbal_deg': 0,
	}
	
	
	# Ping quadcopter
	ping = { 'tag':1 }
	ping_json = json.dumps (ping)
	r.publish (GCSconnStr, ping_json)
	msg = p.listen ()  # Will block until message recieved from quadcopter

	# Tell quadcopter to start listening for waypoints
	quadModeSet = { 'tag':2 }
	quadModeSet_json = json.dumps (quadModeSet)
	r.publish (GCSconnStr, quadModeSet_json)

	interval = 10
	counter  = 0

	# Main Loop 
	while (True == True):  # As long as you don't stop script
	
		# Update quadcopter status
		msg = p.get_message ()
		if msg:
			if msg['type'] == 'message':
				msg_data = json.loads(msg['data'].decode("utf-8"))
				if msg_data['tag'] == 5:
					camera['xGimbal_deg'] = msg_data['xGimbal_deg']
					camera['yGimbal_deg'] = msg_data['yGimbal_deg']
		
		# Update target status
		for t in targets:
			msg = t['pubsub'].get_message ()
			if msg:
				# Time of received
				timestamp = time.time ()

				if (msg['type'] == 'message'):
					msg_data = json.loads (msg['data'].decode("utf-8"))
					t['pos_x'] = msg_data['pos_x']
					t['pos_y'] = msg_data['pos_y']
					t['dest_x'] = msg_data['dest_x']
					t['dest_y'] = msg_data['dest_y']

		# Set waypoint and point heading toward
		quad['waypoint']['x'] = float(args.xcoord)
		quad['waypoint']['y'] = float(args.ycoord)
		quad['waypoint']['heading_x'] = float(args.xheading)
		quad['waypoint']['heading_y'] = float(args.yheading)

		# Set camera target
		quad['target'] = {}

		# !!!! DEBUG: Set this version to have qcoord look directly downward
		#quad['target']['x'] = float(args.xcoord)
		#quad['target']['y'] = float(args.ycoord)

		quad['target']['x'] = float(args.xheading)
		quad['target']['y'] = float(args.yheading)



		

		# Send waypoint to quadcopter
		msgSend = quad['waypoint']
		msgSend['tag'] = 3
		msgSend_json = json.dumps (msgSend)
		r.publish (GCSconnStr, msgSend_json)

		# Send camera target to quadcopter
		msgSend = quad['target']
		msgSend['tag'] = 4
		msgSend_json = json.dumps (msgSend)
		r.publish (GCSconnStr, msgSend_json)

		if (counter % interval == 0):

			# Calc footprint
			#camera["xGimbal_deg"] = camera["yGimbal_deg"] + 57.29
			#camera["xGimbal_deg"] = 15
			camera["yGimbal_deg"] = 0
			position = (float(args.xcoord), float(args.ycoord))
			tpose = [quad["waypoint"]["heading_x"], quad["waypoint"]["heading_y"]]
			print (position, tpose, "X-deg", camera['xGimbal_deg'], "Y-deg", camera['yGimbal_deg'])
			footprint = getGroundFootprint (position, tpose, camera, quad['altitude'])
			footprint_poly = geometry.Polygon([[p[0], p[1]] for p in footprint])
			print ("---")
			# Calc centroid of footprint
			footprint_centroid = calcCentroid(footprint)

			#print (footprint)
			#print (footprint_centroid)


			distances = []
			# Distance between point and each target
			for t in targets:
				if t['pos_x'] is not None:
					distances.append( { 'target' : t['name'],
					    'distFromCenter'      : nm.sqrt(((footprint_centroid[0] - t['pos_x']) ** 2) + \
							                    ((footprint_centroid[1] - t['pos_y']) ** 2)),
					    'distFromBoundary'    : footprint_poly.boundary.distance( geometry.Point([t['pos_x'], t['pos_y']])),
					    'distFromFootprint'   : footprint_poly.distance( geometry.Point([t['pos_x'], t['pos_y']])),
					})  
					distances[len(distances) - 1]['distFromFootprintRel'] = \
					    distances[len(distances) - 1]['distFromBoundary'] if distances[len(distances) - 1]['distFromFootprint'] != 0 \
					    else distances[len(distances) - 1]['distFromBoundary'] * (-1)

			for d in distances:
				if d["distFromFootprintRel"] > 0:
					print ("%s is outside camera view" % d["target"])
				else:
					print ("%s is within camera view"  % d["target"])

		counter = counter + 1

		# Delay
		time.sleep (0.5)
		
		




main()
