# Script : Follow_Targets
# Implements coordination between a quadcopter and a group of UGVs

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


def getTargetByName (targets, name):
	for t in targets:
		if t['name'] == name:
			return t

	return None

def calcCentroid (targets, posORway):
	"""
	Function: calcCentroid
	Arguments:
		targets: List of 'Target' structs
		posORway: Whether you are looking for centroid of positions ("pos") or of their next waypoints ("way")
	Purpose:
	    Returns the center point of the current positions in all targets
	Source:
	    http://stackoverflow.com/questions/23020659/fastest-way-to-calculate-the-centroid-of-a-set-of-coordinate-tuples-in-python-wi
	"""
	if posORway == "pos":
		points = [(t['pos_x'], t['pos_y']) for t in targets if t["comm"]]
	elif posORway == "way":
		points = [(t['dest_x'], t['dest_y']) for t in targets if t["comm"]]
	x, y = zip (*points)
	l = len (x)
	return sum (x) / l, sum (y) / l

def calcFootprintCentroid(points):
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
	return (xView, yView)

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

	#FoV = calcFieldOfView (camera)
	## WARNING: Hard-coded. Assumes MORSE default camera dimensions and focallen
	FoV = [768, 768]

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
	posLowerLeft = (position[0] + distLeft, position[1] + distBehind)
	posLowerRight = (position[0] + distRight, position[1] + distBehind)
	posUpperLeft = (position[0] + distLeft, position[1] + distFront)
	posUpperRight = (position[0] + distRight, position[1] + distFront)
	return (posLowerLeft, posUpperLeft, posUpperRight, posLowerRight)


def getMaxMinFromPosition (targets, position):
	"""
	Function: getMaxMinFromPosition
	Arguments:
		targets: List of 'Target' structs
		position: A len-2 list whose first value is x coord and whose second is y coord.
	Purpose:
		Returns the closest and furthest target name and distances
	"""

	closestTarget = None
	farthestTarget = None

	count = 0
	for t in targets:
		if t["comm"] is False:
			continue
		distance = nm.sqrt(((position[0] - t['pos_x']) ** 2 ) + ((position[1] - t['pos_y']) ** 2 ))
		if count == 0:
			farthestTarget = { 'name':t['name'], 'value':distance }
			closestTarget = { 'name':t['name'], 'value':distance }
		else:
			if distance > farthestTarget['value']:
				farthestTarget =  { 'name':t['name'], 'value':distance }
			if distance < closestTarget['value']:
				closestTarget = { 'name':t['name'], 'value':distance }

	return { 'farthest':farthestTarget, 'closest':closestTarget }

def rotatePoint(centerPoint,point,angle):
	"""Rotates a point around another centerPoint. Angle is in degrees.
	Rotation is counter-clockwise"""
	# Source: https://gist.github.com/somada141/d81a05f172bb2df26a2c
	angle = angle
	temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
	temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
	temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
	return temp_point


def getGroundFootprint (position, target, camera, altitude):
        # Calculate angle
        i = nm.subtract (target, position)
        theta = nm.arctan2(i[1], i[0])
        if (theta < 0):
                theta = theta + 2 * math.pi
        theta = theta - (math.pi / 2)

        footprint =  calcGroundFootprint (camera, altitude, position)
        footprint = [rotatePoint (position, footprint[0], theta),
                     rotatePoint (position, footprint[1], theta),
                     rotatePoint (position, footprint[2], theta),
                     rotatePoint (position, footprint[3], theta)]
        return footprint



def main ():

	reposition = False

	# Initialize null robots
	quadcopter = {}
	targets = []

	# Parse arguments
	parser = argparse.ArgumentParser ()
	parser.add_argument ("-q", "--quadcopter", help = "Name of quadcopter")
	parser.add_argument ("-t", "--targets", help = "Comma-separated list of target names")
	args = parser.parse_args ()

	# Must have a quadcopter
	if (args.quadcopter is None):
		print ("[-] No targets specified")
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
	quad = { 'name':quadName, 'connStr':quadConnStr, 'waypoint':{}, 'x':0, 'y':0}

	# Init targets
	for name in targetNames:
		targetConnStr = 'Morse-Marisa-' + name
		target = {'name':name, 'connStr':targetConnStr, 'comm':False}
		target["distance"] = None
		targets.append (target)

	# Subscribe to quadcopter
	quadConnStr = 'Morse-QCOORD-' + quadName
	p.subscribe (quadConnStr)

	# Ping quadcopter
	ping = { 'tag':1 }
	ping_json = json.dumps (ping)
	r.publish (GCSconnStr, ping_json)
	msg = p.listen ()  # Will block until message recieved from quadcopter

	# Subscribe to each target
	for t in targets:
		t['pubsub'] = r.pubsub ()
		t['pubsub'].subscribe (t['connStr'])

	# Tell quadcopter to start listening for waypoints
	quadModeSet = { 'tag':2 }
	quadModeSet_json = json.dumps (quadModeSet)
	r.publish (GCSconnStr, quadModeSet_json)

	# Setup camera
	# ASSUMES the constant altitude of 30m
	camera = { 'xSensor_mm': 255,
		   'ySensor_mm': 255,
		   'focallen_mm': 93.0909,
		   'xGimbal_deg': 0,
		   'yGimbal_deg': 0,
	}
	altitude = 30

	# Control logic
	haveWaypoint = False
	firstWaypoint = True
	haveTargetPose = False

	position = None

	# Main Loop
	nComm = 0
	nTargets = len(targets)
	while (nTargets > 0):  # As long as targets remain

		# Control flags
		newWaypoint = False

		# World Model
		centroidPrev = None
		centroidPrevTimestamp = None
		centroidSpeed_s = None

		# Update quadcopter status
		msg = p.get_message ()
		if msg:
			if msg['type'] == 'message':
				msg_data = json.loads(msg['data'].decode("utf-8"))
				if msg_data['tag'] == 5:
					camera['xGimbal_deg'] = abs(msg_data['xGimbal_deg'])
					camera['yGimbal_deg'] = 0
					quad["x"] = msg_data['pos_x']
					quad['y'] = msg_data['pos_y']
					position = (float(quad["x"]), float(quad["y"]))

		# Find camera footprint
		if haveWaypoint == True and position is not None:

			tpose = [float(centroid[0]), float (centroid[1])]

			footprint = getGroundFootprint (position, tpose, camera, altitude)
			footprint_poly = geometry.Polygon([[p[0], p[1]] for p in footprint])
			footprint_centroid = calcFootprintCentroid(footprint)

		for t in targets:
			# Update targets' status
			msg = t['pubsub'].get_message ()

			if msg:
				# Time of received
				timestamp = time.time ()

				if (msg['type'] == 'message'):
					#! Should actually check tag to see what kind of message

					# Has received a new waypoint
					newWaypoint = True

					msg_data = json.loads (msg['data'].decode("utf-8"))

					t['pos_x'] = msg_data['pos_x']
					t['pos_y'] = msg_data['pos_y']
					t['dest_x'] = msg_data['dest_x']
					t['dest_y'] = msg_data['dest_y']
					t['comm'] = True

			try:
			    # Update distance from footprint information
			    if haveWaypoint == True:
				    distFromCenter =  nm.sqrt(((footprint_centroid[0] - t['pos_x']) ** 2) + \
							      ((footprint_centroid[1] - t['pos_y']) ** 2))
				    distFromBoundary = footprint_poly.boundary.distance( geometry.Point([t['pos_x'], t['pos_y']]))
				    distFromFootprint = footprint_poly.distance( geometry.Point([t['pos_x'], t['pos_y']]))
				    distFromFootprintRel = distFromBoundary if distFromFootprint != 0 else \
					    distFromBoundary * (-1)
				    t['distance'] = {'fromCenter' : distFromCenter,
						     'fromFootprint' : distFromFootprint,
						     'fromBoundary' : distFromBoundary,
						     'fromFootprintRel' : distFromFootprintRel,
				    }
				    #print ("------")
				    #print (footprint)
				    #print ("POS",  position)
				    #print ("TPOS", tpose)
				    #print ("X-deg", camera['xGimbal_deg'])
				    if t['distance']['fromFootprintRel'] + 4 > 0:
					    print ("%s is outside camera view" % t['name'])
					    reposition = True
				    #
				    else:
					    print ("%s is within camera view"  % t['name'])
				    print ("------")
			except:
			    continue

		if newWaypoint:

			#Get centroid of targets' positions
			centroid = calcCentroid (targets, "pos")

			#print ("centroid", centroid)


			# Get centroid of target's next waypoint positions
			waypoint_centroid = calcCentroid (targets, "way")

			targetsMaxMin = getMaxMinFromPosition (targets, centroid)
			furthestTarget = targetsMaxMin['farthest']

			# Align at stand-off distance
			# 1: l = Waypoint - Centroid
			l = nm.subtract (waypoint_centroid, centroid)
			# 2: Reverse l
			lrev = ( (-1) * l[0], (-1) * l[1] )
			# 3: Get angle of lrev
			theta = nm.arctan2 (lrev[1], lrev[0])
			if theta < 0:
				theta = theta * 2 * math.pi
			# 4: Go standoff-dist in lrev dir
			Nstraight = (2 + furthestTarget['value'], 0)
			N = rotatePoint ([0, 0], Nstraight, theta)
			# Newpoint = Centroid + N
			way = nm.add (N, centroid)


			# Set waypoint and point heading toward
			haveWaypoint = True
			if reposition == True or firstWaypoint == True:
				print("hi")
				quad['waypoint']['x'] = way[0]
				quad['waypoint']['y'] = way[1]
			else:
				quad['waypoint']['x'] = quad['waypoint']['x']
				quad['waypoint']['y'] = quad['waypoint']['y']
			quad['waypoint']['heading_x'] = centroid[0]
			quad['waypoint']['heading_y'] = centroid[1]


			#print (quad['waypoint'])
			# Send waypoint to quadcopter
			msgSend = quad['waypoint']
			msgSend['tag'] = 3
			msgSend_json = json.dumps (msgSend)
			r.publish (GCSconnStr, msgSend_json)

			firstWaypoint = False
			reposition = False


			# Set targets' centroid as camera target
			quad['target'] = {}
			quad['target']['x'] = quad['waypoint']['x']  #centroid[0]
			quad['target']['y'] = quad['waypoint']['y']  #centroid[1]

			# Send camera target to quadcopter
			msgSend = quad['target']
			msgSend['tag'] = 4
			msgSend_json = json.dumps (msgSend)
			r.publish (GCSconnStr, msgSend_json)

			# Estimate centroid speed
			if centroidPrev is not None:
				# Calc distance between centroids
				dist_m = ( ( centroidPrev[0] - centroid[0] ) ** 2 + ( centroidPrev[1] - centroid[1] ) ** 2 ) ** (0.5)
				# Calc time difference
				time_s = timestamp - centroidPrevTimestamp
				# Calc speed
				centroidSpeed_s = dist_m / time_s

			# Set current centroid as previous centroid
			centroidPrev = centroid
			centroidPrevTimestamp = timestamp


		# Reset control flags
		newWaypoint = False

		# Delay
		time.sleep (.001)



if __name__ == "__main__":
	main ()


