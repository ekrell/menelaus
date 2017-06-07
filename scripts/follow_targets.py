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
		points = [(t['pos_x'], t['pos_y']) for t in targets]
	elif posORway == "way":
		points = [(t['dest_x'], t['dest_y']) for t in targets]
	x, y = zip (*points)
	l = len (x)
	return sum (x) / l, sum (y) / l

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


def main ():
	
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
	quad = { 'name':quadName, 'connStr':quadConnStr, 'waypoint':{}  }

	# Init targets
	for name in targetNames:
		targetConnStr = 'Morse-Marisa-' + name
		target = {'name':name, 'connStr':targetConnStr}
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

	# Tell quadcopter to start recording with video camera
	quadMSG = { 'tag':4 }
	quadMSG_json = json.dumps (quadMSG)
	r.publish (GCSconnStr, quadMSG_json)

	# Main Loop 
	while (len (targets) > 0):  # As long as targets remain
		
		# Control flags	
		newWaypoint = False	

		# World Model
		centroidPrev = None
		centroidPrevTimestamp = None
		centroidSpeed_s = None

		# Update quadcopter status
		msg = p.get_message ()
		if msg:
			print (msg)

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

		if newWaypoint:
			
			#Get centroid of targets' positions
			centroid = calcCentroid (targets, "pos")
			
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

			# Set waypoint and point to head toward
			quad['waypoint']['x'] = way[0]
			quad['waypoint']['y'] = way[1]
			quad['waypoint']['heading_x'] = waypoint_centroid[0]
			quad['waypoint']['heading_y'] = waypoint_centroid[1]
						
			# Send waypoint to quadcopter
			msgSend = quad['waypoint']
			msgSend['tag'] = 3
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
		time.sleep (.5)





if __name__ == "__main__":
	main ()


