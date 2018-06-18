import math
import numpy as nm
import pylab as pl
import argparse
import re
from matplotlib.path import Path
import matplotlib.patches as patches
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

F = 1
A = 0
B = 0


def calcFieldOfView (camera):
    """
    Function: calcFieldOfView
    Arguments:
        camera: Camera object
    Purpose:
        Calculate the field of view (FoV) for a camera
        given the x and y sensor dimensions and the focal length
    """
    xView = 2 * math.atan (camera['xSensor_mm'] / (2 * camera['focallen_mm']))
    yView = 2 * math.atan (camera['ySensor_mm'] / (2 * camera['focallen_mm']))
    return (math.degrees (xView), math.degrees(yView))

def calcGroundFootprintDimensions (camera, altitude_m):
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
    global F
    FoV = [F, F]
    #print ("FoV", FoV)
    distFront = altitude_m * (math.tan(math.radians(camera['xGimbal_deg'] + 0.5 * FoV[0])))
    distBehind = altitude_m * (math.tan(math.radians(camera['xGimbal_deg'] - 0.5 * FoV[0])))
    distLeft = altitude_m * (math.tan(math.radians(camera['yGimbal_deg'] - 0.5 * FoV[1])))
    distRight = altitude_m * (math.tan(math.radians(camera['yGimbal_deg'] + 0.5 * FoV[1])))
    return (distFront, distBehind, distLeft, distRight)

def calcGroundFootprint (camera, altitude_m, position):
    """
    Function: calcGroundFootprint
    Arguments:
        camera: Camera object
        altitude_m: Altitude of camera in meters
        position: ground (x, y) coordinates of camera
    """
    (distFront, distBehind, distLeft, distRight) = calcGroundFootprintDimensions (camera, altitude_m)

    #print (distFront, distBehind, distLeft, distRight)
    #print ("Height", distFront - distBehind, "  Width", distLeft - distRight)
    posLowerLeft = (position[0] + distLeft, position[1] + distBehind)
    posLowerRight = (position[0] + distRight, position[1] + distBehind)
    posUpperLeft = (position[0] + distLeft, position[1] + distFront)
    posUpperRight = (position[0] + distRight, position[1] + distFront)
    return (posLowerLeft, posUpperLeft, posUpperRight, posLowerRight)



def rotatePoint(centerPoint,point,angle):
    """Rotates a point around another centerPoint. Angle is in degrees.
    Rotation is counter-clockwise"""
    # Source: https://gist.github.com/somada141/d81a05f172bb2df26a2c
    angle = angle
    temp_point = point[0]-centerPoint[0] , point[1]-centerPoint[1]
    temp_point = ( temp_point[0]*math.cos(angle)-temp_point[1]*math.sin(angle) , temp_point[0]*math.sin(angle)+temp_point[1]*math.cos(angle))
    temp_point = temp_point[0]+centerPoint[0] , temp_point[1]+centerPoint[1]
    return temp_point


def getFootprint (position, target, camera, altitude):

    # Calculate angle
    i = nm.subtract (target, position)
    theta = nm.arctan2(i[1], i[0])
    if (theta < 0):
        theta = theta + 2 * math.pi
    theta = theta - (math.pi / 2)
    #print (theta)

    footprint =  calcGroundFootprint (camera, altitude, position)
    footprint = [rotatePoint (position, footprint[0], theta),
             rotatePoint (position, footprint[1], theta),
             rotatePoint (position, footprint[2], theta),
             rotatePoint (position, footprint[3], theta)]
    #print(footprint)
    return footprint


altitude = 30
camera = {'xSensor_mm':30,
          'ySensor_mm':30,
          'focallen_mm':50,
          'xGimbal_deg':0,
          'yGimbal_deg':0
}
position = (0, 0)



def fitLinear(a, b, x):
    return a * x + b




def seq(start, stop, step=1):
    n = int(round((stop - start)/float(step)))
    if n > 1:
        return([start + step*i for i in range(n+1)])
    elif n == 1:
        return([start])
    else:
        return([])

global F
targetA1 = 13
targetA2 = -13
targetB1 = 42
targetB2 = 4
for x in seq(1, 1000, 0.01):
        
    F = x

    target   = (0, 20)
    camera['xGimbal_deg'] = 0
    footprintA = getFootprint (position, target, camera, altitude)
    
    target   = (0, 20)
    camera['xGimbal_deg'] = 31
    footprintB = getFootprint (position, target, camera, altitude)
    
    #print (footprintA[0], footprintA[1])
    #print (footprintB[2], footprintB[1])

    if footprintA[0][1] > -14 and \
       footprintA[0][1] < -12 and \
       footprintA[1][1] > 12  and \
       footprintA[1][1] < 14  and \
       footprintB[0][1] > 3   and \
       footprintB[0][1] < 5   and \
       footprintB[1][1] > 41  and \
       footprintB[1][1] < 43:

            print (F)
            print (footprintA)
            print (footprintB)
            print ("----")

#global A
#global B
#for x in range (0, 100):
#    for y in range (0, 100):
#        for w in range(0, 100):
#            for z in range(0, 100):
#
#                lla = fitLinear (0, x, y)
#	        ula = fitLinear (0, w, z)
#
#                llb = fitLinear (31, x, y)
#                ulb = fitLinear (31, w, z)
#
#                if lla > -14 - 10 and \
#                   lla < -12 + 10 and \
#                   ula > 13  - 10 and \
#                   ula < 14  + 10 and \
#                   llb > 3   - 10 and \
#                   llb < 5   + 10 and \
#                   ulb > 41  - 10 and \
#                   ulb < 43  + 10:
#
#                       print (x, y, z, w)
#                       print ( [lla, ula], [llb, ulb])
#   


#target   = (100, 120)
#camera['xGimbal_deg'] = 20
#print (target, camera['xGimbal_deg'])
#getFootprint (position, target, camera, altitude)
#print ("----")

#target   = (100, 120)
##camera['xGimbal_deg'] = 30
##print (target, camera['xGimbal_deg'])
##getFootprint (position, target, camera, altitude)
##print ("----")
##
##
##
##print ("\nSection 2\n")
##
##
##
##target   = (120, 100)
##camera['xGimbal_deg'] = 0
##print (target, camera['xGimbal_deg'])
##getFootprint (position, target, camera, altitude)
##print ("----")
##
##target   = (120, 100)
##camera['xGimbal_deg'] = 30
##print (target, camera['xGimbal_deg'])
##getFootprint (position, target, camera, altitude)
##print ("----")

#target   = (0, -10)
#print (target)
#getFootprint (position, target, camera, altitude)
#print ("----")
#
#target   = (-10, 0)
#print (target)
#getFootprint (position, target, camera, altitude)
#print ("----")
#
#print ("")
#
#target   = (10, 10)
#print (target)
#getFootprint (position, target, camera, altitude)
#print ("----")
#
#target   = (10, -10)
#print (target)
#getFootprint (position, target, camera, altitude)
#print ("----")
#
#target   = (-10, -10)
#print (target)
#getFootprint (position, target, camera, altitude)
#print ("----")
#
#target   = (-10, 10)
#print (target)
#getFootprint (position, target, camera, altitude)
#print ("----")
#
#


