#!/usr/bin/env python

##PLEASE DO NOT DELETE THIS LINE
#only comment and move. debugger for the win
#import pdb; pdb.set_trace()
   

#General imports
import rospy
import tf
import time
import math

#other files
from mapCreate import *
from drive import *
from aStar import *

import globalVars

# for driving
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import degrees

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D

# for Searching
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

#Calls subscriptions, run in main
def subsSetup():
    mapMeta = rospy.Subscriber("map_metadata", MapMetaData, metaDataCB)
    mapSub = rospy.Subscriber("map", OccupancyGrid, mapCB)
    robotPose = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, poseCB)
    goalPose = rospy.Subscriber("move_base_simple/goal", PoseStamped, goalPoseCB)
    getOdom()

#Callback to get width
def metaDataCB(msg):
    globalVars.mapWidth = msg.width

#Callback for map information
def mapCB(msg):
    resScale = 3 # must be an Int

    globalVars.mapOrigin = msg.info.origin
    globalVars.mapRes = resScale * msg.info.resolution
    print "originalRes: %f" % (msg.info.resolution)
    print "globalVars.mapRes : %f" % (globalVars.mapRes)
    rawData = msg.data 

    globalVars.mapData = remakeMapData(rescale(rawData, resScale), resScale)
    globalVars.mapWidth /= resScale

#Subscriber for odometry
def getOdom():
    sub = rospy.Subscriber("odom", Odometry, OdomCallback)

#Callback function for odometry
def OdomCallback(data):
    px = data.pose.pose.position.x # gets odom x
    py = data.pose.pose.position.y # gets odom y
    quat = data.pose.pose.orientation # gets odom theta
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw, = euler_from_quaternion(q)

    x = px
    y = py
    globalVars.theta = yaw

#Callback for robot position
def poseCB(msg):
    tempx = round(msg.pose.pose.position.x, 3)
    tempy = round(msg.pose.pose.position.y, 3)

    globalVars.robotPoseX = tempx - (tempx % globalVars.mapRes) 
    globalVars.robotPoseY = tempy - (tempy % globalVars.mapRes)
    
#Callback for goal position
def goalPoseCB(msg):
    tempx = msg.pose.position.x
    tempy = msg.pose.position.y

    globalVars.goalPoseX = round((tempx - (tempx % globalVars.mapRes)),3)
    globalVars.goalPoseY = round((tempy - (tempy % globalVars.mapRes)),3)


if __name__ == '__main__':
    rospy.init_node('FinalProject')
    print "Hellllo"

    i = 1

    #setup subscriptions
    rospy.sleep(1)
    subsSetup()
    rospy.sleep(0.5)

    #wait for start and goal to be defined
    while globalVars.goalPoseX == 0:
        pass
    rospy.sleep(1)

    #Do we still need this?
    x = globalVars.robotPoseX
    y = globalVars.robotPoseY
    
    #Create a list of nodes 
    mapList = makeGraph2()

    createCell(0.1, 0.1, "blueCells")

    updatedMapList = expandObstacles(mapList)
    finalPath = aStar2(updatedMapList)

    createCell(globalVars.robotPoseX, globalVars.robotPoseY, "redCells")
    for pathElement in finalPath:
        addCell(pathElement.x,pathElement.y, "redCells")
        rospy.sleep(0.1)

    loWP = waypoints(finalPath)

    drivePath(loWP)
    publishTwist(0, 0)
            
    while not rospy.is_shutdown():
        rospy.spin()
