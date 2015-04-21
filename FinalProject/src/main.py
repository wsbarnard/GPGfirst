#!/usr/bin/env python

#General imports
import rospy
import tf
import time
import math

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
    global mapWidth

    mapWidth = msg.width

#
#Callback for map information
def mapCB(msg):
    global mapData
    global mapRes
    global mapOrigin
    global mapWidth

    resScale = 3 # must be an Int

    mapOrigin = msg.info.origin
    mapRes = resScale * msg.info.resolution
    print "originalRes: %f" % (msg.info.resolution)
    print "mapRes : %f" % (mapRes)
    rawData = msg.data 

    mapData = remakeMapData(rescale(rawData, resScale), resScale)
    mapWidth /= resScale

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

    # sets the global
    global x
    global y
    global theta

    x = px
    y = py
    theta = yaw

#Callback for robot position
def poseCB(msg):
    global robotPosex
    global robotPosey
    global mapRes

    tempx = round(msg.pose.pose.position.x, 3)
    tempy = round(msg.pose.pose.position.y, 3)

    robotPosex = tempx - (tempx%mapRes) 
    robotPosey = tempy - (tempy%mapRes)
    #print "Xpose: %f" % (robotPosex)
    #print "Ypose: %f" % (robotPosey)
    
#Callback for goal position
def goalPoseCB(msg):
    global goalPosex
    global goalPosey
    global mapRes

    tempx = msg.pose.position.x
    tempy = msg.pose.position.y

    goalPosex = round((tempx - (tempx%mapRes)),3)
    goalPosey = round((tempy - (tempy%mapRes)),3)


if __name__ == '__main__':
    rospy.init_node('gpg_lab3_node')

    global sub
    global greenCells
    global redCells
    global blueCells
    global purpleCells
    global robotPosex
    global robotPosey
    global goalPosex
    global goalPosey
    global mapData 
    global mapWidth
    global mapOrigin
    global nodeList

    #for drive
    global pub
    global pose
    global odom_tf
    global odom_list
    global theta
    print "Hellllo"

    i = 1

    robotPosex = 0
    robotPosey = 0
    goalPosex = 0
    goalPosey = 0
    mapWidth = 0
    mapData = []
    mapList = []
    greenCells = GridCells()
    redCells = GridCells()
    blueCells = GridCells()
    purpleCells = GridCells()

    #setup subscriptions
    rospy.sleep(1)
    subsSetup()
    rospy.sleep(0.5)

    #wait for start and goal to be defined
    while goalPosex == 0:
        pass
    rospy.sleep(1)

    #Do we still need this?
    x = robotPosex
    y = robotPosey
    
    #Create a list of nodes 
    mapList = makeGraph2()

    createCell(0.1, 0.1, "blueCells")

    updatedMapList = expandObstacles(mapList)
    finalPath = aStar2(updatedMapList)

    createCell(robotPosex, robotPosey, "redCells")
    for pathElement in finalPath:
        addCell(pathElement.x,pathElement.y, "redCells")
        rospy.sleep(0.1)

    loWP = waypoints(finalPath)
    print len(loWP)
    drivePath(loWP)
    publishTwist(0, 0)
            

    # sleep to allow initialization, spin waits for an event (btn press)
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()
