#!/usr/bin/env python
import rospy
import math
import time
import globalVars

from geometry_msgs.msg import Twist
from calculate import *

#publishes a twist message t othe cmd_vel_mux/input/teleop topic
def publishTwist(speed, angSpeed):
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)
    
    twist = Twist() 
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angSpeed
    pub.publish(twist)

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, t):
    r = 0.035 # wheel radius
    b = 0.23 # distance between wheels

    #calculates speed and angSpeed  
    speed = (r/2)*(u1 + u2)
    angSpeed = (r/b)*(u1 - u2)

    start = time.time()

    #publish until stop time
    while(time.time() - start < t):
        publishTwist(speed, angSpeed)

    publishTwist(0, 0) #stop when done

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, dist):
    poleRate = .05
    tol = .2
    
    rospy.sleep(.5)

    # set the desired x y positions
    xGoal = x + dist * math.cos(theta) 
    yGoal = y + dist * math.sin(theta)

    #while not at goal 
    while((x < xGoal - tol or x > xGoal + tol) or 
            (y < yGoal - tol or y > yGoal + tol)):

        # keep driving forward
        publishTwist(speed, 0)

        # delay by poleRate
        rospy.sleep(poleRate)

    publishTwist(0, 0) # stop robot

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    poleRate = .1
    tol = .1
    
    time.sleep(.5)
    print angle

    # set desired globalVars.theta, if user pases a globalVars.theta > 2pi or < -2pi, 
    # angle is moduloed to make put it in range of -2pi to 2pi
    angGoal = globalVars.theta + (angle%(2*math.pi))

    # constrain further to -pi to pi because odom range is -pi to pi
    if angGoal >= math.pi:
        angGoal -= (2*math.pi)

    if angGoal < -math.pi:
        angGoal += (2*math.pi)

    #while not at goal 
    while(globalVars.theta < angGoal - tol or globalVars.theta > angGoal + tol):
        #spin direction based on sign of input
        if angle >= 0:
            publishTwist(0, .4)
        else:
            publishTwist(0, -.4)

        time.sleep(poleRate)

    publishTwist(0, 0) # stop robot


#drive from node to node, not from waypoint to waypoint
def drivePath (waypointList):
    #start coordinates = robot coordinates
    prev = 0
    cur = 0
    nxt = cur + 1
    print "waypoint list %f" %(len(waypointList))
    #move from node to node 
    #use indexing - for each integer n in the list of waypoints ranging from 
    while cur < len(waypointList):
        print "nxt begin %f" % (nxt)
        desiredTheta = findTheta2(waypointList[cur], waypointList[nxt])
        rotate(desiredTheta)
        print "rotated"
        driveStraight(0.25, distance(waypointList[cur].x, waypointList[nxt].x, waypointList[cur].y, waypointList[nxt].y))
        print "drove straight"

        cur += 1
        nxt += 1
        if cur >= 1:
            prev += 1
        print "done with path segment"
        print "nxt end %f" % (nxt)
        publishTwist(0,0)
    print "dive path finished"
    publishTwist(0,0)
