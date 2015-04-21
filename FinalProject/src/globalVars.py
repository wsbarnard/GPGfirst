#!/usr/bin/env python

from nav_msgs.msg import GridCells

global sub
global greenCells
global redCells
global blueCells
global purpleCells
global robotPoseX
global robotPoseY
global goalPoseX
global goalPoseY
global mapData 
global mapWidth
global mapOrigin
global nodeList
global mapRes


#for drive
global pub
global pose
global odom_tf
global odom_list
global theta

theta = 0
robotPoseX = 0
robotPoseY = 0
goalPoseX = 0
goalPoseY = 0
mapRes = 0
mapWidth = 0
mapData = []
mapList = []
greenCells = GridCells()
redCells = GridCells()
blueCells = GridCells()
purpleCells = GridCells()
