#!/usr/bin/env python
import globalVars
import math
from structures import *
#given two coordinates, find the angle between them considering the robot's current orientation
def findTheta2(cur, nxt):
    deltax = round(nxt.x - cur.x, 3) 
    deltay = round(nxt.y - cur.y, 3)

    # desired angle in radian3
    angleInRad = round(math.atan2(deltay, deltax), 3)
    
    return (angleInRad - globalVars.theta)

#find the distance between (x1, y1) and (x2, y2)
def distance(x1, x2, y1, y2):
    #basic magnitude function
    distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
    #print distance
    return distance

#heuristic for a point to the goal
def calcHeuristic(point): 
    return ((point.x - globalVars.goalPoseX)**2 + (point.y - globalVars.goalPoseY)**2)**0.5

#straight line distance between two coordinates
def coordHeuristic(pointX, pointY):
    return ((pointX - globalVars.goalPoseX)**2 + (pointY - globalVars.goalPoseY)**2)**0.5

#Straight line distance between the coordinates of two nodes
def twoPointHeuristic(node1, node2):
    return ((node1.x - node2.x)**2 + (node1.y - node2.y)**2)**0.5

#Calculate the cost of a path to a node from the start
def costOfPath(node):
    path = getPath(node)
    cost = 0
    for node in path:
        if node.parent is not None:
            cost += twoPointHeuristic(node, node.parent)
    return cost

#Takes a list of nodes and returns the one with the lowest cost estimate  
def findLowestCost(fList):
    lowNode = Node(0, 0, 10000, 0)
    lowNode.est = 10000
    for e in fList:
        if e.est < lowNode.est:
            lowNode = e
            
    return lowNode



