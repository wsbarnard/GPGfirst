#!/usr/bin/env python

import globalVars

from cells import *
from calculate import *


#Calculate the path to the goal using A*
def aStar2(graph):
    print "Running A*"

    frontier = []
    visited = []
    pathList = []

    #find the start node 
    currentNode = isStart(graph)
   
    print "%f %f" % (globalVars.robotPoseX, globalVars.robotPoseY)
    print "%f %f" % (currentNode.x, currentNode.y)

    createCell(globalVars.robotPoseX,globalVars.robotPoseY,"greenCells")

    frontier.append(currentNode)
   
    currentNode.est = currentNode.h
    while len(frontier) > 0:
   
        currentNode = findLowestCost(frontier)
       
        xdif = currentNode.x - globalVars.goalPoseX
        ydif = currentNode.y - globalVars.goalPoseY
       
        if((currentNode.x > globalVars.goalPoseX - globalVars.mapRes) and (currentNode.x < globalVars.goalPoseX + globalVars.mapRes) and (currentNode.y > globalVars.goalPoseY - globalVars.mapRes) and (currentNode.y < globalVars.goalPoseY + globalVars.mapRes)):
       
            aStarPath = getPath(currentNode)
            aStarPath.reverse()
            return aStarPath
           
        frontier.remove(currentNode)
        visited.append(currentNode)
        currentNeighbors = getNeighbors(currentNode, graph)

        for neighbor in currentNeighbors:
            addCell(neighbor.x,neighbor.y, "greenCells")
            tentativeCostSoFar = currentNode.cost + twoPointHeuristic(neighbor, currentNode)
            tentativeEstCost = tentativeCostSoFar + neighbor.h
            if (neighbor in visited) and (tentativeEstCost >= (neighbor.h + tentativeCostSoFar)):
                continue
            
            if (neighbor not in frontier) or (tentativeEstCost < (neighbor.h + tentativeCostSoFar)):
                neighbor.parent = currentNode
                neighbor.cost = tentativeCostSoFar
                neighbor.est = tentativeEstCost 
                frontier.append(neighbor)
       
    print "A STAR DIDNT RETURN"

#Finds the start node in the list of nodes
def isStart(listofNodes):
        print len(listofNodes)
        for node in listofNodes:
            xdif = round((node.x - globalVars.robotPoseX), 3)
            ydif = round((node.y - globalVars.robotPoseY), 3)
            if ((xdif > -globalVars.mapRes) and (xdif < globalVars.mapRes) and (ydif > -globalVars.mapRes) and (ydif < globalVars.mapRes)):
                print "Found the Start!!!!"
                return node

#Create waypoints from the A* path based off when the robot changes direction
def waypoints(path):
    listOfWP = []
    createCell(path[0].x,path[0].y,"purpleCells")
    listOfWP.append(Coord(path[0].x, path[0].y))
    x = 0
    for node in path:
        if node.parent == None:
            createCell(node.x,node.y,"purpleCells")
            continue
        if node.parent.parent == None:
            oldDeltaX = round((node.x - node.parent.x), 3)
            oldDeltaY = round((node.y - node.parent.y), 3)
            continue

        newDeltaX = round((node.x - node.parent.x), 3)
        newDeltaY = round((node.y - node.parent.y), 3)

        if (oldDeltaX != newDeltaX) or (oldDeltaY != newDeltaY):
            addCell(node.parent.x,node.parent.y,"purpleCells")
            listOfWP.append(Coord(node.parent.x, node.parent.y))
            rospy.sleep(0.1)

        oldDeltaX = newDeltaX
        oldDeltaY = newDeltaY 
 
    listOfWP.append(Coord(len(path)-1, len(path)-1))
    addCell(path[len(path)-1].x,path[len(path)-1].y,"purpleCells")
    return listOfWP

#Create the path based off the start and goal nodes, tracking through the parents
def getPath(current):
    finalPath = [current]
    while current.parent is not None:
        finalPath.append(current.parent)
        current = current.parent
    return finalPath



def getNeighbors(ofNode, ofGraph):
    nList = []

    for n in ofGraph:
        if isNeighbor(n, ofNode):
            nList.append(n)
            if n.parent is not None:
                n.cost= n.parent.cost + twoPointHeuristic(n, n.parent)

    return nList

def isNeighbor(node1, node2):
    diffx1 = node1.x - node2.x
    diffx2 = node2.x - node1.x
    diffy1 = node1.y - node2.y 
    diffy2 = node2.y - node1.y
    
    #if same coordinate, return 0
    if ((diffx1 > -(globalVars.mapRes/2)) and (diffx1 < (globalVars.mapRes/2))) and ((diffy1 > -(globalVars.mapRes/2)) and (diffy1 < (globalVars.mapRes/2))):
        return 0

    #if surrounding nodes, return 1
    if ((diffx1 > (globalVars.mapRes/2)) and (diffx1 < (3*globalVars.mapRes/2))) or ((diffx2 > (globalVars.mapRes/2)) and (diffx2 < (3*globalVars.mapRes/2))) or ((diffx1 > -(globalVars.mapRes/2)) and (diffx1 < (globalVars.mapRes/2))) :
        if ((diffy1 > (globalVars.mapRes/2)) and (diffy1 < (3*globalVars.mapRes/2))) or ((diffy2 > (globalVars.mapRes/2)) and (diffy2 < (3*globalVars.mapRes/2))) or ((diffy1 > -(globalVars.mapRes/2)) and (diffy1 < (globalVars.mapRes/2))):
            return 1
    return 0 
