#!/usr/bin/env python


import rospy
import tf
import time
import math

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


def subsSetup():
    mapMeta = rospy.Subscriber("map_metadata", MapMetaData, metaDataCB)
    mapSub = rospy.Subscriber("map", OccupancyGrid, mapCB)
    robotPose = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, poseCB)
    goalPose = rospy.Subscriber("move_base_simple/goal", PoseStamped, goalPoseCB)



def createCell(x, y, topic):
    global greenCells
    global redCells
    global blueCells
    global purpleCells
    pub = rospy.Publisher(topic, GridCells, queue_size = 10)

    array = [Point()]

    array[0].x = x
    array[0].y = y
    array[0].z = 0

    if topic == "greenCells" :
        greenCells.header.frame_id = "map"
        greenCells.cell_width = .2
        greenCells.cell_height = .2
        greenCells.cells = array
        rospy.sleep(.3)
        pub.publish(greenCells)

    elif topic == "redCells" :
        redCells.header.frame_id = "map"
        redCells.cell_width = .2
        redCells.cell_height = .2
        redCells.cells = array
        rospy.sleep(.3)
        pub.publish(redCells)

    elif topic == "blueCells" :
        blueCells.header.frame_id = "map"
        blueCells.cell_width = .2
        blueCells.cell_height = .2
        blueCells.cells = array
        rospy.sleep(.3)
        pub.publish(blueCells)

    elif topic == "purpleCells" :
        purpleCells.header.frame_id = "map"
        purpleCells.cell_width = .2
        purpleCells.cell_height = .2
        purpleCells.cells = array
        rospy.sleep(.3)
        pub.publish(purpleCells)

    else:
        print "***cell topic not defined in createCell()***"


def addCell(x, y, topic):
    pub = rospy.Publisher(topic, GridCells, queue_size = 10)
    global greenCells
    global redCells
    global blueCells
    global purpleCells

    newCell = Point()
    newCell.x = x
    newCell.y = y
    newCell.z = 0

    if topic == "greenCells" :
        greenCells.cells.append(newCell)
        pub.publish(greenCells)

    elif topic == "redCells" :
        redCells.cells.append(newCell)
        pub.publish(redCells)

    elif topic == "blueCells" :
        blueCells.cells.append(newCell)
        pub.publish(blueCells)

    elif topic == "purpleCells" :
        purpleCells.cells.append(newCell)
        pub.publish(purpleCells)

    else:
        print "***cell topic not defined in addCell()***"

def metaDataCB(msg):
    global mapWidth

    mapWidth = msg.width

    print "width: %f" % (msg.width)

# def mapUpdateCB(msg):
#   print ""


def mapCB(msg):
    global mapData
    global mapRes
    global mapOrigin

    mapOrigin = msg.info.origin
    mapRes = msg.info.resolution
    mapData = msg.data 

    print "nav_msgs/MapMetaData: %f" % (msg.info.resolution)




def poseCB(msg):
    global robotPosex
    global robotPosey

    tempx = round(msg.pose.pose.position.x, 1)
    tempy = round(msg.pose.pose.position.y, 1)

    robotPosex = tempx - (tempx%0.2)
    robotPosey = tempy - (tempy%0.2)
    print "Xpose: %f" % (robotPosex)
    print "Ypose: %f" % (robotPosey)
    



def goalPoseCB(msg):
    global goalPosex
    global goalPosey

    tempx = msg.pose.position.x
    tempy = msg.pose.position.y

    goalPosex = round((tempx - (tempx%0.2)),1)
    goalPosey = round((tempy - (tempy%0.2)),1)
    print "XgoalPose %f" % (goalPosex)
    print "YgoalPose %f" % (goalPosey)


def calcHeuristic(point): 
    global robotPosex
    global robotPosey
    global goalPosex
    global goalPosey

    #DOUBLE CHECKTHIS was robotPosex and robotPosey
    #return abs(point.x - goalPosex) + abs(point.y - goalPosey)
    return ((point.x - goalPosex)**2 + (point.y - goalPosey)**2)**0.5

def coordHeuristic(pointX, pointY):
    global goalPosex
    global goalPosey
    return ((pointX - goalPosex)**2 + (pointY - goalPosey)**2)**0.5
    #return abs(pointX - goalPosex) + abs(pointY - goalPosey)

def twoPointHeuristic(node1, node2):
    return ((node1.x - node2.x)**2 + (node1.y - node2.y)**2)**0.5
    #return abs(node1.x - node2.x) + abs(node1.y - node2.y)

class Node:
    def __init__(self, x, y, h, chance): #removed neighbors.
        self.x = x
        self.y = y
        self.h = h
        self.chance = chance
        self.cost = 0
        self.est = 0
        self.parent = None

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            if(self.x == other.x) and (self.y == other.y):
                return True
            else:
                return False
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

def expandObstacles(listOfNodes):
    expandedList = []
    expandBy = 1
    obstacleList = getObstacles(listOfNodes)
    for node in obstacleList:
        neighborList = getNeighbors(node, listOfNodes)
        for neighborNode in neighborList:
            print len(neighborList)
            if (neighborNode not in expandedList) and (isObstacle(neighborNode) == False):
                neighborNode.chance = 100 #this change is not being seen
                expandedList.append(neighborNode)
                addCell(neighborNode.x, neighborNode.y, "purpleCells")
                rospy.sleep(0.05)
    #this was the ineffective test of the above issue
    # print len(expandedList)
    # for n in listOfNodes:
    #     if n in expandedList:
    #         n.chance = 100
    return listOfNodes

#takes a node and returnts true if it's an obstacle
def isObstacle(node):
    if node.chance > 0:
        return True
    else:
        return False

#takes all the obstacles found in isOBstacle and creates a list of them
def getObstacles(listofNodes):
    obstacleList = []
    for node in listofNodes:
        #print node.chance
        if isObstacle(node):
            #print "added"
            obstacleList.append(node) 
            #ddCell(neighborNode.x, neighborNode.y, "purpleCells")

    #print len(obstacleList)
    return obstacleList



def makeGraph2():
    global mapWidth
    global mapData
    global mapOrigin

    mapGraph = []
    
    xtemp = 0
    ytemp = 0

    for ind in range(len(mapData)):
        xMapCoord = round(((mapRes * ((ind % (mapWidth)))) + mapOrigin.position.x), 1) + 0.1
        yMapCoord = round(((mapRes * ((ind / mapWidth))) + mapOrigin.position.y), 1) + 0.1

        #print "temp %f, %f" %(xtemp, ytemp)
        
        # xMapCoord = float(xtemp) / 10
        # if xMapCoord > 0:
        #     xMapCoord += 0.1
        # yMapCoord = float(ytemp) / 10
        # if yMapCoord > 0:
        #     yMapCoord += 0.1
        # print "legit %f, %f" %(xMapCoord, yMapCoord)

        h = coordHeuristic(xMapCoord, yMapCoord) + (mapData[ind]*100) #make it very difficult to go through obstacle

        N = Node(xMapCoord, yMapCoord, h, mapData[ind])
        
        mapGraph.append(N)
    #for each in mapGraph:
        #print "%f, %f" %(each.x, each.y)
        
    return mapGraph

def aStar2(graph):
    global robotPosex
    global robotPosey
    global goalPosex 
    global goalPosey
    global mapWidth

    frontier = []
    visited = []
    pathList = []

    currentNode = isStart(graph)
    print "%f %f" % (currentNode.x, currentNode.y)
    createCell(robotPosex,robotPosey,"greenCells")

    frontier.append(currentNode)
    currentNode.est = currentNode.h
    while len(frontier) > 0:
        print len(frontier)
        currentNode = findLowestCost(frontier)
        print "%f %f" % (currentNode.x, currentNode.y)
     
        #if nothing works check tolerances
        xdif = currentNode.x - goalPosex
        ydif = currentNode.y - goalPosey
        # if (xdif < 0.1 and xdif > -0.1 and ydif < 0.1 and ydif > -0.1):
        #     return getPath(currentNode)
        if((round(currentNode.x) == round(goalPosex)) and (round(currentNode.y) == round(goalPosey))):
            print "currentNode coords %f %f" % (currentNode.x, currentNode.y)
            print "goal coords        %f %f" % (goalPosex, goalPosey)
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
                # print len(frontier)
                # print len(visited)
    print "A STAR DIDNT RETURN"

def costOfPath(node):
    path = getPath(node)
    cost = 0
    for node in path:
        if node.parent is not None:
            cost += twoPointHeuristic(node, node.parent)
    return cost

def isStart(listofNodes):
        global robotPosex
        global robotPosey

        for node in listofNodes:
            xdif = round((node.x - robotPosex), 1)
            ydif = round((node.y - robotPosey), 1)
            print "%f %f" % (xdif, ydif)
            # if (xdif < 0.1 and xdif > -0.1 and ydif < 0.1 and ydif > -0.1):
            #     return node
            if (xdif == 0 and ydif == 0):
                print "Found the Start!!!!"
                return node

def waypoints(path):
    createCell(path[0].x,path[0].y,"blueCells")
    x = 0
    for node in path:
        if node.parent == None:
            createCell(node.x,node.y,"blueCells")
            print "start cell made blue"
            continue
        if node.parent.parent == None:
            oldDeltaX = round((node.x - node.parent.x), 1)
            oldDeltaY = round((node.y - node.parent.y), 1)
            continue

        newDeltaX = round((node.x - node.parent.x), 1)
        newDeltaY = round((node.y - node.parent.y), 1)

        if (oldDeltaX != newDeltaX) or (oldDeltaY != newDeltaY):
            addCell(node.parent.x,node.parent.y,"blueCells")
            rospy.sleep(0.1)

        oldDeltaX = newDeltaX
        oldDeltaY = newDeltaY 
        
        # print x
        # x += 1
    addCell(path[len(path)-1].x,path[len(path)-1].y,"blueCells")

def getPath(current):
    finalPath = [current]
    while current.parent is not None:
        finalPath.append(current.parent)
        current = current.parent
    return finalPath
  
def findLowestCost(fList):
    lowNode = Node(0, 0, 10000, 0)
    lowNode.est = 10000
    for e in fList:
        if e.est < lowNode.est:
            lowNode = e
            
    return lowNode

def getNeighbors(ofNode, ofGraph):
    nList = []
    #print "START %f, %f" %(ofNode.x, ofNode.y)
    for n in ofGraph:
        if isNeighbor(n, ofNode):
            nList.append(n)
            if n.parent is not None:
                n.cost= n.parent.cost + twoPointHeuristic(n, n.parent)
            #print "%f, %f" %(n.x, n.y)
    return nList

def isNeighbor(node1, node2):
    diffx1 = node1.x - node2.x
    diffx2 = node2.x - node1.x
    diffy1 = node1.y - node2.y 
    diffy2 = node2.y - node1.y
    
    #if same coordinate, return 0
    if ((diffx1 > -0.1) and (diffx1 < 0.1)) and ((diffy1 > -0.1) and (diffy1 < 0.1)):
        return 0

    #if surrounding nodes, return 1
    if ((diffx1 > 0.1) and (diffx1 < 0.3)) or ((diffx2 > 0.1) and (diffx2 < 0.3)) or ((diffx1 > -0.1) and (diffx1 < 0.1)) :
        if ((diffy1 > 0.1) and (diffy1 < 0.3)) or ((diffy2 > 0.1) and (diffy2 < 0.3)) or ((diffy1 > -0.1) and (diffy1 < 0.1)):
            return 1
    # if (node1.x == node2.x and node1.y == node2.y):
    #     return 0

    # if ((node1.x == node2.x) and ((node1.y == (node2.y+0.2)) or (node1.y == (node2.y-0.2)))):
    #     print "neighbor found"
    #     return 1

    # if ((node1.y == node2.y) and ((node1.x == (node2.x+0.2)) or (node1.x == (node2.x-0.2)))):
    #     print "neighbor found"  
    #     return 1

    #if (diffx1 > 0.1) and (diffx1 < 0.3) 
    return 0 


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
    #global mapList
    global mapOrigin
    global nodeList

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


    rospy.sleep(1)
    subsSetup()
    rospy.sleep(0.5)
    while goalPosex == 0:
        pass
    rospy.sleep(1)
    
    mapList = makeGraph2()
    import pdb; pdb.set_trace()
    createCell(-3, -3, "purpleCells")
    updatedMapList = expandObstacles(mapList)
    #getObstacles(mapList)
    finalPath = aStar2(updatedMapList)


    #print len(finalPath)
    
    createCell(robotPosex, robotPosey, "redCells")
    for pathElement in finalPath:
        addCell(pathElement.x,pathElement.y, "redCells")
        rospy.sleep(0.1)

    waypoints(finalPath)
            

# sleep to allow initialization, spin waits for an event (btn press)
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()
