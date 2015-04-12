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
    pub = rospy.Publisher(topic, GridCells, queue_size = 10)
    global greenCells
    global redCells

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

    if topic == "redCells" :
        redCells.header.frame_id = "map"
        redCells.cell_width = .2
        redCells.cell_height = .2
        redCells.cells = array
        rospy.sleep(.3)
        pub.publish(redCells)


def addCell(x, y, topic):
    pub = rospy.Publisher(topic, GridCells, queue_size = 10)
    global greenCells
    global redCells

    newCell = Point()
    newCell.x = x
    newCell.y = y
    newCell.z = 0

    if topic == "greenCells" :
        greenCells.cells.append(newCell)
        pub.publish(greenCells)

    if topic == "redCells" :
        redCells.cells.append(newCell)
        pub.publish(redCells)


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
    return abs(point.x - goalPosex) + abs(point.y - goalPosey)


def coordHeuristic(pointX, pointY):
    global goalPosex
    global goalPoxey
    # return float((int(abs(pointX - goalPosex) + abs(pointY - goalPosey)) * 10)) / 10
    return abs(pointX - goalPosex) + abs(pointY - goalPosey)

def twoPointHeuristic(node1, node2):
    return abs(node1.x - node2.x) + abs(node1.y - node2.y)

class Node:
    def __init__(self, x, y, h, chance): #removed neighbors.
        self.x = x
        self.y = y
        self.h = h
        self.chance = chance
        self.cost = 0
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
    createCell(robotPosex,robotPosey,"greenCells")

    frontier.append(currentNode)
    estCost = currentNode.h
    while len(frontier) > 0:
        currentNode = findLowestCost(frontier)
     
        #if nothing works check tolerances
        xdif = currentNode.x - goalPosex
        ydif = currentNode.y - goalPosey
        # if (xdif < 0.1 and xdif > -0.1 and ydif < 0.1 and ydif > -0.1):
        #     return getPath(currentNode)
        if((round(currentNode.x) == round(goalPosex)) and (round(currentNode.y) == round(goalPosey))):
            print "currentNode coords %f %f" % (currentNode.x, currentNode.y)
            print "goal coords        %f %f" % (goalPosex, goalPosey)
            return getPath(currentNode)
           
        frontier.remove(currentNode)
        visited.append(currentNode)

        currentNeighbors = getNeighbors(currentNode, graph)

        for neighbor in currentNeighbors:
            addCell(neighbor.x,neighbor.y, "greenCells")
            
            if (neighbor in visited):
                continue
            
            if (neighbor not in frontier) or (neighbor.cost< costOfPath(neighbor)):
                neighbor.parent = currentNode
                
                    
                frontier.append(neighbor)
                print len(frontier)
                print len(visited)
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
                print "this is a statement!!!!"
                return node

def getPath(current):
    finalPath = [current]
    while current.parent is not None:
        finalPath.append(current.parent)
        current = current.parent
    return finalPath
  
def findLowestCost(fList):
    lowNode = Node(0, 0, 10000, 0)
    for e in fList:
        if e.h < lowNode.h:
            lowNode = e
            
    return lowNode

def getNeighbors(ofNode, ofGraph):
    nList = []
    print "START %f, %f" %(ofNode.x, ofNode.y)
    for n in ofGraph:
        if isNeighbor(n, ofNode):
            nList.append(n)
            if n.parent is not None:
                n.cost= n.parent.cost + twoPointHeuristic(n, n.parent)
            print "%f, %f" %(n.x, n.y)
    return nList

def isNeighbor(node1, node2):
    diffx1 = node1.x - node2.x
    diffx2 = node2.x - node1.x
    diffy1 = node1.y - node2.y 
    diffy2 = node2.y - node1.y
    
    if ((diffx1 > -0.1) and (diffx1 < 0.1)) and ((diffy1 > -0.1) and (diffy1 < 0.1)):
        return 0

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
    global robotPosex
    global robotPosey
    global goalPosex
    global goalPosey
    global mapData 
    global mapWidth
    global mapList
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


    rospy.sleep(1)
    subsSetup()
    rospy.sleep(0.5)
    while goalPosex == 0:
        pass
    rospy.sleep(1)
    
    mapList = makeGraph2()
    finalPath = aStar2(mapList)

    print len(finalPath)
    
    createCell(robotPosex, robotPosey, "redCells")
    for pathElement in finalPath:
        addCell(pathElement.x,pathElement.y, "redCells")
        rospy.sleep(0.1)
            

# sleep to allow initialization, spin waits for an event (btn press)
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()
