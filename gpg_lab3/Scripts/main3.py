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

    tempx = msg.pose.pose.position.x
    tempy = msg.pose.pose.position.y

    robotPosex = (tempx) - (tempx%0.2)
    robotPosey = (tempy) - (tempy%0.2)
    print "Xpose: %f" % (robotPosex)
    print "Ypose: %f" % (robotPosey)
    



def goalPoseCB(msg):
    global goalPosex
    global goalPosey

    tempx = msg.pose.position.x
    tempy = msg.pose.position.y

    goalPosex = (tempx) - (tempx%0.2)
    goalPosey = (tempy) - (tempy%0.2)
    print "XgoalPose %f" % (goalPosex)
    print "YgoalPose %f" % (goalPosey)


def calcHeuristic(point): #point was previously not a thing...but now we can hero any point
    global robotPosex
    global robotPosey
    global goalPosex
    global goalPosey
    global hero 

    #DOUBLE CHECKTHIS was robotPosex and robotPosey
    hero = abs(point.x - goalPosex) + abs(point.y - goalPosey)
    print "Heuristic: %f" % (hero)

def coordHeuristic(pointX, pointY):
    global goalPosex
    global goalPoxey
    return float((int(abs(pointX - goalPosex) + abs(pointY - goalPosey)) * 10)) / 10

class Node:
    def __init__(self, x, y, h, chance): #removed neighbors.
        self.x = x
        self.y = y
        self.h = h
        self.chance = chance


def makeGraph():
    global mapWidth
    global mapData 
    global mapList
    global mapOrigin
    global nodeList

    nodeList = []
    mapList = []

    tempx = 0
    tempy = 0
    
    for ind in range(len(mapData)):
        # import pdb; pdb.set_trace()
        tempx = int((((mapRes * ((ind % (mapWidth)))) + mapOrigin.position.x + 0.1)) * 10)
        tempy = int((((mapRes * int((ind / mapWidth))) + mapOrigin.position.y + 0.1)) * 10)

        xPlace = float(tempx) / 10
        yPlace = float(tempy) / 10

        h = coordHeuristic(xPlace, yPlace) + (mapData[ind]*100) #make it very difficult to go t\hrough obstacle 
        N = Node(xPlace, yPlace, h, mapData[ind])
        # print "%f, %f"%(N.x, N.y)
        # print N.heuristic
        # print N.chance
        nodeList.append(N)
        #print nodeList

        i = 0
        if (ind % mapWidth) == mapWidth - 1:
            mapList.append(nodeList)
            nodeList = []
            i+1
        N.self = (i*(mapWidth-1))+ ind
        # print N.self

def makeGraph2():
    global mapWidth
    global mapData
    global mapOrigin

    mapGraph = []
    
    xtemp = 0
    ytemp = 0

    for ind in range(len(mapData)):
        xtemp = int((((mapRes * ((ind % (mapWidth)))) + mapOrigin.position.x)) * 10)
        ytemp = int((((mapRes * ((ind / mapWidth))) + mapOrigin.position.y)) * 10)

        #print "temp %f, %f" %(xtemp, ytemp)
        
        xMapCoord = float(xtemp) / 10
        if xMapCoord > 0:
            xMapCoord += 0.1
        yMapCoord = float(ytemp) / 10
        if yMapCoord > 0:
            yMapCoord += 0.1
        #print "legit %f, %f" %(xMapCoord, yMapCoord)

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
    costSoFar = 0
    currentNode = None
    startx = int(robotPosex / mapWidth)
    starty = int(robotPosey % mapWidth)

    currentNode = graph[starty][startx]
    frontier.append(currentNode)
    costSoFar += currentNode.h
    while frontier is not empty:
        currentNode = findLowestCost(frontier)
     
        #if nothing works check tolerances
        if(currentNode.x == goalPosex) and (currentNode.y == goalPosey):
            return pathList

        frontier.remove(currentNode)
        visited.append(currentNode)

        getNeighbors(currentNode, graph)
        
  
def findLowestCost(fList):
    lowNode = Node(0, 0, 10000, 0)
    for e in fList:
        if e.h < lowNode.h:
            lowNode = e
            
    return e

def getNeighbors(ofNode, ofGraph):
    nList = []
    print "START %f, %f" %(ofNode.x, ofNode.y)
    for n in ofGraph:
        if isNeighbor(n, ofNode):
            nList.append(n)
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

    #if (diffx1 > 0.1) and (diffx1 < 0.3) 
    return 0



def aStar(graph):
    global robotPosex
    global robotPosey
    global goalPosex
    global goalPosey
    global mapWidth

    frontier = []
    pathList = []
    visited = []
    costSoFar = 0
    startNode = 0
    startx = 0
    starty = 0

    print robotPosex
    print robotPosey

    for ind in range(mapWidth**2):
        currentNode = graph[ind%mapWidth][ind/mapWidth]
        if (currentNode.x == robotPosex) and (currentNode.y == robotPosey):
            startNode = currentNode
            rospy.sleep(.5)
            startx = ind/mapWidth
            rospy.sleep(.5)
            starty = ind%mapWidth
            # print "this happened, and then therapy"
            break
    # print "exited for loop"


    # frontier.append(startNode)

    #check heuristic, return one that has smallest total cost
    #add to path
    print "WTF A*"
    createCell(startNode.x, startNode.y, "redCells")

    # import pdb; pdb.set_trace()
    while (startNode.x != goalPosex) or (startNode.y != goalPosey): #CHANGE THAT TO SOMETHNG LEGIT
        print "dats what im talkin bout"
        visited.append(startNode)
        pathList.append(startNode)
        cost1 = 10000000
        cost2 = cost1
        cost3 = cost1
        cost4 = cost1

        if (startx + 1 < mapWidth) and (graph[starty][startx+1] not in visited):
            frontier.append(graph[starty][startx + 1])
            cost1 = graph[starty][startx + 1].h
        if (startx - 1 > 0) and (graph[starty][startx-1] not in visited):
            frontier.append(graph[starty][startx - 1])
            cost2 = graph[starty][startx - 1].h
        if (starty - 1 > 0) and (graph[starty - 1][startx] not in visited):
            frontier.append(graph[starty - 1][startx])
            cost3 = graph[starty - 1][startx].h
        if (starty + 1 < mapWidth) and (graph[starty +1][startx] not in visited):
            frontier.append(graph[starty + 1][startx])
            cost4 = graph[starty + 1][startx].h
        costSoFar += min(cost1, cost2, cost3, cost4)
        if costSoFar == cost1:
            startx=startx + 1
        elif costSoFar == cost2:
            startx=startx - 1
        elif costSoFar == cost3:
            starty=starty - 1
        elif costSoFar == cost4:
            starty=starty + 1
        # frontier.remove(startNode)
        addCell(startNode.x, startNode.y, "redCells")
        startNode = graph[starty][startx]
        rospy.sleep(.3)
        

    print "check path"
    rospy.sleep(.3)
    createCell(robotPosex,robotPosey,"greenCells")
    for ind in pathList:
        rospy.sleep(.3)
        addCell(ind.x,ind.y,"greenCells")
        print ind.self



      

if __name__ == '__main__':
    rospy.init_node('gpg_lab3_node')

    global sub
    global greenCells
    global redCells
    global robotPosex
    global robotPosey
    global goalPosex
    global goalPosey
    global hero
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
    rospy.sleep(1)
    #makeGraph2()
    rospy.sleep(0.5)
    while goalPosex == 0:
        pass
    rospy.sleep(1)
    #makeGraph()
   # aStar(mapList)
    #rospy.sleep(1)
 
    #print mapList
    mapList = makeGraph2()
    #aStar2(mapList)
    #for e in mapList:
        #print "%f, %f" %(e.x, e.y)
    print getNeighbors(Node(goalPosex, goalPosey, 100, 100), mapList)
    
    createCell(4.2,4.2, "redCells")
    rospy.sleep(0.3)
    addCell(-3, -3, "redCells")
    rospy.sleep(0.3)

    createCell(0, 0, "greenCells")
    rospy.sleep(0.1)
    addCell(1, 1, "greenCells")


# sleep to allow initialization, spin waits for an event (btn press)
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()
