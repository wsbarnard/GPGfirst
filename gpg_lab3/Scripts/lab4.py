#!/usr/bin/env python


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
from kobuki_msgs.msg import BumperEvent


# for Searching
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

    if topic == "purpleCells" :
        purpleCells.header.frame_id = "map"
        purpleCells.cell_width = mapRes
        purpleCells.cell_height = mapRes
        purpleCells.cells = array
        rospy.sleep(.3)
        pub.publish(purpleCells)

    elif topic == "blueCells" :
        blueCells.header.frame_id = "map"
        blueCells.cell_width = mapRes
        blueCells.cell_height = mapRes
        blueCells.cells = array
        rospy.sleep(.3)
        pub.publish(blueCells)

    elif topic == "redCells" :
        redCells.header.frame_id = "map"
        redCells.cell_width = mapRes
        redCells.cell_height = mapRes
        redCells.cells = array
        rospy.sleep(.3)
        pub.publish(redCells)

    elif topic == "greenCells" :
        greenCells.header.frame_id = "map"
        greenCells.cell_width = mapRes
        greenCells.cell_height = mapRes
        greenCells.cells = array
        rospy.sleep(.3)
        pub.publish(greenCells)

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

    # print "width: %f" % (msg.width)

# def mapUpdateCB(msg):
#   print ""


def mapCB(msg):
    global mapData
    global mapRes
    global mapOrigin

    resScale = 3 # must be an Int

    mapOrigin = msg.info.origin
    mapRes = resScale * msg.info.resolution
    rawData = msg.data 

    mapData = rescale(rawData, resScale)


    print "nav_msgs/MapMetaData: %f" % (msg.info.resolution)

def getOdom():
    sub = rospy.Subscriber("odom", Odometry, OdomCallback)

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
    poleRate = .1
    tol = .1
    
    time.sleep(.5)

    # set the desired x y positions
    xGoal = x + dist * math.cos(theta) 
    yGoal = y + dist * math.sin(theta)

    #while not at goal 
    while((x < xGoal - tol or x > xGoal + tol) or 
            (y < yGoal - tol or y > yGoal + tol)):

        # keep driving forward
        publishTwist(speed, 0)
        print "x %f" % (x)
        print "y %f" % (y)

        # delay by poleRate
        time.sleep(poleRate)

    publishTwist(0, 0) # stop robot

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    poleRate = .1
    tol = .1
    
    time.sleep(.5)

    # set desired theta, if user pases a theta > 2pi or < -2pi, 
    # angle is moduloed to make put it in range of -2pi to 2pi
    angGoal = theta + (angle%(2*math.pi))

    # constrain further to -pi to pi because odom range is -pi to pi
    if angGoal >= math.pi:
        angGoal -= (2*math.pi)

    if angGoal < -math.pi:
        angGoal += (2*math.pi)

    #while not at goal 
    while(theta < angGoal - tol or theta > angGoal + tol):
        #spin direction based on sign of input
        if angle >= 0:
            publishTwist(0, .4)
        else:
            publishTwist(0, -.4)
        
        print "angGoal %f theta %f" % (angGoal, theta)

        time.sleep(poleRate)

    publishTwist(0, 0) # stop robot

def readBumper(msg):
    if (msg.state == 1):
        return true;


def rescale(rawListData, resScale):
    global mapRes
    global mapWidth
    rospy.sleep(0.01) 
    oldWidth = mapWidth
    mapWidth = mapWidth - (mapWidth % resScale)
    # print mapWidth
    clusterList = [] 

    rawList = []
    for e in range(len(rawListData)):
        # print e
        if (e%oldWidth <= mapWidth) and (e < mapWidth**2):
            rawList.append(rawListData[e])
            # print len(rawList)
            # print "added a thing"    
    #print len(rawList)
    #turn raw list into 2D array

    start = 0
    end = mapWidth

    list2D = []
    while end <= (mapWidth**2):
        list2D.append(rawList[start:end])
        start += mapWidth
        end += mapWidth

    ##PLEASE DO NOT DELETE THIS LINE
    #only comment and move. debugger for the win
    #import pdb; pdb.set_trace()
    #thing = slice(list2D, 6, 20)
    thing2 = getClusters(list2D, 5)
    print twoToOneD(thing2)
    #print list2D


 



    # for yInd in range (0, ((len(rawList)/(resScale**3))*(resScale*mapWidth)) + resScale, resScale*mapWidth):
    #     for xInd in range (0, ((len(rawList)/(resScale**3))*(resScale)) + resScale, resScale):
    #         clusterData = []
    #         for y in range (0, resScale):
    #             for x in range (0, resScale):
    #                 cell = (mapWidth*y) + x + xInd + yInd
    #                 # print "%d %d %d %d = %d" % (yInd, xInd, (y*mapWidth), x,cell)
    #                 # if (cell <= mapWidth**2):
    #                 clusterData.append(rawList[cell])

    #     clusterList.append(checkCluster(clusterData))
    #     print clusterData
    # print clusterList
    mapWidth /= resScale
    return clusterList

#makes clusters of given square dimension for the given 2d array
def getClusters(array, dim):
    start = 0
    end = dim

    clusters = []

    while end <= len(array):
        print start
        clusters.append(slice(array, start, end))
        start += dim
        end += dim

    print clusters
    return clusters

#turns a 2D array into a list
def twoToOneD(array):
    #j = len(array[0])
    j = 0
    finalList = []
    for i in range(len(array)):
        if j > len(array[0]):
            j = 0
        finalList.append(array[i])
        j += 1
 
    print finalList


#slices 2D array to given square
def slice(array, start, end):
    list = []
    for i in range(start,end):
        list.append(array[i][start:end])

    return list



def checkCluster(listOfCells):
    for cell in listOfCells:
        if cell > 0:
            return 100
    return 0


def poseCB(msg):
    global robotPosex
    global robotPosey

    tempx = round(msg.pose.pose.position.x, 1)
    tempy = round(msg.pose.pose.position.y, 1)

    robotPosex = tempx - (tempx%mapRes)
    robotPosey = tempy - (tempy%mapRes)
    print "Xpose: %f" % (robotPosex)
    print "Ypose: %f" % (robotPosey)
    



def goalPoseCB(msg):
    global goalPosex
    global goalPosey

    tempx = msg.pose.position.x
    tempy = msg.pose.position.y

    goalPosex = round((tempx - (tempx%mapRes)),1)
    goalPosey = round((tempy - (tempy%mapRes)),1)
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

def expandObstacles(graph):
    print "Expanding obstacles"
    expandedList = []
    listOfNodes = graph
    expandBy = 1
    obstacleList = getObstacles(listOfNodes)
    for node in obstacleList:
        neighborList = getNeighbors(node, listOfNodes)
        for neighborNode in neighborList:
            # print len(neighborList)
            if (neighborNode not in expandedList) and (isObstacle(neighborNode) == False):
                neighborNode.chance = 100 #this change is not being seen
                neighborNode.h = neighborNode.h + (1000)
                expandedList.append(neighborNode)
                addCell(neighborNode.x, neighborNode.y, "blueCells")
                rospy.sleep(0.011)
    
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
            addCell(node.x, node.y, "purpleCells")

    #print len(obstacleList)
    return obstacleList



def makeGraph2():
    global mapWidth
    global mapData
    global mapOrigin

    mapGraph = []
    
    xtemp = 0
    ytemp = 0
    # print len(mapData)
    for ind in range(len(mapData)):
        xMapCoord = round(((mapRes * ((ind % (mapWidth)))) + mapOrigin.position.x), 1) + 0.1
        yMapCoord = round(((mapRes * ((ind / mapWidth))) + mapOrigin.position.y), 1) + 0.1

        #print "temp %f, %f" %(xtemp, ytemp)
        
        # print "legit %f, %f" %(xMapCoord, yMapCoord)

        h = coordHeuristic(xMapCoord, yMapCoord) + (mapData[ind]*100) #make it very difficult to go through obstacle

        N = Node(xMapCoord, yMapCoord, h, mapData[ind])
        
        mapGraph.append(N)
    #for each in mapGraph:
        #print "%f, %f" %(each.x, each.y)
        
    return mapGraph

def aStar2(graph):
    print "Running A*"
    global robotPosex
    global robotPosey
    global goalPosex 
    global goalPosey
    global mapWidth

    frontier = []
    visited = []
    pathList = []

    currentNode = isStart(graph)
    # print "%f %f" % (currentNode.x, currentNode.y)
    createCell(robotPosex,robotPosey,"greenCells")

    frontier.append(currentNode)
    print "len of frontier at start of aStar2: %d" % (len(frontier))
    currentNode.est = currentNode.h
    while len(frontier) > 0:
        # print len(frontier)
        currentNode = findLowestCost(frontier)
        # print "%f %f" % (currentNode.x, currentNode.y)
     
        #if nothing works check tolerances
        xdif = currentNode.x - goalPosex
        ydif = currentNode.y - goalPosey
        # if (xdif < 0.1 and xdif > -0.1 and ydif < 0.1 and ydif > -0.1):
        #     return getPath(currentNode)
        if((round(currentNode.x) == round(goalPosex)) and (round(currentNode.y) == round(goalPosey))):
            # print "currentNode coords %f %f" % (currentNode.x, currentNode.y)
            # print "goal coords        %f %f" % (goalPosex, goalPosey)
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
            # print "%f %f" % (xdif, ydif)
            # if (xdif < 0.1 and xdif > -0.1 and ydif < 0.1 and ydif > -0.1):
            #     return node
            if (xdif == 0 and ydif == 0):
                print "Found the Start!!!!"
                return node

def waypoints(path):
    createCell(path[0].x,path[0].y,"purpleCells")
    x = 0
    for node in path:
        if node.parent == None:
            createCell(node.x,node.y,"purpleCells")
            continue
        if node.parent.parent == None:
            oldDeltaX = round((node.x - node.parent.x), 1)
            oldDeltaY = round((node.y - node.parent.y), 1)
            continue

        newDeltaX = round((node.x - node.parent.x), 1)
        newDeltaY = round((node.y - node.parent.y), 1)

        if (oldDeltaX != newDeltaX) or (oldDeltaY != newDeltaY):
            addCell(node.parent.x,node.parent.y,"purpleCells")
            rospy.sleep(0.1)

        oldDeltaX = newDeltaX
        oldDeltaY = newDeltaY 
        
        # print x
        # x += 1
    addCell(path[len(path)-1].x,path[len(path)-1].y,"purpleCells")

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
    global mapOrigin
    global nodeList

    #for drive
    #initialize x,y,theta
    x = 0
    y = 0
    theta = 0

    

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

    #for drive
    global pub
    global pose
    global odom_tf
    global odom_list
    global btn


    rospy.sleep(1)
    subsSetup()
    rospy.sleep(0.5)
    while goalPosex == 0:
        pass
    rospy.sleep(1)
    
    mapList = makeGraph2()
    print "len of mapList %f" %(len(mapList))
    createCell(0.1, 0.1, "blueCells")
    updatedMapList = expandObstacles(mapList)
    print "len of updatedMapList %f" %(len(updatedMapList))
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