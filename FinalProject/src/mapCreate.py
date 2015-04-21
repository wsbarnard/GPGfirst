#!/usr/bin/env python
import rospy
import time
import globalVars
from calculate import *
from cells import *
from structures import *
from aStar import *

#Creates a graph of nodes containing the obstacle information from the updated map data
def makeGraph2():
    mapGraph = []
    
    xtemp = 0
    ytemp = 0
    
    for ind in range(len(globalVars.mapData)):
        xMapCoord = round(((globalVars.mapRes * ((ind % (globalVars.mapWidth)))) + globalVars.mapOrigin.position.x) + .1, 3)
        yMapCoord = round(((globalVars.mapRes * ((ind / globalVars.mapWidth))) + globalVars.mapOrigin.position.y) + .1 , 3) 
        h = coordHeuristic(xMapCoord, yMapCoord) + (globalVars.mapData[ind]*100) #make it very difficult to go through obstacle

        N = Node(xMapCoord, yMapCoord, h, globalVars.mapData[ind])
        
        mapGraph.append(N)
   
    return mapGraph

#rescale map data to given scale
def rescale(rawListData, resScale):
    rospy.sleep(0.01) 
    oldWidth = globalVars.mapWidth
    globalVars.mapWidth = globalVars.mapWidth - (globalVars.mapWidth % resScale)
    clusterList = [] 

    rawList = []
    for e in range(len(rawListData)):

        if (e % oldWidth <= globalVars.mapWidth) and (e < globalVars.mapWidth**2):
            rawList.append(rawListData[e])

    start = 0
    end = globalVars.mapWidth

    list2D = []
    while end <= (globalVars.mapWidth**2):
        list2D.append(rawList[start:end])
        start += globalVars.mapWidth
        end += globalVars.mapWidth

    clusterList = getClusters(list2D, resScale)
    
    return clusterList

#Returns list of wall map data from a list of clusters
def remakeMapData(clusterList, res):
    newList = []
    for cluster in clusterList:
        cluster1D = twoToOneD(cluster, res, res)

        newList.append(checkCluster(cluster1D))

    return newList



#makes clusters of given square dimension for the given 2d array
def getClusters(array, dim):
    start = 0
    end = dim

    clusters = []

    for start in range(0, len(array), dim):
        for end in range(0, len(array), dim):
            clusters.append(slice(array, start, end, dim))

    #print clusters
    return clusters

#turns a 2D array into a list
def twoToOneD(array, xdim, ydim):
    finalList = []

    for i in range(ydim):
        for j in range(xdim):
            # print array[i][j]
            finalList.append(array[i][j])
 
    return finalList


#slices 2D array to given square
def slice(array, startx, starty, end):
    list = []
    for i in range(startx, startx + end):
        list.append(array[i][starty:starty + end])

    return list


#Takes a list of cells and returns 100 if there is should be an obstacle, 0 if not
def checkCluster(listOfCells):
    if len(filter(lambda x: x == 100, listOfCells)) > 0:
        return 100
    return 0

#expand obstacles by 1 in each direction. Drawn in blue
def expandObstacles(graph):
    print "Expanding obstacles"
    expandedList = []
    listOfNodes = graph
    expandBy = 1
    obstacleList = getObstacles(listOfNodes)
    for node in obstacleList:
        neighborList = getNeighbors(node, listOfNodes)
        for neighborNode in neighborList:

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
        
        if isObstacle(node):
            obstacleList.append(node) 
            #questionable because in expandObstacles we add blue cells for expansion... 
            addCell(node.x, node.y, "purpleCells")

    return obstacleList

