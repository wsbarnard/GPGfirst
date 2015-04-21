#!/usr/bin/env python
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
import rospy
import globalVars

#Create the first cell of given color, allow next cells to be added
def createCell(x, y, topic):
    pub = rospy.Publisher(topic, GridCells, queue_size = 10)

    array = [Point()]

    array[0].x = x
    array[0].y = y
    array[0].z = 0

    if topic == "purpleCells" :
        globalVars.purpleCells.header.frame_id = "map"
        globalVars.purpleCells.cell_width = globalVars.mapRes
        globalVars.purpleCells.cell_height = globalVars.mapRes
        globalVars.purpleCells.cells = array
        rospy.sleep(.3)
        pub.publish(globalVars.purpleCells)

    elif topic == "blueCells" :
        globalVars.blueCells.header.frame_id = "map"
        globalVars.blueCells.cell_width = globalVars.mapRes
        globalVars.blueCells.cell_height = globalVars.mapRes
        globalVars.blueCells.cells = array
        rospy.sleep(.3)
        pub.publish(globalVars.blueCells)

    elif topic == "redCells" :
        globalVars.redCells.header.frame_id = "map"
        globalVars.redCells.cell_width = globalVars.mapRes
        globalVars.redCells.cell_height = globalVars.mapRes
        globalVars.redCells.cells = array
        rospy.sleep(.3)
        pub.publish(globalVars.redCells)

    elif topic == "greenCells" :
        globalVars.greenCells.header.frame_id = "map"
        globalVars.greenCells.cell_width = globalVars.mapRes
        globalVars.greenCells.cell_height = globalVars.mapRes
        globalVars.greenCells.cells = array
        rospy.sleep(.3)
        pub.publish(globalVars.greenCells)

    else:
        print "***cell topic not defined in createCell()***"

#Add to existing cells of given color
def addCell(x, y, topic):
    pub = rospy.Publisher(topic, GridCells, queue_size = 10)
    newCell = Point()
    newCell.x = x
    newCell.y = y
    newCell.z = 0

    if topic == "greenCells" :
        globalVars.greenCells.cells.append(newCell)
        pub.publish(globalVars.greenCells)

    elif topic == "redCells" :
        globalVars.redCells.cells.append(newCell)
        pub.publish(globalVars.redCells)

    elif topic == "blueCells" :
        globalVars.blueCells.cells.append(newCell)
        pub.publish(globalVars.blueCells)

    elif topic == "purpleCells" :
        globalVars.purpleCells.cells.append(newCell)
        pub.publish(globalVars.purpleCells)

    else:
        print "***cell topic not defined in addCell()***"
