#!/usr/bin/env python

#Create the first cell of given color, allow next cells to be added
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

#Add to existing cells of given color
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
