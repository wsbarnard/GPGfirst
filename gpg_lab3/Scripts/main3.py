#!/usr/bin/env python


import rospy
import tf
import time
import math

from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid


def subsSetup():
    mapMeta = rospy.Subscriber("map_metadata", MapMetaData, metaDataCB)
    mapSub = rospy.Subscriber("map", OccupancyGrid, mapCB)


def metaDataCB(msg):
    print "width %f" % (msg.width)

# def mapUpdateCB(msg):
# 	print ""


def mapCB(msg):
    print "nav_msgs/MapMetaData: %f" % (msg.info.resolution)


if __name__ == '__main__':
    rospy.init_node('gpg_lab3_node')

    subsSetup()

# sleep to allow initialization, spin waits for an event (btn press)
    time.sleep(1)

    while not rospy.is_shutdown():
        rospy.spin()
