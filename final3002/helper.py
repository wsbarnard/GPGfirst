#!/usr/bin/env python

import rospy, tf, time, math

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D


def subscribeOriginal():
    orig = rospy.Subscriber("cmd_vel", Twist, origCB)

def origCB(data):
    global speed
    global angSpeed

    speed = data.linear.x
    angSpeed = data.angular.z




#publishes a twist message t othe cmd_vel_mux/input/teleop topic
def publishCopy():
    global speed
    global angSpeed
    global pub1
    pub1 = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 1)

    twist = Twist() 
    twist.linear.x = speed/3; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angSpeed
    pub1.publish(twist)


def publishOriginal():
    global speed
    global angSpeed
    global pub2
    pub2 = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    twist = Twist()
    twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angSpeed
    pub2.publish(twist)

if __name__ == '__main__':
    #define globals
    global speed
    global angSpeed
    global pub1
    global pub2
    speed = 0
    angSpeed = 0
 
    subscribeOriginal()
    rospy.init_node('wsbarnard_lab2_node')
    
    while(1):
        publishOriginal()
        publishCopy()
        
    rospy.spin()
    


    



