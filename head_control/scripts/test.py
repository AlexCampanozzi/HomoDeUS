#! /usr/bin/env python
import os
import rospy
import roslib
import actionlib
import control_msgs.msg
from geometry_msgs.msg import PoseStamped
import math
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice

if __name__ == "__main__":
    rospy.init_node('test', anonymous=False)
    pub = rospy.Publisher('tiago_head_controller', PoseStamped, queue_size=5)

    poseStamped = PoseStamped()


    poseStamped.pose.position.x = 30.0
    poseStamped.pose.position.y = 0

    rospy.sleep(0.1)
    pub.publish(poseStamped)
    rospy.spin()