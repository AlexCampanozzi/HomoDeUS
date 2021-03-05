#! /usr/bin/env python

import rospy
import roslib
import actionlib
from geometry_msgs.msg import PoseStamped
import geometry_msgs

class FaceRecognition:

    def __init__(self):
        rospy.init_node('faceTracking', anonymous=False)

        pub = rospy.Publisher('tiago_head_controller', geometry_msgs.msg.PoseStamped, queue_size=None)
        rate = rospy.Rate(1) # 1hz

        while not rospy.is_shutdown():
            poseStamped = geometry_msgs.msg.PoseStamped()

            poseStamped.pose.position.x = 30.0

            rospy.loginfo("sent pose")
            pub.publish(poseStamped)
            rate.sleep()

if __name__ == "__main__":

    try:
        faceRecognition = FaceRecognition()

    except rospy.ROSInterruptException:
        pass