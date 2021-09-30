#! /usr/bin/env python

import os
import rospy
import roslib
import actionlib
import control_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import math
from pal_startup_msgs.srv import StartupStart, StartupStop
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rosservice

from std_msgs.msg import String

class HeadActionClient:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self):
        rospy.Subscriber("tiago_head_controller", PoseStamped, self.callback)
        rospy.Subscriber("tiago_head_controller_home_reset", Empty, self.homeCB)
        self.pub_abs = rospy.Publisher("head_controller/command", JointTrajectory, queue_size=5)

        # Disabling the pal_head_manager to prevent unwanted head motion while moving the head
        service_list = rosservice.get_service_list()
        if '/pal_startup_control/stop' in service_list:
            try:
                rospy.wait_for_service('/pal_startup_control/stop', 2)
            except rospy.ROSException or rospy.ServiceException as e:
                rospy.logerr('Could not reach pal_startup_control/stop : %s', e.message)
            pal_stop = rospy.ServiceProxy('/pal_startup_control/stop', StartupStop)
            try:
                rospy.loginfo("disabling pal_head_manager.")
                pal_stop("head_manager")
            except rospy.ROSException and rospy.ServiceException as e:
                rospy.logerr('Could not stop head_manager: %s', e.message)

        rospy.loginfo("head_controller init")

    def GotoPosition(self, x, y, duration=1.):
        """
        This method publishes a command to move the robot head in absolute
        x (float): The x position in the absolute frame that the robot must reach
        y (float): The y position in the absolute frame that the robot must reach
        """
        cmd = JointTrajectory()
        cmd.joint_names = ["head_1_joint", "head_2_joint"]

        points = JointTrajectoryPoint()
        points.positions = [x,y]
        points.time_from_start = rospy.Duration(duration)
        cmd.points.append(points)

        rospy.sleep(0.1)
        self.pub_abs.publish(cmd)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.pose.position.y)

        self.GotoPosition(data.pose.position.x, data.pose.position.y)

    def homeCB(self, data):
        self.GotoPosition(0.0, 0.0)

if __name__ == '__main__':

    rospy.init_node('headAction', anonymous=False)
    client = HeadActionClient()
    client.GotoPosition(0, 0)
    rospy.spin()
