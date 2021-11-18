#! /usr/bin/env python

import os
import math
import rospy
import roslib
import actionlib
import rosservice
import control_msgs.msg
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped
from pal_startup_msgs.srv import StartupStart, StartupStop
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint





class headController:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self, head_control_topic='/head_controller/command'):
        #So it can be accessed from topic or used as an object
        rospy.Subscriber("tiago_head_controller", PoseStamped, self.callback)
        rospy.Subscriber("head_home_reset", Empty, self.homeCB)
        
        self.pub_abs = rospy.Publisher(head_control_topic, JointTrajectory, queue_size=5)
        
        # Disabling the pal_head_manager to prevent unwanted head motion while moving the head
        self.startup_verification()

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
        self.pub_abs.publish(cmd)

    def callback(self, data):
        self.GotoPosition(data.pose.position.x, data.pose.position.y)

    def homeCB(self, data):
        self.GotoPosition(0.0, 0.0)

    def startup_verification(self):
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

if __name__ == '__main__':

    rospy.init_node('headAction', anonymous=False)
    client = headController()
    rospy.logwarn("=================== head_controller action =====================")
    rospy.spin()
