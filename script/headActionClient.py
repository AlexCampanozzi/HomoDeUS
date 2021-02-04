#! /usr/bin/env python

import os
import rospy
import roslib
import actionlib
import control_msgs.msg
import geometry_msgs
import math
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice

from std_msgs.msg import String

class HeadActionClient:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self):
        os.system("export ROS_MASTER_URI=http://10.68.0.1:11311")
        os.system("export ROS_IP=10.68.0.127")

        rospy.init_node('headAction', anonymous=False)
        self.client = actionlib.SimpleActionClient(
            "/head_controller/point_head_action", control_msgs.msg.PointHeadAction)

        rospy.Subscriber("chatter", String, self.callback)

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

        rospy.loginfo("init")
        
        # wait for the action server to come up
        while(not self.client.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the action server to come up")

    def GotoPosition(self, x, y):
        """
        This method sends a goal to the actionlib in order to move the robot's head
        x (float): The x position in the optical frame that the robot must reach
        y (float): The y position in the optical frame that the robot must reach
        """
        goal = control_msgs.msg.PointHeadGoal()

        cameraFrame = "/xtion_rgb_optical_frame"

        pointStamped = geometry_msgs.msg.PointStamped()

        pointStamped.header.frame_id = cameraFrame

        pointStamped.point.x = x
        pointStamped.point.y = y
        pointStamped.point.z = 1.0
        
        goal.pointing_frame = cameraFrame
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(1.0)
        goal.max_velocity = 0.25
        goal.target = pointStamped
        
        self.client.send_goal(goal)
        rospy.sleep(0.5)

    def GotoAngle(self, Ytheta, Xtheta):
        """
        This method converts angles value into position before using the GotoPosition function
        Ytheta (float): The x position in the optical frame that the robot must reach
        xTheta (float): The y position in the optical frame that the robot must reach
        """
        Ytheta = Ytheta*math.pi/180
        Xtheta = Xtheta*math.pi/180

        self.GotoPosition(math.tan(Ytheta), math.tan(Xtheta))

    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)