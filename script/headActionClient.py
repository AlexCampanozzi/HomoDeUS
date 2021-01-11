#! /usr/bin/env python

import rospy
import roslib
import actionlib
import control_msgs.msg
import geometry_msgs
import math

class HeadActionClient:
    def __init__(self):
        rospy.init_node('headAction', anonymous=False)
        self.client = actionlib.SimpleActionClient("/head_controller/point_head_action", control_msgs.msg.PointHeadAction)
        rospy.loginfo("init")

        # wait for the action server to come up
        while(not self.client.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the action server to come up")

    def gotoPosition(self, x, y):
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

        def gotoAngle(self, Ytheta, Xtheta):
        goal = control_msgs.msg.PointHeadGoal()

        cameraFrame = "/xtion_rgb_optical_frame"

        pointStamped = geometry_msgs.msg.PointStamped()

        pointStamped.header.frame_id = cameraFrame
        Ytheta = Ytheta*math.pi/180
        Xtheta = Xtheta*math.pi/180
        pointStamped.point.x = math.tan(Ytheta)
        pointStamped.point.y = math.tan(Xtheta)
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

if __name__ == '__main__':

    client = HeadActionClient()
    client.gotoPosition()