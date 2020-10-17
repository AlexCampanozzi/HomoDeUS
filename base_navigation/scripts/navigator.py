#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Transform
# import tf
import tf_lookup.srv

class Navigator:
    def __init__(self):
        rospy.init_node('base_cmds', anonymous=False)
        self.landmarks = {}
        
        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")

    def goto(self, xGoal, yGoal, oriGoal):
        goal = MoveBaseGoal()

        # set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 1
        goal.target_pose.pose.orientation.w = oriGoal

        self.gotoGoal(goal)

    def gotoGoal(self, goal):
        rospy.loginfo("Sending goal location ...")
        self.ac.send_goal(goal)

        self.ac.wait_for_result(rospy.Duration(60))

        if(self.ac.get_state() == GoalStatus.SUCCEEDED):
                rospy.loginfo("The robot reached the destination")
                return True
        else:
                rospy.loginfo("The robot failed to reach the destination")
                return False

    def registerLandmark(self, name, x = None, y = None, w = None):
        landmarkGoal = MoveBaseGoal()
        if (x is None or y is None or w is None):
            rospy.wait_for_service('/lookupTransform')
            lookup = rospy.ServiceProxy("/lookupTransform", tf_lookup.srv.lookupTransform)
            try:
                response = lookup("map", "base_footprint", rospy.Time())
            except rospy.ServiceException as exc:
                print("lookup did not process request: " + str(exc))
            landmarkGoal.target_pose.pose.position = response.transform.transform.translation
            landmarkGoal.target_pose.pose.orientation = response.transform.transform.rotation
        else:
            landmarkGoal.target_pose.pose.position = Point(x, y, 0)
            landmarkGoal.target_pose.pose.orientation.x = 0.0
            landmarkGoal.target_pose.pose.orientation.y = 0.0
            landmarkGoal.target_pose.pose.orientation.z = 1
            landmarkGoal.target_pose.pose.orientation.w = w
        
        landmarkGoal.target_pose.header.frame_id = "map"
        #Time will have to be overwritten before actually sending the goal
        landmarkGoal.target_pose.header.stamp = rospy.Time.now()
        print landmarkGoal
        self.landmarks[name] = landmarkGoal

    def goToLandmark(self, name):
        if name not in self.landmarks:
            rospy.loginfo("Name does not correspond to any known landmark")
            return
        goal = self.landmarks[name]
        goal.target_pose.header.stamp = rospy.Time.now()
        self.gotoGoal(goal)

    def cancelAllGoto(self):
        self.ac.cancel_all_goals()
        self.ac.wait_for_result
