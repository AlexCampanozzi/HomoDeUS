#!/usr/bin/env python2

import os 
import json 
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import HomoDeUS_common_py.HomoDeUS_common_py as common



class Navigator:
    def __init__(self):
        self.landmarks = {}
        landmarks_file = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))+'/homodeus_common/landmarks.json'

        with open(landmarks_file) as json_file:
            self.landmarks = json.load(json_file)

        # define a client to send goal requests to the move_base server through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the move_base action server to come up")

    def goto(self, xGoal, yGoal, oriGoal, blocking=False):
        goal = MoveBaseGoal()

        # set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time(0)

        goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
        quaternion = quaternion_from_euler(0, 0, oriGoal)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.gotoGoal(goal, blocking=blocking)

    def gotoGoal(self, goal, blocking=False):
        rospy.loginfo("Sending goal location ...")
        if blocking:
            self.ac.send_goal(goal)
            result = self.ac.wait_for_result(rospy.Duration(60))
            state = self.ac.get_state()
            return self.handles_result_state(state)
        else:
            self.ac.send_goal(goal, self.gotoDoneCB)
            return True

    def gotoDoneCB(self, state, result):
        # We do a bit of witchcraft here and call a method from the child class (HBBA_nav_listener)
        print("result" + str(result))
        self.handles_result_state(state)
    
    def handles_result_state(self,state):
        if(state == GoalStatus.SUCCEEDED):
                rospy.loginfo("The robot reached the destination")
                return True
        else:
                rospy.loginfo("The robot failed to reach the destination")
                return False

    def goalToLandmark(self, goal):
        goal = MoveBaseGoal()
        position = goal.target_pose.pose.position
        orientation = goal.target_pose.pose.orientation
        orientation  = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]
        landmark = [position.x, position.y, 0, 0, 0, orientation]
        return landmark

    def landmarkToGoal(self, landmark):
        x = landmark[0]
        y = landmark[1]
        yaw = landmark[5]

        goal  =  MoveBaseGoal()
        goal.target_pose.pose.position = Point(x, y, 0)
        orientation_list = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = orientation_list[0]
        goal.target_pose.pose.orientation.y = orientation_list[1]
        goal.target_pose.pose.orientation.z = orientation_list[2]
        goal.target_pose.pose.orientation.w = orientation_list[3]
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time(0)
        return goal

    def registerLandmark(self, name, x = None, y = None, yaw = None):
        landmarkGoal = MoveBaseGoal()
        if (x is None or y is None or yaw is None):
            curPose = self.getCurPose()
            landmarkGoal.target_pose.pose = curPose
            # For testing purposes
            # landmarkGoal.target_pose.pose.position.x += 0.0
            # landmarkGoal.target_pose.pose.position.y += 0.0
            # landmarkGoal.target_pose.pose.orientation.w += 3.1415
        else:
            landmarkGoal.target_pose.pose.position = Point(x, y, 0)
            orientation_list = quaternion_from_euler(0, 0, yaw)
            landmarkGoal.target_pose.pose.orientation.x = orientation_list[0]
            landmarkGoal.target_pose.pose.orientation.y = orientation_list[1]
            landmarkGoal.target_pose.pose.orientation.z = orientation_list[2]
            landmarkGoal.target_pose.pose.orientation.w = orientation_list[3]
        landmarkGoal.target_pose.header.frame_id = "map"
        #Time will have to be overwritten before actually sending the goal
        landmarkGoal.target_pose.header.stamp = rospy.Time(0)
        self.landmarks[name] = self.goalToLandmark(landmarkGoal)

    def gotoLandmark(self, name):
        if name not in self.landmarks:
            rospy.loginfo("Name does not correspond to any known landmark")
            return
        goal = self.landmarkToGoal(self.landmarks[name])

        goal.target_pose.header.stamp = rospy.Time(0)
        self.gotoGoal(goal)

    def cancelAllGoto(self):
        self.ac.cancel_all_goals()

    def getCurPose(self):
        return common.get_relative_pose('map', 'base_footprint')