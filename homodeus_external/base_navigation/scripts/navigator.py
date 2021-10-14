#!/usr/bin/env python2

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point, Pose
import tf_lookup.srv
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import json 
import os 
class Navigator:
    def __init__(self):
        self.landmarks = {}
        with open('/home/nickgb/catkin_ws/src/HomoDeUS/homodeus_common/landmarks.json') as json_file:
            self.landmarks = json.load(json_file)

        # define a client to send goal requests to the move_base server through a SimpleActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # wait for the action server to come up
        while(not self.ac.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the move_base action server to come up")

    def goto(self, xGoal, yGoal, oriGoal):
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

        self.gotoGoal(goal)

    def gotoGoal(self, goal):
        rospy.loginfo("Sending goal location ...")

        self.ac.send_goal(goal, self.gotoDoneCB)

        # Old way of doing it, we non-blocking now
        # self.ac.wait_for_result(rospy.Duration(60))

    def gotoDoneCB(self, state, result):
        # We do a bit of witchcraft here and call a method from the child class (HBBA_nav_listener)
        print("result" + str(result))
        if(state == GoalStatus.SUCCEEDED):
                rospy.loginfo("The robot reached the destination")
                self.doneCB(True)
        else:
                rospy.loginfo("The robot failed to reach the destination")
                self.doneCB(False)

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
        rospy.wait_for_service('/lookupTransform')
        lookup = rospy.ServiceProxy("/lookupTransform", tf_lookup.srv.lookupTransform)
        try:
            response = lookup("map", "base_footprint", rospy.Time())
        except rospy.ServiceException as exc:
            print("lookup did not process request: " + str(exc))
            raise rospy.ServiceException
        pose = Pose()
        pose.position = response.transform.transform.translation
        pose.orientation = response.transform.transform.rotation
        print(self.landmarks)
        return pose
