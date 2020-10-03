#!/usr/bin/env python

# =============================================================================
# Simple program that publishes geometry_msgs/PoseStamped messages on the
# /move_base_simple/goal topic to move the robot to a set point int the map
# =============================================================================

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
# from geometry_msgs.msg import PoseStamped


class Navigator:
    def __init__(self):
#        self.pose_msg = PoseStamped()
#        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.init_node('base_cmds', anonymous=False)

    def goto(self, xGoal, yGoal, oriGoal):

        # define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    
        # wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
          
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

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() == GoalStatus.SUCCEEDED):
                rospy.loginfo("The robot reached the destination")
                return True
        else:
                rospy.loginfo("The robot failed to reach the destination")
                return False


#        self.pose_msg.header.seq = 0  # Dunno what this is
#        self.pose_msg.header.stamp = rospy.get_rostime()  # Stamp current time
#        self.pose_msg.header.frame_id = "map"  # always navigating in map frame
#        self.pose_msg.pose.position.x = posX
#        self.pose_msg.pose.position.y = posY
#        self.pose_msg.pose.position.z = 0  # 2D: z is always 0
#        self.pose_msg.pose.orientation.x = 0  # 2D: always rotated only about z
#        self.pose_msg.pose.orientation.y = 0  # 2D: always rotated only about z
#        self.pose_msg.pose.orientation.z = 1  # 2D: always rotated only about z
#        self.pose_msg.pose.orientation.w = oriZ
#        self.pub.publish(self.pose_msg)

#def setter():
#    pose_msg = PoseStamped()
#    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
#    rospy.init_node('base_cmds', anonymous=True)
#    x = float(input("Enter desired x position: "))
#    y = float(input("Enter desired y position: "))
#    w = float(input("Enter desired orientation: "))
#    pose_msg.header.seq = 0  # Dunno what this is
#    pose_msg.header.stamp = rospy.get_rostime()  # Stamp current time
#    pose_msg.header.frame_id = "map"  # always navigating in map frame
#    pose_msg.pose.position.x = x
#    pose_msg.pose.position.y = y
#    pose_msg.pose.position.z = 0  # 2D: z is always 0
#    pose_msg.pose.orientation.x = 0  # 2D: always rotated only about z
#    pose_msg.pose.orientation.y = 0  # 2D: always rotated only about z
#    pose_msg.pose.orientation.z = 1  # 2D: always rotated only about z
#    pose_msg.pose.orientation.w = w
#    pub.publish(pose_msg)
#
#
#if __name__ == '__main__':
#    try:
#        setter()
#    except rospy.ROSInterruptException:
#        pass
