#! /usr/bin/env python

import roslib
import rospy
import actionlib

from HomoDeUS.msg import DoDishesAction, DoDishesGoal

if _name_ == '_main_':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('do_dishes', DoDishesAction)
    client.wait_for_server()

    goal = DoDishesGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))