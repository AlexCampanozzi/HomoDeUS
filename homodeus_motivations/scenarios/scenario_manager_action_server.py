#!/usr/bin/env python

import rospy

import actionlib

import custom_msgs.msg

class ScenarioManagerAction(object):
    # TODO: get messages for feedback and result once they're built
    _feedback = custom_msgs.msg._scenario_managerFeedback # Feedback indicating current state when transitions happen
    _result = custom_msgs.msg._scenario_managerResult # Result given when execution terminates

    custom_msgs.msg.scenario_managerAction

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, custom_msgs.msg.scenario_managerAction, execute_cb=self.execute_cb, auto_start = False)
        print("komdkomngdf komngdkmp nd ")
        self._as.start()
