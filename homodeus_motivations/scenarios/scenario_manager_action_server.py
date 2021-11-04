#!/usr/bin/env python

import rospy

import actionlib

from custom_msgs.msg import scenario_managerFeedback, scenario_managerResult, scenario_managerAction

class ScenarioManagerAction(object):
    # TODO: get messages for feedback and result once they're built
    _feedback = scenario_managerFeedback() # Feedback indicating current state when transitions happen
    _result = scenario_managerResult() # Result given when execution terminates

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, scenario_managerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
