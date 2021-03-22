#!/usr/bin/env python

import rospy

import actionlib

import actionlib_tutorials.msg

class ScenarioManagerAction(object):
    # TODO: get messages for feedback and result once they're built
    _feedback = actionlib_tutorials.msg.FibonacciFeedback() # Feedback indicating current state when transitions happen
    _result = actionlib_tutorials.msg.FibonacciResult() # Result given when execution terminates

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, actionlib_tutorials.msg.FibonacciAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def execute_cb(self, goal):
        # redefine in child scenarios with actual execution
        if goal is True:
            # do stuff
            pass
        else:
            # handle 
