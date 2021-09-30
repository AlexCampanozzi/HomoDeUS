#!/usr/bin/env python

# A simple module to publish events based on the results of GoTo actions

import rospy
import actionlib
from yaml import safe_load
# from json import loads
# from ast import literal_eval
from hbba_msgs.msg import Desire, DesiresSet, Event
from custom_msgs.msg import GoToResult

def equalWithinTolerance(a, b, tol):
    return abs(a-b) <= tol

class GoToLandmarkResultObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenGoToResult(self):
        self.goToResultSubscriber = rospy.Subscriber("bhvr_output_res_nav_result", GoToResult, self.listenGoToResultCB, queue_size=5)

    def listenGoToResultCB(self, result):
        if result.result == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "GoToLandmark":
                    print "looking at a GoToLandmark"
                    paramsDict = safe_load(desire.params)
                    if result.landmark == paramsDict["name"]:
                        print "Attained Landmark found in GoToLandmark desires"
                        event = Event()
                        event.desire = desire.id
                        event.desire_type = desire.type
                        event.type = Event.ACC_ON
                        self.eventPublisher.publish(event)
                    else:
                        print "name did not match"
                        
        """else:
            for desire in self.curDesireSet.desires:
                if desire.type == "GoToLandmark":
                    print "looking at a GoToLandmark"
                    paramsDict = safe_load(desire.params)
                    if result.landmark == paramsDict["name"]:
                        print "Failed Landmark found in GoToLandmark desires"
                        event = Event()
                        event.desire = desire.id
                        event.desire_type = desire.type
                        event.type = Event.ACC_OFF
                        self.eventPublisher.publish(event)
                    else:
                        print "name did not match"
        """

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_results_observer")

        node = GoToLandmarkResultObserver()
        node.listenDesiresSet()
        node.listenGoToResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
