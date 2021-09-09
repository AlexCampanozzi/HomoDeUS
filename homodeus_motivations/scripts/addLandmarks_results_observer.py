#!/usr/bin/env python

# A simple module to publish events based on the results of GoTo actions

import rospy
import actionlib
from yaml import safe_load
from hbba_msgs.msg import Desire, DesiresSet, Event
from std_msgs.msg import String

class AddLandmarkObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenGoToResult(self):
        self.goToResultSubscriber = rospy.Subscriber("bhvr_output_nav_added_landmark", String, self.listenGoToResultCB)

    def listenGoToResultCB(self, result):
        name = result.data
        for desire in self.curDesireSet.desires:
            if desire.type == "AddLandmark":
                paramsDict = safe_load(desire.params)
                if paramsDict["name"] == name:
                    print "Landmark we wanted was added"
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_results_observer")

        node = GoToResultObserver()
        node.listenDesiresSet()
        node.listenGoToResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
