#!/usr/bin/env python

# A simple module to publish events based on face tracking

import rospy
import actionlib
from yaml import safe_load
from std_msgs.msg import Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

class ApproachClientObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event, queue_size = 5)
        self.curDesireSet = DesiresSet()
        rospy.loginfo("approach client observer")


    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenApproachClient(self):
        self.goToResultSubscriber = rospy.Subscriber("bhvr_output_res_nav_result", GoToResult, self.listenApproachClientCB)


    def listenApproachClientCB(self, result):
        succes = result.result
        if result.result == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "GoTo":
                    paramsDict = safe_load(desire.params)
                    if equalWithinTolerance(result.x, paramsDict["x"], 0.1) and equalWithinTolerance(result.y, paramsDict["y"], 0.1) and equalWithinTolerance(result.t, paramsDict["t"], 0.1):
                        rospy.loginfo("Position found within tolerance of a goal position")
                        event = Event()
                        event.desire = desire.id
                        event.desire_type = desire.type
                        event.type = Event.ACC_ON
                        self.eventPublisher.publish(event)
                    else:
                        print "Position found outside tolerance of a goal position"
                        
        else:
            # What do we do when GoTo fails?
            pass

if __name__ == "__main__":
    try:
        rospy.init_node("obs_approach_client")
        node = ApproachClientObserver()
        node.listenDesiresSet()
        node.listenApproachClient()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
