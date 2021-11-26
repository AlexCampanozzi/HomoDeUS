#!/usr/bin/env python

# A simple module to publish events based on face tracking

import rospy
import actionlib
from yaml import safe_load
from std_msgs.msg import Bool
from hbba_msgs.msg import Desire, DesiresSet, Event
from custom_msgs.msg import GoToResult


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
        # self.goToResultSubscriber = rospy.Subscriber("/obs_approach_client", Bool, self.listenApproachClientCB)
        self.goToResultSubscriber = rospy.Subscriber("/bhvr_approach_client/obs_approach_client", Bool, self.listenApproachClientCB)
        


    def listenApproachClientCB(self, result):
        rospy.loginfo("received something")
        succes = result.data
        if result.data == True:
            rospy.loginfo("data is true")
            for desire in self.curDesireSet.desires:
                rospy.loginfo(desire)
                if desire.type == "approach_client":
                    paramsDict = safe_load(desire.params)
                    rospy.loginfo("Position found within tolerance of a goal position for approach client")
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)
                        
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
