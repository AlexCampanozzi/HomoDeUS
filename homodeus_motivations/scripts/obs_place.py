#!/usr/bin/env python
from __future__ import print_function
# A simple module to publish events based on the results of Place actions

import rospy
from hbba_msgs.msg import Desire, DesiresSet, Event
from std_msgs.msg import Bool


class PlaceObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenPlaceResult(self):
        self.goToResultSubscriber = rospy.Subscriber("/bhvr_output_place_result", Bool, self.listenPlaceResultCB)

    def listenPlaceResultCB(self, result):
        success = result.data
        if success == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "ListenForPlace" or desire.type == "Place":
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)
                        
        else:
            # What do we do when Place fails?
            pass

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_results_observer")

        node = PlaceObserver()
        node.listenDesiresSet()
        node.listenPlaceResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
