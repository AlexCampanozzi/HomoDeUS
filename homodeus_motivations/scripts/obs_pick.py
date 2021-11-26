#!/usr/bin/env python
from __future__ import print_function
# A simple module to publish events based on the results of Pick actions

import rospy
from hbba_msgs.msg import Desire, DesiresSet, Event
from std_msgs.msg import Bool


class PickObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenPickResult(self):
        self.goToResultSubscriber = rospy.Subscriber("/bhvr_output_pick_result", Bool, self.listenPickResultCB)

    def listenPickResultCB(self, result):
        rospy.logwarn("************* PICK RESULT: " + str(result.data) + " **************")
        success = result.data
        if success == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "ListenForPick" or desire.type == "Pick":
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)
                        
        else:
            # What do we do when Pick fails?
            pass

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_results_observer")

        node = PickObserver()
        node.listenDesiresSet()
        node.listenPickResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
