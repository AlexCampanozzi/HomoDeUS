#!/usr/bin/env python

# A simple module to publish events based on the results of GoTo actions

import rospy
from std_msgs.msg import Bool
from hbba_msgs.msg import DesiresSet, Event
import HomoDeUS_common_py.HomoDeUS_common_py as common

class lookingAroundObserver:
    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()
        self.eventSent = False

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenLookAroundResult(self):
        self.dialogSubscriber = rospy.Subscriber("bhvr_output_res_looking_around", Bool, self.listenLookingAroundCb)

    def listenLookingAroundCb(self, result):
        if result.data == False:
            self.eventSent = False
        elif not self.eventSent:
            for desire in self.curDesireSet.desires: # just showing it is working
                if desire.type == "Look":
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)
                    self.eventSent = True


if __name__ == "__main__":
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = lookingAroundObserver()
        node.listenDesiresSet()
        node.listenLookAroundResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
