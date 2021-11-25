#!/usr/bin/env python

# A simple module to publish events based on the results of GoTo actions

import rospy
from hbba_msgs.msg import DesiresSet, Event
from darknet_ros_msgs.msg import BoundingBoxes
import HomoDeUS_common_py.HomoDeUS_common_py as common

class trackingObjectObserver:
    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenTrackingObjectResult(self):
        self.dialogSubscriber = rospy.Subscriber("bhvr_output_trackingObject_boxes", BoundingBoxes, self.listenTrackingObjectCB)

    def listenTrackingObjectCB(self, boxes):
        for desire in self.curDesireSet.desires:
            if desire.type == "Track_object":
                event = Event()
                event.desire = desire.id
                event.desire_type = desire.type
                event.type = Event.ACC_ON
                self.eventPublisher.publish(event)

if __name__ == "__main__":
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = trackingObjectObserver()
        node.listenDesiresSet()
        node.listenTrackingObjectResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
