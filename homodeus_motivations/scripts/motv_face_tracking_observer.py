#!/usr/bin/env python

# A simple module to publish events based on face tracking

import rospy
import actionlib
from yaml import safe_load
from std_msgs import Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

class FaceTrackingResultObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenFaceTracking(self):
        self.faceTrackingETASubscriber = rospy.Subscriber("FaceTrackingETA", Bool, self.listenFaceTrackingCB)

    def listenFaceTrackingCB(self, success):
        if success == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "FaceTracking":
                    rospy.log("looking at a FaceTracking")
                    paramsDict = safe_load(desire.params)
                    
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)                  
        else:
            # here things to do if we have a false too often
            pass

if __name__ == "__main__":
    try:
        rospy.init_node("motv_face_tracking_observer")

        node = FaceTrackingResultObserver()
        node.listenDesiresSet()
        node.listenFaceTracking()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
