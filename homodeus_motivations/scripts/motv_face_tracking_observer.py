#!/usr/bin/env python

# A simple module to publish events based on face tracking

import rospy
import actionlib
from yaml import safe_load
from std_msgs import Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

def equalWithinTolerance(a, b, tol):
    return abs(a-b) <= tol

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

    def listenFaceTrackingCB(self, result):
        succes = result.result
        ## to do
        if result.result == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "GoToLandmark":
                    print "looking at a GoToLandmark"
                    paramsDict = safe_load(desire.params)
                    if result.landmark == paramsDict["name"]:
                        print "Attained Landmark found in gotoLandmark desires"
                        event = Event()
                        event.desire = desire.id
                        event.desire_type = desire.type
                        event.type = Event.ACC_ON
                        self.eventPublisher.publish(event)
                    else:
                        print "name did not match"
                        
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
