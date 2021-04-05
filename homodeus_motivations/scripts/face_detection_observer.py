#!/usr/bin/env python

# A simple module to publish events based on face tracking

import rospy
import actionlib
from yaml import safe_load
from std_msgs.msg import Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

class FaceDetectionObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event, queue_size = 10)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenFaceDetection(self):
        self.faceDetectionSubscriber = rospy.Subscriber("/face_detection_observer", Bool, self.listenFaceTrackingCB)

    def listenFaceTrackingCB(self, success):
        if success.data == True:
            for desire in self.curDesireSet.desires:
                if desire.type == "face_detection":
                    paramsDict = safe_load(desire.params)
                    
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)                  
        else:
            # here things to do if we have a false too often
            pass

if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = FaceDetectionObserver()
        node.listenDesiresSet()
        node.listenFaceDetection()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())