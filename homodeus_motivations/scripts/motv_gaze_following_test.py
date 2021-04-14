#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib
from hbba_msgs.msg import Desire, DesiresSet, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

class GazeFollowingManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self):
        # faceDetectionDesire = Desire()
        # faceDetectionDesire.id          = "test_face_detection"
        # faceDetectionDesire.type        = "face_detection"
        # faceDetectionDesire.utility     = 1.0
        # faceDetectionDesire.intensity   = 1.0

        moveHeadDesire = Desire()
        moveHeadDesire.id          = "test_face_tracking"
        moveHeadDesire.type        = "face_tracking"
        moveHeadDesire.utility     = 1.0
        moveHeadDesire.intensity   = 1.0
        #des.params      = "{frame_id: '/xtion_rgb_optical_frame', x: 30.0, y: 0.0, t: 0.0}"

        self.add_desires.call([moveHeadDesire])
        # self.add_desires.call([faceDetectionDesire, moveHeadDesire])

    def remove(self):
        self.rem_desires.call(["test_face_detection"])
        self.rem_desires.call(["test_face_tracking"])

    def removeOnEvent(self, event):
        if event.desire_type == "face_detection" and event.type == Event.ACC_ON:
            pass # in this case we just want to know if a face has been detected
        elif event.desire_type == "face_tracking" and event.type == Event.ACC_ON:
            #self.rem_desires.call([event.desire])
            pass
        else:
            pass

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":

    try:
        rospy.init_node("tiago_motv_gaze_following", anonymous=False)

        node = GazeFollowingManager()
        node.add()

        node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass