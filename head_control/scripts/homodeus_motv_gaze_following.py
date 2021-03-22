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
<<<<<<< HEAD
        des = Desire()
        des.id          = "face_detection"
        des.type        = "GoTo"
        des.utility     = 1.0
        des.intensity   = 1.0
=======
        faceDetectionDesire = Desire()
        faceDetectionDesire.id          = "test_face_detection"
        faceDetectionDesire.type        = "face_detection"
        faceDetectionDesire.utility     = 1.0
        faceDetectionDesire.intensity   = 1.0

        moveHeadDesire = Desire()
        moveHeadDesire.id          = "test_move_head"
        moveHeadDesire.type        = "move_head"
        moveHeadDesire.utility     = 1.0
        moveHeadDesire.intensity   = 1.0
>>>>>>> eb974631031dac9be9e1aedd2608932da1341782
        #des.params      = "{frame_id: '/xtion_rgb_optical_frame', x: 30.0, y: 0.0, t: 0.0}"

        self.add_desires.call([faceDetectionDesire, moveHeadDesire])

    def remove(self):
        self.rem_desires.call(["test_face_detection"])
        self.rem_desires.call(["test_move_head"])

    def removeOnEvent(self, event):
        if event.desire_type == "face_detection" and event.type == Event.ACC_ON:
            self.rem_desires.call([event.desire])
        elif event.desire_type == "move_head" and event.type == Event.ACC_ON:
            self.rem_desires.call([event.desire])
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