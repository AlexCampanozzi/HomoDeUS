#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib
from hbba_msgs.msg import Desire, DesiresSet, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

class GotoManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self):
        des = Desire()
        des.id          = "face_detection"
        des.type        = "GoTo"
        des.utility     = 1.0
        des.intensity   = 1.0
        #des.params      = "{frame_id: '/xtion_rgb_optical_frame', x: 30.0, y: 0.0, t: 0.0}"

        self.add_desires.call([des])

    def remove(self):
        self.rem_desires.call(["test_goto"])

    def removeOnEvent(self, event):
        if event.desire_type == "GoTo" and event.type == Event.ACC_ON:
            self.rem_desires.call([event.desire])
        else:
            pass

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":

    try:
        rospy.init_node("tiago_motv_gaze_following", anonymous=False)

        node = GotoManager()
        node.add()

        node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass