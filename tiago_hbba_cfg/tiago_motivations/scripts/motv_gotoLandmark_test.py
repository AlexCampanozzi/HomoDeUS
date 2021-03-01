#!/usr/bin/env python

# A simple motivation module to add a GoToLandmark desire and remove it when it is accomplished

import rospy
import actionlib
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

class GotoLandmarkManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self):
        des = Desire()
        des.id          = "test_gotoLandmark"
        des.type        = "GoToLandmark"
        des.utility     = 2.0
        des.intensity   = 2.0
        des.params      = "{name: 'origin'}"

        rospy.sleep(8)
        self.add_desires.call([des])

    def remove(self):
        self.rem_desires.call(["test_goto"])

    def removeOnEvent(self, event):
        if event.desire_type == "GoToLandmark" and event.type == Event.ACC_ON:
            self.rem_desires.call([event.desire])
        else:
            pass

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_gotoLandmark_test")

        node = GotoLandmarkManager()
        node.add()
        # Note to self: see about using IW ruleset instead to remove desires
        node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
