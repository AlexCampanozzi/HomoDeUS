#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

class GotoManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self):
        des = Desire()
        des.id          = "test_goto"
        des.type        = "GoTo"
        des.utility     = 1.0
        des.intensity   = 1.0
        des.params      = "{frame_id: 'map', x: -1, y: -1, t: 1}"

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
        rospy.init_node("motv_goto_test")

        node = GotoManager()
        node.add()
        # Note to self: see about using IW ruleset instead to remove desires
        node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
