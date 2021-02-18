#!/usr/bin/env python

# A motivation for environment-based suggestions (close the door, etc.)

import rospy
import actionlib
from hbba_msgs.msg import Desire
from hbba_msgs.srv import AddDesires, RemoveDesires

class GotoManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)

    def add(self):
        des = Desire()
        des.id          = "test_goto"
        des.type        = "GoTo"
        des.utility     = 1.0
        des.intensity   = 1.0
        des.params      = "{fram_id: map, x: -1, y: -1, t: 0}"

        self.add_desires.call([des])

    def remove(self):
        self.rem_desires.call(["test_goto"])

if __name__ == "__main__":
    rospy.init_node("motv_goto_test")

    node = GotoManager()
    node.add()

    rospy.spin()

