#!/usr/bin/env python

# A simple motivation module to add a GoTo desire

import rospy
import actionlib
from hbba_msgs.msg import Desire, DesiresSet
from hbba_msgs.srv import AddDesires, RemoveDesires

class GotoManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        self.add_desires.wait_for_service()
        while(not self.add_desires.wait_for_service(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the hbba desire server to come up")

    def add(self):
        des = Desire()
        des.id          = "test_goto"
        des.type        = "GoTo"
        des.utility     = 1.0
        des.intensity   = 1.0
        des.params      = "{frame_id: map, x: -1, y: -1, t: 0}"

        desSet = DesiresSet()
        desSet.desires = [des, Desire()]

        self.add_desires.call(desSet)

    def remove(self):
        self.rem_desires.call(["test_goto"])

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_test")

        node = GotoManager()
        node.add()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
