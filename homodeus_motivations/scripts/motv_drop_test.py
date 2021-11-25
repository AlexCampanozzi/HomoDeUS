#!/usr/bin/env python

# A simple motivation module to add a test_cloudProc desire and remove it when it is accomplished

import rospy
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from std_msgs.msg import String

class CloudProcManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self):
        des = Desire()
        des.id          = "test_drop_proc"
        des.type        = "DropSpotPerception"
        des.utility     = 2.0
        des.intensity   = 1.0

        drop_des = Desire()
        drop_des.id          = "test_drop_listen"
        drop_des.type        = "ListenForPlace"
        drop_des.utility     = 2.0
        drop_des.intensity   = 1.0

        rospy.sleep(5)
        rospy.logwarn("Adding DropSpotPerception desire")
        self.add_desires.call([des])
        rospy.logwarn("Adding ListenForPlace desire")
        self.add_desires.call([drop_des])

    def remove(self):
        self.rem_desires.call(["test_drop_proc"])
        self.rem_desires.call(["test_drop_listen"])

    def removeOnEvent(self, event):
        if event.desire_type == "test_drop_proc" and event.type == Event.ACC_ON:
            self.rem_desires.call([event.desire])
        else:
            pass
        if event.desire_type == "test_drop_listen" and event.type == Event.ACC_ON:
            self.rem_desires.call([event.desire])
        else:
            pass

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_cloudProc_test")

        node = CloudProcManager()
        node.add()
        # Note to self: see about using IW ruleset instead to remove desires
        node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
