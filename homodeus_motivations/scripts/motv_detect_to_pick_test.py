#!/usr/bin/env python

# A simple motivation module to add a test_cloudProc desire and remove it when it is accomplished

import rospy
import actionlib
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
        des.id          = "test_cloudProc"
        des.type        = "PointCloudPerception"
        des.utility     = 2.0
        des.intensity   = 1.0

        pick_des = Desire()
        pick_des.id          = "test_pickListenBhvr"
        pick_des.type        = "ListenForPick"
        pick_des.utility     = 2.0
        pick_des.intensity   = 1.0

        rospy.sleep(2)
        rospy.logwarn("Adding PointCloudPerception desire")
        self.add_desires.call([des])
        rospy.logwarn("Adding ListenForPick desire")
        self.add_desires.call([pick_des])
        rospy.sleep(2)
        rospy.logwarn("Publishing desired_object")
        pub = rospy.Publisher("/desired_object", String, queue_size=5)
        rospy.sleep(1)
        pub.publish("bottle")

    def remove(self):
        self.rem_desires.call(["test_cloudProc"])

    def removeOnEvent(self, event):
        if event.desire_type == "test_cloudProc" and event.type == Event.ACC_ON:
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
