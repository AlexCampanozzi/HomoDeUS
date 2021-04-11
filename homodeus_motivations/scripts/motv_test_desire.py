#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib
import time
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

class testManager:
    """
    This class is only useful for testing the reaction following a particular desire
    """
    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self):
        des = Desire()
        des.id          = "testing_desire"
        des.type        = "Listening"
        des.utility     = 3.0
        des.intensity   = 1.0
        des.params      = "{context: 'order_ready'}"

        self.add_desires.call([des])
        # Help to spot when the desire has been added so it is more easy to follow after
        rospy.loginfo("-------------------------ADDING DESIRE----------------------------")

    def remove(self):
        self.rem_desires.call(["testing_desire"])

    def removeOnEvent(self, event):
        if event.desire_type == "Listening" and event.type == Event.ACC_ON:
            rospy.loginfo("REMOVING DESIRE LISTENING")
            self.rem_desires.call([event.desire])
        else:
            pass

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_test")

        node = testManager()
        time.sleep(5)
        node.add()
        # Note to self: see about using IW ruleset instead to remove desires
        node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
