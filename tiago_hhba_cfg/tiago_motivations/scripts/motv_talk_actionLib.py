#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib

from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires


class ListenTalkManager:

    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)

        rospy.wait_for_service("add_desires")

    def add(self):
        des = Desire()
        des.id          = "test_talk"
        des.type        = "Talking"
        des.utility     = 1.0
        des.intensity   = 1.0

        self.add_desires.call([des])

    def remove(self):
        self.rem_desires.call(["test_talk"])

    def removeOnEvent(self, event):
        if event.desire_type == "Talking" and event.type == Event.ACC_ON:
            print "removing desire Talking"
            self.rem_desires.call([event.desire])
        else:
            pass
    
    def procModuleCB(self,feedback):
        if 'script' in feedback:
            self.add()
        else:
            pass


    def observeProcModule(self):
        rospy.Subscriber("proc_listen_module", String, self.procModuleCB, queue_size=5)


    def observeEvent(self):
        rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_test")

        node = ListenTalkManager()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
