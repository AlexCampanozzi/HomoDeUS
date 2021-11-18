#!/usr/bin/env python

# A simple motivation module to add a GoTo desire and remove it when it is accomplished

import rospy
import actionlib
import time
from std_msgs.msg import UInt16, Empty
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires

class testManager:
    """
    This class is only useful for testing the reaction following a particular desire
    """
    def __init__(self):
        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        self.sub            = rospy.Subscriber('select_desire',UInt16, self.motv_cb, queue_size=5)
        self.sub            = rospy.Subscriber('remove_desire',Empty, self.remove_cb, queue_size=5)
        self.desire_track_object = 'test_track'
        self.desire_look_around = 'test_look'
        self.current_desire = []
        rospy.wait_for_service("add_desires")

    def motv_cb(self, desire):
        desire = desire.data
        rospy.logwarn('adding new desire ' + str(desire))
        if desire == 1:
            self.add_track_object()
        if desire == 2:
            self.add_look_around()
        if desire == 3:
            self.add_track_object()
            self.add_look_around()
        
    def remove_cb(self,empty):
        rospy.logwarn('removing current desire')
        rospy.logwarn(str(self.current_desire))
        if self.current_desire:
            self.rem_desires.call(self.current_desire)
            del self.current_desire[:]

    def add_track_object(self):
        des = Desire()
        des.id          = self.desire_track_object
        des.type        = "Track_object"
        des.utility     = 6.0
        des.intensity   = 3.0
        des.params      = "{object: 'bottle'}"

        self.add_desires.call([des])
        self.current_desire.append(self.desire_track_object)
        # Help to spot when the desire has been added so it is more easy to follow after

    def add_look_around(self):
        des = Desire()
        des.id          = self.desire_look_around
        des.type        = "Look"
        des.utility     = 6.0
        des.intensity   = 1.0

        self.add_desires.call([des])
        self.current_desire.append(self.desire_look_around)
        # Help to spot when the desire has been added so it is more easy to follow after

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
        # Note to self: see about using IW ruleset instead to remove desires
        #node.observe()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
