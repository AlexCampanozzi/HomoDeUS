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
        rospy.loginfo("in init of motv_talk_actionLib")
        rospy.wait_for_service("add_desires")
        self.observeProcModule()
        self.observeEvent()
        self.eventPublisher = rospy.Publisher("events", Event, queue_size=5)

    def add(self):
        des = Desire()
        des.id          = "test_talk"
        des.type        = "Talking"
        des.utility     = 1.0
        des.intensity   = 1.0
        self.add_desires.call([des])
        rospy.loginfo(self.__class__.__name__ + "DESIRE SENT!!!!!!!!! !! hooooo we are closer every minute*********")
        
        rospy.loginfo(self.__class__.__name__ + "CHANGING EVENT OF LISTENING")
        event = Event()
        event.desire = "ListenToHuman"
        event.desire_type = "Listening"
        event.type = Event.ACC_ON
        self.eventPublisher.publish(event)

    def remove(self):
        self.rem_desires.call(["test_talk"])

    def removeOnEvent(self, event):
        rospy.loginfo( self.__class__.__name__ + "desire remove ??????????? !!")
        print(str(event))

        if event.desire_type == "Talking" and event.type == Event.ACC_ON:
            print "removing desire Talking"
            self.rem_desires.call([event.desire])
        else:
            pass
    
    def procModuleCB(self,feedback):
        rospy.loginfo(self.__class__.__name__ + "in motv_talk_actionLib procModuleCB, CA AVANCE")
        print(str(feedback))
        if 'script' in str(feedback):
            rospy.loginfo("did it go in script searching??")
            self.add()
        else:
            pass


    def observeProcModule(self):
        rospy.Subscriber("/proc_listen_module", String, self.procModuleCB, queue_size=5)


    def observeEvent(self):
        rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_test")

        node = ListenTalkManager()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
