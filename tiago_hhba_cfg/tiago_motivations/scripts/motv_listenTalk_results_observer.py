#! /usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import String
from hbba_msgs.msg import Desire, DesiresSet, Event


class GoToResultObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event, queue_size = 10)
        self.curDesireSet = DesiresSet()
        rospy.loginfo("init motv_listenTalk_results_observer")

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        print(str(desireSet))
        self.curDesireSet = desireSet
        rospy.loginfo(self.__class__.__name__ + "------------SHOULD GOTORESULTOBSERVER AFTER DESIRE SENT----------")


    def listenTalkResult(self):
        rospy.Subscriber("/talkManagerSimul_topic",String, self.listenTalkResultCB, queue_size=5)

    def listenTalkResultCB(self, text):
        rospy.loginfo(self.__class__.__name__ + "*****************I am in the LAST step of everything******************")
        print(text)
        for desire in self.curDesireSet.desires:
            if desire.type == "Talking":
                if text.data == "Command receive, I'm starting scenario 1!" :
                    rospy.loginfo(self.__class__.__name__ +"CHANGING EVENT TALKING")
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)
                    break
            else:
                print "not the desire I was looking for"
                pass

if __name__ == "__main__":
    try:
        rospy.init_node("motv_goto_results_observer")

        node = GoToResultObserver()
        node.listenDesiresSet()
        node.listenTalkResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass