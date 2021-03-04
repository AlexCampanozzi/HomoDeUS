#! /usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import String
from hbba_msgs.msg import Desire, DesiresSet, Event


class GoToResultObserver:

    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event, queue_size = 10)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenTalkResult(self):
        self.goToResultSubscriber = rospy.Subscriber("/talkManagerSimul_topic",String, self.listenTalkResultCB)

    def listenTalkResultCB(self, text):
        if text == "Command receive, I'm starting scenario 1!" :
            for desire in self.curDesireSet.desires:
                if desire.type == "Talking":
                    print "Position found within tolerance of a goal position"
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    event.type = Event.ACC_ON
                    self.eventPublisher.publish(event)
                    break
                else:
                    pass           
        else:
            print "not the text I was looking for"
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