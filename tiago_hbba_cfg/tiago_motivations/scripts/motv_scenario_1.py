import rospy
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from action import ScenarioManagerAction

class Scenario1Manager(ScenarioManagerAction):

    def __init__(self):
        self.desires = {}

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.removeOnEvent, queue_size=5)

    def eventCB(self, event):
        if event.desire in self.desires:
            self.desires[event.desire] = event.type
    
    def execute_cb(self, goal):
        if goal is True:
            # stuff
        else:
            # no stuff


if __name__ == "__main__":
    try:
        rospy.init_node("scenario_1_manager)

        node = GoToResultObserver()
        node.listenDesiresSet()
        node.listenGoToResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
