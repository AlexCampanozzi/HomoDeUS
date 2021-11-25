import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State03(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        rospy.loginfo('------------------ State 03 Init -----------------------')
        self.command = ""

    def _set_id(self):
        return "Say_Command"

    def add_state_desires(self):
        print("03 init")
        self.add("inform_cook_03", "Talking",  params = "{TtsText: 'A client ordered: " + self.command + "'}")
        self.add("find_order_03", "Track_object", params = "{object: '" + self.command + "'}")
        self.add("looking_around_03", "Look")
        self.stateDict["inform_cook_03"] = Event.DES_ON
        self.stateDict["find_order_03"] = Event.DES_ON
        self.stateDict["looking_around_03"] = Event.DES_ON

    def react_to_event(self): ## Ignore the other desires
        if self.stateDict["find_order_03"] == Event.ACC_ON:
            return True
        elif self.stateDict["find_order_03"] == Event.ACC_OFF:
            return False
        else:
            return None

    def cleanup(self):
        self.remove("inform_cook_03")
        self.remove("find_order_03")
        self.remove("looking_around_03")

        self.stateDict.pop("inform_cook_03")
        self.stateDict.pop("find_order_03")
        self.stateDict.pop("looking_around_03")
