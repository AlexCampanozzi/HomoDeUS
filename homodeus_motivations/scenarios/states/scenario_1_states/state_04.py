import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State03(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.command = ""

    def _set_id(self):
        return "Find_Object"

    def add_state_desires(self):
        print("04 init")
        self.add("inform_cook_03", "Talking",  params = "{TtsText: 'A client ordered: " + self.command + "'}")
        self.stateDict["inform_cook_03"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return True
            elif self.stateDict[desire] == Event.ACC_OFF:
                return False
            else:
                return None

    def cleanup(self):
        self.remove("inform_cook_03")
        self.stateDict.pop("inform_cook_03")
