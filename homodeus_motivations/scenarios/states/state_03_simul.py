import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State03(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "Say_Command"

    def add_state_desires(self):
        print("00 init")
        self.add("inform_cook_03", "GoToLandmark",  params = "{name: 'table1'}")
        self.stateDict["inform_cook_03"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return True
            elif self.stateDict[desire] == Event.ACC_OFF:
                return False

    def cleanup(self):
        self.remove("inform_cook_03")
        self.stateDict.pop("inform_cook_03")
