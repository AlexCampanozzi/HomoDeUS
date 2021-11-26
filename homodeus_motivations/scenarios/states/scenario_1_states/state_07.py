import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State07(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        
    def _set_id(self):
        return "Go_back_kitchen"

    def add_state_desires(self):
        print("07 init")

        self.add("move_to_start_07", "GoToLandmark",  params = "{name: 'start'}")
        self.stateDict["move_to_start_07"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return True
            elif self.stateDict[desire] == Event.ACC_OFF:
                return False
            else:
                return None

    def cleanup(self):
        self.remove("move_to_start_07")
        self.stateDict.pop("move_to_start_07")
