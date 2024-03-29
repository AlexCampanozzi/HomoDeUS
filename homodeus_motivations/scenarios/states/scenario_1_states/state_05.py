import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State05(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        
    def _set_id(self):
        return "Approach_client"

    def add_state_desires(self):
        print("05 init")

        self.add("approach_client_05", "approach_client")
        self.stateDict["approach_client_05"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return True
            elif self.stateDict[desire] == Event.ACC_OFF:
                return False
            else:
                return None

    def cleanup(self):
        self.remove("approach_client_05")
        self.stateDict.pop("approach_client_05")
