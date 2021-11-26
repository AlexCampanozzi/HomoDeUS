import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State06(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        
    def _set_id(self):
        return "GoTo_Table"

    def add_state_desires(self):
        print("06 init")

        self.add("move_to_table_06", "GoToLandmark",  params = "{name: 'table1'}")
        self.stateDict["move_to_table_06"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return True
            elif self.stateDict[desire] == Event.ACC_OFF:
                return False
            else:
                return None

    def cleanup(self):
        self.remove("move_to_table_06")
        self.stateDict.pop("move_to_table_06")
