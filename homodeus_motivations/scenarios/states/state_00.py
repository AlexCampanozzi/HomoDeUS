import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State00(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_00"

    def add_state_desires(self):
        print("00 init")
        #self.add("move_to_table_00", "GoToLandmark",  params = "{name: 'table1'}")
        #self.stateDict["move_to_table_00"] = Event.DES_ON
        self.add("track_customer_00", "face_detection")
        self.stateDict["track_customer_00"] = Event.DES_ON
        
    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                print("going to state 03")
                return "state_03"

    def cleanup(self):
        self.remove("track_customer_00")
        self.stateDict.pop("track_customer_00")
        #self.remove("move_to_table_00")
        #self.stateDict.pop("move_to_table_00")