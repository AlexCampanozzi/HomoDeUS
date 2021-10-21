from state import StateBase
from hbba_msgs.msg import Desire, Event
import rospy

class State01(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_01"

    def add_state_desires(self):
        rospy.logerr("state1 ok")
        self.add("take_plate_01", "take_plate")
        self.stateDict["take_plate_01"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                print("ok state 1")
                return "state_02"

    def cleanup(self):
        self.remove("take_plate_01")
        self.stateDict.pop("take_plate_01")