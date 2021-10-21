from state import StateBase
from hbba_msgs.msg import Desire, Event
import rospy

class State00(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_00"

    def add_state_desires(self):
        rospy.logerr("state0 ok")
        self.add("goto_00", "goto")
        self.stateDict["goto_00"] = Event.DES_ON



    def react_to_event(self):
        for desire in self.stateDict:
            print("reacted")
            if self.stateDict[desire] == Event.ACC_ON:
                print("ok state 0")
                return "state_01"

    def cleanup(self):
        self.remove("goto_00")
        self.stateDict.pop("goto_00")
