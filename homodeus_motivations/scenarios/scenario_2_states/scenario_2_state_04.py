from state import StateBase
from hbba_msgs.msg import Desire, Event
import rospy

class State04(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_04"

    def add_state_desires(self):
        rospy.logerr("state4 ok")
        self.add("goto_04", "goto")
        self.stateDict["goto_04"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                print("ok state 4")
                return "Done"

    def cleanup(self):
        self.remove("goto_04")
        self.stateDict.pop("goto_04")
