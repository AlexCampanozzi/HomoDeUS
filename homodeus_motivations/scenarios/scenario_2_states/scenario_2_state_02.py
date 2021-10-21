from state import StateBase
from hbba_msgs.msg import Desire, Event
import rospy

class State02(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_02"

    def add_state_desires(self):
        rospy.logerr("state2 ok")
        self.add("goto_02", "goto")
        self.stateDict["goto_02"] = Event.DES_ON
        self.add("approach_client_02", "ApproachClient")
        self.stateDict["approach_client_02"] = Event.DES_ON
        self.add("face_tracking_02", "face_tracking")
        self.stateDict["face_tracking_02"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                print("ok state 2")
                return "state_03"

    def cleanup(self):
        self.remove("goto_02")
        self.stateDict.pop("goto_02")
        self.remove("approach_client_02")
        self.stateDict.pop("approach_client_02")
        self.remove("face_tracking_02")
        self.stateDict.pop("face_tracking_02")