from state import StateBase
from hbba_msgs.msg import Desire, Event
import rospy

class State03(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_03"

    def add_state_desires(self):
        rospy.logerr("state3 ok")
        self.add("give_plate_03", "give_plate")
        self.stateDict["give_plate_03"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                print("ok state 3")
                return "state_04"

    def cleanup(self):
        self.remove("give_plate_03")
        self.stateDict.pop("give_plate_03")