import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State01(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "Get_Command"

    def add_state_desires(self):
        print("01 init")
        self.add("listen_for_menu", "Dialoguing",  params = "{context: 'menu_selection'}")
        self.stateDict["listen_for_menu"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return True
            elif self.stateDict[desire] == Event.ACC_OFF:
                return False
            else:
                return None

    def cleanup(self):
        self.remove("listen_for_menu")
        self.stateDict.pop("listen_for_menu")
