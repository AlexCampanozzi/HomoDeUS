import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State01(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_01"

    def add_state_desires(self):
        self.add("say_goodbye_01", "Talking",  params="{TtsText: 'Goodbye, come again Human!'}")
        self.stateDict["say_goodbye_01"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "say_goodbye_01":
                if self.stateDict[desire] == Event.ACC_ON:
                    return "state_00"

        return None

    def cleanup(self):
        self.remove("say_goodbye_01")
        self.stateDict.pop("say_goodbye_01")
