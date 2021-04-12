import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State08(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_08"

    def add_state_desires(self):
        self.add("ask_for_help_08", "Talking",  params = "{TtsText: 'I appear not to be getting where I want to be, I require help. What should I do?'}")
        self.stateDict["ask_for_help_08"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "ask_for_help_08":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("ask_for_help_08")
                    self.stateDict.pop("ask_for_help_08")
                    self.add("Listening_for_answer_08", "Listening",  params="{context: 'continue_or_reset'}")
                    self.stateDict["Listening_for_answer_08"] = Event.DES_ON
                    return None

            # ACC_OFF: reset heard
            if desire == "Listening_for_answer_08": 
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("Listening_for_answer_08")
                    self.stateDict.pop("Listening_for_answer_08")
                    return "state_00"

            # ACC_ON: ok heard
            if desire == "Listening_for_answer_08": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("Listening_for_answer_08")
                    self.stateDict.pop("Listening_for_answer_08")
                    return "state_07"

        return None

    def cleanup(self):
        pass
