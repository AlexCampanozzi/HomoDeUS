from state import StateBase

class State08(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_08"

    def add_state_desires(self):
        self.add(self, "ask_for_help_08", "Talking",  params = "what do?") # TODO Fix class and params
        self.stateDict["ask_for_help_08"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "ask_for_help_08":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("ask_for_help_08")
                    self.stateDict.pop("ask_for_help_08")
                    self.add(self, "listen_for_answer_08", "Listen",  params="ok or reset") # TODO Fix class and params
                    self.stateDict["listen_for_answer_08"] = Event.DES_ON
                    return None

            # ACC_OFF: reset heard
            if desire == "listen_for_answer_08": 
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("listen_for_answer_08")
                    self.stateDict.pop("listen_for_answer_08")
                    return "state_00"

            # ACC_ON: ok heard
            if desire == "listen_for_answer_08": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_answer_08")
                    self.stateDict.pop("listen_for_answer_08")
                    return "state_07"

        return None

    def cleanup(self):
        pass
