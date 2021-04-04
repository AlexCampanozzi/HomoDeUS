from state import StateBase

class State11(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_11"

    def add_state_desires(self):
        self.add(self, "ask_for_help_11", "Talking",  params = "{TtsText: 'I appear not to be getting where I want to be, I require help. What should I do?'}")
        self.stateDict["ask_for_help_11"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "ask_for_help_11":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("ask_for_help_11")
                    self.stateDict.pop("ask_for_help_11")
                    self.add(self, "listen_for_answer_11", "Listen",  params="{context: 'continue_or_reset'}") # TODO Fix params
                    self.stateDict["listen_for_answer_11"] = Event.DES_ON
                    return None

            # ACC_OFF: reset heard
            if desire == "listen_for_answer_11": 
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("listen_for_answer_11")
                    self.stateDict.pop("listen_for_answer_11")
                    return "state_00"

            # ACC_ON: ok heard
            if desire == "listen_for_answer_11": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_answer_11")
                    self.stateDict.pop("listen_for_answer_11")
                    return "state_10"

        return None

    def cleanup(self):
        pass
