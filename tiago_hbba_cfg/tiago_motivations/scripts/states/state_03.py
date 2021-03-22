from state import StateBase

class State03(StateBase):
    def __init__(self):
        StateBase.__init__(self)

    def _set_id(self):
        return "state_03"

    def add_state_desires(self):
        self.add(self, "track_customer_03", "Face_tracking",  params="")
        self.add(self, "hear_hotword_00", "Keyword_detection",  params="robot")
        return ("detect_customer_00", "hear_hotword_00")

    def get_next_state(self, stateDict):
        for state in stateDict:
            if stateDict[state] == Event.ACC_ON:
                return "state_03"

    def cleanup(self, nextState):
        self.remove("detect_customer_00")
        self.remove("hear_hotword_00")
