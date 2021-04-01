from state import StateBase

class State00(StateBase):
    def __init__(self, stateDict):
        print("in 00 init")
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_00"

    def add_state_desires(self):
        self.add(self, "detect_customer_00", "Face_detection",  params="")
        self.add(self, "hear_hotword_00", "Keyword_detection",  params="robot")
        self.stateDict["detect_customer_00"] = Event.DES_ON
        self.stateDict["hear_hotword_00"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return "state_03"

    def cleanup(self):
        self.remove("detect_customer_00")
        self.stateDict.pop("detect_customer_00")
        self.remove("hear_hotword_00")
        self.stateDict.pop("hear_hotword_00")
