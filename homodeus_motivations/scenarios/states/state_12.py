from state import StateBase

class State12(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_12"

    def add_state_desires(self):
        self.add(self, "track_customer_12", "face_tracking",  params="")
        self.stateDict["track_customer_12"] = Event.DES_ON
        self.add(self, "inform_customer_12", "Talking",  params = "{TtsText: 'I have brought your order, please take it.'}")
        self.stateDict["inform_customer_12"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "inform_customer_12":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("inform_customer_12")
                    self.stateDict.pop("inform_customer_12  ")
                    # what even is wait as a desire?
                    self.add(self, "wait_12", "i dont know",  params="") # TODO Fix class and params
                    self.stateDict["wait_12"] = Event.DES_ON
                    return None

            if desire == "wait_12": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("wait_12")
                    self.stateDict.pop("wait_12")
                    return "Done" # Not a state, but indication to manager that scenario is over

        return None

    def cleanup(self):
        self.remove("track_customer_12")
        self.stateDict.pop("track_customer_12")
