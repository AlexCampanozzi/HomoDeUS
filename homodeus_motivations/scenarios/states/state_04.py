from state import StateBase

class State04(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_04"

    def add_state_desires(self):
        self.add(self, "track_customer_04", "face_tracking")
        self.stateDict["track_customer_04"] = Event.DES_ON
        self.add(self, "incite_customer_04", "Talking",  params="{TtsText: 'Could you repeat your order please?'}")
        self.stateDict["incite_customer_04"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "incite_customer_04":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("incite_customer_04")
                    self.stateDict.pop("incite_customer_04")
                    self.add(self, "listen_for_order_04", "Listen",  params="{context: 'menu'}") # TODO Fix params
                    self.stateDict["listen_for_order_04"] = Event.DES_ON
                    return None

            if desire == "listen_for_order_04":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_order_04")
                    self.stateDict.pop("listen_for_order_04")
                    return "state_06"

            if desire == "listen_for_order_04":
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("listen_for_order_04")
                    self.stateDict.pop("listen_for_order_04")
                    return "state_02"

            if desire == "listen_for_order_04":
                if self.stateDict[desire] == Event.IMP_ON:
                    self.remove("listen_for_order_04")
                    self.stateDict.pop("listen_for_order_04")
                    return "state_01"

        return None

    def cleanup(self):
        self.remove("track_customer_04")
        self.stateDict.pop("track_customer_04")
