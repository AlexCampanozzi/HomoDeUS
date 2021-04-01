from state import StateBase

class State05(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_05"

    def add_state_desires(self):
        self.add(self, "track_customer_05", "face_tracking",  params="")
        self.stateDict["track_customer_05"] = Event.DES_ON
        self.add(self, "ask_for_order_05", "Talking",  params="gib ordrer") # TODO Fix class and params
        self.stateDict["ask_for_order_05"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "ask_for_order_05":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("ask_for_order_05")
                    self.stateDict.pop("ask_for_order_05")
                    self.add(self, "listen_for_order_05", "Listen",  params="menu") # TODO Fix class and params
                    self.stateDict["listen_for_order_05"] = Event.DES_ON
                    return None

            if desire == "listen_for_order_05":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_order_05")
                    self.stateDict.pop("listen_for_order_05")
                    return "state_06"

            if desire == "listen_for_order_05":
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("listen_for_order_05")
                    self.stateDict.pop("listen_for_order_05")
                    return "state_02"

            if desire == "listen_for_order_05":
                if self.stateDict[desire] == Event.IMP_ON:
                    self.remove("listen_for_order_05")
                    self.stateDict.pop("listen_for_order_05")
                    return "state_04"

        return None

    def cleanup(self):
        self.remove("track_customer_05")
        self.stateDict.pop("track_customer_05")
