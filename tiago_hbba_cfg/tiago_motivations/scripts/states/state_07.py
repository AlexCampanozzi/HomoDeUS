from state import StateBase

class State07(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.fail_count = 0
        self.max_fail_count = 3
        # TODO CB to update order form observer

    def _set_id(self):
        return "state_07"

    def add_state_desires(self):
        self.add(self, "inform_customer_07", "Talk",  params = "going to kitchen") # TODO Fix class and params
        self.stateDict["inform_customer_07"] = Event.DES_ON
        self.add(self, "move_to_kitchen_07", "GoToLandmark",  params = "{name: 'kitchen'}")
        self.stateDict["move_to_kitchen_07"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "move_to_kitchen_07":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("move_to_kitchen_07")
                    self.stateDict.pop("move_to_kitchen_07")
                    return "state_09"

            if desire == "move_to_kitchen_07":
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("move_to_kitchen_07")
                    self.stateDict.pop("move_to_kitchen_07")
                    return "state_08"

        return None

    def cleanup(self):
        self.remove("inform_customer_07")
        self.stateDict.pop("inform_customer_07")
