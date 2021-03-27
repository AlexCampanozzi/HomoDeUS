from state import StateBase

class State10(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_10"

    def add_state_desires(self):
        # Add a search before tracking? Or maybe have search be part of tracking?
        self.add(self, "thank_cook_10", "Talk",  params = "thanks dude") # TODO Fix class and params
        self.stateDict["thank_cook_10"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "thank_cook_10":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("thank_cook_10")
                    self.stateDict.pop("thank_cook_10")
                    # Keep tracking as we continue sequence: no remove here
                    self.add(self, "move_to_customer_10", "GoToLandmark",  params = "{name: 'customer'}")
                    self.stateDict["move_to_customer_10"] = Event.DES_ON
                    return None

            if desire == "move_to_customer_10": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("move_to_customer_10")
                    self.stateDict.pop("move_to_customer_10")  
                    return "state_12"

            if desire == "move_to_customer_10": 
                # Not sure which, may stay both
                if self.stateDict[desire] == Event.ACC_OFF or self.stateDict[desire] == Event.IMP_ON:
                    self.remove("move_to_customer_10")
                    self.stateDict.pop("move_to_customer_10")
                    return "state_11"

        return None

    def cleanup(self):
        pass
