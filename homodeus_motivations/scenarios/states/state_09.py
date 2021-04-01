from state import StateBase

class State09(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.order = ""
        # TODO CB to update order form observer

    def _set_id(self):
        return "state_09"

    def add_state_desires(self):
        # Add a search before tracking? Or maybe have search be part of tracking?
        self.add(self, "track_cook_09", "face_tracking",  params="")
        self.stateDict["track_cook_09"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "track_cook_09":
                if self.stateDict[desire] == Event.ACC_ON:
                    # Keep tracking as we continue sequence: no remove here
                    self.add(self, "inform_cook_09", "Talking",  params = self.order) # TODO Fix class and params
                    self.stateDict["inform_cook_09"] = Event.DES_ON
                    return None

            if desire == "inform_cook_09": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("inform_cook_09")
                    self.stateDict.pop("inform_cook_09")
                    self.add(self, "listen_cook_09", "Listen",  params="order up") # TODO Fix class and params
                    self.stateDict["listen_cook_09"] = Event.DES_ON   
                    return None

            if desire == "listen_cook_09": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_cook_09")
                    self.stateDict.pop("listen_cook_09")
                    return "state_10"

        return None

    def cleanup(self):
        self.remove("track_cook_09")
        self.stateDict.pop("track_cook_09")
