from state import StateBase

class State09(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.order = ""
        # TODO set topic to correct name
        rospy.Subscriber("/obs_client_order", String, self.order_cb, queue_size=5)

    def order_cb(self, order_string):
        self.order = "{TtsText: 'A client ordered: " + order_string.data + "'}"

    def _set_id(self):
        return "state_09"

    def add_state_desires(self):
        # Add a search before tracking? Or maybe have search be part of tracking?
        self.add(self, "track_cook_09", "face_tracking")
        self.stateDict["track_cook_09"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "track_cook_09":
                if self.stateDict[desire] == Event.ACC_ON:
                    # Keep tracking as we continue sequence: no remove here
                    self.add(self, "inform_cook_09", "Talking",  params = self.order)
                    self.stateDict["inform_cook_09"] = Event.DES_ON
                    return None

            if desire == "inform_cook_09": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("inform_cook_09")
                    self.stateDict.pop("inform_cook_09")
                    self.add(self, "Listening_cook_09", "Listening",  params="{context: 'order_ready'}") # TODO Fix params
                    self.stateDict["Listening_cook_09"] = Event.DES_ON   
                    return None

            if desire == "Listening_cook_09": 
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("Listening_cook_09")
                    self.stateDict.pop("Listening_cook_09")
                    return "state_10"

        return None

    def cleanup(self):
        self.remove("track_cook_09")
        self.stateDict.pop("track_cook_09")
