import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State07(StateBase):
    def __int__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_07"

    def add_state_desires(self):
        self.add("inform_customer_07", "Talking",  params = "{TtsText: 'Thank you for ordering, I'm headed for the kitchen now.'}")
        self.stateDict["inform_customer_07"] = Event.DES_ON
        self.add("move_to_kitchen_07", "GoToLandmark",  params = "{name: 'kitchen'}")
        self.stateDict["move_to_kitchen_07"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "move_to_kitchen_07":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.add("register_customer_location_07", "AddLandmark",  params = "{name: 'customer'}")
                    self.stateDict["register_customer_location_07"] = Event.DES_ON
                    self.remove("move_to_kitchen_07")
                    self.stateDict.pop("move_to_kitchen_07")
                    return "state_09"

            if desire == "register_customer_location_07":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("register_customer_location_07")
                    self.stateDict.pop("register_customer_location_07")
                    return None

            if desire == "move_to_kitchen_07":
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("move_to_kitchen_07")
                    self.stateDict.pop("move_to_kitchen_07")
                    return "state_08"

        return None

    def cleanup(self):
        self.remove("inform_customer_07")
        self.stateDict.pop("inform_customer_07")
