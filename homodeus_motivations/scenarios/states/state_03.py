import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State03(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_03"

    def add_state_desires(self):
        self.add("track_customer_03", "face_tracking")
        self.add("greet_customer_03", "Talking",  params="{TtsText: 'Hello human, thank you for coming.'}")
        self.stateDict["track_customer_03"] = Event.DES_ON
        self.stateDict["greet_customer_03"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "greet_customer_03":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("greet_customer_03")
                    self.stateDict.pop("greet_customer_03")
                    self.add("ask_for_order_03", "Talking",  params="{TtsText: 'Please order something from our menu so I may be of service.'}")
                    self.stateDict["ask_for_order_03"] = Event.DES_ON
                    return None

            if desire == "ask_for_order_03":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("ask_for_order_03")
                    self.stateDict.pop("ask_for_order_03")
                    self.add("listen_for_order_03", "Listening",  params="{context: 'menu'}") # TODO Fix params
                    self.stateDict["listen_for_order_03"] = Event.DES_ON
                    return None

            if desire == "listen_for_order_03":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_order_03")
                    self.stateDict.pop("listen_for_order_03")
                    return "state_06"

                # heard stuff not on menu
                elif self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("listen_for_order_03")
                    self.stateDict.pop("listen_for_order_03")
                    return "state_02"

                # timeout
                elif self.stateDict[desire] == Event.IMP_ON:
                    self.remove("listen_for_order_03")
                    self.stateDict.pop("listen_for_order_03")
                    return "state_04"

        return None

    def cleanup(self):
        self.remove("track_customer_03")
        self.stateDict.pop("track_customer_03")
