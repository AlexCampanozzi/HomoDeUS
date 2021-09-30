import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State02(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.fail_count  = 0
        self.max_fail_count  = 3

    def _set_id(self):
        return "state_02"

    def add_state_desires(self):
        self.add("track_customer_02", "face_tracking")
        self.stateDict["track_customer_02"] = Event.DES_ON
        self.add("apology_02", "Talking",  params="{TtsText: 'Sorry, I did not catch that. Make sure that you are asking for something on the menu.'}")
        self.stateDict["apology_02"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "apology_02":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("apology_02")
                    self.stateDict.pop("apology_02")
                    self.add("ask_for_order_02", "Talking",  params="{TtsText: 'Hello Human, what kind of food do you want? Please choose from the menu.'}")
                    self.stateDict["ask_for_order_02"] = Event.DES_ON
                    return None

            if desire == "ask_for_order_02":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("ask_for_order_02")
                    self.stateDict.pop("ask_for_order_02")
                    self.add("listen_for_order_02", "Listening",  params="{context: 'menu'}")
                    self.stateDict["listen_for_order_02"] = Event.DES_ON
                    return None

            if desire == "listen_for_order_02":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_order_02")
                    self.stateDict.pop("listen_for_order_02")
                    return "state_06"

            if desire == "listen_for_order_02":
                if self.stateDict[desire] == Event.ACC_OFF and self.fail_count < self.max_fail_count:
                    self.remove("listen_for_order_02")
                    self.stateDict.pop("listen_for_order_02")
                    self.add("apology_02", "Talking",  params="{TtsText: 'Sorry, I did not catch that. Make sure that you are asking for something on the menu.'}") # TODO Fix params?
                    self.stateDict["apology_02"] = Event.DES_ON
                    self.fail_count += 1
                    return None
                elif self.stateDict[desire] == Event.ACC_OFF and self.fail_count >= self.max_fail_count:
                    self.remove("listen_for_order_02")
                    self.stateDict.pop("listen_for_order_02")
                    return "state_04"

            if desire == "listen_for_order_02":
                if self.stateDict[desire] == Event.IMP_OFF:
                    self.remove("listen_for_order_02")
                    self.stateDict.pop("listen_for_order_02")
                    return "state_04"

        return None

    def cleanup(self):
        self.remove("track_customer_02")
        self.stateDict.pop("track_customer_02")
        self.fail_count = 0
