import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State06(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.fail_count = 0
        self.max_fail_count = 3
        self.order = ""
        rospy.Subscriber("/obs_client_order", String, self.order_cb, queue_size=5)

    def order_cb(self, order_string):
        self.order = "{TtsText: 'You ordered: " + order_string.data + ", correct?" + "'}"

    def _set_id(self):
        return "state_06"

    def add_state_desires(self):
        self.add("track_customer_06", "face_tracking")
        self.stateDict["track_customer_06"] = Event.DES_ON

        self.add("repeat_order_06", "Talking",  params = self.order)
        self.stateDict["repeat_order_06"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:

            if desire == "repeat_order_06":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("repeat_order_06")
                    self.stateDict.pop("repeat_order_06")
                    self.add("listen_for_answer_06", "Listening",  params="{context: 'yes_or_no'}") # TODO Fix params
                    self.stateDict["listen_for_answer_06"] = Event.DES_ON
                    return None

            if desire == "listen_for_answer_06":
                if self.stateDict[desire] == Event.ACC_ON:
                    self.remove("listen_for_answer_06")
                    self.stateDict.pop("listen_for_answer_06")
                    return "state_07"

            if desire == "listen_for_answer_06":
                if self.stateDict[desire] == Event.ACC_OFF:
                    self.remove("listen_for_answer_06")
                    self.stateDict.pop("listen_for_answer_06")
                    return "state_05"

            if desire == "listen_for_answer_06":
                if self.stateDict[desire] == Event.IMP_ON and self.fail_count < self.max_fail_count:
                    self.remove("listen_for_answer_06")
                    self.stateDict.pop("listen_for_answer_06")
                    self.add("repeat_order_06", "Talking",  params = self.order)
                    self.stateDict["repeat_order_06"] = Event.DES_ON
                    self.fail_count += 1
                    return None

                elif self.stateDict[desire] == Event.IMP_ON and self.fail_count >= self.max_fail_count:
                    self.remove("listen_for_answer_06")
                    self.stateDict.pop("listen_for_answer_06")
                    return "state_02"

        return None

    def cleanup(self):
        self.remove("track_customer_06")
        self.stateDict.pop("track_customer_06")
        self.fail_count = 0
