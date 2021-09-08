import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from state import StateBase

class State00(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)

    def _set_id(self):
        return "state_00"

    def add_state_desires(self):
        print("00 init")
        self.add("detect_customer_00", "face_detection")
        self.add("hear_hotword_00", "Keyword_detection",  params="{value: 'roboto'}")
        self.add("register_customer_location_00", "AddLandmark",  params = "{name: 'kitchen', x: 1, y: 1, yaw: 0}")
        self.stateDict["register_customer_location_00"] = Event.DES_ON
        self.stateDict["detect_customer_00"] = Event.DES_ON
        self.stateDict["hear_hotword_00"] = Event.DES_ON

    def react_to_event(self):
        for desire in self.stateDict:
            if self.stateDict[desire] == Event.ACC_ON:
                return "state_03"

    def cleanup(self):
        self.remove("detect_customer_00")
        self.stateDict.pop("detect_customer_00")
        self.remove("hear_hotword_00")
        self.stateDict.pop("hear_hotword_00")
        self.remove("register_customer_location_00")
        self.stateDict.pop("register_customer_location_00")
