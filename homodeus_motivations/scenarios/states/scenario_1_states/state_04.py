import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Desire, Event
from ..state import StateBase

class State04(StateBase):
    def __init__(self, stateDict):
        StateBase.__init__(self, stateDict)
        self.command = ""

    def _set_id(self):
        return "Place_Object"

    def add_state_desires(self):
        print("04 init")
        self.add("thank_client_04", "Talking",  params = "{TtsText: 'Here is your " + self.command + " Have a great meal!'}")
        self.add("place_object_04", "ListenForPlace")
        self.add("get_drop_pose_04","DropSpotPerception")
        self.stateDict["place_object_04"] = Event.DES_ON
        self.stateDict["thank_client_04"] = Event.DES_ON
        self.stateDict["get_drop_pose_04"] = Event.DES_ON
        

    def react_to_event(self): ## Ignore the other desires
        if self.stateDict["place_object_04"] == Event.ACC_ON:
            return True
        elif self.stateDict["place_object_04"] == Event.ACC_OFF:
            return False
        else:
            return None

    def cleanup(self):
        self.remove("thank_client_04")
        self.remove("place_object_04")
        self.remove("get_drop_pose_04")

        self.stateDict.pop("place_object_04")
        self.stateDict.pop("thank_client_04")
        self.stateDict.pop("get_drop_pose_04")
