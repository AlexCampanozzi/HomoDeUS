#! /usr/bin/env python

import rospy
import actionlib
import traceback

from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

import HomoDeUS_common_py.HomoDeUS_common_py as common

class Rest_the_robot:

     def __init__(self):

        self.input_motv =rospy.Subscriber("/proc_output_batterie_level", String, self.battery_cb, queue_size=10)

        # the output of the module
        self.event_Subcriber = rospy.Subscriber("events",Event, self.listen_event_cb , queue_size = 10)

        self.add_desires_service = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires_service = rospy.ServiceProxy('remove_desires', RemoveDesires)

    def add_desire(self,desire_id, desire_type,desire_utility,desire_intensity, desire_params=None):
        des = Desire()
        des.id          = desire_id
        des.type        = desire_type
        des.utility     = desire_utility
        des.intensity   = desire_intensity
        if desire_param is not None:
            des.params = desire_params
        
        self.add_desires_service.call([des])

    def battery_cb(self, level):
        if level == "high":
            pass # do nothing
        elif level == "medium":
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=5)
        elif level == "low":
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=20)
        elif level == "critical low":
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=100)
    
    def wait_to_long(self):
        #TODO sur quoi je veux me timer( dernier mvt,dernier scenario entrepris, etc)
        if timer > 10 #en minute, surment changer en seconde
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=50)

if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Rest_the_robot()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())