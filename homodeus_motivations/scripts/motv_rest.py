#! /usr/bin/env python

import rospy
import actionlib
import traceback
import time

from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from custom_actions.msg import ScenarioSelectorActivatedAction, ScenarioSelectorActivatedGoal

import HomoDeUS_common_py.HomoDeUS_common_py as common

class Rest_the_robot:

    def __init__(self):

        self.input_motv =rospy.Subscriber("/proc_output_battery_level", String, self.battery_cb, queue_size=10)

        # the output of the module
        #self.event_Subcriber = rospy.Subscriber("events",Event, self.wait_to_long , queue_size = 10)

        self.add_desires_service = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires_service = rospy.ServiceProxy('remove_desires', RemoveDesires)
        #rospy.wait_for_service("add_desires")
        self.timer_since_event = time.time()
        self.timer_event = time.time()

    def add_desire(self,desire_id, desire_type,desire_utility,desire_intensity, desire_params=None):
        des = Desire()
        des.id          = desire_id
        des.type        = desire_type
        des.utility     = desire_utility
        des.intensity   = desire_intensity
        if desire_param is not None:
            des.params = desire_params

        rospy.loginfo("adding desire: " + des.id)
        self.add_desires_service.call([des])
    
    def remove(self):
        self.rem_desires.call(["rest"])

    #def go_rest(self):
        """
        this method will loop between checking battery level and wait time
        """
        #while not rospy.is_shutdown():
            #self.battery_cb()
            #self.wait_to_long()

    def battery_cb(self, level_giving):
        """
        This method monitor the battery level of the robot. It will creat desire to go recharge, getting stronger as the battery level diminish
        """
        #TODO this is need to be tested
        self.level = level_giving
        if self.level == "high":
            rospy.loginfo("%s", self.level)
            pass # do nothing

        elif self.level == "medium":
            rospy.loginfo("%s", self.level)
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=5)

        elif self.level == "low":
            rospy.loginfo("%s", self.level)
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=20)
            
        elif self.level == "critical low":
            rospy.loginfo("%s", self.level)
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=100)
    
    def wait_to_long(self):
        
        self.timer_since_event = time.time()
        #TODO this is untested 
        if goal.start_scenario == True: #TODO check version final de scenario selector
            self.timer_event = time.time()
        
        timer = self.timer_since_event-self.timer_event

        if timer > 1200: #1200 second is 20 min
            self.add_desire(desire_id="rest",desire_type="Recharge",desire_utility=8,desire_intensity=5)
        
        
    
    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        common.loginfo(self,"have been shutdown")

if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Rest_the_robot()
        rospy.on_shutdown(node.node_shutdown)
        #node.go_rest()
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())