#! /usr/bin/env python
import rospy
import actionlib
import traceback

from std_msgs.msg import String, Bool, Float32
from hbba_msgs.msg import Desire, DesiresSet, Event

import HomoDeUS_common_py.HomoDeUS_common_py as common

class Battery_level_observer:
    """
    This class follows the state of the battery level/survival desire
    """
    def __init__(self):
        self.input_motv =rospy.Subscriber("/power_status/charge", Float32,self.desire_event_change, queue_size=10)

        # the output of the module
        self.event_publisher = rospy.Publisher("events",Event, queue_size = 10)

        # following DesireSet
        self.desires_set_subscriber = rospy.Subscriber("desires_set", DesiresSet, self.listen_desires_set_Cb)

    def listen_desires_set_Cb(self,desireSet):
        """
        This method updated the desireSet used in the desire_event_change method.

        Arguments
        ---------
        desireSet: DesiresSet
            The most recent desireSet
        """
        self.curDesireSet = desireSet

    def desire_event_change(self, battery_percent):
        #TODO add desire section to change
        if self.accomplish_criterion(battery_percent):
            event.type = Event.ACC_ON
            self.event_publisher.publish(event)

    def accomplish_criterion(self, battery_percent):
        """
        This method looks the result of the module and return True if it accomplished the criterion the motivation is looking for
        """
        
        if battery_percent < 50.0: #TODO, presentement hardcoder, mais pourrais etre controler HBBA par une combinaison de cost des desire, 
            ropsy.loginfo("Power level below norme")
            return True
        else:
            return False
        
    def cancel_criterion(self):
        """
        This method follows up the environment of the robot and return True if
         it is now impossible to accomplish the purpose of the desire.
        """
        #should look for the ambiant noise and if it is too high for a certain time, consider it is impossible
        return False
        
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
        node = Battery_level_observer()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())

