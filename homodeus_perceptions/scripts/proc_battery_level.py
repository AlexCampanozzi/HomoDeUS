#! /usr/bin/env python
import os
import rospy
import traceback

from std_msgs.msg import String
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Battery_level:
    """
    This class publishes the battery level
    """

    def __init__(self):
        """
        This method initializes the perception module 
        """
        self.output_perc = rospy.Publisher("/proc_output_batterie_level", String, queue_size=10)

        self.input_perc = rospy.Subscriber("/power_status/charge", Float32, self.perc_to_level)

    def transform(self):
        """
        This method transform the input which is a percentage into a battery level which is a string
        """
        while not rospy.is_shutdown():
            batteryLevel = self.perc_to_level()
            self.output_perc.publish(batteryLevel)

    def perc_to_level(self):
        """
        This method take a percentage and output 
        """
        batteryPerc = self.input_perc()

        if 100 > batteryPerc >= 20:
            level = "high"
            return level

        elif 20 > batteryPerc >= 10:
            level = "medium"
            return level

        elif 10 > batteryPerc >= 5:
            level = "low"
            return level

        elif 5 > batteryPerc:
            level = "critical low"
            return level

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
        node = Battery_level()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())
        