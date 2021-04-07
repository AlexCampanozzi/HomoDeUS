#! /usr/bin/env python
import os
import rospy
import traceback

from std_msgs.msg import String, Float32
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Battery_level:
    """
    This class publishes the battery level
    """

    def __init__(self):
        """
        This method initializes the perception module 
        """
        self.output_perc = rospy.Publisher("/proc_output_battery_level", String, queue_size=10)

        self.input_perc = rospy.Subscriber("/power_status/charge", Float32, self.perc_to_level)
        #self.input_perc = rospy.Subscriber("fake_battery_percent", Float32, self.perc_to_level) #use for test,put power_status/charge in comment
        self.level = "high"

    def transform(self):
        """
        This method transform the input which is a percentage into a battery level which is a string
        """
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            #batteryLevel = self.input_perc.callback()
            rospy.loginfo("%s", self.level)
            self.output_perc.publish(self.level)
            rate.sleep()

    def perc_to_level(self, perc):
        """
        This method take a percentage and output 
        """
        self.batteryPerc = perc

        if 100 > self.batteryPerc.data >= 20:
            self.level = "high"
            

        elif 20 > self.batteryPerc.data >= 10:
            self.level = "medium"
            

        elif 10 > self.batteryPerc.data >= 5:
            self.level = "low"
            

        elif 5 > self.batteryPerc.data:
            self.level = "critical low"

        #self.output_perc.publish(self.level)
            

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
        node.transform()
        rospy.on_shutdown(node.node_shutdown)

        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())
        