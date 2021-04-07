#! /usr/bin/env python
import os
import rospy
import traceback

from std_msgs.msg import Float32
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Tester_battery_Level:

    def __init__(self):

        self.ouput_perc = rospy.Publisher("fake_battery_percent", Float32, queue_size=10)

    def fake_batt_lev(self):
        while not rospy.is_shutdown():
            rate = rospy.Rate(10)
            battery_lie = 15.0
            rospy.loginfo(battery_lie)
            self.ouput_perc.publish(battery_lie)
            rate.sleep()

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
        node = Tester_battery_Level()
        node.fake_batt_lev()

        rospy.on_shutdown(node.node_shutdown)
        
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())