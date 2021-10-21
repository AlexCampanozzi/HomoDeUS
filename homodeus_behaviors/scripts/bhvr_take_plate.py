#! /usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import HomoDeUS_common_py.HomoDeUS_common_py as common  

class GivePlate:
    def __init__(self):
        rospy.loginfo("bhvr_give_plate constructing")

        #rospy.Subscriber('/proc_output_face_positions', FacePositions, self._head_callback, queue_size=5)

        self.pub = rospy.Publisher('tiago_arm_controller', PoseStamped, queue_size=5)
        self.pubObserver = rospy.Publisher('/Face_tracking_observer', Bool, queue_size=5)

    def _arm_callback(self):

        poseStamped = PoseStamped()

        poseStamped.pose.position.x = 0
        poseStamped.pose.position.y = 0

        #rospy.loginfo("sent pose")

        self.pub.publish(poseStamped)

if __name__ == "__main__":

    try:
        rospy.init_node(common.get_file_name(__file__))
        givePlate = GivePlate()
        rospy.spin()

    except rospy.ROSInterruptException:
        print("except")
        pass
