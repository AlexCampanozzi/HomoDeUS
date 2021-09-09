#!/usr/bin/env python

import roslib
import rospy
from hbba_msgs.msg import *
from hbba_msgs.srv import *
from emotions_msgs.msg import Intensity

import os
import sys

if ("-c" in sys.argv):
    if (os.fork() == 0):
        os.system("roslaunch /root/tiago_ws/src/HomoDeUS/homodeus_hbba_cfg/./hbba_struct.launch")
        sys.exit(0)

rospy.init_node("hbba_struct", anonymous=True)

rospy.wait_for_service("hbba/add_desires", 30.0)
add_desires = rospy.ServiceProxy("hbba/add_desires", AddDesires)

pubEmoIntensity = rospy.Publisher("/emo_intensity", 
                                  Intensity,
                                  latch=True,
                                  queue_size=1)

desire_tiago_slam_static = Desire()
desire_tiago_slam_static.id = "tiago_slam_static"
desire_tiago_slam_static.type = "SLAM"
desire_tiago_slam_static.utility = 1
desire_tiago_slam_static.intensity = 1.0
desire_tiago_slam_static.params = ""
desire_tiago_slam_static.security = False
add_desires([desire_tiago_slam_static])



print "Stop this script with Ctrl-C when ready."
rospy.spin()
