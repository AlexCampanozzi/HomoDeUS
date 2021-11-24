#! /usr/bin/env python
import os
import time
import rospy
import roslib
import actionlib
import threading
import control_msgs.msg
from geometry_msgs.msg import PoseStamped
import math
from pal_startup_msgs.srv import StartupStart, StartupStop
import rosservice

test_time = 4

def testThread():
    print(test_time)
    time.sleep(test_time)
    print(test_time)

if __name__ == "__main__":
    sending_thread = threading.Thread(target=testThread)
    sending_thread.start()
    time.sleep(2)
    test_time = 20

