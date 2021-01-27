#! /usr/bin/env python
import sys
import rospy

sys.path.append(".")
from headActionClient import HeadActionClient

if __name__ == '__main__':

    client = HeadActionClient()
    client.GotoAngle(-30, 0)
    rospy.sleep(2)
    client.GotoAngle(30, 0)
