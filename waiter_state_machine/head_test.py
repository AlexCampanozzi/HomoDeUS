#! /usr/bin/env python

import rospy
from headActionClient import *

head_client = HeadActionClient()

def main():
    # rospy.init_node("head_test_node")

    mode = input("Mode (angle or cart): ")

    if mode == "angle":
        while not rospy.is_shutdown():
            theta = input("Angle 1: ")
            azimuth = input("Angle 2: ")
            print(" ")

            head_client.GotoAngle(float(theta), float(azimuth))
            rospy.sleep(0.1)

    else:
        while not rospy.is_shutdown():
            x = input("X: ")
            y = input("Y: ")
            print(" ")

            head_client.GotoPosition(float(x), float(y))
            rospy.sleep(0.1)

main()


