#! /usr/bin/env python

import rospy
from headActionClient import * 

rospy.init_node("head_test_node")

head_client = HeadActionClient()

def main():import

    #mode = input("Mode (angle or cart): ")
    mode = "pos_abs"
    if mode == "angle":
        while not rospy.is_shutdown():
            theta = input("Angle 1: ")
            azimuth = input("Angle 2: ")
            print(" ")

            head_client.GotoAngle(float(theta), float(azimuth))
            rospy.sleep(0.1)

    elif mode == "pos_abs":
        while not rospy.is_shutdown():
            x = input("X: ")
            y = input("Y: ")
            print(" ")

            head_client.GotoPositionAbsolute(float(x), float(y))
            rospy.sleep(0.1)

    else:
        while not rospy.is_shutdown():
            x = input("X: ")
            y = input("Y: ")
            print(" ")

            head_client.GotoPosition(float(x), float(y))
            rospy.sleep(0.1)

main()


