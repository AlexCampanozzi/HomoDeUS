#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  2 21:51:00 2020

@author: CM
"""

import sys

sys.path.append(".")
from navigator import Navigator

import rospy

if __name__ == '__main__':
    try:
        nav = Navigator()
        # Easy destination: 
        # print "attempting to go to (x=1, y=1, w=1)"
        # nav.goto(1, 1, 1)
        # Harder destination (on the other side of a wall):
        rospy.Subscriber('/nav_landmark', Bool, nav.getCurPose, queue_size=5)
    
    except rospy.ROSInterruptException:
        # use if cancel fucks up when server is unavailalble
        # pass
        nav.cancelAllGoto
