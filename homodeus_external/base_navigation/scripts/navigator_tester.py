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
        print "attempting to go to (x=0, y=-5, w=1). Will have to find a way around the wall"
        nav.goto(0, -5, 1)
        print "registering this point as a landmark"
        nav.registerLandmark("testPoint")
        print "going back to origin"
        nav.goto(0,0,0)
        print "going to registered landmark"
        nav.goToLandmark("testPoint")
    
    except rospy.ROSInterruptException:
        # use if cancel fucks up when server is unavailalble
        # pass
        nav.cancelAllGoto
