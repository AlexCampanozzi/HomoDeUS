#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  2 21:51:00 2020

@author: root
"""


import sys

sys.path.append(".")
from navigator import Navigator

import rospy

if __name__ == '__main__':
    try:
        nav = Navigator()
        print "attempting to go to (x=1, y=1, w=1)"
        nav.goto(1, 1, 1)
        print "registering this point as a landmark"
        nav.registerLandmark("testPoint")
        print "going back to origin"
        nav.goto(0,0,0)
        print "going to registered landmark"
        nav.goToLandmark("testPoint")
    except rospy.ROSInterruptException:
        nav.cancelAllGoto
