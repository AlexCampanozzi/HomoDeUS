#! /usr/bin/env python

import roslib
#import sys
import unittest
#import rostest
import rospy
#import script we are testing
import navigator
# test Cheat sheet
# assertEqual(a, b)     check=> a == b
# assertNotEqual(a, b)  check=> a != b
# assertTrue(x)         check=> bool(x) is True
# assertFalse(x)        check=> bool(x) is False
# assertIs(a, b)        check=> a is b
# assertIsNot(a, b)     check=> a is not b
# assertIsNone(x)       check=> x is None
# assertIsNotNone(x)    check=> x is not None

###run test###
#rostest base_navigation test_nav.test



#setting your test class
class NavigatorTest(unittest.TestCase):
    # each def is a different test that we are testing from the script
    # all test name must start with "test_" to be run

    def test_goto(self):
        self.assertTrue(True)
    def test_testing(self):
        pass
    #def test_gotoGoal
    #def test_registerLandmark
    #def test_goToLandmark
    #def test_cancelAllGoto
    #def test_getCurPose
        
if __name__ == '__main__':
    # next two ligne require to wrap test with rosunit to produce XML result
    import rostest
    rostest.rosrun('base_navigation', 'test_nav', NavigatorTest)
    #rostest.rosrun(package_name, test_name, test_case_class)