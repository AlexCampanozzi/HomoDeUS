#! /usr/bin/env python

import roslib
#import sys
import unittest
import rostest
import rospy
#import script we are testing
#import speech_recognizer
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
#rostest HomoDeUS test_speech.test



#setting your test class
class SpeechRecognizerTest(unittest.TestCase):
    # each def is a different test that we are testing from the script
    #def setUp(self):
        #self.headActionClient = HeadActionClient()

    def test_gotoPosition(self):
        self.assertTrue(True)
    def test_testing(self):
        pass
    #def test_say(self):
    #def test_interrupt(self):
    #def test_wait_for_keyword(self):
    #def test_speech_to_text(self):
    #def test_set_keyword(self):
    #def test_run(self):
        
if __name__ == '__main__':
    # next two ligne require to wrap test with rosunit to produce XML result
    #import rostest
    rostest.rosrun('HomoDeUS', 'test_speech', SpeechRecognizerTest)
    #rostest.rosrun(package_name, test_name, test_case_class)