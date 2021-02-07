#! /usr/bin/env python

from behaviors import *

test = "face_tracking"  

if test == "voice":
    rospy.init_node('keyword_speech_multi_recognizer_server')

    params = {"speech": "Yo I am TIAGo", "language": "en_GB"}
    voice = Voice()
    voice.activate()
    voice.run(params)

    rospy.spin()

elif test == 'face_tracking':
    rospy.init_node('behaviors_test_node')

    face_tracker = Facetracking()
    face_tracker.activate()
    
    rospy.spin()
