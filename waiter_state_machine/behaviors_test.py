#! /usr/bin/env python

from behaviors import *

test = "voice_recognition"  

if test == "voice":
    rospy.init_node('keyword_speech_multi_recognizer_server')

    params = {"speech": "Yo I am TIAGo", "language": "en_GB"}
    voice = Voice()
    voice.activate()
    voice.run(params)

    rospy.spin()

elif test == 'face_tracking':
    rospy.init_node('behaviors_test_node')

    face_tracker = FaceTracking()
    face_tracker.activate()
    
    rospy.spin()

elif test == "voice_recognition":
    rospy.init_node('behaviors_test_node')

    voice_recognition = VoiceRecognition()
    voice_recognition.activate()

    params = {
        "language": "en-us",
        "skip_keyword": "False",
        "tell_back": "True"
    }

    voice_recognition.run(params)

    rospy.spin()

elif test == "locomotion":
    locomotion = Locomotion()

    params = {
        "x" : "1",
        "y" : "1",
        "orientation" : "0"
    }

    locomotion.run(params)

    rospy.spin()