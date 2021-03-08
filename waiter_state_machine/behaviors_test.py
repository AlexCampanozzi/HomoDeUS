#! /usr/bin/env python

from behaviors import *

test = "locomotion"
print(test)

if test == "voice":
    rospy.init_node('keyword_speech_multi_recognizer_server')

    params = {"speech": "Yo I am TIAGo", "language": "en_GB"}
    voice = Voice()
    voice.activate()
    voice.run(params)

    rospy.spin()

elif test == 'face_tracking':

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
    print(voice_recognition.speech)

    rospy.spin()

elif test == "locomotion":
    rospy.init_node('base_cmds', anonymous=False)
    locomotion = Locomotion()
    locomotion.activate()
    
    params = {
        "x" : "0",
        "y" : "0.3",
        "orientation" : "-0.90"
    }

    locomotion.run(params)

    rospy.spin()
