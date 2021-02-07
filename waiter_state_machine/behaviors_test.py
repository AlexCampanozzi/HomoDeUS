#! /usr/bin/env python

from behaviors import *

test = "voice"  

if test == "voice":
    rospy.init_node('keyword_speech_multi_recognizer_server')

    params = {"speech": "Yo I am TIAGo", "language": "en_GB"}
    voice = Voice()
    voice.activate()
    voice.run(params)

    rospy.spin()
