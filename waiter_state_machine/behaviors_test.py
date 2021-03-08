#! /usr/bin/env python

""" /!\ THIS FILE IS ONLY USED FOR TESTING PURPOSES /!\ """

from behaviors import *

test = "locomotion"
print("Testing behavior: " + test)


if test == "voice":

    # Init the node
    rospy.init_node('voice_test_node')

    # Setting up the parameters
    params = {
        "speech": "Hello, I am TIAGo",
        "language": "en_GB"
    }

    # Running the behavior
    voice = Voice()
    voice.activate()
    voice.run(params)


elif test == 'face_tracking':

    # Init the node
    rospy.init_node('face_tracking_test_node')

    # Running the behavior
    face_tracker = FaceTracking()
    face_tracker.activate()


elif test == "voice_recognition":

    # Init the node
    rospy.init_node('voice_recognition_test_node')

    # Setting up the parameters
    params = {
        "language": "en-us",
        "skip_keyword": "False",
        "tell_back": "True"
    }

    # Running the behavior
    voice_recognition = VoiceRecognition()
    voice_recognition.activate()
    voice_recognition.run(params)

    # Print the results
    print(voice_recognition.speech)


elif test == "locomotion":

    # Init the node
    rospy.init_node('locomotion_test_node')

    # Setting up the parameters
    params = {
        "x": "0",
        "y": "0.3",
        "orientation": "-0.90"
    }

    # Running the behavior
    locomotion = Locomotion()
    locomotion.activate()
    locomotion.run(params)


rospy.spin()
