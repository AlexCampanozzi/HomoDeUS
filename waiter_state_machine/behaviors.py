#! /usr/bin/env python

# TODO: Include dependencies
import time
import math
import threading
import rospy
import actionlib
from pal_detection_msgs.msg import FaceDetections
from speech_recognition_server.msg import SpeechRecognitionActivatedAction
from speech_recognition_server.msg import SpeechRecognitionActivatedGoal
from speech_recognition_server.msg import SpeechRecognitionActivatedFeedback


#section for Locomotion
from navigator import *

import pal_interaction_msgs.msg
from std_msgs.msg import String

class BehaviorBase:
    """
    Base class from which every behavior inherits from. Each child has
    to provide an implementation for the following methods:
        0. __init__()
        1. _run()
    """
    def __init__(self):
        self.active = False

    def activate(self):
        """
        Activate the behavior.
        """
        self.active = True

    def deactivate(self):
        """
        Deactivate the behavior.
        """
        self.active = False

    def run(self, params=None):
        """
        Execute the _run() method.

        Arguments
        ---------
        params : dict
            A dictionary to pass custom parameters to the _run() method.
        """
        if self.active:
            return self._run(params)

    def _run(self, params):
        """
        Actions associated to the behavior.

        Arguments
        ---------
        params : dict
            A dictionary with the needed parameters.
        """
        raise NotImplementedError()


"""
+-------------------------------------------------+
|                    Behaviors                    |
+-------------------------------------------------+
"""


class FaceTracking(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)

        # Setting up a head action client and a subscriber to /faces
        self.head_client = HeadActionClient() # <------ WARNING: init_node is used in this class!!
        rospy.Subscriber('/pal_face/faces', FaceDetections, self._head_callback)

        # Collecting image settings
        self.img_width = 320 #rospy.get_param('processing_img_width')
        self.img_height = 240 #rospy.get_param('processing_img_height')

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

        self.threshold = 10

    def _run(self, params):
        # This method is not necessary for this behavior.
        pass

    def _head_callback(self, detections):

        if not self.active:
            return

        main_face_x = 0
        main_face_y = 0
        main_face_dist = 1000000

        # Find the closest face to the image center (main face)
        for face in detections.faces:
            face_x, face_y = _get_face_center_position(face)
            face_dist = _distance_from_img_center(face_x, face_y)

            if (face_dist < main_face_dist):
                main_face_x = face_x
                main_face_y = face_y
                main_face_dist = face_dist

        # If the main face is inside the limit, don't move the head
        if main_face_dist ­­­< self.threshold:
            return

        # Simple proportional controller
        K = 0.5
        
        error_x = (main_face_x - self.img_center_x) // self.img_center_x
        error_y = (main_face_y - self.img_center_y) // self.img_center_y

        cmd_x = K * error_x
        cmd_y = K * error_y

        # Send angle command to move the head
        self.head_client.GoToAngle(cmd_x, cmd_y)
        
    def _distance_from_img_center(self, x, y):
        return math.sqrt((self.img_center_x - x)**2 + (self.img_center_y)**2)

    def _get_face_center_position(face):
        x = face.x + (face.width // 2)
        y = face.y + (face.height // 2)

        return x, y


class VoiceRecognition(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)
        
        # Setting up the client
        self.stt_client = actionlib.SimpleActionClient("speech_recognition_action_server", SpeechRecognitionActivatedAction)
        self.stt_client.wait_for_server()

        self.speech = ""

    def _run(self, params):
        language = params["language"]
        skip_keyword = (params["skip_keyword"] == "True")
        tell_back = (params["tell_back"] == "True")

        goal = SpeechRecognitionActivatedGoal()
        goal.language = language
        goal.skip_keyword = skip_keyword
        goal.tell_back = tell_back

        self.stt_client.send_goal(goal, feedback_cb=self._feedback_callback)

        # Wait for the server to finish
        self.stt_client.wait_for_result()

    def _feedback_callback(self, feedback):
        self.speech = feedback.recognition_results


class Voice(BehaviorBase):
    def __init__(self):
        """
        This method initializes the submodule used for text-to speech.
        
        Arguments
        ---------
            None
        """
        BehaviorBase.__init__(self)

        # Setting up Text-to-Speech
        self.tts_client = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)
        self.tts_client.wait_for_server()
        self.language = "en_GB"

    def _run(self, params):
        """
        Actions associated to the behavior.

        Arguments
        ---------
        params : dict
            A dictionary with the needed parameters.
        """
        self.speech = params["speech"]
        self.language = params["language"]
        
        if not self.active:
            return
        else:
            goal = pal_interaction_msgs.msg.TtsGoal()
            goal.rawtext.lang_id = self.language
            goal.rawtext.text = self.speech

            self.tts_client.send_goal(goal)


class Locomotion(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)
        # if we can't use the navigator, we need to init the client directly
        self.navigator = Navigator()

    def _run(self, params):
	    # if we can't use the navigator, the client need to do the calls
        x = float(params["x"])
        y = float(params["y"])
        orientation = float(params["orientation"])
        
        self.navigator.goto(x, y, orientation)
