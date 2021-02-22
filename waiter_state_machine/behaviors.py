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
from speech_recognition_server.msg import SpeechRecognitionActivatedResult
from headActionClient import *

from face_detection.msg import FacePosition
from face_detection.msg import FacePositions

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
        print("Face tracking constructing")
        # Setting up a head action client and a subscriber to /faces
        print("Face tracking constructing 2")
        rospy.Subscriber('/pal_face/faces', FacePositions, self._head_callback)

        # Collecting image settings
        self.img_width = 320 #rospy.get_param('processing_img_width')
        self.img_height = 240 #rospy.get_param('processing_img_height')

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

        self.threshold = 1

        # Timestamp updated each time a face is detected
        self.timestamp = time.time()

        self.head_controller = self.HeadController(self.img_center_x, self.img_center_y)

    def _run(self, params):
        # This method is not necessary for this behavior.
        pass

    def _head_callback(self, detections):
        print("HeadCallBack")
        # TODO: Maybe this should go after the activation check?
        self.timestamp = time.time()

        if not self.active:
            return

        main_face_x = 0
        main_face_y = 0
        main_face_dist = 1000000

        # Find the closest face to the image center (main face)
        for face in detections.faces:
            face_x, face_y = self._get_face_center_position(face)
            face_dist = self._distance_from_img_center(face_x, face_y)

            if (face_dist < main_face_dist):
                main_face_x = face_x
                main_face_y = face_y
                main_face_dist = face_dist

        # If the main face is inside the limit, don't move the head
        if main_face_dist < self.threshold:
            return

        # Simple proportional controller
        self.head_controller.goal_x = float(main_face_x)
        self.head_controller.goal_y = float(main_face_y)

    def _distance_from_img_center(self, x, y):
        return math.sqrt((self.img_center_x - x)**2 + (self.img_center_y)**2)

    def _get_face_center_position(self, face):
        x = face.x + (face.width // 2)
        y = face.y + (face.height // 2)

        return x, y

    class HeadController:
        def __init__(self, center_x, center_y):
            self.goal_x = float(center_x)
            self.goal_y = float(center_y)
            self.center_x = float(center_x)
            self.center_y = float(center_y)
            
            self.K = 0.1

            self.head_client = HeadActionClient() # <------ WARNING: init_node is used in this class!!

            thread = threading.Thread(target=self.control_loop)
            thread.start()

        def control_loop(self):
            while not rospy.is_shutdown():
                error_x = (self.goal_x - self.center_x) / self.center_x
                error_y = (self.goal_y - self.center_y) / self.center_y

                cmd_x = self.K * error_x
                cmd_y = self.K * error_y
                #print(cmd_x)
                #print(cmd_y)
                self.head_client.GotoPosition(cmd_x, cmd_y)

                rospy.sleep(0.01)


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

        self.stt_client.send_goal(goal)
        self.stt_client.wait_for_result(timeout=rospy.Duration(30.))
        result = self.stt_client.get_result()
        if result == None:
            self.speech = ""
        else:
            self.speech = result.recognition_results
        if self.speech != None:
            print("understood: " + self.speech)
        # Wait for the server to finish
        #self.stt_client.wait_for_result()

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
        elif self.speech == None:
            return
        else:
            goal = pal_interaction_msgs.msg.TtsGoal()
            goal.rawtext.lang_id = self.language
            goal.rawtext.text = self.speech

            self.tts_client.send_goal(goal)
            rospy.sleep(5.)


class Locomotion(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)
        # if we can't use the navigator, we need to init the client directly
        self.navigator = Navigator()
        print("initLocomotion")
    def _run(self, params):
	    # if we can't use the navigator, the client need to do the calls
        x = float(params["x"])
        y = float(params["y"])
        orientation = float(params["orientation"])
        print("runLocomotion")
        return(self.navigator.goto(x, y, orientation))
