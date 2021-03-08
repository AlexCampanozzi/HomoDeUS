#! /usr/bin/env python

import math
import rospy
import threading
import actionlib
from navigator import *
from headActionClient import *
import pal_interaction_msgs.msg
from std_msgs.msg import String
from face_detection.msg import FacePosition
from face_detection.msg import FacePositions
from pal_detection_msgs.msg import FaceDetections
from speech_recognition_server.msg import SpeechRecognitionActivatedAction
from speech_recognition_server.msg import SpeechRecognitionActivatedGoal
from speech_recognition_server.msg import SpeechRecognitionActivatedResult


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
        Executes the _run() method.

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
        """
        This method subscribes to the face detection topic and instantiates a
        head controller for the face tracking functionality.

        Arguments
        ---------
            None
        """
        BehaviorBase.__init__(self)

        # Subscribing to the face detection topic
        rospy.Subscriber('/pal_face/faces', FacePositions, self._head_callback)

        # Extracting image settings
        self.img_width = 320
        self.img_height = 240

        self.img_center_x = self.img_width // 2
        self.img_center_y = self.img_height // 2

        self.threshold = 1

        # Timestamp updated each time a face is detected
        self.timestamp = rospy.get_time()

        # The head controller that will be running in another thread
        self.head_controller = self.HeadController(
            self.img_center_x,
            self.img_center_y
        )

        rospy.loginfo("[Behaviors] Face tracking is ready to run")

    def _run(self, params):
        # This method is not necessary for this behavior.
        pass

    def _head_callback(self, detections):
        """
        This method is called every time a face is detected. If the behavior is
        active, it will update the controller goal to move the head toward the
        most centered face.

        Arguments
        ---------
            detections : FaceDetections
                The positions and sizes of the faces that were detected
        """
        self.timestamp = rospy.get_time()

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

        # If the main face is already centered, we can stop right here
        if main_face_dist < self.threshold:
            return

        # Updating the controller goal
        self.head_controller.goal_x = float(main_face_x)
        self.head_controller.goal_y = float(main_face_y)

    def _distance_from_img_center(self, x, y):
        """
        Computes the distance of a point from the image center and returns
        the result.

        Arguments
        ---------
            x : int
                The x position
            y : int
                The y position
        """
        return math.sqrt((self.img_center_x - x)**2 + (self.img_center_y - y)**2)

    def _get_face_center_position(self, face):
        """
        Returns a face center position (instead of the top left corner).

        Arguments
        ---------
            face : Face
                The face to get the center position from
        """
        x = face.x + (face.width // 2)
        y = face.y + (face.height // 2)

        return x, y

    class HeadController:
        def __init__(self, center_x, center_y):
            """
            This method initializes the head controller by setting up its
            attributes and by starting its separate thread.

            Arguments
            ---------
                center_x : int or float
                    The x position of the image center

                center_y : int or float
                    The y position of the image center
            """
            # Setting up the attributes
            self.goal_x = float(center_x)
            self.goal_y = float(center_y)
            self.center_x = float(center_x)
            self.center_y = float(center_y)

            # The controller gain
            self.K = 0.1

            # Instantiating a client to move the head
            self.head_client = HeadActionClient()

            # Creating a thread running self.control_loop
            thread = threading.Thread(target=self.control_loop)
            thread.start()

        def control_loop(self):
            """
            This method moves the head toward a goal.

            Arguments
            ---------
                None
            """
            while not rospy.is_shutdown():
                error_x = (self.goal_x - self.center_x) / self.center_x
                error_y = (self.goal_y - self.center_y) / self.center_y

                cmd_x = self.K * error_x
                cmd_y = self.K * error_y

                self.head_client.GotoPosition(cmd_x, cmd_y)

                rospy.sleep(0.01)


class VoiceRecognition(BehaviorBase):
    def __init__(self):
        """
        This method creates a client used for the speech-to-text functionality.

        Arguments
        ---------
            None
        """
        BehaviorBase.__init__(self)

        # Setting up a speech-to-text client
        self.stt_client = actionlib.SimpleActionClient(
            "speech_recognition_action_server",
            SpeechRecognitionActivatedAction
        )

        self.stt_client.wait_for_server()

        self.speech = ""

        rospy.loginfo("[Behaviors] Voice recognition is ready to run")

    def _run(self, params):
        """
        This method converts speech to text. The result of the conversion
        is stored in self.speech.

        Arguments
        ---------
        params : dict
            A dict with the parameters. It must contains the following keys:
            - language : string
                The language in which the user speaks.
            - skip_keyword : string
                If set to True, the robot won't wait for its keyword and it'll
                directly use the Google API to understand complex sentences.
            - tell_back : string
                If set to True, the robot tells back what it understood.
        """

        # Resetting the speech attribute each time
        self.speech = ""

        # Extracting the parameters
        language = params["language"]
        skip_keyword = (params["skip_keyword"] == "True")
        tell_back = (params["tell_back"] == "True")

        # Sending the goal to the server
        goal = SpeechRecognitionActivatedGoal()
        goal.language = language
        goal.skip_keyword = skip_keyword
        goal.tell_back = tell_back

        self.stt_client.send_goal_and_wait(goal)

        # TODO: Test if still needed
        self.stt_client.wait_for_result(timeout=rospy.Duration(30.))

        # Updating the speech attribute
        result = self.stt_client.get_result()

        if result is None:
            self.speech = ""

        else:
            self.speech = result.recognition_results

        rospy.loginfo("[Voice recognition] TIAGo understood: " + self.speech)


class Voice(BehaviorBase):
    def __init__(self):
        """
        This method creates a client used for the text-to-speech functionality.

        Arguments
        ---------
            None
        """
        BehaviorBase.__init__(self)

        # Setting up a text-to-speech client
        self.tts_client = actionlib.SimpleActionClient(
            "tts",
            pal_interaction_msgs.msg.TtsAction
        )

        self.tts_client.wait_for_server()

        rospy.loginfo("[Behaviors] Voice is ready to run")

    def _run(self, params):
        """
        This method uses the voice of the robot say something.

        Arguments
        ---------
        params : dict
            A dict with the parameters. It must contains the following keys:
            - speech : string
                The text to convert to speech.
            - language : string
                The language of the speech.
        """

        # Extracting the parameters
        speech = params["speech"]
        language = params["language"]

        # Saving time if speech is empty
        if speech is None:
            return

        # Sending the goal to the server
        goal = pal_interaction_msgs.msg.TtsGoal()
        goal.rawtext.lang_id = language
        goal.rawtext.text = speech

        self.tts_client.send_goal_and_wait(goal)


class Locomotion(BehaviorBase):
    def __init__(self):
        """
        This method initializes a client use for the locomotion functionality.

        Arguments
        ---------
            None
        """
        BehaviorBase.__init__(self)

        # Creating a move_base client
        self.navigator = Navigator()

        rospy.loginfo("[Behaviors] Locomotion is ready to run")

    def _run(self, params):
        """
        This method moves the robot to a desired location. The method returns
        True if the robot was able to reach the goal, False otherwise.

        Arguments
        ---------
        params : dict
            A dict with the parameters. It must contains the following keys:
            - x : string
                The x position for the robot to move to.

            - y : string
                The y position for the robot to move to.

            - orientation : string
                The orientation for the robot to move to.
        """

        # Extracting the parameters
        x = float(params["x"])
        y = float(params["y"])
        orientation = float(params["orientation"])

        # Sending the goal through the client
        success = self.navigator.goto(x, y, orientation)
        return success
