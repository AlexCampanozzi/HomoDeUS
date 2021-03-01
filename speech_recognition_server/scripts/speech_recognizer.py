#!/usr/bin/env python
import sys
import os
import time
import rospy
import pyaudio
import actionlib
import pal_interaction_msgs.msg
from std_msgs.msg import String
import speech_recognition as sr
from sphinxbase.sphinxbase import *
from pocketsphinx.pocketsphinx import *


class SpeechRecognizer:
    """
    This class provides functionalities such as Speech-to-Text and Text-
    to-Speech for the robot.
    """
    def __init__(self, keyword='legacy', threshold=1e-20, timeout=30):
        """
        This method initializes the submodules used for listening and talking.
            * Keyword (microphones) = PyAudio (PortAudio)
            * Keyword (detection) = Pocket Sphinx
            * Speech-to-Text = Google API
            * Text-to-Speech = Action server from PAL Robotics

        Arguments
        ---------
        keyword : string
            The keyword used to activate the voice interface of the robot. By
            default, its value is set to 'legacy', but it can be changed for
            a word in Pocket Sphinx's dictionary.

        threshold : float
            A number to avoid false positives when listening for a keyword.
            It can be seen as the sensitivity. By default, its value is set to
            1e-20.

        timeout : int
            The maximum duration (in seconds) of the wait when the robot is
            waiting for a keyword to be said. By default, its value is set to
            30 seconds.
        """
        # Time out for keywords detection
        self.timeout = timeout

        # Setting up Pocket Sphinx        
        model_dir = os.path.join(os.path.dirname(__file__), '../include/pocketsphinx-5prealpha/model')
        
        self.config = Decoder.default_config()
        self.config.set_string(
            '-hmm', os.path.join(model_dir, 'en-us/en-us'))
        self.config.set_string(
            '-dict', os.path.join(model_dir, 'en-us/cmudict-en-us.dict'))

        self.keyword_recognized_text = "Yes I'm listening"

        self.config.set_string('-keyphrase', keyword)
        self.config.set_float('-kws_threshold', threshold)

        self.decoder = Decoder(self.config)
        self.interrupted = False

        # Setting up PortAudio
        pa = pyaudio.PyAudio()
        self.stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=1024)

        # Setting up SpeechRecognition
        self.recognizer = sr.Recognizer()

        # Setting up Text-to-Speech
        #self.tts_client = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)
        #self.tts_client.wait_for_server()

    def say(self, text):
        """
        This method converts text to speech using the proper action server
        made by PAL Robotics.

        Arguments
        ---------
        text : string
            The text the robot has to say.
        """
        goal = pal_interaction_msgs.msg.TtsGoal()
        goal.rawtext.lang_id = "en_GB"
        goal.rawtext.text = text

        #self.tts_client.send_goal(goal)

    def interrupt(self):
        """
        This method can be used to interrupt the whole recognition pipeline.
        """
        self.interrupted = True

    def wait_for_keyword(self):
        """
        This method waits for the keyword to be said. If the keyword
        was recognized in time (before timeout), the method returns True.
        Otherwise, it returns False.
        """
        # Opening up the microphone stream with PortAudio
        self.stream.start_stream()
        self.decoder.start_utt()

        max_time_waiting = time.time() + self.timeout

        # Waiting for the keyword to be said
        while (
            (self.decoder.hyp() is None) and
            (time.time() < max_time_waiting) and
            not rospy.is_shutdown()
        ):
            buffer = self.stream.read(1024)

            if buffer:
                self.decoder.process_raw(buffer, False, False)

        self.stream.stop_stream()

        # If the keyword was recognized
        if self.decoder.hyp() is not None:
            self.decoder.end_utt()
            return True

        else:
            rospy.loginfo("SpeechRecognition: Reached timeout!")
            self.decoder.end_utt()
            return False

    def speech_to_text(self, language='en-US'):
        """
        This method uses the Google Speech API to recognize complex sentences
        and return what was said as a string.

        Arguments
        ---------
        lang : string
            The language used for the speech recognition. By default its
            value is set to american english.
        """
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1) 
            # #if problem with listen, uncomment this to see if it is caused by the ambiant noise
            rospy.loginfo("SpeechRecognition: Listening...")
            audio = self.recognizer.listen(source)

            try:
                speech = self.recognizer.recognize_google(audio, language=language)
                return speech

            except LookupError:
                rospy.loginfo("SpeechRecognition: LookupError")
                return ""

            except sr.UnknownValueError:
                rospy.loginfo("SpeechRecognition: UnknownValue")
                return ""

    def set_keyword(self, keyword, threshold=1e-20):
        """
        This method reconfigures Pocket Sphinx to change the keyword.

        Arguments
        ---------
        keyword : string
            The new keyword.

        threshold : float
            A small number to avoid false positives when listening
            for a keyword. It can be seen as the sensitivity. By default,
            its value is set to 1e-20.
        """
        model_dir = "/home/pal/Downloads/pocketsphinx-5prealpha/model"

        self.config = Decoder.default_config()
        self.config.set_string(
            '-hmm', os.path.join(model_dir, 'en-us/en-us'))
        self.config.set_string(
            '-dict', os.path.join(model_dir, 'en-us/cmudict-en-us.dict'))

        self.config.set_string('-keyphrase', keyword)
        self.config.set_float('-kws_threshold', threshold)

        self.decoder = Decoder(self.config)

    def run(self, lang='en-US', skip_keyword=False, tell_back=False):
        """
        This method manages the whole recognition pipeline. That's what
        the ROS Action server uses.

        Arguments
        ---------
        lang : string
            The language used for the speech recognition. By default
            its value is set on american english.

        skip_keyword : boolean
            This parameter informs the method if the robot needs to
            wait for the keyword before using the Google Speech API.
            By default, its value is set to False which means that the
            robot will wait for the keyword to be said.

        tell_back : boolean
            This parameter informs the method if the robot should repeat
            what it understood with the speech recognition. By default,
            its value is set to False.
        """
        try:
            result = ""

            if not skip_keyword:
                rospy.loginfo("SpeechRecognition: skip_keyword is False")
                self.interrupted = False

                if (
                    rospy.is_shutdown() or
                    self.interrupted or
                    not self.wait_for_keyword()
                ):
                    return ""

                rospy.loginfo("SpeechRecognition: Recognized keyword!")
                #self.say(self.keyword_recognized_text)
                #self.tts_client.wait_for_result()
            
            result = self.speech_to_text()

            #if tell_back and result:
                #self.say(result)
                #self.tts_client.wait_for_result()

            return result

        except sr.WaitTimeoutError:
            pass

        except sr.UnknownValueError:
            rospy.loginfo("SpeechRecognition: Unknown value error (Sphinx)")

        except sr.RequestError as e:
            rospy.loginfo("SpeechRecognition: Sphinx error; {0}".format(e))

        return ""
