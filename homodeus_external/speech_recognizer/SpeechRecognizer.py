#! /usr/bin/env python
import os
import rospy
import traceback
import speech_recognition as sr

from std_msgs.msg import String
import HomoDeUS_common_py.HomoDeUS_common_py as common

class SpeechRecognizer:
    """
    This class publishes a String of what is heard by the Robot
    """
    def __init__(self, language = 'en-US'):
        """
        This method initializes the perception module by initializing the output topic and
        the Speech_recognition object 

        Arguments
        ---------
        language : string
            The language used for the speech recognition. By default its
            value is set to american english.
        """ 
        #The output of the module
        self.language = language
        self.speech_recognizer = sr.Recognizer()
    
    def speech_to_text(self):
        """
        This method uses the Google Speech API to recognize complex sentences
        and return what was said as a string.
        """
        with common.noalsaerr():
            with sr.Microphone() as source:
                self.speech_recognizer.adjust_for_ambient_noise(source, duration=1) 
                rospy.loginfo("Listening...")
                audio = self.speech_recognizer.listen(source)

                try:
                    speech = self.speech_recognizer.recognize_google(audio, language=self.language)
                    return speech

                except LookupError:
                    rospy.loginfo("LookupError")
                    return ""

                except sr.UnknownValueError:
                    rospy.loginfo("UnknownValue")
                    return ""
