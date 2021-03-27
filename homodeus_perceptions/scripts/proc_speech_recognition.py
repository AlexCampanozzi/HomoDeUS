#! /usr/bin/env python
import os
import rospy
import traceback
from keyword_detector.KeywordRecognizer import KeywordRecognizer as kr
import speech_recognition as sr

from std_msgs.msg import String
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Speech_recognition:
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
        self.output_perc = rospy.Publisher("/proc_output_listenText", String, queue_size=10)

        self.language = language
        self.speech_recognizer = sr.Recognizer()

    def transform(self):
        """
        This method transform the input which is sound into text and publish it
        to the output topic
        """
        while not rospy.is_shutdown():

            speechText = self.speech_to_text()
            self.output_perc.publish(speechText)
    
    def speech_to_text(self):
        """
        This method uses the Google Speech API to recognize complex sentences
        and return what was said as a string.
        """
        with sr.Microphone() as source:
            self.speech_recognizer.adjust_for_ambient_noise(source, duration=1) 
            common.loginfo(self,"Listening...")
            audio = self.speech_recognizer.listen(source)

            try:
                speech = self.speech_recognizer.recognize_google(audio, language=self.language)
                return speech

            except LookupError:
                common.loginfo(self,"LookupError")
                return ""

            except sr.UnknownValueError:
                common.loginfo(self,"UnknownValue")
                return ""

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        common.loginfo(self,"have been shutdown")

    

if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Speech_recognition()
        node.transform()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())
        