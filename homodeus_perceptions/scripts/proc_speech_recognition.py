#! /usr/bin/env python
import os
import rospy
import traceback
import speech_recognizer.SpeechRecognizer as sr
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
        self.speech_recognizer = sr.SpeechRecognizer()

    def transform(self):
        """
        This method transform the input which is sound into text and publish it
        to the output topic
        """
        while not rospy.is_shutdown():

            speechText = self.speech_recognizer.speech_to_text()
            if speechText:
                self.output_perc.publish(speechText)

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        rospy.loginfo("have been shutdown")

    

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
        rospy.logerr(traceback.format_exc())
        