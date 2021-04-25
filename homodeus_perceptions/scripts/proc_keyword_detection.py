#! /usr/bin/env python
import os
import rospy
import traceback
import keyword_detector.KeywordRecognizer as kr
from std_msgs.msg import Bool, String
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Keyword_detection:
    """
    This class publishes the state of detection of the wanted keyword
    """
    def __init__(self, keyword='legacy', timeout=30):
        """
        This method initializes the perception module by initializing the output topic and
        the keywordRecognizer object 
        Arguments
        ---------
        keyword : string
            The keyword used to activate the voice interface of the robot. By
            default, its value is set to 'legacy', but it can be changed for
            a word in Pocket Sphinx's dictionary.
        timeout : int
            The maximum duration (in seconds) of the wait when the robot is
            waiting for a keyword to be said. By default, its value is set to
            30 seconds.
        """ 
        #The input of the module
        self.intput_perc_keyword = rospy.Subscriber("inter_keyword_detection", Bool, self.transform_Cb,queue_size=2)

        #The output of the module
        self.output_perc = rospy.Publisher("/proc_output_keywordDetect", Bool, queue_size=10)

    def transform_Cb(self, detection):
        """
        This method transform the input which is sound into a boolean representing the detection or not of the keyword
        and send this boolean to the output topic
        """
        self.output_perc.publish(detection.data)

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        rospy.loginfo(self, "is shutting down")
    

if __name__ == '__main__':
    """
    This if condition tells what to do if the script is called directely. Otherwise, this part should be ignored.
    It vreates a node and starts the speechRecognition server in it.
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Keyword_detection()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(__file__,traceback.format_exc())
