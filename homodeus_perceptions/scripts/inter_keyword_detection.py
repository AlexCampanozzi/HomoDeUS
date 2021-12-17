#! /usr/bin/env python
import os
import rospy
import traceback
import keyword_detector.KeywordRecognizer as kr
from std_msgs.msg import Bool, String
import HomoDeUS_common_py.HomoDeUS_common_py as common

class Inter_keyword_detection:
    """
    This class publishes the state of detection of the wanted keyword
    """
    def __init__(self, keyword='legacy', timeout=30):
        """
        This method initializes the intermediate module of keyword detection perception
        by initializing the output topic and the keywordRecognizer object 
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
        self.intput_perc_keyword = rospy.Subscriber("/desire_keyword", String, self.set_keyword,queue_size=5)

        #The output of the module
        self.output_perc = rospy.Publisher("/inter_keyword_detection", Bool, queue_size=10)


        # param_name = rospy.search_param('keyword')
        # keyword = rospy.get_param(param_name,"alfred")
        # rospy.loginfo(keyword)

        self.keyword_recognizer = kr.KeywordRecognizer(keyword=keyword, timeout=timeout)

    def set_keyword(self,keyword):
        """
        This method set a new keyword for which the robot will recognize itself
        ---------
        keyword : string
            The keyword used to activate the voice interface of the robot. By
            default, its value is set to 'legacy', but it can be changed for
            a word in Pocket Sphinx's dictionary.
        """
        if keyword.data:
            self.keyword_recognizer.set_keyword(keyword.data)

    def transform(self):
        """
        This method transform the input which is sound into a boolean representing the detection or not of the keyword
        and send this boolean to the output topic
        """
        while not rospy.is_shutdown():
            detection = self.keyword_recognizer.wait_for_keyword()
            if detection:
                rospy.logwarn("---------------------KEYWORD DETECTED---------------------")
            self.output_perc.publish(detection)

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
        node = Inter_keyword_detection()
        node.transform()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(__file__,traceback.format_exc())
