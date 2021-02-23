#! /usr/bin/env python

import os
import rospy
import roslib
import actionlib
import control_msgs.msg
import geometry_msgs
import math
from pal_startup_msgs.srv import StartupStart, StartupStop
from speech_recognition_server.msg import SpeechRecognitionActivatedAction
from speech_recognition_server.msg import SpeechRecognitionActivatedGoal
from speech_recognition_server.msg import SpeechRecognitionActivatedFeedback
import rosservice

from std_msgs.msg import String

class listenManager:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "speech_recognition_action_server", SpeechRecognitionActivatedAction)

        rospy.loginfo("init")
        self.feedback_Pub = rospy.Publisher("speech_recognition", String, queue_size=10)

        # wait for the action server to come up
        while(not self.client.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the action server to come up")
        rospy.Subscriber("/speech_recognition_action_server/feedback", String, self.callback)
        rospy.loginfo("Connection to server done")

    def send_goal(self):
        """
        This method sends a goal to the actionlib in order to move the robot's head
        x (float): The x position in the optical frame that the robot must reach
        y (float): The y position in the optical frame that the robot must reach
        """
        goal = SpeechRecognitionActivatedGoal()
        
        goal.language="en-US"
        goal.skip_keyword=False
        self.client.send_goal(goal)
        rospy.sleep(0.5)

    def send_feedback(self, result):
        """
        This method converts angles value into position before using the GotoPosition function
        Ytheta (float): The x position in the optical frame that the robot must reach
        xTheta (float): The y position in the optical frame that the robot must reach
        """        
        self.feedback_Pub.publish(result)

    def callback(self, feedback):
        if (self.client.wait_for_result(timeout=20)):
            result = self.client.get_result()
            self.send_feedback(result)
        else:
            rospy.loginfo("I heard %s", feedback)
            rospy.loginfo("But there was no result from that")    

if __name__ == "__main__":
    try:
        rospy.init_node("proc_listen_test")

        node = listenManager()
        node.send_goal()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass