#! /usr/bin/env python

import rospy
import actionlib
from HomoDeUS_common.HomoDeUS_common import convert_char_array_to_string 
from pal_startup_msgs.srv import StartupStart, StartupStop
from speech_recognition_server.msg import SpeechRecognitionActivatedAction
from speech_recognition_server.msg import SpeechRecognitionActivatedGoal
from speech_recognition_server.msg import SpeechRecognitionActivatedFeedback

from bondpy import bondpy
from functools import partial

from std_msgs.msg import String

class listenManager:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "/speech_recognition_action_server", SpeechRecognitionActivatedAction)

        bond_cb = partial(self.client_shutdown, "the server is now down")
        self.bond = bondpy.Bond(str("/listenManager_bond_topic"),str("1234"),bond_cb)
        self.bond.start()

        self.feedback_Pub = rospy.Publisher("proc_listen_module", String, queue_size=10)
        
        # wait for the action server to come up
        while(not self.client.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the action server to come up")
 
        print("Connection to server speech_recognition")
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
        if (self.client.wait_for_result(timeout=rospy.Duration(20))):
            result = self.client.get_result()
            if result is not None:    
                rospy.loginfo(result.recognition_results)
                result = convert_char_array_to_string(result.recognition_results)
                self.send_feedback(result)
                self.client_shutdown("Goal succeeded, a result was receive")
    
        self.client_shutdown("Goal failed, nothing received")
            
        
    def client_shutdown(self,reason):
        rospy.signal_shutdown(self.__class__.__name__ + reason)

    def end_Communication_with_server(self):
        self.client.cancel_goal()
        self.bond.break_bond()
        rospy.loginfo("Goood Bye my friend")
        print("End_Communication_with_server")

if __name__ == "__main__":
    try:
        rospy.init_node("proc_listen_test")

        node = listenManager()
        node.send_goal()
        rospy.on_shutdown(node.end_Communication_with_server)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass