#! /usr/bin/env python

import rospy
import argparse
import actionlib
import pal_interaction_msgs.msg

from bondpy import bondpy
from functools import partial
from std_msgs.msg import String

class talkManager:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self):
        # Setting up Text-to-Speech
        self.client = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)
        self.client.wait_for_server()
        
        bond_cb = partial(self.client_shutdown, "the server is now down")
        self.bond = bondpy.Bond(str("/talkManager_bond_topic"),str("1234"),bond_cb)
        self.bond.start()

        # wait for the action server to come up
        while(not self.client.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the action server to come up")
 
        rospy.loginfo("Connection to server done")

    def send_goal(self, text):
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

        goal_achieve_cb = partial(self.client_shutdown, "goal achieved, " + self.__class__.__name__ + " done")

        self.client.send_goal(goal=goal,done_cb=goal_achieve_cb)

    def client_shutdown(self,reason=""):
        rospy.signal_shutdown(self.__class__.__name__ + reason)

    def end_Communication_with_server(self):
        self.client.cancel_goal()
        self.bond.break_bond()
        rospy.loginfo("Goood Bye my friend")

class talkManagerSimul:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self):
        self.feedback_Pub = rospy.Publisher("/talkManagerSimul_topic", String, queue_size=10)

    def send_goal(self, text):
        """
        This method converts text to speech using the proper action server
        made by PAL Robotics.

        Arguments
        ---------
        text : string
            The text the robot has to say.
        """
        self.feedback_Pub.publish(text)
        self.client_shutdown(reason="goal achieved, the robot talked")


    def client_shutdown(self,reason=""):
        rospy.signal_shutdown(self.__class__.__name__ + reason)

    def end_Communication_with_server(self):
        self.feedback_Pub.unregister()
        rospy.loginfo("Goood Bye my friend")



if __name__ == "__main__":

    #parser = argparse.ArgumentParser(description='fulfill the desire of talking')
    #parser.add_argument('--mode', choices=['real', 'simul'], help="the environment where it's called", required=True)
    #parser.add_argument('--text', type=str,default="Command receive, I'm starting scenario 1!", help='text the robot will tell' )

    #args = parser.parse_args()

    #if args.mode == 'real':
    if False: 
        try:
            rospy.init_node("bhvr_talk_test")
            node = talkManager()
            #node.send_goal(args.text)
            rospy.on_shutdown(node.end_Communication_with_server)
            rospy.spin()

        except rospy.ROSInterruptException:
            pass
    #elif args.mode == 'simul':
    elif True:
        try:
            rospy.init_node("bhvr_talk_test")
            node = talkManagerSimul()
            node.send_goal("Command receive, I'm starting scenario 1!")
            rospy.on_shutdown(node.end_Communication_with_server)
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass

