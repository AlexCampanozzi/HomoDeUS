#! /usr/bin/env python
import rospy
import traceback
import sys
import actionlib
import speech_recognizer as sr
from speech_recognition_server.msg import SpeechRecognitionActivatedAction
from speech_recognition_server.msg import SpeechRecognitionActivatedResult
from speech_recognition_server.msg import SpeechRecognitionActivatedFeedback

from bondpy import bondpy

class SpeechRecognitionServer():
    """
    This class provides a server to which you can send goal about speech recognition tasks.
    """
    def __init__(self):
        """
        This method initializes and start the server to which you send your goals and initialize the recognizer able to understand 
        what is said to the robot.
        """
        self.actionServer = actionlib.SimpleActionServer(
            "speech_recognition_action_server",
            SpeechRecognitionActivatedAction,
            execute_cb=self.execute_cb,
            auto_start=False)
        self.recognizer = sr.SpeechRecognizer()

        self.actionServer.register_preempt_callback(self.recognizer.interrupt())

        self.actionServer.start()
        
        #bool used for bond_connexion
        self.not_shutting_down = True
        self.bond_connexion_loop()
        rospy.loginfo("SpeechRecognitionServer: Running.")

    def bond_connexion_loop(self):
        if hasattr(self,'bond'):
            self.bond.break_bond()
            self.bond.shutdown()
        if not self.actionServer.is_preempt_requested():
            self.bond = bondpy.Bond("/listenManager_bond_topic", "1234",self.bond_connexion_loop)
            self.bond.start()

    def execute_cb(self, goal):
        """
        This method receive a goal and deal with it by translating it into a command for the recognizer. 
        It also deals with what to send as feedback to the action server by sending the word(s) recognize by the recognizer

        Arguments
        ---------
        goal : 3 variables
            language: string
                The language in which the recognizer has to understand (usually en-US).
            skip_keyword: bool
                A bool pointing out if the robot has to wait for a keyword before trying to recognize what is it said to it
            tell_back: bool
                A bool pointing out if the robot has to repeat the words just said to it
        """
        if self.actionServer.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self.actionServer.__class__.__name__)
            self.actionServer.set_preempted()
        
        rospy.loginfo("SpeechRecognitionServer: Received a goal")
        action_feedback = SpeechRecognitionActivatedFeedback()

        # If the user specifies a language
        if bool(goal.language and goal.language.strip()):
            lang = goal.language
        else:
            lang = "en-US"
        
        action_feedback.feedback_on_goal="A goal was sent setting the language at: " + lang + " and skip_keyword at:" + str(goal.skip_keyword)
        self.actionServer.publish_feedback(action_feedback)
        
        action_result = SpeechRecognitionActivatedResult()
        
        # Run the main recognition loop
        while not rospy.is_shutdown() and not self.actionServer.is_preempt_requested() and self.actionServer.is_active():
            try:
                results = self.recognizer.run(lang,
                    skip_keyword=goal.skip_keyword)
                action_result.recognition_results = results
                rospy.loginfo(results)
                self.actionServer.set_succeeded(result=action_result,text="Goal achieved")

            except Exception as e:
                rospy.logerr(str(e))

    def shutting_down(self):
        self.not_shutting_down = False
        self.bond_connexion_loop()
        
if __name__ == '__main__':
    """
    This if condition tells what to do if the script is called directely. Otherwise, this part should be ignored.
    It vreates a node and starts the speechRecognition server in it.
    """
    try:
        rospy.init_node('speech_recognition_server_node')
        node = SpeechRecognitionServer()
        rospy.on_shutdown(node.shutting_down)
        rospy.spin()

    except Exception:
        rospy.logerr(traceback.format_exc())