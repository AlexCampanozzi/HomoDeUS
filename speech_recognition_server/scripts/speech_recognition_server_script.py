#! /usr/bin/env python
import rospy
import traceback
import sys
import actionlib
import speech_recognizer as sr
from speech_recognition_server.msg import SpeechRecognitionActivatedAction
from speech_recognition_server.msg import SpeechRecognitionActivatedFeedback


class SpeechRecognitionServer():
    def __init__(self):

        self.actionServer = actionlib.SimpleActionServer(
            "speech_recognition_action_server",
            SpeechRecognitionActivatedAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.recognizer = sr.SpeechRecognizer()

        self.actionServer.register_preempt_callback(self.recognizer.interrupt())

        self.actionServer.start()
        rospy.loginfo("SpeechRecongitionServer: Running.")

    def execute_cb(self, goal):
        rospy.loginfo("SpeechRecognitionServer: Received a goal")

        # If the user specifies a language
        if bool(goal.language and goal.language.strip()):
            lang = goal.language
        else:
            lang = "en-US"

        feedback = SpeechRecognitionActivatedFeedback()

        # Run the main recognition loop
        while not rospy.is_shutdown() and not self.actionServer.is_preempt_requested():
            try:
                results = self.recognizer.run(
                    lang,
                    skip_keyword=goal.skip_keyword,
                    tell_back=goal.tell_back)

                feedback.recognition_results = results
                self.actionServer.publish_feedback(feedback)

            except Exception as e:
                rospy.logerr(str(e))

        self.actionServer.set_preempted()


if __name__ == '__main__':
    rospy.init_node('keyword_speech_multi_recognizer_server')
    try:
        SpeechRecognitionServer()
        rospy.spin()

    except Exception:
        rospy.logerr(traceback.format_exc())
