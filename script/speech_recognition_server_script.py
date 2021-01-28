#! /usr/bin/env python
import rospy
import traceback
import sys
import actionlib
import speech_recognizer as sr
from speech_recognition_server.msg import SpeechRecognitionActivatedAction, SpeechRecognitionActivatedFeedback

class SpeechRecognitionServer():
  def __init__(self):
    self.actionServer = actionlib.SimpleActionServer("speech_recognition_action_server",
                                            SpeechRecognitionActivatedAction,
                                            execute_cb=self.execute_cb,
                                            auto_start = False)
    self.recognizer = sr.SpeechRecognizer()
    self.actionServer.register_preempt_callback(self.recognizer.interrupt())

    rospy.loginfo("Speech Recognizer running.")
    self.actionServer.start()

  def execute_cb(self, goal):
    rospy.loginfo("Received a goal")

    if bool(goal.language and goal.language.strip()):  # language field is NOT empty
      lang = goal.language
    else:
      lang = "en-US"  # English by default
  
    feedback = SpeechRecognitionActivatedFeedback()
    while not rospy.is_shutdown() and not self.actionServer.is_preempt_requested():
        try:
            print(lang)
            results = self.recognizer.run(lang, skip_keyword=goal.skip_keyword, tellBack=goal.tellBack) 
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
