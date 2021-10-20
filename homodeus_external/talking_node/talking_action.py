#! /usr/bin/env python
# coding=utf-8

import os
import time
import rospy
import pygame
import actionlib
import threading
import traceback
from gtts import gTTS
from pytictoc import TicToc
from pydub import AudioSegment
from custom_msgs.msg import ttsActionAction, ttsActionResult, ttsActionGoal
from std_msgs.msg import String
import HomoDeUS_common_py.HomoDeUS_common_py as common
import wave
import contextlib


import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

TALK_DELAY = 0.4
TIMEOUT = 2
SOURCE_MP3 = "talking_file.mp3"
SOURCE_WAV = "talking_file.wav"

class talkingSynthesizer:
    """
    This class publishes a String of what is heard by the Robot
    """

    def __init__(self, language='fr-FR'):
        """
        This method starts the talking action server used by the simulation
        Arguments
        ---------
        language : string
            The language used for the speech recognition. By default its
            value is set to american english.
        """
        # The output of the module
        pygame.mixer.init()
        self.language = language
        self.is_connected = common.check_connection()
        self.file_path_mp3 = os.path.dirname(os.path.abspath(__file__))+"/"+SOURCE_MP3
        self.file_path_wav = os.path.dirname(os.path.abspath(__file__))+"/"+SOURCE_WAV
        self.tts_action = actionlib.SimpleActionServer("tts", ttsActionAction,
                                                             self.__goal_action_cb, auto_start=False)
        self.tts_action.register_preempt_callback(self.__interrupt_action_cb)
        self.tts_action.start()

        # self.thread_check_connection = threading.Thread(target=self.check_connection)
        #self.thread_check_connection.start()

    def say(self, text):
        try:
            text_gtts = common.remove_accents(text)
            myText = gTTS(text=text_gtts, lang=self.language, slow=False)
            myText.save(self.file_path_mp3)
            sound = AudioSegment.from_mp3(self.file_path_mp3)
            sound.export(self.file_path_wav, format="wav")
        except Exception as e:
            text = "\"" + text + "\""
            os.system("pico2wave  -l " + self.language + " -w " + self.file_path_wav + " " + text)

        pygame.mixer.music.load(self.file_path_wav)
        pygame.mixer.music.play(loops=0)

    def __goal_action_cb(self, goal):
        rospy.loginfo("tts_action receive a goal")
        if goal.lang_id == '':
            self.language = 'fr-FR'
        else:
            self.language = goal.lang_id
        self.say(goal.text)
        time.sleep(abs(self.get_wav_duration(self.file_path_wav)-TALK_DELAY))
        result = ttsActionResult()
        result.success = True
        self.tts_action.set_succeeded(result)

    def __interrupt_action_cb(self): # pygame interrompt lui même le texte avec une nouvelle requête "say"
        rospy.loginfo('tts action is being interrupted')
        pygame.mixer.music.stop()
        result = ttsActionResult()
        result.success = True
        self.tts_action.set_preempted(result)

    def __node_shutdown(self):
        """
        This method cancel goal if their is a sudden shutdown. It also informs by a log that the node was shutdown
        """
        pygame.mixer.music.stop()
        rospy.loginfo("have been shutdown")

    def __get_wav_duration(self, fname):
        with contextlib.closing(wave.open(fname,'r')) as f:
            frames = f.getnframes()
            rate = f.getframerate()
            duration = frames / float(rate)
            return duration


if __name__ == '__main__':

    try:
        rospy.init_node(common.get_file_name(__file__))
        node = talkingSynthesizer()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(traceback.format_exc())

#if __name__ == '__main__':
    # PROBLEME POSSIBLE: TROP LONG TEXTE EST LONG A GÉNÉRÉ
    #rospy.init_node('test', anonymous=False)
#    talkClient = talkingSynthesizer()
#    t = TicToc()
#    t.tic()
#    print("---------------")
#    talkClient.say("Bonjour je test ma classe pendant que je t'écris un texte énorme parlant de la signification du bien et du mal en rapport avec les grains de cafais. En effet, il faut se souvenir que le grain de café est quelque chose qui peut amener beaucoup de mal, mais beaucoup de création ce qui est bien")
#    print("---------------")
#    t.toc()
#    time.sleep(15)
#    t.tic()
#    print("---------------")
#    talkClient.say("Bonjour je test ma classe pendant que je t'écris un texte énorme parlant de la signification du bien et du mal en rapport avec les grains de cafés. En effet, il faut se souvenir que le grain de café est quelque chose qui peut amener beaucoup de mal, mais beaucoup de création ce qui est bien")
#    print("---------------")
#    t.toc()
#    time.sleep(5)
#    t.tic()
#    talkClient.say("Interruption?")
#    t.toc()
#    time.sleep(5)
#    exit()

