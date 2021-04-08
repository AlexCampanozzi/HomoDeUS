#!/usr/bin/env python
import os
import time
import rospy
import pyaudio
from sphinxbase.sphinxbase import *
from pocketsphinx.pocketsphinx import *
import HomoDeUS_common_py.HomoDeUS_common_py as common

class KeywordRecognizer:
    """
    This class provides functionality as Keyword recognition
    """
    def __init__(self, keyword='legacy', threshold=1e-20, timeout=30):
        """
        This method initializes the submodules used for listening the keyword.
            * Keyword (microphones) = PyAudio (PortAudio)
            * Keyword (detection) = Pocket Sphinx

        Arguments
        ---------
        keyword : string
            The keyword used to activate the voice interface of the robot. By
            default, its value is set to 'legacy', but it can be changed for
            a word in Pocket Sphinx's dictionary.

        threshold : float
            A number to avoid false positives when listening for a keyword.
            It can be seen as the sensitivity. By default, its value is set to
            1e-20.

        timeout : int
            The maximum duration (in seconds) of the wait when the robot is
            waiting for a keyword to be said. By default, its value is set to
            30 seconds.
        """

        # Time out for keywords detection
        self.timeout = timeout
        self.set_new_keyword = False

        # Setting up Pocket Sphinx        
        self.model_dir = os.path.join(os.path.dirname(__file__), './include/pocketsphinx-5prealpha/model')
        
        self.set_keyword(keyword)

        # Setting up PortAudio
        pa = pyaudio.PyAudio()
        self.stream = pa.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=512)

    def wait_for_keyword(self):
        """
        This method waits for the keyword to be said. If the keyword
        was recognized in time (before timeout), the method returns True.
        Otherwise, it returns False.
        """
        if not self.set_new_keyword:
            # Opening up the microphone stream with PortAudio
            self.stream.start_stream()
            self.decoder.start_utt()

            max_time_waiting = time.time() + self.timeout

            # Waiting for the keyword to be said
            
            while (
                (self.decoder.hyp() is None) and
                (time.time() < max_time_waiting) and
                not(self.set_new_keyword) and
                not rospy.is_shutdown()
            ):
                buffer = self.stream.read(1024)

                if buffer:
                    self.decoder.process_raw(buffer, False, False)

            self.stream.stop_stream()

            # If the keyword was recognized
            if self.decoder.hyp() is not None:
                self.decoder.end_utt()
                return True

            else:
                rospy.loginfo("Reached timeout!")
                self.decoder.end_utt()
                return False
        else:
            time.sleep(0.5)
            return False

    def set_keyword(self, keyword, threshold=1e-20):
        """
        This method reconfigures Pocket Sphinx to change the keyword.

        Arguments
        ---------
        keyword : string
            The new keyword.

        threshold : float
            A small number to avoid false positives when listening
            for a keyword. It can be seen as the sensitivity. By default,
            its value is set to 1e-20.
        """
        rospy.loginfo("--------------------------------- " + keyword + "----------------------------------")
        self.set_new_keyword = True
        time.sleep(0.2)

        self.config = Decoder.default_config()
        self.config.set_string(
            '-hmm', os.path.join(self.model_dir, 'en-us/en-us'))
        self.config.set_string(
            '-dict', os.path.join(self.model_dir, 'en-us/cmudict-en-us.dict'))

        self.config.set_string('-keyphrase', keyword)
        self.config.set_float('-kws_threshold', threshold)

        self.decoder = Decoder(self.config)

        self.set_new_keyword = False

        
