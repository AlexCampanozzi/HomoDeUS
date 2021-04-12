#! /usr/bin/env python
import re
import os
import rospy
import actionlib
import traceback
import xml.etree.ElementTree as ET


from threading import Timer
from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event
from HomoDeUS_common_py.HomoDeUS_common_py import no_caps_and_whitespace


class Speech_recognition_observer:
    """
    This class follows observe the speech_recognition perception and handles the state of the Listening desire based on an XML
    """
    def __init__(self):
        # The input of the module
        self.input_motv = rospy.Subscriber("/proc_output_listenText", String, self.listen_Cb, queue_size=5)
        self.input_context = rospy.Subscriber("/proc_goal_context", String,self.set_context, queue_size=5 )

        # the output of the module
        self.event_publisher = rospy.Publisher("events",Event, queue_size = 10)
        self.menu_publisher = rospy.Publisher("/obs_client_order", String, queue_size = 10)

        # following DesireSet
        self.desires_set_subscriber = rospy.Subscriber("desires_set", DesiresSet, self.listen_desires_set_Cb)

        self.actual_context = ""
        
        # Setting up Pocket Sphinx        
        speech_context_xml_path = os.path.join(os.path.dirname(__file__), '../../homodeus_external/xml_folder/speech_context.xml') 
        self.speech_context_parsed = ET.parse(speech_context_xml_path).getroot()

        self.cancel = False

    def listen_desires_set_Cb(self,desireSet):
        """
        This method updated the desireSet used in the desire_event_change method.

        Arguments
        ---------
        desireSet: DesiresSet
            The most recent desireSet
        """
        self.curDesireSet = desireSet
    
    def set_context(self, context):
        """
        This method updated the context used to detect the waited answer from Listening desire.
        It also set a timer following the time before receiving a relevant answer and sending a IMP state if no 
        relevant answer before reaching the set time.

        Arguments
        ---------
        context: string
            the context given to the xml parser
        """
        self.actual_context = context.data
        for context in self.speech_context_parsed.iter(self.actual_context):
            rospy.loginfo(context.get('timer'))
            self.timer = Timer(int(context.get('timer')), self.timeout_cb)
            break
        self.timer.start()

    def timeout_cb(self):
        """
        This method send an IMP_ON event.type following the timer reaching it's set time.
        """
        self.desire_event_change(Event.IMP_ON)

    def desire_event_change(self, Event_type):
        """
        This method send an event depening from which it's type depend of the argument
        Arguments
        ---------
        Event_type: Event.type
            The type of event given by the caller representing the states of the desire
        """
        # S'il y a beaucoup de desirs du meme type... il me semble que ca ne fonctionnera pas
        event = Event()
        rospy.loginfo(self.curDesireSet.desires)
        for desire in self.curDesireSet.desires:
            if desire.type == "Listening":
                event.desire = desire.id
                event.desire_type = desire.type
                event.type = Event_type
                self.event_publisher.publish(event)

        #reset 
        self.timer.cancel()
        self.actual_context=""
                
    def listen_Cb(self, Stt_text):
        """
        This method is the Cb of the speech_recognition perception
        Arguments
        ---------
        Stt_text: String()
            the string received from speech_recognition perception
        """

        text_to_use = no_caps_and_whitespace(Stt_text.data)
        if self.actual_context:
            if self.actual_context !="menu":
                self._look_answer(text_to_use)
            else:
                self._look_answer_menu(text_to_use)
        else:
            rospy.loginfo("Listening without receiving a context")

    def _look_answer(self, Stt_text):
        """
        This method analyse if the desire is accomplish or not depending on the context and the xml doc
        Arguments
        ---------
        Stt_text: string
            the string received from speech_recognition perception
        """
        xml_path_to_search= "./"+self.actual_context+"/answer"
        for answer in self.speech_context_parsed.findall(xml_path_to_search):
            if re.search(r"\b{}\b".format(answer.text), Stt_text) is not None:
                if answer.get('acc_type') == 'ON':
                    self.desire_event_change(Event.ACC_ON)
                    break
                elif answer.get('acc_type') == 'OFF':
                    self.desire_event_change(Event.ACC_OFF)
                    break
        return

    def _look_answer_menu(self, Stt_text):
        """
        This method analyse if the desire is accomplish or not depending on the context and the xml doc.
        It also send information of the order of the client to the appropriate topic
        Arguments
        ---------
        Stt_text: string
            the string received from speech_recognition perception
        """
        order_string = ""
        xml_path_to_search= "./"+self.actual_context+"/answer"
        for answer in self.speech_context_parsed.findall(xml_path_to_search):
            if re.search(r"\b{}\b".format(answer.text), Stt_text) is not None:
                order_string += answer.text
                order_string += " "

        if order_string:
            self.desire_event_change(Event.ACC_ON)
            self.menu_publisher.publish(order_string)
        else:
            self.desire_event_change(Event.ACC_OFF)

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        rospy.loginfo("have been shutdown")

    

if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node('motv_speech_recognition_observer')
        node = Speech_recognition_observer()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        pass
