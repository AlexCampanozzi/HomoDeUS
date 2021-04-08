#! /usr/bin/env python
import re
import rospy
import actionlib
import traceback
import xml.etree.ElementTree as ET


from threading import Timer
from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event


class Speech_recognition_observer:
    """
    This class follows the state of the keyword_detection desire
    """
    def __init__(self):
        # The input of the module
        self.input_motv = rospy.Subscriber("/proc_output_listenText", String, self.listen_Cb, queue_size=10)
        self.input_context = rospy.Subscriber("/proc_goal_context", String,self.set_context, queue_size=5 )

        # the output of the module
        self.event_publisher = rospy.Publisher("events",Event, queue_size = 10)
        self.menu_publisher = rospy.Publisher("/obs_client_order", String, queue_size = 10)

        # following DesireSet
        self.desires_set_subscriber = rospy.Subscriber("desires_set", DesiresSet, self.listen_desires_set_Cb)

        self.actual_context = ""
        
        speech_context_xml_path="/home/pal/catkin_ws/src/HomoDeUS/homodeus_external/xml_folder/speech_context.xml"
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
    
    def set_context(self,context):
        self.actual_context = context.data
        if context.data == "menu" or context.data == "order_ready":
            self.timer = Timer(60, self.timeout_cb)
        else:
            self.timer = Timer(30,self.timeout_cb)
        self.timer.start()

    def timeout_cb(self):
        self.desire_event_change(Event.IMP_ON)

    def desire_event_change(self, Event_type):
        """
        This method is the callback of the output of the module it is connected to, it then send an event depening of the result
        get from the topic.

        Arguments
        ---------
        SpeechText: String
            The result of the output of the module it is connected to. In this case it is the speech recognized
        """
        # S'il y a beaucoup de desirs du meme type... il me semble que ca ne fonctionnera pas
        event = Event()
        for desire in self.curDesireSet.desires:
            if desire.type == "Speech_recognition":
                event.desire = desire.id
                event.desire_type = desire.type
                event.type = Event.IMP_ON
                self.event_publisher.publish(event)
        #reset 
        self.timer.cancel()
        self.actual_context=""
                
    def listen_Cb(self,Stt_text):
        text_to_use = self._prepare_sentence(Stt_text.data)
        if self.actual_context:
            if self.actual_context !="menu":
                self._look_answer(text_to_use)
            else:
                self._look_answer_menu(text_to_use)
        else:
            rospy.loginfo("Listening without receiving a context")

    def _look_answer(self,Stt_text):
        xml_path_to_search= "./"+self.actual_context+"/answer"
        for answer in self.speech_context_parsed.findall(xml_path_to_search):
            if re.search(r"\b{}\b".format(answer.text), Stt_text) is not None:
                if answer.get('acc_type') == 'ON':
                    self.desire_event_change(Event.ACC_ON)
                    return
                elif answer.get('acc_type') == 'OFF':
                    self.desire_event_change(Event.ACC_OFF)
                    return

    def _look_answer_menu(self,Stt_text):
        order_string = ""
        xml_path_to_search= "./"+self.actual_context+"/answer"
        for answer in self.speech_context_parsed.findall(xml_path_to_search):
            if re.search(r"\b{}\b".format(answer.text), Stt_text) is not None:
                order_string += answer.text
                order_string += " "

        if order_string:
            self.desire_event_change(Event.ACC_ON)
            order_string = "You want " + order_string + "right?"
            self.menu_publisher.publish(order_string)
        
    def _prepare_sentence(self, sentence):
        # Removing punctuation and capital letters
        prepared_sentence = re.sub(r'[^\w\s]', '', sentence).lower()
        return prepared_sentence

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        rospy.loginfo(self,"have been shutdown")

    

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
