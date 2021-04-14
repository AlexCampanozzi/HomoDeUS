#! /usr/bin/env python
import re
import os
import rospy
import actionlib
import traceback
import random as rand
import speech_recognizer.SpeechRecognizer as sr
import pal_interaction_msgs.msg
import xml.etree.ElementTree as ET
from enum import Enum
from std_msgs.msg import String, Bool
import HomoDeUS_common_py.HomoDeUS_common_py as common

REPEAT_MAX = 2

class Dialog_state(Enum):
    begin = 1
    listen = 2
    confirmation = 3
    done = 4

class Dialoguing_module:
    """
    This class provide control to the robot's head as an actionlib server
    """
    def __init__(self, language = "en_GB"):
        dialog_context_xml_path=os.path.join(os.path.dirname(__file__), '../../homodeus_external/xml_folder/dialog_context.xml') 
        self.dialog_context = ET.parse(dialog_context_xml_path).getroot()

        self.language = language
        self.selected_context =  ""
        self.repeat = 0     
        self.relevant_info= ""
        self.dialog_state = Dialog_state.begin

        param_name = rospy.search_param('on_robot')
        self.on_robot = rospy.get_param(param_name,False)
        

        # Goal input
        self.input_bhvr_goal_context = rospy.Subscriber("/bhvr_input_goal_dialContext",data_class=String, callback=self.set_context_Cb,queue_size=10)
        
        # Output
        self.output_bhvr_result = rospy.Publisher("/bhvr_output_res_dialBool", Bool, queue_size=10)
        self.output_bhvr_relevant = rospy.Publisher("/bhvr_output_res_dialRelevant", String, queue_size=10)

        if self.on_robot:
            self.connect_to_tts_server()
        
        self.speech_recognizer = sr.SpeechRecognizer()
        
        
    def connect_to_tts_server(self):
        self.output_bhvr_command = actionlib.SimpleActionClient("tts", pal_interaction_msgs.msg.TtsAction)

        # wait for the action server to come up
        while(not self.output_bhvr_command.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the action server to come up")

    def set_context_Cb(self,context):
        """
        This method converts text to speech using the proper action server
        made by PAL Robotics.

        Arguments
        ---------
        TtsText : string
            The text the robot has to say.
        """
        self._reset_value()
        self.selected_context = context.data
        self.dialoguing()

    def talking(self,TtsText):
        if self.on_robot:
            goal = pal_interaction_msgs.msg.TtsGoal()
            goal.rawtext.lang_id = self.language
            goal.rawtext.text = TtsText

            self.output_bhvr_command.send_goal_and_wait(goal=goal)

        rospy.loginfo(TtsText)

    def dialoguing(self):
        context =self.selected_context
        rospy.loginfo(context)
        print(self.dialog_state.name)

        while self.dialog_state != Dialog_state.done:
            if self.dialog_state == Dialog_state.begin:
                self.talking(self._select_talking_text(self.selected_context,self.dialog_state.name))
                self._increasing_state()
                context =self.selected_context
            listen_text = self._prepare_sentence(self.speech_recognizer.speech_to_text())
            print(listen_text)
            if self.dialog_state == Dialog_state.confirmation:
                context =  "confirmation"

            relevant_infos = self._get_relevant_info(context,listen_text)

            talking_state = self._update_talking_state(relevant_infos[0])

            self._answer_client(context, talking_state, relevant_infos)

            self._update_dialog_state(talking_state,relevant_infos[0])

            if self.dialog_state == Dialog_state.done:
                self._send_relevant_info(self.relevant_info) 

    def _get_relevant_info(self,context,client_answer):
        relevant_info = ""
        prefix = ""
        suffix = ""
        xml_path_to_search = "./"+context+"/listen/*" 
        for answer in self.dialog_context.findall(xml_path_to_search):
            if re.search(r"\b{}\b".format(answer.text), client_answer) is not None:
                if relevant_info:
                    relevant_info+= " "
                relevant_info += answer.get('rel_info')
                prefix = str(answer.get('prefix') or '')
                suffix = str(answer.get('suffix') or '')
        if self.dialog_state == Dialog_state.listen:
            self.relevant_info = relevant_info
        return relevant_info, prefix, suffix


    def _update_talking_state(self,relevant_info):
        talking_state = ""
        if relevant_info:
            talking_state="feedback"
        elif self.repeat <= REPEAT_MAX or self.dialog_state == Dialog_state.confirmation:
            talking_state="repeat"
        elif self.repeat > REPEAT_MAX :
            talking_state="nothing"
        return talking_state
        

    def _update_dialog_state(self,talking_state,relevant_info):
        print(talking_state)
        print(relevant_info)
        if talking_state == "feedback":
            if self.dialog_state == Dialog_state.confirmation:
                if 'negative' in relevant_info:
                    self._reset_value()
                else:
                    self._increasing_state()
            else:
                self._increasing_state()
        elif talking_state == "repeat":
            self.repeat+=1
        elif talking_state == "nothing":
            self.dialog_state = Dialog_state.done
            self.relevant_info = ""

    def _reset_value(self):
        self.dialog_state = Dialog_state.begin
        self.repeat = 0

    def _increasing_state(self):
        self.dialog_state = Dialog_state(self.dialog_state.value + 1)
        self.repeat = 0
    
    def _answer_client(self,context, talking_state,relevant_infos):
        talking_text = ""
           
        if talking_state == "feedback":
            rel_info = ""
            if self.selected_context == "menu_selection":
                rel_info = relevant_infos[0]
            else:
                rel_info = relevant_infos[0].split()
                rel_info = rel_info[0]         
            talking_text = self._select_talking_text(context,talking_state) + \
                 relevant_infos[1] + rel_info + relevant_infos[2] 
        else:
            talking_text = self._select_talking_text(context,talking_state)
        self.talking(talking_text)

    def _send_relevant_info(self,relevant_info):
        self.output_bhvr_relevant.publish(relevant_info)
        self.output_bhvr_result.publish(True)

    def _prepare_sentence(self, sentence):
            # Removing punctuation and capital letters
            prepared_sentence = re.sub(r'[^\w\s]', '', sentence).lower()
            return prepared_sentence

    def _select_talking_text(self,global_context, specific_context):
        global_context_iter = self.dialog_context.getiterator(global_context)
        outlist = global_context_iter[0].findall("./"+specific_context+"/*")
        return outlist[rand.randint(0,(len(outlist)-1))].text

    def node_shutdown(self):
        self.output_bhvr_command.cancel_goal()
        rospy.loginfo("have been shutdown")


if __name__ == "__main__":
    """
    It creates a node and starts the tts server in it and connects to all topics needed for this module
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Dialoguing_module()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(traceback.format_exc())

