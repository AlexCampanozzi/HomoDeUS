#! /usr/bin/env python
import re
import random as rand
# import rospy
# import roslib
# import actionlib
import xml.etree.ElementTree as ET

class DialogBehavior:
    def __init__(self):
        # rospy.loginfo("init_DialogBehavior")
        self.xml_path = "dialog_context.xml"
        #doublecheck the path
        self.dialog_context = ET.parse(self.xml_path).getroot()

    def _dialog_callback(self, dialog_text_rcv):
        #recieve the dialog, select the best answer and send it to talk
        dialog_text_rcv = self._prepare_sentence(dialog_text_rcv)
        for current_context in self.dialog_context.findall('*'):
            if self._is_supported_context(current_context, dialog_text_rcv):
                #send self._select_outanswer(current_context) to talk
                print('    ' + self._select_outanswer(current_context))
                return True
        #send self.dialog_context.find('no_data').findall('outanswer').text to talk
        print('    ' + self._select_outanswer(self.dialog_context.find('no_data')))
        return False


    def _is_supported_context(self, context, sentence):
        # Checking if there's an intersection
        found_flag = 0
        for inanswer in self.dialog_context.find(context.tag).findall('inanswer'):
            elements = inanswer.text.split(',')
            for element in elements:
                if element in sentence:
                    found_flag = found_flag + 1
                else :
                    found_flag = -1
                    break
            if found_flag == len(elements):
                return True
            found_flag = 0
        return False

    def _prepare_sentence(self, sentence):
        # Removing punctuation and capital letters
        prepared_sentence = re.sub(r'[^\w\s]', '', sentence).lower()
        return prepared_sentence

    def _select_outanswer(self, context):
        #select one of the outanswer randomly and return the string
        outlist = context.findall('outanswer')
        return outlist[rand.randint(0,(len(outlist)-1))].text

# if __name__ == "__main__":

#     try:
#         rospy.init_node('DialogBhvr_node', anonymous=False)
#         dialog_behavior = DialogBehavior()
#         rospy.spin()

#     except rospy.ROSInterruptException:
#         print("except")
#         pass
