#! /usr/bin/env python

import xml.etree.ElementTree as ET
import os
    
#parse the xml containing the contexts
speech_context_xml_path = "src/HomoDeUS/tiago_hhba_cfg/tiago_motivations/others/speech_context.xml"
speech_context = ET.parse(speech_context_xml_path).getroot()
for element in speech_context.findall('*'):
    print(element.tag)
    for answer in element.findall('answer'):
        print('     ' + answer.text)


# print ('step2')
# teststring = " i am named bob"
# if 'bob' in teststring:
#     print('bobfound')
# print('bobend')