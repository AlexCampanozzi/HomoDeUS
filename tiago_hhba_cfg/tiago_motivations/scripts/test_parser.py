#! /usr/bin/env python

import xml.etree.ElementTree as ET
    
#parse the xml containing the contexts
speech_context_xml_path = 'speech_context.xml'
speech_context = ET.parse(speech_context_xml_path).getroot()
for element in speech_context.findall('*'):
    print(element.tag)
    for answer in element.findall('answer'):
        print('     ' + answer.text)
