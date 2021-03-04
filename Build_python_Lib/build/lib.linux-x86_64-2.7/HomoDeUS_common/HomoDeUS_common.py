# General library usable by many code part
import rospy

def convert_char_array_to_string(char_array):
    str=""
    try:
        return str.join(char_array)
    except:
        return "Error occured: variable non string"
