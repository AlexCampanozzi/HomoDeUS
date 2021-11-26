#!/usr/bin/env python

# A simple module to publish events based on the results of GoTo actions

import os
import time
import json
import rospy
import actionlib
from yaml import safe_load
# from json import loads
# from ast import literal_eval
from std_msgs.msg import String
from hbba_msgs.msg import Desire, DesiresSet, Event
import HomoDeUS_common_py.HomoDeUS_common_py as common

FILE_LOCATION = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))+'/homodeus_common/dialog_answer.json'
class dialogObserver:
    def __init__(self):
        self.eventPublisher = rospy.Publisher("events", Event)
        self.curDesireSet = DesiresSet()

    def listenDesiresSet(self):
        self.desiresSetSubscriber = rospy.Subscriber("desires_set", DesiresSet, self.listenDesiresSetCB)

    def listenDesiresSetCB(self, desireSet):
        self.curDesireSet = desireSet

    def listenDialogResult(self):
        self.dialogSubscriber = rospy.Subscriber("bhvr_output_res_dialRelevant", String, self.listenDialogCB)

    def listenDialogCB(self, relevant_infos):
            result = relevant_infos.data
            for desire in self.curDesireSet.desires:
                if desire.type == "Dialoguing":
                    event = Event()
                    event.desire = desire.id
                    event.desire_type = desire.type
                    if result != '':
                        rospy.loginfo("Dialogue return a relevant information")
                        self.write_dialog_info(result, FILE_LOCATION)
                        event.type = Event.ACC_ON
                    elif result == 'interruption':
                        # Nothing to do here since the desire is already being removed
                        pass
                    else:
                        rospy.loginfo('Dialogue did not return anything')
                        event.type = Event.ACC_OFF
                    self.eventPublisher.publish(event)

    def write_dialog_info(self, result, file_location):
        data = {}
        data['dialog'] = []
        data['dialog'].append({
            'info': result
        })
        with open(file_location, 'w') as outfile:
            json.dump(data, outfile)


if __name__ == "__main__":
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = dialogObserver()
        node.listenDesiresSet()
        node.listenDialogResult()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
