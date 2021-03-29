#! /usr/bin/env python
import traceback

import actionlib
import HomoDeUS_common_py.HomoDeUS_common_py as common
import rospy
from hbba_msgs.msg import Desire, DesiresSet, Event
from std_msgs.msg import Bool, String

import test_parser


class Scenario_Selector:
    """
    This class select the scenario that we will do
    """
    def __init__(self):
        #TODO
        #parse_xml_context_speech
        self.actionServer1 = actionlib.SimpleActionClient("scenario1",scenario_selector)#actionclient
        self.actionServer2 = actionlib.SimpleActionClient("scenario2",scenario_selector)
        self.actionServer3 = actionlib.SimpleActionClient("scenario3",scenario_selector)
        self.actionServer3 = actionlib.SimpleActionClient("scenario4",scenario_selector)

        # The input of the module
        self.input_motv = rospy.Subscriber("/proc_output_keywordDetect", Bool, self.listen_Keyword_cb, queue_size=10)

        # the output of the module
        self.event_SUbcriber = rospy.Subscriber("events",Event, self.listen_event_cb , queue_size = 10)


        #ajout de desire
        self.add_keyword_desire()

        self.pause_all = False
        self.desire_type_dict = dict()
    
 
    def add_keyword_desire(self):
        des = Desire()
        des.id          = "Scenario_selector_keyword"
        des.type        = "Keyword_detection"
        des.utility     = 5.0
        des.intensity   = 100.0
        des.params      = "robot"

        self.add_desires.call([des])

    #def desire_event_change(self, context):
       
        #pas avec un event quil est controler
    def listen_Keyword_cb(self, detection):
        if detection:
            self.pause_all = True
            for key in self.desire_type_dict:
                if key is not "speech_recognition":
                    self.add_pause_desire(key)
              
        
        #pause all desire
    def add_pause_desire(self, desire_type):
        des = Desire()
        des.id          = "Pause_"+desire_type
        des.type        = desire_type
        des.utility     = 0.0
        des.intensity   = 1000.0
        
        self.add_desires.call([des])

    def listen_event_cb(self,event):
        if event.type = Event.DES_ON:
            if event.desire_type not in self.desire_type_dict:
               self.desire_type_dict[event.desire_type] = event.desire_type
                if self.pause_all is True:
                    self.add_pause_desire(event.desire_type)
                
        #todo
        #si pause, envoyer tout les desire avec utility 0
        #pause = "Pause_"
        #ajouter desire_id dans dict
        # if pause in self.desire_type_dict:
        #   


    def scenario_selection(self,context):
        scenario = context
        
        self.scenario.wait_for_server()
        
        #voir avec les scenario comment on les start pour choisir le bon goal
        goal = start()#TBC

        self.scenario.send_goal(self, goal)#send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None)
        self.scenario.wait_for_result()
    def cb_speech ();

    def cb_desire();

    def cb_event();

    def accomplish_criterion(self, context):
        #TODO
        # voir la structure de accomplish/voir si un mot es rechercher est detecter
        #return context

    def cancel_criterion(self):
        #TODO
        #besoin d'avoir un cancel du scenario present exemple: client deside de partir avant fin d'un scenario
        #return False

    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        common.loginfo(self,"have been shutdown")


if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Scenario_Selector()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())
