#! /usr/bin/env python
import rospy
import actionlib
import traceback

from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

import HomoDeUS_common_py.HomoDeUS_common_py as common

class Scenario_Selector:
    """
    This class select the scenario that we will choose
    """
    def __init__(self):
        #TODO
        #parse_xml_context_speech
        scenario_1 = actionlib.SimpleActionServer("scenario1",,auto_start=False)
        scenario_2 = actionlib.SimpleActionServer("scenario2",,auto_start=False)
        scenario_3 = actionlib.SimpleActionServer("scenario3",,auto_start=False)
        scenario_4 = actionlib.SimpleActionServer("scenario4",,auto_start=False)
    
    def listen_desires_set_Cb(self,desireSet):
        self.curDesireSet = desireSet

    def desire_event_change(self, context):
        #TODO
        #pas avec un event 
    
    def scenario_selection(self,context);
        scenario = context
        scenario.actionServer.start()
        scenario.wait_for_server()
        
        #voir avec les scenario comment on les start pour choisir le bon goal
        goal = start()#TBC

        scenario.send_goal(goal)
        scenario.wait_for_result()

    def accomplish_criterion(self, context):
        #TODO
        # voir la structure de accomplish
        return context

    def cancel_criterion(self):
        #besoin d'avoir un cancel du scenario present exemple: client deside de partir avant fin d'un scenario
        return False

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