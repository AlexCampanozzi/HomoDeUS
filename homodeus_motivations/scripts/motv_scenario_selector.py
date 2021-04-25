#! /usr/bin/env python
import traceback

import actionlib
import rospy
from std_msgs.msg import Bool, String
from hbba_msgs.msg import Desire, DesiresSet, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from custom_actions.msg import ScenarioSelectorActivatedAction, ScenarioSelectorActivatedGoal

import HomoDeUS_common_py.HomoDeUS_common_py as common


class Scenario_Selector:
    """
    This class select the scenario to be executed by the robot
    """
    def __init__(self):
        #initialise scenario_client so it can be modified later
        self.scenario_client = actionlib.SimpleActionClient("Empty",ScenarioSelectorActivatedAction)

        # The input of the module
        self.input_motv_keyword = rospy.Subscriber("/proc_output_keywordDetect", Bool, self.listen_Keyword_cb, queue_size=10)
        self.input_motv_dialog_relevant = rospy.Subscriber("/bhvr_output_res_dialRelevant", String,self.listen_dialog_cb, queue_size=10)

        rospy.wait_for_service('add_desires')
        self.add_desires_service = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires_service = rospy.ServiceProxy('remove_desires', RemoveDesires)

        self.actual_scenario = ""

        # desire called in this motv
        self.desire_pause_id = "pause_for_keyword"
        self.desire_dialoguing_id = "listen_for_task"
        self.desire_keyword_id = "hear_his_name"
        
        # ajout de desire
        self.add_desire(desire_id=self.desire_keyword_id,desire_type="Keyword_detection",desire_utility=8, \
            desire_intensity=50, desire_params = "{value: 'roberto'}")
       

    def listen_Keyword_cb(self, detection):
        """
        This method adds the desire of dialoguing with the person asking for the robot and remove the desire of hearing for it's name 

        Arguments
        ---------
        detection: Bool()
            if the robot heard or not it's name
        """
        if detection.data:
            # self.add_desire(desire_id=self.desire_pause_id,desire_type="pause",desire_utility=5.0,desire_intensity=100.00)
            self.add_desire(desire_id=self.desire_dialoguing_id, desire_type= "Dialoguing", desire_utility=5.0, \
                desire_intensity=100.0, desire_params = "{context: 'scenario_selection'}")
            self.rem_desires_service([self.desire_keyword_id])

    def listen_dialog_cb(self,speechText):
        """
        This method receive the information send from the dialogue bhvr and starts a scenario or cancel the actual one
        if needed

        Arguments
        ---------
        speechText: String()
            what informations the robot got from it's dialog bhvr
        """
        if 'scenario' in str(speechText.data):
            self.startScenario(speechText.data)

        if 'cancel' in str(speechText.data):
            self.cancel_scenario()
        self.rem_desires_service.call([self.desire_pause_id, self.desire_dialoguing_id])
        self.add_desire(desire_id=self.desire_keyword_id,desire_type="Keyword_detection",desire_utility=8, \
            desire_intensity=50, desire_params = "{value: 'roberto'}")

    def cancel_scenario(self):
        """
        This method receive cancel the actual scenario so the robot stop doing it
        """
        self.scenario_client.cancel_all_goals()
        self.actual_scenario = ""
    
    def startScenario(self, scenario):
        """
        This method send a goal the the right action server related to the scenario the robot has to do.
        By sending a goal, the scenario will begin
        
        Arguments
        ---------
        scenario: string
            The name of the action_server to connect to and send a goal
        """
        if self.actual_scenario is not "":
            self.cancel_scenario()
        
        self.scenario_client = actionlib.SimpleActionClient(scenario.split()[0],ScenarioSelectorActivatedAction)

        self.scenario_client.wait_for_server()
        
        goal = ScenarioSelectorActivatedGoal()
        goal.start_scenario = True
        self.scenario_client.send_goal(goal)

        self.actual_scenario = scenario


    def add_desire(self,desire_id, desire_type,desire_utility,desire_intensity, desire_params=None):
        """
        This method adds a desire to the iw 
        
        Arguments
        ---------
        desire_id: string
            The name of the desire
        desire_type: string
            the type of the desire, should fit with one listed in desires.txt in homodeus_hbba_cfg
        desire_utility: float
            The utility of the desire so hbba knows which strategy to use
        desire_intensity: float
            The intensity reprensenting the importance of the desire to be done
        desire_params: string
            other important informations used by the strategy to fulfill the desire
        """
        des = Desire()
        des.id          = desire_id
        des.type        = desire_type
        des.utility     = desire_utility
        des.intensity   = desire_intensity
        if desire_params is not None:
            des.params = desire_params
        
        rospy.loginfo("adding desire: " + des.id)
        self.add_desires_service.call([des])

                
    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        rospy.loginfo("have been shutdown")


if __name__ == '__main__':
    """
    It starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Scenario_Selector()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        rospy.logerr(__file__,traceback.format_exc())
