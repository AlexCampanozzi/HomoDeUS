#! /usr/bin/env python
import traceback
import actionlib
import rospy
from std_msgs.msg import Bool, String
from hbba_msgs.msg import Desire, DesiresSet, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from custom_msgs.msg import scenario_managerAction, scenario_managerGoal, scenario_managerResult, ttsActionAction,ttsActionGoal

import HomoDeUS_common_py.HomoDeUS_common_py as common


class Scenario_Selector:
    """
    This class select the scenario to be executed by the robot
    """
    def __init__(self):
        #initialise scenario_client so it can be modified later
        self.scenario_client = None


        # The input of the module
        self.input_motv_keyword = rospy.Subscriber("/proc_output_keywordDetect", Bool, self.listen_Keyword_cb, queue_size=10)
        self.input_motv_dialog_relevant = rospy.Subscriber("/bhvr_output_res_dialRelevant", String,self.listen_dialog_cb, queue_size=10)
        rospy.wait_for_service('add_desires')
        self.add_desires_service = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires_service = rospy.ServiceProxy('remove_desires', RemoveDesires)

        self.actual_scenario = ""

        self.on_robot = rospy.get_param('on_robot',False)

        # desire called in this motv
        #self.desire_pause_id = "pause_for_keyword"
        self.desire_dialoguing_id = "listen_for_task"
        self.desire_keyword_id = "hear_his_name"
        
        # ajout de desire
        common.add_desire(self,desire_id=self.desire_keyword_id,desire_type="Keyword_detection",desire_utility=8, \
            desire_intensity=50, desire_params = "{value: 'robot'}")

        rospy.loginfo('--------------BOOT SEQUENCE COMPLETED----------------------')

    def listen_Keyword_cb(self, detection):
        """
        This method adds the desire of dialoguing with the person asking for the robot and remove the desire of hearing for it's name 

        Arguments
        ---------
        detection: Bool()
            if the robot heard or not it's name
        """
        if detection.data:
            rospy.logwarn("------------------------")
            # self.add_desire(desire_id=self.desire_pause_id,desire_type="pause",desire_utility=5.0,desire_intensity=100.00)
            common.add_desire(self,desire_id=self.desire_dialoguing_id, desire_type= "Dialoguing", desire_utility=5.0, \
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

        self.rem_desires_service.call([self.desire_dialoguing_id])
        common.add_desire(self,desire_id=self.desire_keyword_id,desire_type="Keyword_detection",desire_utility=8, \
            desire_intensity=50, desire_params = "{value: 'robot'}")

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
        action_name = "/motv_scenario1/"+scenario.split()[0]+"_manager"
        rospy.loginfo(action_name)
        self.scenario_client = actionlib.SimpleActionClient(action_name,scenario_managerAction)

        self.scenario_client.wait_for_server()
        
        goal = scenario_managerGoal()
        goal.execute = True
        self.scenario_client.send_goal(goal)

        self.actual_scenario = scenario

                
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
        rospy.logerr(traceback.format_exc())
