#!/usr/bin/env python
import os
import rospy
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from scenario_manager_action_server import ScenarioManagerAction
import actionlib
from custom_msgs.msg import scenario_managerAction, scenario_managerResult, scenario_managerFeedback
from states import state_00_simul, state_01_simul, state_02_simul, state_03_simul


FILE_LOCATION = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))+'/homodeus_common/dialog_answer.json'
class Scenario1Manager(ScenarioManagerAction):
    def __init__(self):
        ScenarioManagerAction.__init__(self, name="scenario_1_manager")
        self.desires = {}
        self.states = {}
        self.current_state = None
        self.reaction_events = [Event.ACC_ON, Event.ACC_OFF, Event.IMP_ON, Event.IMP_OFF]
        self.rem_desires = rospy.ServiceProxy('remove_desires', RemoveDesires)
        # Get and add all states
        self.state_00 = state_00_simul()
        self.state_01 = state_00_simul()
        self.state_02 = state_00_simul()
        self.state_03 = state_00_simul()
        self.sequence = [self.state_00,self.state_01,self.state_02,self.state_03]
        self._as.register_preempt_callback(self.canceled_cb)
        rospy.wait_for_service("remove_desires")

    def add_state(self, state):

        key = state.get_id()

        if key not in self.states.keys():
            self.states[key] = state

            if self.current_state is None:
                self.current_state = key

    def observe(self):
        self.sub_desires = rospy.Subscriber("events", Event, self.eventCB, queue_size=5)

    def stopObserving(self):
        self.sub_desires.unregister()

    def eventCB(self, event):
        # Update own desire states w/ events seen
        if event.desire in self.desires:
            if event.type in self.reaction_events:
                self.desires[event.desire] = event.type

                success = self.states[self.current_state].react_to_event()

                if success is True:
                    self.states[self.current_state].cleanup()
                    if self.current_state.name == 'dialog':
                        dialog_info = self.read_dialog_info(FILE_LOCATION)
                        if dialog_info == 'nothing':
                            
                            self._as.set_aborted(result=scenario_result)

                    if react_result == "Done":
                        print("done")
                        self._result.result = True
                        self._as.set_succeeded(self._result)

                        self.current_state = "state_00"
                        self.stopObserving()
                        
                    else:
                        self._feedback.prev_state = self.current_state
                        self.current_state = react_result

                        self._feedback.state = react_result
                        self._as.publish_feedback(self._feedback)
                        print(self._feedback.state)

                        self.states[self.current_state].add_state_desires()
    
    
    def read_dialog_info(self,file_location):
        with open(file_location) as json_file:
            data = json.load(json_file)
            for dialog in data['dialog']:
                print('info: ' + dialog['info'])

    def execute_cb(self, goal):
        rospy.logdebug("---------------- scenario 1 has been called ----------------")
        if goal.execute is True:
            self.current_state = self.state1
            while not rospy.is_shutdown() and self.current_state is not None:
                result = self.current_state.execute()
                self.state_transition(result)
            self.state_transition()
            self.state1.execute() #goto table
            self.state_transition()
            self.state2.execute() #Dialogue
            self.state3.execute() #GotoKitchen
            self.state4.execute() #Talk
            # initial desire addition
        else:
            pass
            # no stuff

    def state_transition(self,result):
        rospy.loginfo("changing scenario 1 state")
        feedback = scenario_managerFeedback()
        scenario_result = scenario_managerResult()
        if result == 'FAILED':
            scenario_result.result = result
            self._as.set_aborted(result=scenario_result)
        else:
            pass
        
        self._as.publish_feedback()

    def canceled_cb(self):
        print("cancelling scen")
        self.current_sate.cancel()
        self._as.set_preempted()

class Scenario1Tester:
    def __init__(self):
        print("test ini")
        client = actionlib.SimpleActionClient("scenario_1_manager", custom_msgs.msg.scenario_managerAction)
        print("made client")
        client.wait_for_server()
        print("scen server found")
        goal = custom_msgs.msg.scenario_managerGoal(execute=True)
        print(goal)
        client.send_goal(goal)
        print("goal sent to scen")


if __name__ == "__main__":
    try:
        print(FILE_LOCATION)
        #rospy.init_node("scenario_1_manager")
        #node = Scenario1Manager()
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass
