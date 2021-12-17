#!/usr/bin/env python
import os
import json
import rospy
import time
from std_msgs.msg import String, UInt16
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from scenario_manager_action_server import ScenarioManagerAction
import actionlib
from custom_msgs.msg import scenario_managerAction, scenario_managerResult, scenario_managerFeedback
from states.scenario_1_states import state_00, state_01, state_02, state_03, state_04, state_05, state_06, state_07


FILE_LOCATION = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))+'/homodeus_common/dialog_answer.json'

class Scenario1Manager(ScenarioManagerAction):
    def __init__(self):
        ScenarioManagerAction.__init__(self, name="scenario_1_manager")
        self.desires = {}
        self.states = {}
        self.sub_desires = None
        self.current_state = None

        self.reaction_events = [Event.ACC_ON, Event.ACC_OFF, Event.IMP_ON, Event.IMP_OFF]
        self.rem_desires = rospy.ServiceProxy('remove_desires', RemoveDesires)
        self.test_state = rospy.Subscriber("/test_state_scenario_1", UInt16, self.test_state_cb, queue_size=5)

        self.menu_selection = ''
        # Get all the states of the current scenario

        self.state_00 = state_00.State00(self.desires)
        self.state_01 = state_01.State01(self.desires)
        self.state_02 = state_02.State02(self.desires)
        self.state_03 = state_03.State03(self.desires)
        self.state_04 = state_04.State04(self.desires)
        self.state_05 = state_05.State05(self.desires) # approach client
        self.state_06 = state_06.State06(self.desires)
        self.state_07 = state_07.State07(self.desires)

        # Add the states to the current states dict for the follow up
        self.add_state(self.state_00)
        self.add_state(self.state_01)
        self.add_state(self.state_02)
        self.add_state(self.state_03)
        self.add_state(self.state_04)
        self.add_state(self.state_05)
        self.add_state(self.state_06)
        self.add_state(self.state_07)


        # Build the normal scenario_sequence (when everything goes as it should)
        self.scenario_sequence = [self.state_00,self.state_05,self.state_01,self.state_02,self.state_03,self.state_06, self.state_04, self.state_07]
        self.index = 0

        self.current_state = self.scenario_sequence[self.index]
        self._as.register_preempt_callback(self.canceled_cb)
        rospy.wait_for_service("remove_desires")
        rospy.logwarn("================================= SCENARIO 1 READY ======================================")


    def add_state(self, state):
        key = state.get_id()

        if key not in self.states.keys():
            self.states[key] = state

    def observe(self):
        self.sub_desires = rospy.Subscriber("events", Event, self.eventCB, queue_size=5)

    def stopObserving(self):
        self.sub_desires.unregister()

    def eventCB(self, event):
        # Update own desire states w/ events seen
        if event.desire in self.desires:
            #Check if event type is relevant
            if event.type in self.reaction_events:
                self.desires[event.desire] = event.type

                success = self.current_state.react_to_event()
                
                if success is not None:
                    self.current_state.cleanup()
                    #If success, go to next state
                    #If failure, should have already try some redo in their behaviour so scenario failed
                    if success:
                        if self.specific_outcome():
                            return
                        elif self.last_state():
                            return
                        self.start_next_step()

                    else:
                        self._result.result = False
                        self._as.set_aborted(result=self._result)

    def execute_cb(self, goal):
        rospy.loginfo("---------------- scenario 1 has been called ----------------")
        if goal.execute is True:
            if self.sub_desires is None:
                self.observe()
                # initial desire addition
            else: 
                self.__cancel_current_desires()

            # self.current_state = self.state_00
            self.current_state = self.scenario_sequence[self.index]
            self.current_state.add_state_desires()
            while self._as.is_active():
                time.sleep(2)
        else:
            pass
            # no stuff
    def test_state_cb(self, state_number):
        state_number = state_number.data
        
        if state_number < 0 or state_number > 3:
            rospy.logwarn("This state number currently doesn't exist")
            return
        
        if self.sub_desires is None:
            self.observe()
        else:
            self.__cancel_current_desires()

        rospy.loginfo("Testing state " + str(state_number))
        self._feedback.prev_state = self.current_state.get_id()
        
        if state_number == 0:
            self.current_state = self.state_00
        elif state_number == 1:
            self.current_state = self.state_01
        elif state_number == 2:
            self.current_state = self.state_02
        elif state_number == 3:
            self.current_state = self.state_03
        elif state_number == 4:
            self.current_state = self.state_04
            self.current_state.command = 'apple'
        elif state_number == 5:
            self.current_state = self.state_05
            

        self._feedback.state = self.current_state.get_id()
        
        self._as.publish_feedback(self._feedback)
        
        self.current_state.add_state_desires()

    def canceled_cb(self):
        rospy.loginfo("---------------- scenario 1 has been cancelled ----------------")
        self.__cancel_current_desires()
        self.current_state = self.state_00
        self._as.set_preempted()
        self.stopObserving()
    
    def __cancel_current_desires(self):
        self.current_state.cleanup()
        self.desires.clear()

    def specific_outcome(self):
        if self.current_state.get_id() == 'Get_Command':
            dialog_info = self.read_dialog_info(FILE_LOCATION)
            if dialog_info == 'nothing':
                self._result.result = False
                self._as.set_aborted(result=self._result)
                return True

            else:
                self.menu_selection = dialog_info
                self.state_03.command = self.menu_selection
                self.state_04.command = self.menu_selection

    def read_dialog_info(self, file_location):
        with open(file_location) as file:
            data = json.load(file)
            for info in data['dialog']:
                return info['info']
    
    def last_state(self):
        if len(self.scenario_sequence)-1 <= self.index:
            self.reset_scenario()
            self._result.result = True
            self._as.set_succeeded(result=self._result)
            return True
        return False

    def reset_scenario(self):
        self.index = 0
        self.current_state = None

    def start_next_step(self):
        self.index = self.index+1
        self._feedback.prev_state = self.current_state.get_id()
        
        self.current_state = self.scenario_sequence[self.index]        
        self._feedback.state = self.current_state.get_id()
        
        self._as.publish_feedback(self._feedback)
        self.current_state.add_state_desires()



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
        rospy.init_node("scenario_1_manager")
        node = Scenario1Manager()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
