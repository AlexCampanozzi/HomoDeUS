#!/usr/bin/env python
import os
import rospy
import time
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

        self.reaction_events = [Event.ACC_ON, Event.ACC_OFF, Event.IMP_ON, Event.IMP_OFF]
        self.rem_desires = rospy.ServiceProxy('remove_desires', RemoveDesires)

        self.menu_selection = ''
        # Get and add all states
        self.state_00 = state_00_simul.State00(self.desires)
        self.state_01 = state_01_simul.State01(self.desires)
        self.state_02 = state_02_simul.State02(self.desires)
        self.state_03 = state_03_simul.State03(self.desires)
        self.add_state(self.state_00)
        self.add_state(self.state_01)
        self.add_state(self.state_02)
        self.add_state(self.state_03)
        self.sequence = [self.state_00,self.state_01,self.state_02,self.state_03]
        self.index = 0
        self.current_state = self.sequence[self.index]
        self._as.register_preempt_callback(self.canceled_cb)
        rospy.wait_for_service("remove_desires")

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
            if event.type in self.reaction_events:
                self.desires[event.desire] = event.type

                success = self.states[self.current_state].react_to_event()
                if success is not None:
                    self.states[self.current_state].cleanup()
                    if success:
                        if self.current_state.get_id() == 'Get_Command':
                            dialog_info = self.read_dialog_info(FILE_LOCATION)
                            if dialog_info == 'nothing':
                                self._result.result = False
                                self._as.set_aborted(result=self._result)
                                return

                            else:
                                self.menu_selection = dialog_info

                        elif len(self.sequence)-1 <= self.index:
                            self.index = 0
                            self.current_state = None
                            self._result = True
                            self._as.set_succeeded(result=self._result)
                            return

                        self.index = self.index+1
                        self._feedback.prev_state = self.current_state
                        self.current_state = self.sequence[self.index]
                        self._feedback.state = self.current_state
                        self._as.publish_feedback(self._feedback)
                        self.current_state.add_state_desires()
                    else:
                        self._result.result = False
                        self._as.set_aborted(result=self._result)
    
    def read_dialog_info(self,file_location):
        with open(file_location) as json_file:
            data = json.load(json_file)
            for dialog in data['dialog']:
                print('info: ' + dialog['info'])

    def execute_cb(self, goal):
        rospy.logdebug("---------------- scenario 1 has been called ----------------")
        if goal.execute is True:
            self.observe()
            # initial desire addition
            self.current_state.add_state_desires()
            print(self._as.is_active())
            print("+++++++++++++++++++++++++++++++++++++++++")
            while self._as.is_active():
                time.sleep(2)
        else:
            pass
            # no stuff

    def canceled_cb(self):
        print("cancelling scen")
        for desire in self.desires:
            self.rem_desires.call(desire)
        self.desires.clear()
        self.current_state = self.state_00
        self._as.set_preempted()
        self.stopObserving()

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
