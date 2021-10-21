#!/usr/bin/env python
import rospy
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from scenario_manager_action_server import ScenarioManagerAction
import actionlib
import custom_msgs.msg
from scenario_2_states import scenario_2_state_00
from scenario_2_states import scenario_2_state_01
from scenario_2_states import scenario_2_state_02
from scenario_2_states import scenario_2_state_03
from scenario_2_states import scenario_2_state_04

class Scenario2Manager(ScenarioManagerAction):

    def __init__(self):
        ScenarioManagerAction.__init__(self, name="scenario_2_manager")
        self.desires = {}
        self.states = {}
        self.current_state = None
        self.reaction_events = [Event.ACC_ON, Event.ACC_OFF, Event.IMP_ON, Event.IMP_OFF]
        # Get and add all states
        self.add_state(scenario_2_state_00.State00(self.desires))
        self.add_state(scenario_2_state_01.State01(self.desires))
        self.add_state(scenario_2_state_02.State02(self.desires))
        self.add_state(scenario_2_state_03.State03(self.desires))
        self.add_state(scenario_2_state_04.State04(self.desires))
        self._as.register_preempt_callback(self.canceled_cb)
        self.rem_desires = rospy.ServiceProxy('remove_desires', RemoveDesires)
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
        print("evencb reacted")
        print(event.desire)
        print(self.desires)
        if event.desire in self.desires:
            print("if 1")
            if event.type in self.reaction_events:
                print("if  2")
                self.desires[event.desire] = event.type

                react_result = self.states[self.current_state].react_to_event()

                if react_result is not None:
                    print("if 3")
                    self.states[self.current_state].cleanup()

                    if react_result == "Done":
                        print("if 4")
                        self._result.result = True
                        self._as.set_succeeded(self._result)

                        self.current_state = "state_00"
                        self.stopObserving()
                        
                    else:
                        print("else")
                        self._feedback.prev_state = self.current_state
                        self.current_state = react_result

                        self._feedback.state = react_result
                        self._as.publish_feedback(self._feedback)

                        self.states[self.current_state].add_state_desires()
    
    def execute_cb(self, goal):
        if goal.execute is True:
            self.observe()
            # initial desire addition
            self.states[self.current_state].add_state_desires()
        else:
            pass
            # no stuff

    def canceled_cb(self):
        for desire in self.desires:
            self.rem_desires.call(desire)
        self.desires.clear()
        self.current_state = "state_00"
        self._as.set_preempted()
        self.stopObserving()

class Scenario2Tester:
    def __init__(self):
        client = actionlib.SimpleActionClient("scenario_2_manager", custom_msgs.msg.scenario_managerAction)
        client.wait_for_server()
        goal = custom_msgs.msg.scenario_managerGoal(execute=True)
        print(goal)
        client.send_goal(goal)


if __name__ == "__main__":
    try:
        rospy.init_node("scenario_2_manager")

        node = Scenario2Manager()
        doTest = Scenario2Tester()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
