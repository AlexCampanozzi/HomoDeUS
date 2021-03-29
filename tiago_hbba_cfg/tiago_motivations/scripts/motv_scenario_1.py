import rospy
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesires
from action.scenario_manager_action_server import ScenarioManagerAction
import states

class Scenario1Manager(ScenarioManagerAction):

    def __init__(self):
        ScenarioManagerAction.__init__(self, name="scenario_1_manager")
        self.desires = {}
        self.states = {}
        self.current_state = None
        self.reaction_events = [Event.ACC_ON, Event.ACC_OFF, Event.IMP_ON, Event.IMP_OFF]
        # Get and add all states
        self.add_state(states.state_00.State00(self.desires))
        self.add_state(states.state_01.State01(self.desires))
        self.add_state(states.state_02.State02(self.desires))
        self.add_state(states.state_03.State03(self.desires))
        self.add_state(states.state_04.State04(self.desires))
        self.add_state(states.state_05.State05(self.desires))
        self.add_state(states.state_06.State06(self.desires))
        self.add_state(states.state_07.State07(self.desires))
        self.add_state(states.state_08.State08(self.desires))
        self.add_state(states.state_09.State09(self.desires))
        self.add_state(states.state_10.State10(self.desires))
        self.add_state(states.state_11.State11(self.desires))
        self.add_state(states.state_12.State12(self.desires))

        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("remove_desires")

        self.register_preempt_callback(self.canceled_cb)

    def add_state(self, state):
        key = state.get_id()

        if key not in self.states.keys():
            self.states[key] = state

            if self.current_state is None:
                self.current_state = self.states.get(key)

    def observe(self):
        sub_desires = rospy.Subscriber("events", Event, self.eventCB, queue_size=5)

    def eventCB(self, event):
        # Update own desire states w/ events seen
        if event.desire in self.desires:
            if event.type in self.reaction_events:
                self.desires[event.desire] = event.type

                react_result = self.states[self.current_state].react_to_event()

                if react_result is not None:
                    self.states[self.current_state].cleanup()

                    if react_result == "Done":
                        self._result.result = True
                        self._as.set_succeeded(self._result)

                        self.current_state = "state_00"
                        # TODO opposite of self.observe()
                        
                    else:
                        self._feedback.prev_state = self.current_state
                        self.current_state = react_result

                        self._feedback.state = react_result
                        self._as.publish_feedback(self._feedback)

                        self.states[self.current_state].add_desires()
    
    def execute_cb(self, goal):
        if goal.execute is True:
            self.observe()
            # initial desire addition
            self.states[self.current_state].add_desires()
        else:
            pass
            # no stuff

    def canceled_cb(self):
        for desire in self.desires:
            self.rem_desires.call(desire)
        self.desires.clear()
        self.current_state = "state_00"
        self._as.set_preempted()
        # TODO opposite of self.observe()


if __name__ == "__main__":
    try:
        rospy.init_node("scenario_1_manager)

        node = Scenario1Manager()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
