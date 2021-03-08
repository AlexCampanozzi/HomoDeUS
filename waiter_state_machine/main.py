#! /usr/bin/env python

import rospy
from states import *
from state_machine import *
from behaviors import *


def main():
    # Creating the node
    rospy.init_node('waiter_state_machine')

    # Initializing the behaviors
    initialiseBehaviors()

    # Creating the machine
    state_machine = StateMachine()

    # Assembling the state machine
    state_machine.add_state(State00())
    state_machine.add_state(State01())
    state_machine.add_state(State02())
    state_machine.add_state(State03())
    state_machine.add_state(State04())
    state_machine.add_state(State05())
    state_machine.add_state(State06())
    state_machine.add_state(State07())
    state_machine.add_state(State08())
    state_machine.add_state(State09())
    state_machine.add_state(State10())
    state_machine.add_state(State11())
    state_machine.add_state(State12())

    # Printing the current state id
    current_state_id = state_machine.get_current_state().get_id()
    rospy.loginfo("[Main] Switched to state: " + current_state_id)

    # Main loop
    while not rospy.is_shutdown():
        state_machine.run_current_state()

        if state_machine.check_transitions():
            current_state_id = state_machine.get_current_state().get_id()
            rospy.loginfo("[Main] Switched to state: " + current_state_id)


if __name__ == "__main__":
    main()
