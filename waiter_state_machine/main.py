#! /usr/bin/env python

import rospy
from states import *
from state_machine import *
from behaviors import *


def main():
    # Creating the node
    rospy.init_node('waiter_state_machine')
    initialiseBehaviors()

    # Creating the machine
    bertrand = StateMachine()
    print("StateMachineCreated")

    # Assembling the state machine
    bertrand.add_state(State00())
    bertrand.add_state(State01())
    bertrand.add_state(State02())
    bertrand.add_state(State03())
    bertrand.add_state(State04())
    bertrand.add_state(State05())
    bertrand.add_state(State06())
    bertrand.add_state(State07())
    bertrand.add_state(State08())
    bertrand.add_state(State09())
    bertrand.add_state(State10())
    bertrand.add_state(State11())

    # Printing the current state id
    rospy.loginfo("Switched to state: " + bertrand.get_current_state().get_id())

    # Main loop
    while not rospy.is_shutdown():
        bertrand.run_current_state()
        
        if bertrand.check_transitions():
            rospy.loginfo("Switched to state: " + bertrand.get_current_state().get_id())
        
        #rospy.sleep(0.1)

if __name__ == "__main__":
    main()
