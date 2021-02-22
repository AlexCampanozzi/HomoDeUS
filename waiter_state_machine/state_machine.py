#! /usr/bin/env python

from states import *
import rospy


class StateMachine:

    """
    State Machine to manage states and transitions.

    Example on how to use it:
    >>> from states import *
    >>> from state_machine import *
    >>>
    >>> state_machine = StateMachine()
    >>>
    >>> # Creation
    >>> state_machine.add_state(State00())
    >>> state_machine.add_state(State01())
    >>> state_machine.add_state(State02())
    >>>
    >>> # Main loop
    >>> while state_machine.get_current_state().id is not 'state 03':
    >>>     state_machine.run_current_state()
    >>>     state_machine.check_transitions()
    """
    def __init__(self):
        self.states = {}
        self.current_state = None

    def add_state(self, state):
        """
        Method to add a state to the states dictionary.
        """
        key = state.get_id()

        if key not in self.states.keys():
            self.states[key] = state

            if self.current_state is None:
                self.current_state = self.states.get(key)

            return True

        else:
            return False

    def run_current_state(self):
        """
        Method to execute actions associated to the active state.
        """
        self.current_state.run()

    def check_transitions(self):
        """
        Method to check if a transition should occur and to perform
        the switch of state if necessary.
        """
        print("statemachine checkTransition")
        next_state = self.current_state.get_next_state()

        if (next_state is not None) and (next_state in self.states.keys()):
            self.current_state.reset()
            self.current_state = self.states.get(next_state)
            self.current_state.run_pre_execution = True
            self.current_state.run_execution = True
            self.current_state.run_post_execution = True
            rospy.sleep(0.5)
            return True

        return False

    def get_current_state(self):
        """
        This method returns the current active state
        """
        return self.current_state
