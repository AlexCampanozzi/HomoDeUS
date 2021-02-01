#! /usr/bin/env python

from modules import *


class StateBase:
    """
    Base class from which every state inherits from. Each child has
    to provide an implementation for the following methods:
        0. __init__()
        1. _set_id()
        2. _pre_execution()
        3. _execution()
        4. _post_execution()
        5. _get_next_state()
    """
    def __init__(self):
        self.id = self._set_id()
        self.reset()

        # TODO: Maybe create objects of each module here?

    def reset(self):
        """
        This method gets called by the StateMachine to reset the state
        before transitioning to another one. This allows for a full run
        pass the next time the state gets activated.
        """
        self.run_pre_execution = True
        self.run_execution = True
        self.run_post_execution = True

    def get_id(self):
        """
        Return the state's id.
        """
        return self.id

    def run(self):
        """
        This method gets called by the StateMachine to execute
        actions associated with the state (if active). By default,
        self.run_pre_execution() and self._execution() will only run once
        after the activation while self._post_execution() will be ran in a
        loop and can be used to idle while waiting for a transition.
        """
        if self.run_pre_execution:
            self._pre_execution()
            self.run_pre_execution = False

        elif self.run_execution():
            self._execution()
            self.run_execution = False

        elif self.run_post_execution:
            self._post_execution()

    def _set_id(self):
        """ 
        Set the state's id used by the StateMachine for transitions.
        This method should return a string containing the id.
        """
        raise NotImplementedError()

    def _pre_execution(self):
        """
        Initialization stage of the run method. If self.run_pre_execution
        is not set to true in self._post_execution(), this method will only
        run once after a transition to the state.
        """
        raise NotImplementedError()

    def _execution(self):
        """
        Main stage of the run method. If self.run_execution is not set to
        true in self._post_execution(), this method will only run once after
        a transition to the state.
        """
        raise NotImplementedError()

    def _post_execution(self):
        """
        Post stage of the run method. This method can be used for idling since
        it will be executed in a loop until there's a transition to another
        state. The execution of this method could be blocked by setting
        self.run_post_execution to false.
        """
        raise NotImplementedError()

    def get_next_state(self):
        """
        This method should return the next state's id (string) if an event allows
        a transition. If there is no transition, it should return None.
        """
        raise NotImplementedError()

"""
+-------------------------------------------------+
|                 States 0 to 5                   |
+-------------------------------------------------+
"""


class State00(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 00'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State01(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 01'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State02(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 02'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State03(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 03'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State04(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 04'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State05(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 05'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


"""
+-------------------------------------------------+
|                 States 6 to 11                  |
+-------------------------------------------------+
"""


class State06(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 06'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State07(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 07'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State08(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 08'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State09(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 09'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State10(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 10'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass


class State11(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        # TODO: Add code here if necessary...

    def _set_id(self):
        return 'state 11'

    def _pre_execution(self):
        # TODO: Add code here if necessary...
        pass

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        # TODO: Add code here if necessary...
        pass
