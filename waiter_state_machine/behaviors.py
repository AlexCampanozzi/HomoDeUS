#! /usr/bin/env python

# TODO: Include dependencies


class BehaviorBase:
    """
    Base class from which every behavior inherits from. Each child has
    to provide an implementation for the following methods:
        0. __init__()
        1. _run()
    """
    def __init__(self):
        self.active = False

    def activate(self):
        """
        Activate the behavior.
        """
        self.active = True

    def deactivate(self):
        """
        Deactivate the behavior.
        """
        self.active = False

    def run(self, params=None):
        """
        Execute the _run() method.

        Arguments
        ---------
        params : dict
            A dictionary to pass custom parameters to the _run() method.
        """
        if self.active:
            return self._run(params)

    def _run(self, params):
        """
        Actions associated to the behavior.

        Arguments
        ---------
        params : dict
            A dictionary with the needed parameters.
        """
        raise NotImplementedError()


"""
+-------------------------------------------------+
|                    Behaviors                    |
+-------------------------------------------------+
"""


class FaceTracking(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass


class VoiceRecognition(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass


class Voice(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)

    def _run(self, params):
        speech = params["speech"]
        language = params["language"]

        if self.active:
            


class Locomotion(BehaviorBase):
    def __init__(self):
        BehaviorBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass
