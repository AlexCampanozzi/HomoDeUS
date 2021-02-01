#! /usr/bin/env python

# TODO: Include dependencies


class ModuleBase:
    """
    Base class from which every module inherits from. Each child has
    to provide an implementation for the following methods:
        0. __init__()
        1. _run()
    """
    def __init__(self):
        self.active = False

    def activate(self):
        """
        Activate the module.
        """
        self.active = True

    def deactivate(self):
        """
        Deactivate the module.
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
        Actions associated to the module.

        Arguments
        ---------
        params : dict
            A dictionary with the needed parameters.
        """
        raise NotImplementedError()


"""
+-------------------------------------------------+
|                    Modules                      |
+-------------------------------------------------+
"""


class FaceTracking(ModuleBase):
    def __init__(self):
        ModuleBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass


class VoiceRecognition(ModuleBase):
    def __init__(self):
        ModuleBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass


class Voice(ModuleBase):
    def __init__(self):
        ModuleBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass


class Locomotion(ModuleBase):
    def __init__(self):
        ModuleBase.__init__(self)
        # TODO: Add code here if necessary...

    def _run(self, params):
        # TODO: Add code here if necessary...
        pass
