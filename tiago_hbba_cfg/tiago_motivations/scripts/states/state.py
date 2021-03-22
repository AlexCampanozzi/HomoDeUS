import rospy
from hbba_msgs.msg import Desire, Event
from hbba_msgs.srv import AddDesires, RemoveDesiresc

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

        self.add_desires    = rospy.ServiceProxy('add_desires', AddDesires)
        self.rem_desires    = rospy.ServiceProxy('remove_desires', RemoveDesires)
        rospy.wait_for_service("add_desires")

    def add(self, desId, desType, utility=1.0, intensity=1.0, params=""):
        des = Desire()
        des.id          = desId
        des.type        = desType
        des.utility     = utility
        des.intensity   = intensity
        des.params      = params
        self.add_desires.call([des])

    def remove(self, name):
        self.rem_desires.call([name])

    def get_id(self):
        """
        Return the state's id.
        """
        return self.id

    def _set_id(self):
        """ 
        Set the state's id used by the StateMachine for transitions.
        This method should return a string containing the id.
        """
        raise NotImplementedError()

    def get_next_state(self, stateDict):
        """
        This method should return the next state's id (string) if an event allows
        a transition. If there is no transition, it should return None.
        """
        raise NotImplementedError()

    def add_state_desires(self):
        """
        This method adds the state's desires
        """
        raise NotImplementedError()

    def cleanup(self, nextState):
        """
        This method removes the state's desires not present in the next state
        """
        raise NotImplementedError()
