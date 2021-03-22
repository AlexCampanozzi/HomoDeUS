#! /usr/bin/env python
import rospy
import actionlib
import traceback

from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

from HomoDeUS_common_py.HomoDeUS_common_py as common

class Speech_recognition_observer:
    """
    This class follows the state of the keyword_detection desire
    """
    def __init__(self):
        # The input of the module
        self.input_motv = rospy.Subscriber("/proc_output_listenText", String, self.desire_event_change, queue_size=10)

        # the output of the module
        self.event_publisher = rospy.Publisher("events",Event, queue_size = 10)

        # following DesireSet
        self.desires_set_subscriber = rospy.Subscriber("desires_set", DesiresSet, self.listen_desires_set_Cb)

    def listen_desires_set_Cb(self,desireSet):
        """
        This method updated the desireSet used in the desire_event_change method.

        Arguments
        ---------
        desireSet: DesiresSet
            The most recent desireSet
        """
        self.curDesireSet = desireSet

    def desire_event_change(self, SpeechText):
        """
        This method is the callback of the output of the module it is connected to, it then send an event depening of the result
        get from the topic.

        Arguments
        ---------
        SpeechText: String
            The result of the output of the module it is connected to. In this case it is the speech recognized
        """
        # S'il y a beaucoup de désirs du même type... il me semble que ca ne fonctionnera pas
        event = Event()
        for desire in self.curDesireSet.desires:
            if desire.type == "Speech_recognition":
                event.desire = desire.id
                event.desire_type = desire.type
                if self.accomplish_criterion(SpeechText.data):
                    event.type = Event.ACC_ON
                    self.event_publisher.publish(event)
                elif self.cancel_criterion():
                    #Problème actuel est que si le désir n'est pas activé il peut tout de même changer d'état!
                    event.type = Event.IMP_ON
                    self.event_publisher.publish(event)
                


    def accomplish_criterion(self, SpeechText):
        """
        This method look the result of the module and return True if it accomplished the criterion the motivation is looking for

        Arguments
        ---------
        SpeechText : string
            The text recognizes by the robot, it will be analyzed to know if something 
            relevant have been said and considerer if the desire is now accomplished
        """
        # Pourrait etre un xml avec les mots à retrouvés selon le contexte et le désir donne le contexte
        if 'scenario' in str(SpeechText):
            return True
        else:
            return False

    def cancel_criterion(self):
        """
        This method follows up the environment of the robot and return True if
         it is now impossible to accomplish the purpose of the desire.
        """
        #should look for the ambiant noise and if it is too high for a certain time, consider it is impossible
        return False
        
    def node_shutdown(self):
        """
        This method informs the developper about the shutdown of this node
        """
        common.loginfo(self,"have been shutdown")

    

if __name__ == '__main__':
    """
    This method starts a node with the name of the file and calls
    the transform function. It only shutdown if an extern event ask for it
    """
    try:
        rospy.init_node(common.get_file_name(__file__))
        node = Speech_recognition_observer()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(traceback.format_exc())
