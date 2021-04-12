#! /usr/bin/env python
import rospy
import actionlib
import traceback

from std_msgs.msg import String, Bool
from hbba_msgs.msg import Desire, DesiresSet, Event

import HomoDeUS_common_py.HomoDeUS_common_py as common

class Keyword_detection_observer:
    """
    This class follows the state of the keyword_detection desire
    """
    def __init__(self):
        # The input of the module
        self.input_motv = rospy.Subscriber("/proc_output_keywordDetect", Bool, self.desire_event_change, queue_size=10)

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
        
    def desire_event_change(self, detection):
        """
        This method is the callback of the output of the module it observes. it sends an event depending on the result
        get from the topic.

        Arguments
        ---------
        SpeechText: Bool
            The result of the output of the module it observes. In this case it is the detection of the keyword
        """
        # S'il y a beaucoup de desirs du meme type... il me semble que ca ne fonctionnera pas
        event = Event()
        for desire in self.curDesireSet.desires:
            if desire.type == "Keyword_detection":
                event.desire = desire.id
                event.desire_type = desire.type
                if self.accomplish_criterion(detection.data):
                    event.type = Event.ACC_ON
                    self.event_publisher.publish(event)
                elif self.cancel_criterion():
                #Probleme actuel est que si le desir n'est pas active il peut tout de meme changer d'etat!
                    event.type = Event.IMP_ON
                    self.event_publisher.publish(event)


    def accomplish_criterion(self, detection):
        """
        This method looks the result of the module and return True if it accomplished the criterion the motivation is looking for
        """
        return detection
        
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
        node = Keyword_detection_observer()
        rospy.on_shutdown(node.node_shutdown)
        rospy.spin()

    except Exception:
        common.logerr(__file__,traceback.format_exc())
