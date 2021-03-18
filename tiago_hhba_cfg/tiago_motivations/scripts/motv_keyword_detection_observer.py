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
        self.curDesireSet = desireSet
        
    def desire_event_change(self, detection):
        # S'il y a beaucoup de désirs du même type... il me semble que ca ne fonctionnera pas
        event = Event()
        for desire in self.curDesireSet.desires:
            if desire.type == "Keyword_detection":
                event.desire = desire.id
                event.desire_type = desire.type
                if self.accomplish_criterion(detection):
                    event.type = Event.ACC_ON
                    self.event_publisher.publish(event)
                elif self.cancel_criterion():
                #Problème actuel est que si le désir n'est pas activé il peut tout de même changer d'état!
                    event.type = Event.IMP_ON
                    self.event_publisher.publish(event)


    def accomplish_criterion(self, detection):
        return detection
        
    def cancel_criterion(self):
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
