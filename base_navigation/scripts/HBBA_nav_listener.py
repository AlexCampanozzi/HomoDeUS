#!/usr/bin/env python2
import rospy

from navigator import Navigator
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class HBBA_nav_listener(Navigator):
    def __init__(self):
        Navigator.__init__(self)
        self.result_pub = rospy.Publisher("nav_result", Bool, queue_size=5)
        # navigator inits a node, therefore cant init one here
        # rospy.init_node('HBBA_nav_listener', anonymous=True)

    def callback(self, data):
        nav_result = self.goto(data.pose.position.x, data.pose.position.y, data.pose.orientation.w)
        self.result_pub.publish(nav_result)

    def listen(self):
        rospy.Subscriber('/hbba_nav_goal', PoseStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    try:
        listener = HBBA_nav_listener()
        listener.listen()
    except rospy.ROSInterruptException:
        pass

