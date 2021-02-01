#!/usr/bin/env python2
import rospy

from navigator import Navigator
from geometry_msgs.msg import PoseStamped

class HBBA_nav_listener(Navigator):
    def __init__(self):
        Navigator.__init__(self)
        # navigator inits a node, therefore cant init one here
        # rospy.init_node('HBBA_nav_listener', anonymous=True)

    def callback(self, data):
        self.goto(data.pose.position.x, data.pose.position.y, data.pose.orientation.w)

    def listen(self):
        rospy.Subscriber('hbba_navgoal', PoseStamped, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == '__main__':
    listener = HBBA_nav_listener()
    listener.listen()
